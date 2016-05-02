/*
 * ImageFitter.cpp
 *
 *  Created on: Apr 04, 2016
 *      Author: Roman Käslin
 *   Institute: ETH Zurich, Autonomous Systems Lab
 *
 */

#include <image_fitter/ImageFitter.h>

namespace image_fitter {

ImageFitter::ImageFitter(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle), isActive_(false)
{
  ROS_INFO("Map fitter node started, ready to match some grid maps.");
  readParameters();
  mapImagePublisher_ = nodeHandle_.advertise<sensor_msgs::Image>(mapImageTopic_,1);   // publisher for map_image
  referenceMapImagePublisher_ = nodeHandle_.advertise<sensor_msgs::Image>(referenceMapImageTopic_,1);   // publisher for reference_map_image
  correlationPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(correlationMapTopic_,1);    // publisher for correlation_map
  activityCheckTimer_ = nodeHandle_.createTimer(activityCheckDuration_,
                                                &ImageFitter::updateSubscriptionCallback,
                                                this);
  broadcastTimer_ = nodeHandle_.createTimer(ros::Duration(0.01), &ImageFitter::tfBroadcast, this);
  corrPointPublisher_ = nodeHandle_.advertise<geometry_msgs::PointStamped>("/corrPoint",1);
  SSDPointPublisher_ = nodeHandle_.advertise<geometry_msgs::PointStamped>("/SSDPoint",1);
  SADPointPublisher_ = nodeHandle_.advertise<geometry_msgs::PointStamped>("/SADPoint",1);
  MIPointPublisher_ = nodeHandle_.advertise<geometry_msgs::PointStamped>("/MIPoint",1);
  correctPointPublisher_ = nodeHandle_.advertise<geometry_msgs::PointStamped>("/correctPoint",1);
}

ImageFitter::~ImageFitter()
{
}

bool ImageFitter::readParameters()
{
  nodeHandle_.param("map_topic", mapTopic_, std::string("/elevation_mapping_long_range/elevation_map"));
  nodeHandle_.param("reference_map_topic", referenceMapTopic_, std::string("/uav_elevation_mapping/uav_elevation_map"));
  nodeHandle_.param("map_image_topic", mapImageTopic_, std::string("/elevation_mapping_long_range/map_image"));
  nodeHandle_.param("reference_map_image_topic", referenceMapImageTopic_, std::string("/uav_elevation_mapping/reference_map_image"));
  nodeHandle_.param("correlation_map_topic", correlationMapTopic_, std::string("/correlation_best_rotation/correlation_map"));

  nodeHandle_.param("angle_increment", angleIncrement_, 360);
  nodeHandle_.param("position_increment_search", searchIncrement_, 5);
  nodeHandle_.param("position_increment_correlation", correlationIncrement_, 5);
  nodeHandle_.param("required_overlap", requiredOverlap_, float(0.75));
  nodeHandle_.param("correlation_threshold", corrThreshold_, float(0)); //0.65 weighted, 0.75 unweighted
  nodeHandle_.param("SSD_threshold", SSDThreshold_, float(10));
  nodeHandle_.param("SAD_threshold", SADThreshold_, float(10));
  nodeHandle_.param("MI_threshold", MIThreshold_, float(-10));

  double activityCheckRate;
  nodeHandle_.param("activity_check_rate", activityCheckRate, 10.0);
  activityCheckDuration_.fromSec(1.0 / activityCheckRate);
  cumulativeErrorCorr_ = 0;
  cumulativeErrorSSD_ = 0;
  cumulativeErrorSAD_ = 0;
  cumulativeErrorMI_ = 0;
  correctMatchesCorr_ = 0;
  correctMatchesSSD_ = 0;
  correctMatchesSAD_ = 0;
  correctMatchesMI_ = 0;
}

void ImageFitter::updateSubscriptionCallback(const ros::TimerEvent&)
{
  if (!isActive_) {
    mapSubscriber_ = nodeHandle_.subscribe(mapTopic_, 1, &ImageFitter::callback, this);
    isActive_ = true;
    ROS_DEBUG("Subscribed to grid map at '%s'.", mapTopic_.c_str());
  }
}

void ImageFitter::callback(const grid_map_msgs::GridMap& message)
{
  ROS_INFO("Map fitter received a map (timestamp %f) for matching.",
            message.info.header.stamp.toSec());
  grid_map::GridMapRosConverter::fromMessage(message, map_);

  grid_map::GridMapRosConverter::loadFromBag("/home/parallels/rosbags/reference_map_last.bag", referenceMapTopic_, referenceMap_);
  convertToImages();

  convertToWeightedImages();
  exhaustiveSearch();
}

void ImageFitter::convertToImages()
{

  // TODO Convert map to resolution of refferenceMap if necessary
  grid_map::GridMapCvConverter::toImage<unsigned char, 4>(map_, "elevation", CV_8UC4, -1, 0, mapImage_);

  /*cv::namedWindow("Map_", CV_WINDOW_AUTOSIZE );
  cv::imshow("Map_", mapImage_ );
  cv::waitKey(0);*/

  cv_bridge::CvImage mapImage_msg;
  mapImage_msg.header.stamp = ros::Time::now();
  mapImage_msg.header.frame_id = "grid_map"; //later perhaps map_rotated
  mapImage_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  mapImage_msg.image = mapImage_;
  mapImagePublisher_.publish(mapImage_msg.toImageMsg());

  grid_map::GridMapCvConverter::toImage<unsigned char, 4>(referenceMap_, "elevation", CV_8UC4, -1, 0, referenceMapImage_);

  /*cv::namedWindow("referenceMap_", CV_WINDOW_AUTOSIZE );
  cv::imshow("referenceMap_", referenceMapImage_ );
  cv::waitKey(0);*/

  mapImage_msg.header.stamp = ros::Time::now();
  mapImage_msg.image = referenceMapImage_;
  referenceMapImagePublisher_.publish(mapImage_msg.toImageMsg());

  //generate list of all defined points
  std::vector<cv::Point> referenceDefinedPoints;
  referenceDefinedPoints.reserve(referenceMapImage_.rows*referenceMapImage_.cols);
  for(int i=0; i<referenceMapImage_.rows; ++i)
    for(int j=0; j<referenceMapImage_.cols; ++j)
    {
      if(referenceMapImage_.at<cv::Vec<unsigned char, 4>>(i,j)[3] ==  std::numeric_limits<unsigned char>::max())
      {
        referenceDefinedPoints.push_back(cv::Point(j,i));
      }
    }
  // get bounding rectangle around defined points
  referenceBoundRect_ = cv::boundingRect(referenceDefinedPoints);
}

void ImageFitter::convertToWeightedImages()
{
  // TODO Convert map to resolution of refferenceMap if necessary
  grid_map::GridMapCvConverter::toWeightedImage<unsigned short, 4>(map_, "elevation", "variance", CV_16UC4, -1, 0, weightedMapImage_);

  /*cv::namedWindow("weightedMap_", CV_WINDOW_AUTOSIZE );
  cv::imshow("weightedMap_", weightedMapImage_ );
  cv::waitKey(0);*/

  cv_bridge::CvImage mapImage_msg;
  mapImage_msg.header.stamp = ros::Time::now();
  mapImage_msg.header.frame_id = "grid_map"; //later perhaps map_rotated
  mapImage_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  mapImage_msg.image = weightedMapImage_;
  mapImagePublisher_.publish(mapImage_msg.toImageMsg());

  // rotate template image to check robustness
  cv::Point2f center(weightedMapImage_.cols/2.0, weightedMapImage_.rows/2.0);
  templateRotation_ = 0;
  cv::Mat rotMat = cv::getRotationMatrix2D(center, templateRotation_, 1.0);
  cv::Rect rotRect = cv::RotatedRect(center, weightedMapImage_.size(), templateRotation_).boundingRect();
  rotMat.at<double>(0,2) += rotRect.width/2.0 - center.x;
  rotMat.at<double>(1,2) += rotRect.height/2.0 - center.y;
  warpAffine(weightedMapImage_, weightedMapImage_, rotMat, rotRect.size());

  grid_map::GridMapCvConverter::toWeightedImage<unsigned short, 4>(referenceMap_, "elevation", "variance", CV_16UC4, -1, 0, weightedReferenceMapImage_);

  mapImage_msg.header.stamp = ros::Time::now();
  mapImage_msg.image = weightedReferenceMapImage_;
  referenceMapImagePublisher_.publish(mapImage_msg.toImageMsg());

  //generate list of all defined points
  std::vector<cv::Point> referenceDefinedPoints;
  referenceDefinedPoints.reserve(weightedReferenceMapImage_.rows*weightedReferenceMapImage_.cols);
  for(int i=0; i<weightedReferenceMapImage_.rows; ++i)
    for(int j=0; j<weightedReferenceMapImage_.cols; ++j)
    {
      if(weightedReferenceMapImage_.at<cv::Vec<unsigned short, 4>>(i,j)[3] !=  std::numeric_limits<unsigned short>::min())
      {
        referenceDefinedPoints.push_back(cv::Point(j,i));
      }
    }
  // get bounding rectangle around defined points
  referenceBoundRect_ = cv::boundingRect(referenceDefinedPoints);
}

void ImageFitter::exhaustiveSearch()
{
  // initialize correlationMap
  grid_map::GridMap correlationMap({"correlation","rotationNCC","SSD","rotationSSD","SAD","rotationSAD", "MI", "rotationMI"});
  correlationMap.setGeometry(referenceMap_.getLength(), referenceMap_.getResolution()*searchIncrement_,
                              referenceMap_.getPosition()); //TODO only use submap
  correlationMap.setFrameId("grid_map");

  //initialize parameters
  int rows = int((referenceBoundRect_.br().y-referenceBoundRect_.tl().y)/searchIncrement_);
  int cols = int((referenceBoundRect_.br().x-referenceBoundRect_.tl().x)/searchIncrement_);
  int acceptedThetas[rows][cols];
  for (int i=0; i < rows; i++)
    for (int j=0; j < cols; j++)
      acceptedThetas[i][j]=0;

  float best_corr[int(360/angleIncrement_)];
  int corr_row[int(360/angleIncrement_)];
  int corr_col[int(360/angleIncrement_)];

  float best_SSD[int(360/angleIncrement_)];
  int SSD_row[int(360/angleIncrement_)];
  int SSD_col[int(360/angleIncrement_)];

  float best_SAD[int(360/angleIncrement_)];
  int SAD_row[int(360/angleIncrement_)];
  int SAD_col[int(360/angleIncrement_)];

  float best_MI[int(360/angleIncrement_)];
  int MI_row[int(360/angleIncrement_)];
  int MI_col[int(360/angleIncrement_)];

  grid_map::Position correct_position = map_.getPosition();
  grid_map::Position position = referenceMap_.getPosition();
  grid_map::Length length = referenceMap_.getLength();
  float resolution = referenceMap_.getResolution();
  //float correlation[referenceMapImage_.rows][referenceMapImage_.cols][int(360/angleIncrement_)];

  cv::Point2f center(weightedMapImage_.cols/2.0, weightedMapImage_.rows/2.0);
  ros::Time time = ros::Time::now();
  for (float theta = 0; theta < 360; theta+=angleIncrement_)
  {
    cv::Mat rotMat = cv::getRotationMatrix2D(center, theta, 1.0);
    cv::Rect rotRect = cv::RotatedRect(center, weightedMapImage_.size(), theta).boundingRect();
    rotMat.at<double>(0,2) += rotRect.width/2.0 - center.x;
    rotMat.at<double>(1,2) += rotRect.height/2.0 - center.y;
    cv::Mat weightedRotatedImage;
    warpAffine(weightedMapImage_, weightedRotatedImage, rotMat, rotRect.size());
    warpAffine(mapImage_, templateImage_, rotMat, rotRect.size());
   
    /*//generate list of all defined points
    std::vector<cv::Point> definedPoints;
    definedPoints.reserve(weightedRotatedImage.rows*weightedRotatedImage.cols);
    for(int i=0; i<weightedRotatedImage.rows; ++i)
    {
      for(int j=0; j<weightedRotatedImage.cols; ++j)
      {
        if(weightedRotatedImage.at<cv::Vec<unsigned short, 4>>(i,j)[3] !=  std::numeric_limits<unsigned short>::min())
        {
          definedPoints.push_back(cv::Point(j,i));
        }
      }
    }*/
    /*float mutInfoTest = mutualInformation( int((position(0) + length(0)/2 - correct_position(0))/resolution), int((position(1) + length(1)/2 - correct_position(1))/resolution));
  }/**/
    best_corr[int(theta/angleIncrement_)] = -1;
    best_SSD[int(theta/angleIncrement_)] = 10;
    best_SAD[int(theta/angleIncrement_)] = 10;
    best_MI[int(theta/angleIncrement_)] = -10;
    // only iterate through points within referenceBoundRect, get top left and bottom right coordinate
    for (int row = referenceBoundRect_.tl().y; row < referenceBoundRect_.br().y; row+=searchIncrement_)
    {
      for (int col = referenceBoundRect_.tl().x; col < referenceBoundRect_.br().x; col+=searchIncrement_)
      {
        float errSAD = 10;
        float errSSD = 10;
        float corrNCC = -1;
        float mutInfo = -10;

        bool success = findMatches(&weightedRotatedImage, row, col);
        if (success) 
        {

          //errSAD = errorSAD();
          //errSSD = errorSSD();
          //corrNCC = correlationNCC();

          errSAD = weightedErrorSAD();
          errSSD = weightedErrorSSD();
          corrNCC = weightedCorrelationNCC();
          mutInfo = mutualInformation( row, col);

          acceptedThetas[(int(row-referenceBoundRect_.tl().y)/searchIncrement_)][int((col-referenceBoundRect_.tl().x)/searchIncrement_)] += 1;

          // save calculated correlation in correlationMap
          grid_map::Position xy_position;
          xy_position(0) = position(0) + length(0)/2 - row*resolution - resolution/2;
          xy_position(1) = position(1) + length(1)/2 - col*resolution - resolution/2; 
          if (correlationMap.isInside(xy_position)) 
          {
            grid_map::Index correlation_index;
            correlationMap.getIndex(xy_position, correlation_index);

            bool valid = correlationMap.isValid(correlation_index, "correlation");
            // if no value so far or correlation smaller or correlation higher than for other thetas
            if (((valid == false) || (corrNCC+1,5 > correlationMap.at("correlation", correlation_index) ))) 
            {
              correlationMap.at("correlation", correlation_index) = corrNCC+1.5;  //set correlation
              correlationMap.at("rotationNCC", correlation_index) = theta;    //set theta
            }

            valid = correlationMap.isValid(correlation_index, "SSD");
            // if no value so far or correlation smaller or correlation higher than for other thetas
            if (((valid == false) || (errSSD*5 < correlationMap.at("SSD", correlation_index) ))) 
            {
              correlationMap.at("SSD", correlation_index) = errSSD*5;  //set correlation
              correlationMap.at("rotationSSD", correlation_index) = theta;    //set theta
            }

            valid = correlationMap.isValid(correlation_index, "SAD");
            // if no value so far or correlation smaller or correlation higher than for other thetas
            if (((valid == false) || (errSSD*5 < correlationMap.at("SAD", correlation_index) ))) 
            {
              correlationMap.at("SAD", correlation_index) = errSAD*5;  //set correlation
              correlationMap.at("rotationSAD", correlation_index) = theta;    //set theta
            }

            valid = correlationMap.isValid(correlation_index, "MI");
            // if no value so far or correlation smaller or correlation higher than for other thetas
            if (((valid == false) || (mutInfo > correlationMap.at("MI", correlation_index) ))) 
            {
              correlationMap.at("MI", correlation_index) = mutInfo;  //set correlation
              correlationMap.at("rotationMI", correlation_index) = theta;    //set theta
            }
          }
          // save best correlation for each theta
          if (corrNCC > best_corr[int(theta/angleIncrement_)])
          {
            best_corr[int(theta/angleIncrement_)] = corrNCC;
            corr_row[int(theta/angleIncrement_)] = row;
            corr_col[int(theta/angleIncrement_)] = col;
          }
          if (errSSD < best_SSD[int(theta/angleIncrement_)])
          {
            best_SSD[int(theta/angleIncrement_)] = errSSD;
            SSD_row[int(theta/angleIncrement_)] = row;
            SSD_col[int(theta/angleIncrement_)] = col;
          }
          if (errSAD < best_SAD[int(theta/angleIncrement_)])
          {
            best_SAD[int(theta/angleIncrement_)] = errSAD;
            SAD_row[int(theta/angleIncrement_)] = row;
            SAD_col[int(theta/angleIncrement_)] = col;
          }
          if (mutInfo > best_MI[int(theta/angleIncrement_)])
          {
            best_MI[int(theta/angleIncrement_)] = mutInfo;
            MI_row[int(theta/angleIncrement_)] = row;
            MI_col[int(theta/angleIncrement_)] = col;
          }
        }
      }
    }
    // publish correlationMap for each theta
    grid_map_msgs::GridMap correlation_msg;
    grid_map::GridMapRosConverter::toMessage(correlationMap, correlation_msg);
    correlationPublisher_.publish(correlation_msg);
  }
  //find highest correlation over all theta
  float bestCorr = -1.0;
  float bestSSD = 10;
  float bestSAD = 10;
  float bestMI = -10;
  int bestThetaCorr;
  int bestThetaSSD;
  int bestThetaSAD;
  int bestThetaMI;
  float bestXCorr;
  float bestYCorr;
  float bestXSSD;
  float bestYSSD;
  float bestXSAD;
  float bestYSAD;
  float bestXMI;
  float bestYMI;
  for (int i = 0; i < int(360/angleIncrement_); i++)
  {
    if (best_corr[i] > bestCorr && best_corr[i] >= corrThreshold_) 
    {
      //std::cout <<acceptedThetas[int((corr_row[i]-referenceBoundRect_.tl().y)/searchIncrement_)][int((corr_col[i]-referenceBoundRect_.tl().x)/searchIncrement_)] <<std::endl;
      if (acceptedThetas[int((corr_row[i]-referenceBoundRect_.tl().y)/searchIncrement_)][int((corr_col[i]-referenceBoundRect_.tl().x)/searchIncrement_)] == int(360/angleIncrement_))
      {
        bestCorr = best_corr[i];
        bestThetaCorr = i*angleIncrement_;
        bestXCorr = position(0) + length(0)/2 - corr_row[i]*resolution;
        bestYCorr = position(1) + length(1)/2 - corr_col[i]*resolution; 
      }
    }
    if (best_SSD[i] < bestSSD && best_SSD[i] <= SSDThreshold_) 
    {
      //std::cout << acceptedThetas[int((SSD_row[i]-referenceBoundRect_.tl().y)/searchIncrement_)][int((SSD_col[i]-referenceBoundRect_.tl().x)/searchIncrement_)] << std::endl;
      if (acceptedThetas[int((SSD_row[i]-referenceBoundRect_.tl().y)/searchIncrement_)][int((SSD_col[i]-referenceBoundRect_.tl().x)/searchIncrement_)] == int(360/angleIncrement_))
      {
        bestSSD = best_SSD[i];
        bestThetaSSD = i*angleIncrement_;
        bestXSSD = position(0) + length(0)/2 - SSD_row[i]*resolution;
        bestYSSD = position(1) + length(1)/2 - SSD_col[i]*resolution; 
      }
    }
    if (best_SAD[i] < bestSAD && best_SAD[i] <= SADThreshold_) 
    {
      //std::cout << acceptedThetas[int((SAD_row[i]-referenceBoundRect_.tl().y)/searchIncrement_)][int((SAD_col[i]-referenceBoundRect_.tl().x)/searchIncrement_)] << std::endl;
      if (acceptedThetas[int((SAD_row[i]-referenceBoundRect_.tl().y)/searchIncrement_)][int((SAD_col[i]-referenceBoundRect_.tl().x)/searchIncrement_)] == int(360/angleIncrement_))
      {
        bestSAD = best_SAD[i];
        bestThetaSAD = i*angleIncrement_;
        bestXSAD = position(0) + length(0)/2 - SAD_row[i]*resolution;
        bestYSAD = position(1) + length(1)/2 - SAD_col[i]*resolution; 
      }
    }
    if (best_MI[i] > bestMI && best_MI[i] >= MIThreshold_) 
    {
      //std::cout << acceptedThetas[int((SAD_row[i]-referenceBoundRect_.tl().y)/searchIncrement_)][int((SAD_col[i]-referenceBoundRect_.tl().x)/searchIncrement_)] << std::endl;
      if (acceptedThetas[int((MI_row[i]-referenceBoundRect_.tl().y)/searchIncrement_)][int((MI_col[i]-referenceBoundRect_.tl().x)/searchIncrement_)] == int(360/angleIncrement_))
      {
        bestMI = best_MI[i];
        bestThetaMI = i*angleIncrement_;
        bestXMI = position(0) + length(0)/2 - MI_row[i]*resolution;
        bestYMI = position(1) + length(1)/2 - MI_col[i]*resolution; 
      }
    }
  }
  float z = findZ(bestXCorr, bestYCorr, bestThetaCorr);

  ros::Time pubTime = ros::Time::now();
  // output best correlation and time used
  if (bestCorr != -1) 
  {
    cumulativeErrorCorr_ += sqrt((bestXCorr - correct_position(0))*(bestXCorr - correct_position(0)) + (bestYCorr - correct_position(1))*(bestYCorr - correct_position(1)));
    if (sqrt((bestXCorr - correct_position(0))*(bestXCorr - correct_position(0)) + (bestYCorr - correct_position(1))*(bestYCorr - correct_position(1))) < 0.5 && fabs(bestThetaCorr - (360-int(templateRotation_))%360) < angleIncrement_) {correctMatchesCorr_ += 1;}
    geometry_msgs::PointStamped corrPoint;
    corrPoint.point.x = bestXCorr;
    corrPoint.point.y = bestYCorr;
    corrPoint.point.z = bestThetaCorr;
    corrPoint.header.stamp = pubTime;
    corrPointPublisher_.publish(corrPoint);
  }
  if (bestSSD != 10) 
  {
    cumulativeErrorSSD_ += sqrt((bestXSSD - correct_position(0))*(bestXSSD - correct_position(0)) + (bestYSSD - correct_position(1))*(bestYSSD - correct_position(1)));
    if (sqrt((bestXSSD - correct_position(0))*(bestXSSD - correct_position(0)) + (bestYSSD - correct_position(1))*(bestYSSD - correct_position(1))) < 0.5 && fabs(bestThetaSSD - (360-int(templateRotation_))%360) < angleIncrement_) {correctMatchesSSD_ += 1;}
    geometry_msgs::PointStamped SSDPoint;
    SSDPoint.point.x = bestXSSD;
    SSDPoint.point.y = bestYSSD;
    SSDPoint.point.z = bestThetaSSD;
    SSDPoint.header.stamp = pubTime;
    SSDPointPublisher_.publish(SSDPoint);
  }
  if (bestSAD != 10) 
  {
    cumulativeErrorSAD_ += sqrt((bestXSAD - correct_position(0))*(bestXSAD - correct_position(0)) + (bestYSAD - correct_position(1))*(bestYSAD - correct_position(1)));
    if (sqrt((bestXSAD - correct_position(0))*(bestXSAD - correct_position(0)) + (bestYSAD - correct_position(1))*(bestYSAD - correct_position(1))) < 0.5 && fabs(bestThetaSAD - (360-int(templateRotation_))%360) < angleIncrement_) {correctMatchesSAD_ += 1;}
    geometry_msgs::PointStamped SADPoint;
    SADPoint.point.x = bestXSAD;
    SADPoint.point.y = bestYSAD;
    SADPoint.point.z = bestThetaSAD;
    SADPoint.header.stamp = pubTime;
    SADPointPublisher_.publish(SADPoint);
  }
  if (bestMI != 0) 
  {
    cumulativeErrorMI_ += sqrt((bestXMI - correct_position(0))*(bestXMI - correct_position(0)) + (bestYMI - correct_position(1))*(bestYMI - correct_position(1)));
    if (sqrt((bestXMI - correct_position(0))*(bestXMI - correct_position(0)) + (bestYMI - correct_position(1))*(bestYMI - correct_position(1))) < 0.5 && fabs(bestThetaMI - (360-int(templateRotation_))%360) < angleIncrement_) {correctMatchesMI_ += 1;}
    geometry_msgs::PointStamped MIPoint;
    MIPoint.point.x = bestXMI;
    MIPoint.point.y = bestYMI;
    MIPoint.point.z = bestThetaMI;
    MIPoint.header.stamp = pubTime;
    MIPointPublisher_.publish(MIPoint);
  }
  geometry_msgs::PointStamped correctPoint;
  correctPoint.point.x = correct_position(0);
  correctPoint.point.y = correct_position(1);
  correctPoint.point.z = 0;
  correctPoint.header.stamp = pubTime;
  correctPointPublisher_.publish(correctPoint);



  ros::Duration duration = ros::Time::now() - time;
  std::cout << "Best correlation " << bestCorr << " at " << bestXCorr << ", " << bestYCorr << " and theta " << bestThetaCorr << " and z: " << z << std::endl;
  std::cout << "Best SSD " << bestSSD << " at " << bestXSSD << ", " << bestYSSD << " and theta " << bestThetaSSD << std::endl;
  std::cout << "Best SAD " << bestSAD << " at " << bestXSAD << ", " << bestYSAD << " and theta " << bestThetaSAD << std::endl;
  std::cout << "Best MI " << bestMI << " at " << bestXMI << ", " << bestYMI << " and theta " << bestThetaMI << std::endl;
  std::cout << "Correct position " << correct_position.transpose() << " and theta " << (360-int(templateRotation_))%360 << std::endl;
  std::cout << "Time used: " << duration.toSec() << " Sekunden" << std::endl;
  std::cout << "Cumulative error NCC: " << cumulativeErrorCorr_ << " matches: " << correctMatchesCorr_ << " SSD: " << cumulativeErrorSSD_ << " matches: " << correctMatchesSSD_ << " SAD: " << cumulativeErrorSAD_ << " matches: " << correctMatchesSAD_ << " MI: " << cumulativeErrorMI_ << " matches: " << correctMatchesMI_ << std::endl;
  ROS_INFO("done");
  isActive_ = false;
  // TODO: write code that saves values in csv

  /*
  int result_cols =  referenceMapImage_.cols - mapImage_.cols + 1;
  int result_rows = referenceMapImage_.rows - mapImage_.rows + 1;
  cv::Mat result;
  result.create(result_rows, result_cols, CV_32FC1 );

  cv::matchTemplate(mapImage_, referenceMapImage_, result, CV_TM_CCORR);

  double minVal; 
  double maxVal; 
  cv::Point minLoc; 
  cv::Point maxLoc;
  minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );
  cv::rectangle(referenceMapImage_, maxLoc, cv::Point( maxLoc.x + mapImage_.cols , maxLoc.y + mapImage_.rows ), cv::Scalar::all(255), 1, 8, 0 );

  cv_bridge::CvImage mapImage_msg;
  mapImage_msg.header.stamp = ros::Time::now();
  mapImage_msg.header.frame_id = "grid_map"; 
  mapImage_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  mapImage_msg.image = referenceMapImage_;

  referenceMapImagePublisher_.publish(mapImage_msg.toImageMsg());
  */
}

float ImageFitter::findZ(float x, float y, int theta)
{
  // initialize
  float shifted_mean = 0;
  float reference_mean = 0;
  int matches = 0;

  grid_map::Matrix& data = map_["elevation"];
  for (grid_map::GridMapIteratorSparse iterator(map_, correlationIncrement_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    float shifted = data(index(0), index(1));
    if (shifted == shifted) {   // check if point is defined, if nan f!= f 
      grid_map::Position xy_position;
      map_.getPosition(index, xy_position);  // get coordinates
      tf::Vector3 xy_vector = tf::Vector3(xy_position(0), xy_position(1), 0.0);

      // transform coordinates from /map_rotated to /grid_map
      tf::Transform transform = tf::Transform(tf::Quaternion(0.0, 0.0, sin(theta/180*M_PI/2), cos(theta/180*M_PI/2)), tf::Vector3(x, y, 0.0));
      tf::Vector3 map_vector = transform*(xy_vector); // apply transformation
      grid_map::Position map_position;
      map_position(0) = map_vector.getX();
      map_position(1) = map_vector.getY();

      // check if point is within reference_map
      if (referenceMap_.isInside(map_position)) {
        float reference = referenceMap_.atPosition("elevation", map_position);
        if (reference == reference) {   // check if point is defined, if nan f!= f 
          matches += 1;
          shifted_mean += shifted;
          reference_mean += reference;
        }
      }
    }
  }
  // calculate mean
  shifted_mean = shifted_mean/matches;
  reference_mean = reference_mean/matches;

  return reference_mean - shifted_mean;
}

bool ImageFitter::findMatches(cv::Mat *rotatedImage, int row, int col)
{
  // initialize
  int points = 0;
  matches_ = 0;

  shifted_mean_ = 0;
  reference_mean_ = 0;
  xy_shifted_.clear();
  xy_reference_.clear();
  xy_shifted_var_.clear();
  //xy_reference_var_.clear();
  row_matches_.clear();
  col_matches_.clear();

  // only iterate through definedPoints
  /*for (int rotPoint = 0; rotPoint < definedPoints.size(); rotPoint+=correlationIncrement_)
  {
    int i = definedPoints[rotPoint].y;
    int j = definedPoints[rotPoint].x;*/
  for (int i = 0; i <= rotatedImage->rows-correlationIncrement_; i+=correlationIncrement_) 
  {
    for (int j = 0; j <= rotatedImage->cols-correlationIncrement_; j+=correlationIncrement_)
    {
      //check if pixel is defined, obsolet if only iterated through defined Points
      if (rotatedImage->at<cv::Vec<unsigned short, 4>>(i,j)[3] != std::numeric_limits<unsigned short>::min())
      {
        points += 1;
        int reference_row = row-rotatedImage->rows/2+i;
        int reference_col = col-rotatedImage->cols/2+j;
        // check if corresponding pixel is within referenceMapImage
        if (reference_row >= 0 && reference_row < weightedReferenceMapImage_.rows &&reference_col >= 0 && reference_col < weightedReferenceMapImage_.cols)
        {
          // check if corresponding pixel is defined
          if (weightedReferenceMapImage_.at<cv::Vec<unsigned short, 4>>(reference_row,reference_col)[3] != std::numeric_limits<unsigned short>::min())
          {
            matches_ += 1;
            float mapHeight = rotatedImage->at<cv::Vec<unsigned short, 4>>(i,j)[0];
            float mapVariance = rotatedImage->at<cv::Vec<unsigned short, 4>>(i,j)[3];
            float referenceHeight = weightedReferenceMapImage_.at<cv::Vec<unsigned short, 4>>(reference_row,reference_col)[0];
            //float referenceVariance = weightedReferenceMapImage_.at<cv::Vec<unsigned short, 4>>(reference_row,reference_col)[3];
            shifted_mean_ += mapHeight;
            reference_mean_ += referenceHeight;
            xy_shifted_.push_back(mapHeight);
            xy_reference_.push_back(referenceHeight);
            //xy_shifted_var_.push_back(1 / mapVariance); // std::numeric_limits<unsigned short>::max() - mapVariance or 1 / mapVariance
            xy_shifted_var_.push_back(std::numeric_limits<unsigned short>::max() - mapVariance);
            //xy_reference_var_.push_back((std::numeric_limits<unsigned short>::max()-referenceVariance)/std::numeric_limits<unsigned short>::max());
            row_matches_.push_back(i);
            col_matches_.push_back(j);
          }
        }
      }
    }
  }
  // check if required overlap is fulfilled
  if (matches_ > points*requiredOverlap_) 
  { 
    shifted_mean_ = shifted_mean_/matches_;
    reference_mean_ = reference_mean_/matches_;
    return true; 
  }
  else { return false; }
}

float ImageFitter::mutualInformation(int row, int col)
{
  //include zero mean!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  cv::Mat hist( 256, 1, cv::DataType<float>::type, 0.0); // 512 with mean
  cv::Mat referenceHist( 256, 1, cv::DataType<float>::type, 0.0); // 512 with mean
  cv::Mat jointHist( 256, 256, cv::DataType<float>::type, 0.0); //512, 512 with mean
  int count = 0;
  std::vector<int> templateI;
  std::vector<int> referenceI;
  //float template_mean = 0;
  //float reference_mean = 0;

  for (int i = 0; i < templateImage_.rows; i++) 
  {
    for (int j = 0; j < templateImage_.cols; j++)
    {
      //check if pixel is defined, obsolet if only iterated through defined Points
      if (templateImage_.at<cv::Vec<unsigned char, 4>>(i,j)[3] == std::numeric_limits<unsigned char>::max())
      {
        int reference_row = row-templateImage_.rows/2+i;
        int reference_col = col-templateImage_.cols/2+j;
        // check if corresponding pixel is within referenceMapImage
        if (reference_row >= 0 && reference_row < referenceMapImage_.rows &&reference_col >= 0 && reference_col < referenceMapImage_.cols)
        {
          // check if corresponding pixel is defined
          if (referenceMapImage_.at<cv::Vec<unsigned char, 4>>(reference_row,reference_col)[3] == std::numeric_limits<unsigned char>::max())
          {
            int temp = templateImage_.at<cv::Vec<unsigned char, 4>>(i,j)[0];
            int ref = referenceMapImage_.at<cv::Vec<unsigned char, 4>>(reference_row,reference_col)[0];
            templateI.push_back(temp);
            referenceI.push_back(ref);
            //template_mean += temp;
            //reference_mean += ref;
            count += 1;
          }
        }
      }
    }
  }
  //template_mean = template_mean/count;
  //reference_mean = reference_mean/count;
  for (int k=0; k < count; k++)
  {
      int i1 = templateI[k]; //- template_mean + 255;
      int i2 = referenceI[k]; //- reference_mean + 255;
      hist.at<float>(i1, 0) += 1;
      referenceHist.at<float>(i2, 0) += 1;
      jointHist.at<float>(i1, i2) += 1;
  }
  /*for (int k=0; k < matches_; k++)
  {
      int i1 = xy_shifted_[k]/256 -1 + 1/256; //- template_mean + 255;
      int i2 = xy_reference_[k]/256 -1 + 1/256; //- reference_mean + 255;
      hist.at<float>(i1, 0) += 1;
      referenceHist.at<float>(i2, 0) += 1;
      jointHist.at<float>(i1, i2) += 1;
      count += 1;
  }*/

  hist = hist/count;
  referenceHist = referenceHist/count;
  jointHist = jointHist/count;

  cv::Mat logP;
  cv::log(hist,logP);
  cv::Mat referenceLogP;
  cv::log(referenceHist,referenceLogP);
  cv::Mat jointLogP;
  cv::log(jointHist,jointLogP);

  cv::Mat weightedHist( 256, 256, cv::DataType<float>::type, 0.0);
  for (int i=0; i < 256; i++)
  {
    for (int j=i; j < 256; j++)
    {
      weightedHist.at<float>(i,j) = float((j-i)+1)/32;
      weightedHist.at<float>(j,i) = weightedHist.at<float>(i,j);
    }
  } 
  float entropy = -1*cv::sum(hist.mul(logP)).val[0];
  float referenceEntropy = -1*cv::sum(referenceHist.mul(referenceLogP)).val[0];

  //float norm = cv::norm(weightedHist, cv::NORM_L1);
  //weightedHist = weightedHist/norm;
  //std::cout << weightedHist <<std::endl;
  weightedHist = weightedHist.mul(jointHist);

  cv::Mat divLogP;
  cv::gemm(hist, referenceHist, 1, cv::Mat(), 0, divLogP, cv::GEMM_2_T);
  cv::divide(jointHist, divLogP, divLogP);
  cv::log(divLogP, divLogP);

  float jointEntropy = -1*cv::sum(weightedHist.mul(jointLogP)).val[0];
  float mutualDiv = cv::sum(weightedHist.mul(divLogP)).val[0];
  //std::cout << " Mutual information: " << entropy+referenceEntropy-jointEntropy << " by division: " << mutualDiv <<std::endl;
  //std::cout << count << " template entropy: " << entropy << " reference entropy: " << referenceEntropy << " joint entropy: " << jointEntropy << " Mutual information: " << entropy+referenceEntropy-jointEntropy <<std::endl;

  /*jointHist = jointHist * 255;

  cv::Mat histImage( 256, 256, cv::DataType<float>::type, 0.0);
  cv::Mat histImage2( 256, 256, cv::DataType<float>::type, 0.0);

  cv::normalize(hist, hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
  cv::normalize(referenceHist, referenceHist, 0, histImage2.rows, cv::NORM_MINMAX, -1, cv::Mat() );

  for( int i = 1; i < 256; i++ )
  {
    cv::line( histImage, cv::Point( (i-1), 255 - cvRound(hist.at<float>(i-1)) ) ,
                     cv::Point(i, 255 - cvRound(hist.at<float>(i)) ),
                     cv::Scalar( 255, 0, 0), 1, 8, 0  );
    cv::line( histImage2, cv::Point( (i-1), 255 - cvRound(referenceHist.at<float>(i-1)) ) ,
                     cv::Point( i, 255 - cvRound(referenceHist.at<float>(i)) ),
                     cv::Scalar( 255, 0, 0), 1, 8, 0  );
  }
  std::vector<cv::Mat> channels; 
  channels.push_back(histImage);
  channels.push_back(histImage2);
  channels.push_back(jointHist);
  cv::merge(channels, histImage);
  cv::namedWindow("calcHist", CV_WINDOW_AUTOSIZE );
  cv::imshow("calcHist", histImage );
  cv::waitKey(0);*/

  return (entropy+referenceEntropy-jointEntropy); // /jointEntropy or /sqrt(entropy*referenceEntropy);
  
  /*cv::Mat hist;
  std::vector<cv::Mat> bgr_planes;
  cv::split( mapImage_, bgr_planes );

  int histSize = 256;
  float range[] = { 0, 256 } ;
  const float* histRange = { range };
  cv::calcHist(&bgr_planes[0], 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, true, false );

  cv::normalize(hist, hist, 1.0, 0.0, cv::NORM_L1);
  
  cv::Mat logP;
  cv::log(hist,logP);

  float entropy = -1*cv::sum(hist.mul(logP)).val[0];
  std::cout << entropy << std::endl;

  cv::Point2f center(mapImage_.cols/2.0, mapImage_.rows/2.0);
  templateRotation_ = 5;
  cv::Mat rotMat = cv::getRotationMatrix2D(center, templateRotation_, 1.0);
  cv::Rect rotRect = cv::RotatedRect(center, mapImage_.size(), templateRotation_).boundingRect();
  rotMat.at<double>(0,2) += rotRect.width/2.0 - center.x;
  rotMat.at<double>(1,2) += rotRect.height/2.0 - center.y;
  cv::Mat rotatedMapImage;
  warpAffine(mapImage_, rotatedMapImage, rotMat, mapImage_.size());//rotRect.size());

  cv::Mat rotatedHist;
  std::vector<cv::Mat> bgr_planes2;
  cv::split( rotatedMapImage, bgr_planes2 );
  
  cv::calcHist(&bgr_planes2[0], 1, 0, cv::Mat(), rotatedHist, 1, &histSize, &histRange, true, false );

  cv::normalize(rotatedHist, rotatedHist, 1.0, 0.0, cv::NORM_L1);
  
  cv::Mat rotatedLogP;
  cv::log(rotatedHist,rotatedLogP);

  float rotatedEntropy = -1*cv::sum(rotatedHist.mul(rotatedLogP)).val[0];
  std::cout << rotatedEntropy << std::endl;

  cv::Mat jointHist( 256, 256, cv::DataType<float>::type, 0.0);
  //cv::calcHist( (&bgr_planes[0], &bgr_planes2[0]), 2, 0, cv::Mat(), jointHist, 2, {&histSize, &histSize}, {&histRange, &histRange}, true, false );
  int count = 0;
  for (int y=0; y < mapImage_.size().width; y++)
  {
    for (int x=0; x < mapImage_.size().height; x++)
    {
      int int1 = mapImage_.at<unsigned char>(x,y);
      int int2 = rotatedMapImage.at<unsigned char>(x,y);
      jointHist.at<float>(int1, int2) += 1;
      count += 1;
    }
  }
  jointHist = jointHist/count;

  cv::Mat jointLogP;
  cv::log(jointHist,jointLogP);

  float jointEntropy = -1*cv::sum(jointHist.mul(jointLogP)).val[0];
  std::cout << jointEntropy << std::endl;

  std::cout << "Mutual information: " << entropy+rotatedEntropy-jointEntropy <<std::endl;
  jointHist = jointHist*std::numeric_limits<unsigned short>::max();

  return entropy+rotatedEntropy-jointEntropy;
  */
  /*cv::namedWindow("calcHist", CV_WINDOW_AUTOSIZE );
  cv::imshow("calcHist", jointHist );
  cv::waitKey(0);*/
  /*int hist_w = 512; 
  int hist_h = 400;
  int bin_w = cvRound( (double) hist_w/histSize );
  cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );

  cv::normalize(jointHist, jointHist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );

  for( int i = 1; i < histSize; i++ )
  {
    cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(jointHist.at<float>(i-1)) ) ,
                     cv::Point( bin_w*(i), hist_h - cvRound(jointHist.at<float>(i)) ),
                     cv::Scalar( 255, 0, 0), 2, 8, 0  );
  }
  cv::namedWindow("calcHist", CV_WINDOW_AUTOSIZE );
  cv::imshow("calcHist", histImage );
  cv::waitKey(0);*/
}

float ImageFitter::errorSAD()
{
  float error = 0;
  for (int i = 0; i < matches_; i++) 
  {
    float shifted = (xy_shifted_[i]-shifted_mean_)/std::numeric_limits<unsigned short>::max();
    float reference = (xy_reference_[i]-reference_mean_)/std::numeric_limits<unsigned short>::max();
    error += fabs(shifted-reference);
  }
  // divide error by number of matches
  //std::cout << error/matches_ <<std::endl;
  return error/matches_;
}

float ImageFitter::weightedErrorSAD()
{
  float error = 0;
  float normalization = 0;
  for (int i = 0; i < matches_; i++) 
  {
    float shifted = (xy_shifted_[i]-shifted_mean_)/std::numeric_limits<unsigned short>::max();
    float reference = (xy_reference_[i]-reference_mean_)/std::numeric_limits<unsigned short>::max();
    error += fabs(shifted-reference) * xy_shifted_var_[i];// * xy_reference_var_[i];
    normalization += xy_shifted_var_[i];// * (xy_reference_var_[i]/std::numeric_limits<unsigned short>::max());
  }
  // divide error by number of matches
  //std::cout << error/normalization <<std::endl;
  return error/normalization;
}

float ImageFitter::errorSSD()
{
  float error = 0;
  for (int i = 0; i < matches_; i++) 
  {
    float shifted = (xy_shifted_[i]-shifted_mean_)/std::numeric_limits<unsigned short>::max();
    float reference = (xy_reference_[i]-reference_mean_)/std::numeric_limits<unsigned short>::max();
    error += (shifted-reference)*(shifted-reference); //sqrt(fabs(shifted-reference)) instead of (shifted-reference)*(shifted-reference), since values are in between 0 and 1
  }
  // divide error by number of matches
  //std::cout << error/matches_ <<std::endl;
  return error/matches_;
}

float ImageFitter::weightedErrorSSD()
{
  float error = 0;
  float normalization = 0;
  for (int i = 0; i < matches_; i++) 
  {
    float shifted = (xy_shifted_[i]-shifted_mean_)/std::numeric_limits<unsigned short>::max();
    float reference = (xy_reference_[i]-reference_mean_)/std::numeric_limits<unsigned short>::max();
    error += (shifted-reference)*(shifted-reference) * xy_shifted_var_[i]*xy_shifted_var_[i];// * xy_reference_var_[i]; //sqrt(fabs(shifted-reference)) instead of (shifted-reference)*(shifted-reference), since values are in between 0 and 1
    normalization += xy_shifted_var_[i]*xy_shifted_var_[i];// * (xy_reference_var_[i]/std::numeric_limits<unsigned short>::max());
  }
  // divide error by number of matches
  //std::cout << error/normalization <<std::endl;
  return error/normalization;
}

float ImageFitter::correlationNCC()
{
  float shifted_normal = 0;
  float reference_normal = 0;
  float correlation = 0;
  for (int i = 0; i < matches_; i++) 
  {
    float shifted_corr = (xy_shifted_[i]-shifted_mean_);
    float reference_corr = (xy_reference_[i]-reference_mean_);
    correlation += shifted_corr*reference_corr;
    shifted_normal += shifted_corr*shifted_corr;
    reference_normal += reference_corr*reference_corr;
  }
  return correlation/sqrt(shifted_normal*reference_normal);
}

float ImageFitter::weightedCorrelationNCC()
{
  // calculate Normalized Cross Correlation (NCC)
  float shifted_normal = 0;
  float reference_normal = 0;
  float correlation = 0;
  for (int i = 0; i < matches_; i++) 
  {
    //for 1/variance
    float shifted_corr = (xy_shifted_[i]-shifted_mean_); // * xy_shifted_var_[i]
    float reference_corr = (xy_reference_[i]-reference_mean_);// * xy_reference_var_[i];
    correlation += shifted_corr*reference_corr * xy_shifted_var_[i];
    shifted_normal += xy_shifted_var_[i]*shifted_corr*shifted_corr;// shifted_corr*shifted_corr * xy_shifted_var_[i];
    reference_normal += xy_shifted_var_[i]*reference_corr*reference_corr;// reference_corr*reference_corr * xy_shifted_var_[i];
    
    //for 1 - variance
    /*float shifted_corr = (xy_shifted_[i]-shifted_mean_)* xy_shifted_var_[i];
    float reference_corr = (xy_reference_[i]-reference_mean_);// * xy_reference_var_[i];
    correlation += shifted_corr*reference_corr;
    shifted_normal += shifted_corr * shifted_corr;
    reference_normal += reference_corr * reference_corr;*/
  }
  return correlation/sqrt(shifted_normal*reference_normal);
}

void ImageFitter::tfBroadcast(const ros::TimerEvent&) 
{
  broadcaster_.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)), 
          ros::Time::now(),"/map", "/grid_map"));
}

} /* namespace */
