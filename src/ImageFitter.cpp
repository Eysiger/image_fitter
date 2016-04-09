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
  //correlationPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(correlationMapTopic_,1);    // publisher for correlation_map
  activityCheckTimer_ = nodeHandle_.createTimer(activityCheckDuration_,
                                                &ImageFitter::updateSubscriptionCallback,
                                                this);
  broadcastTimer_ = nodeHandle_.createTimer(ros::Duration(0.01), &ImageFitter::tfBroadcast, this);
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
  //nodeHandle_.param("correlation_map_topic", correlationMapTopic_, std::string("/correlation_best_rotation/correlation_map"));

  nodeHandle_.param("angle_increment", angleIncrement_, 10);
  nodeHandle_.param("position_increment_search", searchIncrement_, 5);
  nodeHandle_.param("position_increment_correlation", correlationIncrement_, 5);
  nodeHandle_.param("required_overlap", requiredOverlap_, float(0.75));
  nodeHandle_.param("correlation_threshold", corrThreshold_, float(0.75));

  double activityCheckRate;
  nodeHandle_.param("activity_check_rate", activityCheckRate, 1.0);
  activityCheckDuration_.fromSec(1.0 / activityCheckRate);
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
  exhaustiveSearch();
}

void ImageFitter::convertToImages()
{

  // TODO Convert map to resolution of refferenceMap if necessary
  float map_min = -1;   //TODO automate
  float map_max = 0;    //TODO automate
  grid_map::GridMapRosConverter::toCvImage(map_, "elevation", mapImage_, map_min, map_max);

  //generate list of all defined points
  
  definedPoints.reserve(mapImage_.rows*mapImage_.cols);
  for(int j=0; j<mapImage_.rows; ++j)
    for(int i=0; i<mapImage_.cols; ++i)
    {
      if(mapImage_.at<cv::Vec<uchar, 4>>(j,i)[3] == std::numeric_limits<unsigned char>::max())
      {
        definedPoints.push_back(cv::Point(i,j));
      }
    }
  //crop image to bounding rectangle around defined points
  /*cv::Rect boundRect = cv::boundingRect(definedPoints);
  mapImage_ = mapImage_(boundRect);*/

  cv_bridge::CvImage mapImage_msg;
  mapImage_msg.header.stamp = ros::Time::now();
  mapImage_msg.header.frame_id = "map"; //later perhaps map_rotated
  mapImage_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  mapImage_msg.image = mapImage_;

  mapImagePublisher_.publish(mapImage_msg.toImageMsg());

  float referenceMap_min = -1;   //TODO automate
  float referenceMap_max = 0;    //TODO automate
  grid_map::GridMapRosConverter::toCvImage(referenceMap_, "elevation", referenceMapImage_, referenceMap_min, referenceMap_max);

  //generate list of all defined points
  referenceDefinedPoints.reserve(referenceMapImage_.rows*referenceMapImage_.cols);
  for(int j=0; j<referenceMapImage_.rows; ++j)
    for(int i=0; i<referenceMapImage_.cols; ++i)
    {
      if(referenceMapImage_.at<cv::Vec<uchar, 4>>(j,i)[3] == std::numeric_limits<unsigned char>::max())
      {
        referenceDefinedPoints.push_back(cv::Point(i,j));
      }
    }
  //crop image to bounding rectangle around defined points
  /*boundRect = cv::boundingRect(referenceDefinedPoints);
  referenceMapImage_ = referenceMapImage_(boundRect);*/
}

void ImageFitter::exhaustiveSearch()
{
  int result_cols =  referenceMapImage_.cols - mapImage_.cols + 1;
  int result_rows = referenceMapImage_.rows - mapImage_.rows + 1;
  cv::Mat result;
  result.create(result_rows, result_cols, CV_32FC1 );
  //cv::matchTemplate(mapImage_, referenceMapImage_, result, CV_TM_CCORR);
  cv::Point2f center(mapImage_.cols/2.0, mapImage_.rows/2.0);

  float best_corr[int(360/angleIncrement_)];
  int best_row[int(360/angleIncrement_)];
  int best_col[int(360/angleIncrement_)];
  grid_map::Position correct_position = map_.getPosition();
  //float correlation[referenceMapImage_.rows][referenceMapImage_.cols][int(360/angleIncrement_)];
  ros::Time time = ros::Time::now();
  for (float theta = 0; theta < 360; theta+=angleIncrement_)
  {
    cv::Mat rotMat = cv::getRotationMatrix2D(center, theta, 1.0);
    cv::Rect rotRect=cv::RotatedRect(center,mapImage_.size(), theta).boundingRect();
    rotMat.at<double>(0,2) += rotRect.width/2.0 - center.x;
    rotMat.at<double>(1,2) += rotRect.height/2.0 - center.y;
    cv::Mat rotatedImage;
    warpAffine(mapImage_, rotatedImage, rotMat, rotRect.size());
    //cv::imwrite("rotatedImage.png", rotatedImage);
    best_corr[int(theta/angleIncrement_)] = -1.0;
    for (int row = 0; row <= referenceMapImage_.rows-searchIncrement_; row+=searchIncrement_)
    {
      for (int col = 0; col <= referenceMapImage_.cols-searchIncrement_; col+=searchIncrement_)
      {
        int points = 0;
        int matches = 0;

        float shifted_mean = 0;
        float reference_mean = 0;
        std::vector<float> xy_shifted;
        std::vector<float> xy_reference;
        for (int i = 0; i <= rotatedImage.rows-correlationIncrement_; i+=correlationIncrement_) 
        {
          for (int j = 0; j <= rotatedImage.cols-correlationIncrement_; j+=correlationIncrement_)
          {
            if (rotatedImage.at<cv::Vec<uchar, 4>>(i,j)[3] == std::numeric_limits<unsigned char>::max())
            {
              points += 1;
              int reference_row = row-rotatedImage.rows/2+i;
              int reference_col = col-rotatedImage.cols/2+j;
              if (reference_row >= 0 && reference_row < referenceMapImage_.rows &&reference_col >= 0 && reference_col < referenceMapImage_.cols)
              {
                if (referenceMapImage_.at<cv::Vec<uchar, 4>>(reference_row,reference_col)[3] == std::numeric_limits<unsigned char>::max())
                {
                  matches += 1;
                  int mapHeight = rotatedImage.at<cv::Vec<uchar, 4>>(i,j)[0];
                  int referenceHeight = referenceMapImage_.at<cv::Vec<uchar, 4>>(reference_row,reference_col)[0];
                  shifted_mean += mapHeight;
                  reference_mean += referenceHeight;
                  xy_shifted.push_back(mapHeight);
                  xy_reference.push_back(referenceHeight);
                }
              }
            }
          }
        }
        if (matches > points*requiredOverlap_) 
        { 
          // calculate Normalized Cross Correlation (NCC)
          shifted_mean = shifted_mean/matches;
          reference_mean = reference_mean/matches;
          float shifted_normal = 0;
          float reference_normal = 0;
          float correlation = 0;
          for (int i = 0; i < matches; i++) 
          {
            float shifted_corr = (xy_shifted[i]-shifted_mean);
            float reference_corr = (xy_reference[i]-reference_mean);
            correlation += shifted_corr*reference_corr;
            shifted_normal += shifted_corr*shifted_corr;
            reference_normal += reference_corr*reference_corr;
          }
          correlation = correlation/sqrt(shifted_normal*reference_normal);
          if (correlation > best_corr[int(theta/angleIncrement_)])
          {
            best_corr[int(theta/angleIncrement_)] = correlation;
            best_row[int(theta/angleIncrement_)] = row;
            best_col[int(theta/angleIncrement_)] = col;
          }
        }
      }
    }
  }
  float bestCorr = -1.0;
  int bestTheta;
  float bestX;
  float bestY;
  grid_map::Position position = referenceMap_.getPosition();
  grid_map::Length length = referenceMap_.getLength();
  for (int i = 0; i < int(360/angleIncrement_); i++)
  {
    if (best_corr[i] > bestCorr && best_corr[i] >= corrThreshold_) 
    {
      bestCorr = best_corr[i];
      bestTheta = i*angleIncrement_;
      bestX = position(0) + length(0)/2 - best_row[i]*referenceMap_.getResolution(); //TODO consider image shift
      bestY = position(1) + length(1)/2 - best_col[i]*referenceMap_.getResolution(); //TODO consider image shift
    }
  }
  ros::Duration duration = ros::Time::now() - time;
  // output best correlation and time used
  std::cout << "Best correlation " << bestCorr << " at " << bestX << ", " << bestY << " and theta " << bestTheta << std::endl;
  std::cout << "Correct position " << correct_position.transpose() << " and theta 0" << std::endl;
  std::cout << "Time used: " << duration.toSec() << " Sekunden" << std::endl;
  ROS_INFO("done");
  isActive_ = false;
  // TODO: write code that saves values in csv

  /*double minVal; 
  double maxVal; 
  cv::Point minLoc; 
  cv::Point maxLoc;
  minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );
  cv::rectangle(referenceMapImage_, maxLoc, cv::Point( maxLoc.x + mapImage_.cols , maxLoc.y + mapImage_.rows ), cv::Scalar::all(255), 1, 8, 0 );

  cv_bridge::CvImage mapImage_msg;
  mapImage_msg.header.stamp = ros::Time::now();
  mapImage_msg.header.frame_id = "map"; 
  mapImage_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  mapImage_msg.image = referenceMapImage_;

  referenceMapImagePublisher_.publish(mapImage_msg.toImageMsg());*/
}

void ImageFitter::shift(grid_map::Position position, int theta)
{
}
float ImageFitter::correlationNCC(grid_map::Position position, int theta)
{
}

void ImageFitter::tfBroadcast(const ros::TimerEvent&) 
{
  broadcaster_.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)), 
          ros::Time::now(),"/world", "/map"));
}

} /* namespace */
