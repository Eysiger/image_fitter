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

  nodeHandle_.param("angle_increment", angleIncrement_, 45);
  nodeHandle_.param("position_increment_search", searchIncrement_, 5);
  nodeHandle_.param("position_increment_correlation", correlationIncrement_, 5);
  nodeHandle_.param("required_overlap", requiredOverlap_, float(0.75));

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

  //cv_bridge::CvImage mapImage_msg;
  mapImage_msg.header.stamp = ros::Time::now();
  mapImage_msg.header.frame_id = "map"; 
  mapImage_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  mapImage_msg.image = referenceMapImage_;

  referenceMapImagePublisher_.publish(mapImage_msg.toImageMsg());
}

void ImageFitter::exhaustiveSearch()
{
  /*int result_cols =  referenceMapImage_.cols - mapImage_.cols + 1;
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
  */
  
  std::vector<cv::KeyPoint> keypointsA, keypointsB;
  cv::Mat descriptorsA, descriptorsB;

  const int blur = 3;
  //cv::GaussianBlur(referenceMapImage_, referenceMapImage_, cv::Size(blur,blur), 0);
  //cv::GaussianBlur(mapImage_, mapImage_, cv::Size(blur,blur), 0);

  cv::Ptr<cv::FeatureDetector> detector = cv::Algorithm::create<cv::FeatureDetector>("Feature2D.HARRIS");
  detector->detect(mapImage_, keypointsA);
  detector->detect(referenceMapImage_, keypointsB);

  cv::Ptr<cv::DescriptorExtractor> descriptorExtractor =cv::Algorithm::create<cv::DescriptorExtractor>("Feature2D.BRISK");
  descriptorExtractor->compute(mapImage_, keypointsA, descriptorsA);
  descriptorExtractor->compute(referenceMapImage_, keypointsB, descriptorsB);

  cv::Mat keypoints;
  cv::cvtColor(mapImage_, keypoints, cv::COLOR_RGBA2RGB);
  drawKeypoints(keypoints, keypointsA, keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
  cv::imwrite( "Map all Features.png", keypoints );
  cv::cvtColor(referenceMapImage_, keypoints, cv::COLOR_RGBA2RGB);
  drawKeypoints(keypoints, keypointsB, keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
  cv::imwrite( "referenceMap all Features.png", keypoints );
  cv::waitKey(0);

  cv::FlannBasedMatcher matcher(new cv::flann::LshIndexParams(20,10,2));

  std::vector<cv::DMatch> matches;
  matcher.match(descriptorsA, descriptorsB, matches);

  cv::Mat all_matches;
  cv::drawMatches( mapImage_, keypointsA, referenceMapImage_, keypointsB,
                       matches, all_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                       std::vector<char>(),cv::DrawMatchesFlags::DEFAULT); //NOT_DRAW_SINGLE_POINTS );
  IplImage* outrecog = new IplImage(all_matches);
  cvSaveImage( "BRISK All Matches.jpeg", outrecog );

  //-- Localize the object
  std::vector<cv::Point2f> obj;
  std::vector<cv::Point2f> scene;
  for( int i = 0; i < matches.size(); i++ )
  {
    //-- Get the keypoints from the good matches
    obj.push_back( keypointsA[ matches[i].queryIdx ].pt );
    scene.push_back( keypointsB[ matches[i].trainIdx ].pt );
  }

  cv::Mat H = findHomography( obj, scene, CV_RANSAC );

  //-- Get the corners from the image_1 ( the object to be "detected" )
  std::vector<cv::Point2f> obj_corners(4);
  obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( mapImage_.cols, 0 );
  obj_corners[2] = cvPoint( mapImage_.cols, mapImage_.rows ); obj_corners[3] = cvPoint( 0, mapImage_.rows );
  std::vector<cv::Point2f> scene_corners(4);
  perspectiveTransform( obj_corners, scene_corners, H);

  line( all_matches, scene_corners[0] + cv::Point2f( mapImage_.cols, 0), scene_corners[1] + cv::Point2f( mapImage_.cols, 0), cv::Scalar(0, 255, 0), 4 );
  line( all_matches, scene_corners[1] + cv::Point2f( mapImage_.cols, 0), scene_corners[2] + cv::Point2f( mapImage_.cols, 0), cv::Scalar( 0, 255, 0), 4 );
  line( all_matches, scene_corners[2] + cv::Point2f( mapImage_.cols, 0), scene_corners[3] + cv::Point2f( mapImage_.cols, 0), cv::Scalar( 0, 255, 0), 4 );
  line( all_matches, scene_corners[3] + cv::Point2f( mapImage_.cols, 0), scene_corners[0] + cv::Point2f( mapImage_.cols, 0), cv::Scalar( 0, 255, 0), 4 );

  IplImage* outrecog2 = new IplImage(all_matches);
  cvSaveImage( "Homography.jpeg", outrecog2 );

  /*ros::Time time = ros::Time::now();  // initialization
  ros::Duration transform_dur;
  ros::Duration correlation_dur;
  float best_corr = 1;
  grid_map::Position best_position;
  float best_theta;
  grid_map::Position correct_position = map_.getPosition();

  grid_map::Position position;
  position(0) = 6;    // VAR max x considered for search, TODO automate
  position(1) = 3;    // VAR max y considered for search, TODO automate 
  grid_map::Index startIndex;
  referenceMap_.getIndex(position, startIndex);
  grid_map::Size size;
  size(0) = (position(0) - (-1))/referenceMap_.getResolution(); // VAR min x, TODO automate
  size(1) = (position(1) - (-1))/referenceMap_.getResolution(); // VAR min y, TODO automate

  grid_map::GridMap correlationMap_({"correlation","rotation"});
  correlationMap_.setGeometry(referenceMap_.getLength(), referenceMap_.getResolution()*searchIncrement_,
                              referenceMap_.getPosition()); //TODO only use submap
  correlationMap_.setFrameId("map");

  // iterate sparsely through search area
  for (grid_map::SubmapIteratorSparse iterator(referenceMap_, startIndex, size, searchIncrement_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    grid_map::Position xy_position;     // TODO check if isInside necessary
    referenceMap_.getPosition(index, xy_position);    // get coordinates
    for (float theta = 0; theta < 360; theta+=angleIncrement_) {  // iterate over rotation 
      //float corr = CorrelationSAD(Shift(xy_position, theta, map, broadcaster), reference_map, xy_position, theta);    // get correlation of shifted_map
      //float corr = CorrelationSSD(Shift(xy_position, theta, map, broadcaster), reference_map, xy_position, theta);    // get correlation of shifted_map
      ros::Time transform_time = ros::Time::now();
      shift(xy_position, theta);
      transform_dur += ros::Time::now() - transform_time;

  ros::Time correlation_time = ros::Time::now();
      float corr = correlationNCC(xy_position, theta);  // get correlation of shifted_map
      //correlation[index(0)][index(1)][int(theta/angle)] = corr;
  correlation_dur += ros::Time::now() - correlation_time;

      if (correlationMap_.isInside(xy_position)) {
        grid_map::Index correlation_index;
        correlationMap_.getIndex(xy_position, correlation_index);

        bool valid = correlationMap_.isValid(correlation_index, "correlation");
        // if no value so far or correlation smaller and correlation valid
        if (((valid == false) || (corr*1 < correlationMap_.at("correlation", correlation_index) )) && corr != 1) {
          correlationMap_.at("correlation", correlation_index) = corr*1;  //set correlation
          correlationMap_.at("rotation", correlation_index) = theta;    //set theta
          if (corr < best_corr) { // if best correlation store it
            best_corr = corr;
            best_position = xy_position;
            best_theta = theta;
          }
        }
      }
    }
    // publish current correlation_map
    grid_map_msgs::GridMap correlation_msg;
    grid_map::GridMapRosConverter::toMessage(correlationMap_, correlation_msg);
    correlationPublisher_.publish(correlation_msg);
  }
  ros::Duration duration = ros::Time::now() - time;
  // TODO calculate z alignment
  // output best correlation and time used
  std::cout << "Best correlation " << best_corr << " at " << best_position.transpose() << " and theta " << best_theta << std::endl;
  std::cout << "Correct position " << correct_position.transpose() << " and theta 0" << std::endl;
  std::cout << "Time used: " << duration.toSec() << " Sekunden, (corrMap, corr, innerCorr)" << correlation_dur.toSec() << ", " << correlationDur_.toSec()<< ", " << correlationDur2_.toSec() << std::endl;
  ROS_INFO("done");
  isActive_ = false;*/
}

void ImageFitter::shift(grid_map::Position position, int theta)
{
  grid_map::Position zero_position;
  zero_position(0) = 0.0;
  zero_position(1) = 0.0;
  map_.setPosition(zero_position);  // set position to origin

  // broadcast transformation from /map to /map_rotated
  broadcaster_.sendTransform(tf::StampedTransform(
        tf::Transform(tf::Quaternion(0.0, 0.0, sin(theta/180*M_PI/2), cos(theta/180*M_PI/2)), 
    tf::Vector3(position(0), position(1), 0.0)), ros::Time::now(), "/map", "/map_rotated"));
  map_.setFrameId("map_rotated"); // set frame to /map_rotated

  grid_map_msgs::GridMap shifted_msg;
  grid_map::GridMapRosConverter::toMessage(map_, shifted_msg);
  shifted_msg.info.header.stamp = ros::Time::now();
  //shiftedPublisher_.publish(shifted_msg);   // publish shifted_map
}
float ImageFitter::correlationNCC(grid_map::Position position, int theta)
{
  float correlation = 0;  // initialization
  float shifted_mean = 0;
  float reference_mean = 0;
  std::vector<float> xy_shifted;
  std::vector<float> xy_reference;
  float shifted_normal = 0;
  float reference_normal = 0;
  int points = 0;
  int matches = 0;

  
  grid_map::Matrix& data = map_["elevation"];
  // iterate sparsely through template points
  for (grid_map::GridMapIteratorSparse iterator(map_, correlationIncrement_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);

    float shifted = data(index(0), index(1));
    
    if (shifted == shifted) {   // check if point is defined, if nan f!= f
      points += 1;    // increase number of valid points
      grid_map::Position xy_position;
  ros::Time correlation_time = ros::Time::now();
      map_.getPosition(index, xy_position);  // get coordinates
  correlationDur2_ += ros::Time::now() - correlation_time;
      tf::Vector3 xy_vector = tf::Vector3(xy_position(0), xy_position(1), 0.0);

      // transform coordinates from /map_rotated to /map
      tf::Transform transform = tf::Transform(tf::Quaternion(0.0, 0.0, sin(theta/180*M_PI/2), cos(theta/180*M_PI/2)), tf::Vector3(position(0), position(1), 0.0));
      tf::Vector3 map_vector = transform*(xy_vector); // apply transformation
      grid_map::Position map_position;
      map_position(0) = map_vector.getX();
      map_position(1) = map_vector.getY();


      // check if point is within reference_map
      if (referenceMap_.isInside(map_position)) {
  correlation_time = ros::Time::now();
        float reference = referenceMap_.atPosition("elevation", map_position);
  correlationDur_ += ros::Time::now() - correlation_time;
        if (reference == reference) {   // check if point is defined, if nan f!= f 
          matches += 1;   // increase number of matched points
          shifted_mean += shifted;
          reference_mean += reference;
          xy_shifted.push_back(shifted);
          xy_reference.push_back(reference);
        }
      }
    }
  }
  
  
  // check if required overlap is fulfilled
  if (matches > points*requiredOverlap_) 
  { 
    // calculate Normalized Cross Correlation (NCC)
    shifted_mean = shifted_mean/matches;
    reference_mean = reference_mean/matches;
    for (int i = 0; i < matches; i++) {
      float shifted_corr = (xy_shifted[i]-shifted_mean);
      float reference_corr = (xy_reference[i]-reference_mean);
      correlation += shifted_corr*reference_corr;
      shifted_normal += shifted_corr*shifted_corr;
      reference_normal += reference_corr*reference_corr;
    }
    correlation = correlation/sqrt(shifted_normal*reference_normal);
    return 1 - correlation; 
  }
  else { return 1.0; } 
}

void ImageFitter::tfBroadcast(const ros::TimerEvent&) 
{
  broadcaster_.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)), 
          ros::Time::now(),"/world", "/map"));
}

} /* namespace */
