/*
 * ImageFitter.h
 *
 *  Created on: Apr 07, 2016
 *      Author: Roman Käslin
 *   Institute: ETH Zurich, Autonomous Systems Lab
 *
 */

#ifndef IMAGEFITTER_H
#define IMAGEFITTER_H

#include <ros/ros.h>
#include <cstdlib>
#include <math.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIteratorSparse.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_core/iterators/SubmapIteratorSparse.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_msgs/GridMap.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

namespace image_fitter {

class ImageFitter
{
public:
	  /*!
   	 * Constructor.
   	 * @param nodeHandle the ROS node handle.
   	 */
  	ImageFitter(ros::NodeHandle& nodeHandle);

  	/*!
   	 * Destructor.
   	 */
  	virtual ~ImageFitter();

  	/*!
   	 * Callback function for the grid map.
   	 * @param message the grid map message to be visualized.
   	 */
  	void callback(const grid_map_msgs::GridMap& message);

    void convertToImages();

    void exhaustiveSearch();

    float errorSAD(cv::Mat *rotatedImage, int row, int col);

    float errorSSD(cv::Mat *rotatedImage, int row, int col);

    float correlationNCC(cv::Mat *rotatedImage, int row, int col);

private:
    /*!
     * Read parameters from ROS.
     * @return true if successful.
     */
    bool readParameters();

    /*!
     * Check if visualizations are active (subscribed to),
     * and accordingly cancels/activates the subscription to the
     * grid map to safe bandwidth.
     * @param timerEvent the timer event.
     */
    void updateSubscriptionCallback(const ros::TimerEvent& timerEvent);

    void tfBroadcast(const ros::TimerEvent& timerEvent);


    //! ROS nodehandle.
  	ros::NodeHandle& nodeHandle_;

  	//! Topic name of the grid map to be matched.
 	  std::string mapTopic_;

    //! Topic name of the grid map to be matched to.
    std::string referenceMapTopic_;


    std::string correlationMapTopic_;


    std::string mapImageTopic_;


    std::string referenceMapImageTopic_;


    //! Duration of checking the activity of the visualizations.
    ros::Duration activityCheckDuration_;

    //! Timer to check the activity of the visualizations.
    ros::Timer activityCheckTimer_;

    ros::Timer broadcastTimer_;

    //! If the grid map visualization is subscribed to the grid map.
    bool isActive_;

    //! ROS subscriber to the grid map.
    ros::Subscriber mapSubscriber_;

    //! Template grid_map
    grid_map::GridMap map_;

    //! Reference grid_map
    grid_map::GridMap referenceMap_;


    int angleIncrement_;


    int searchIncrement_;


    int correlationIncrement_;
    

    float requiredOverlap_;


    float corrThreshold_;


    tf::TransformBroadcaster broadcaster_;


    cv::Mat mapImage_;


    cv::Mat referenceMapImage_;


    cv::Rect referenceBoundRect_;

    //! Image publisher.
    ros::Publisher mapImagePublisher_;


    ros::Publisher referenceMapImagePublisher_;

    //! Grid map publisher.
    ros::Publisher correlationPublisher_;

};

} /* namespace */

#endif
