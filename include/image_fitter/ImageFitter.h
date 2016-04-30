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
#include <geometry_msgs/PointStamped.h>

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

    void convertToWeightedImages();

    void exhaustiveSearch();

    float findZ(float x, float y, int theta);

    bool findMatches(cv::Mat *rotatedImage, int row, int col);

    float mutualInformation(cv::Mat *templateImage, int row, int col);

    float errorSAD();

    float weightedErrorSAD();

    float errorSSD();

    float weightedErrorSSD();

    float correlationNCC();

    float weightedCorrelationNCC();

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

    float SSDThreshold_;

    float SADThreshold_;


    tf::TransformBroadcaster broadcaster_;


    cv::Mat mapImage_;


    cv::Mat referenceMapImage_;
    

    cv::Mat weightedMapImage_;

    cv::Mat weightedReferenceMapImage_;


    cv::Rect referenceBoundRect_;

    //! Image publisher.
    ros::Publisher mapImagePublisher_;


    ros::Publisher referenceMapImagePublisher_;

    //! Grid map publisher.
    ros::Publisher correlationPublisher_;

    ros::Publisher corrPointPublisher_;
    ros::Publisher SSDPointPublisher_;
    ros::Publisher SADPointPublisher_;
    ros::Publisher correctPointPublisher_;

    float cumulativeErrorCorr_;
    float cumulativeErrorSSD_;
    float cumulativeErrorSAD_;
    int correctMatchesCorr_;
    int correctMatchesSSD_;
    int correctMatchesSAD_;

    float shifted_mean_;
    float reference_mean_;
    int matches_;
    std::vector<float> xy_shifted_;
    std::vector<float> xy_reference_;
    std::vector<float> xy_shifted_var_;
    std::vector<float> xy_reference_var_;
    std::vector<int> row_matches_;
    std::vector<int> col_matches_;

    float templateRotation_;

};

} /* namespace */

#endif
