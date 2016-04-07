/*
 * image_fitter_node.cpp
 *
 *  Created on: Apr 07, 2016
 *      Author: Roman Käslin
 *   Institute: ETH Zurich, Autonomous Systems Lab
 *
 */

#include <ros/ros.h>
#include <image_fitter/ImageFitter.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "image_fitter");
  ros::NodeHandle nodeHandle("~");

  image_fitter::ImageFitter imageFitter(nodeHandle);

  ros::spin();
  return 0; 
}
