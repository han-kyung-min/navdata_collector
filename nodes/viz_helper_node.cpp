/*
 * viz_helper_node.cpp
 *
 *  Created on: Jan 10, 2025
 *      Author: hankm
 */


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include "viz_helper.hpp"

using namespace navdata_collector;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "viz_helper");
  const ros::NodeHandle nh;
  const ros::NodeHandle private_nh("~");

  ros::WallTime start_, end_;

  ROS_INFO("Starting viz_helper_node\n");
  VizHelper viz_helper(private_nh, nh);
  ros::spinOnce();
  //front_detector_dms.initmotion();
  while( !viz_helper.isDone() && ros::ok() )
  {
	  try{
		  ros::spinOnce();
	  }
	  catch(std::runtime_error& e)
	  {
		ROS_ERROR("viz helper exception: %s", e.what());
		return -1;
	  }
  }

  return 0;
}
