/*
 * navdata_collector_node.cpp
 *
 *  Created on: Jan 6, 2025
 *      Author: hankm
 */


#include "navdata_collector.hpp"


using namespace navdata_collector;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "navdata_collector");
	const ros::NodeHandle nh;
	const ros::NodeHandle private_nh("~");

	ros::WallTime start_, end_;
	ROS_INFO("args: %s %s %s\n", argv[0], argv[1], argv[2]);

	NavDataCollector oDataCollector(private_nh, nh);

	ros::Rate rate(30);

	while( ros::ok() & !oDataCollector.isDone()  )
	{
		oDataCollector.publishRobotPose();
		//oDataCollector.publishRobotVel() ;
		ros::spinOnce();
		rate.sleep() ;
	}
}

