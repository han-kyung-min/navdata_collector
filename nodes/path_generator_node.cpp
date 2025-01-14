/*
 * navdata_collector_node.cpp
 *
 *  Created on: Jan 6, 2025
 *      Author: hankm
 */


#include "path_generator.hpp"

using namespace navdata_collector;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "path_generator");
	const ros::NodeHandle nh;
	const ros::NodeHandle private_nh("~");

	ros::WallTime start_, end_;
	ROS_INFO("args: %s %s %s\n", argv[0], argv[1], argv[2]);

	PathGenerator oPathGenerator(private_nh, nh);

	while( ros::ok() & !oPathGenerator.isDone() )
	{
		ros::spinOnce();
	}
}

