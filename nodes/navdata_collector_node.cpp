/*
 * navdata_collector_node.cpp
 *
 *  Created on: Jan 6, 2025
 *      Author: hankm
 */


#include "navdata_collector.hpp"


//
//#include <ros/ros.h>
//#include <ros/console.h>
//#include <move_base/move_base.h>
//#include <move_base_msgs/MoveBaseAction.h>
//
//#include "visualization_msgs/Marker.h"
//#include "visualization_msgs/MarkerArray.h"
//
//#include "geometry_msgs/PoseWithCovarianceStamped.h"
//#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>
//#include <message_filters/sync_policies/approximate_time.h>
//#include <message_filters/sync_policies/exact_time.h>
//#include <sensor_msgs/Image.h>
//#include <sensor_msgs/CameraInfo.h>
//#include <sensor_msgs/LaserScan.h>
//#include <nav_msgs/Odometry.h>
//#include <image_transport/image_transport.h>
//
//#include <geometry_msgs/TwistStamped.h>
//
//#include <tf/transform_listener.h>
//#include "tf/message_filter.h"
//
//#include <std_msgs/Bool.h>
//#include <std_msgs/Int8.h>
//
//#include <rosbag/bag.h>
//#include <rosbag/view.h>
//
//#include "navdata_collector/scan_metadata.h"
//#include "navdata_collector/rgbd_metadata.h"
//
//using namespace message_filters ;
//using namespace std;
//
//
//typedef enum{	FREE 	= 0,
//				UNKNOWN	= 127,
//				OCCUPIED= 255, // is moving but needs to be stopped
//			} OCC_LABEL ;
//
//typedef enum {RGBD=1, SCAN, COMPLETE} METDATA_TYPE;
//
//geometry_msgs::TwistStamped _outvel ;
//navdata_collector::rgbd_metadata _rgbd_metadata;
//navdata_collector::scan_metadata _scan_metadata ;
//std::string _str_bagfile_path, _str_bagfile, _str_metadata_topic ;
//rosbag::Bag _bag;
//
//uint32_t _bag_file_cnt = 0;
//
//std::mutex mutex_bag ;
//bool _b_is_bag_accessible ;
//
//#include <cstdarg>
//std::string fmt(const std::string& fmt, ...) {
//    int size = 200;
//    std::string str;
//    va_list ap;
//    while (1) {
//        str.resize(size);
//        va_start(ap, fmt);
//        int n = vsnprintf((char*)str.c_str(), size, fmt.c_str(), ap);
//        va_end(ap);
//        if (n > -1 && n < size) {
//            str.resize(n);
//            return str;
//        }
//        if (n > -1)
//            size = n + 1;
//        else
//            size *= 2;
//    }
//    return str;
//}
//
//void twistReceiveCallBack(  const geometry_msgs::TwistConstPtr& msg  )
//{
//	//ROS_INFO("got twist msg \n");
//	_outvel.twist = *msg ;
//	_outvel.header.stamp = ros::Time::now() ;
//}
//
//void RGBDMetaDataCallBack( const sensor_msgs::ImageConstPtr& rgb_msg,
//						const sensor_msgs::ImageConstPtr& depth_msg,
//						const geometry_msgs::TwistStampedConstPtr& vel_msg,
//						const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
//{
//	ROS_INFO("rgbd msgs set\n");
//
//	_rgbd_metadata.header 	= (*pose_msg).header ;
//	_rgbd_metadata.rgb 	= *rgb_msg ;
//	_rgbd_metadata.depth	= *depth_msg;
//	_rgbd_metadata.cmd_vel	= (*vel_msg).twist ;
//	_rgbd_metadata.rpose	= (*pose_msg).pose ;
//
//	ROS_INFO(" RGBD metadata is set @ R position <%f %f %f> \n",
//			(*pose_msg).pose.position.x,
//			(*pose_msg).pose.position.y,
//			(*pose_msg).pose.position.z);
//}
//
//void ScanMetaDataCallBack( 	const sensor_msgs::LaserScanConstPtr& scan_msg,
//							const nav_msgs::OccupancyGridConstPtr& map_msg,
//							const geometry_msgs::TwistStampedConstPtr& vel_msg,
//							const geometry_msgs::PoseStamped::ConstPtr& pose_msg )
//{
////	ROS_INFO("got Scan msgs set\n");
//
//	_scan_metadata.header 	= (*scan_msg).header ;
//	_scan_metadata.cmd_vel 	= (*vel_msg).twist ;
//	_scan_metadata.scan	= *scan_msg;
//	_scan_metadata.map = *map_msg,
//	_scan_metadata.rpose	= (*pose_msg).pose ;
//
//	if(_b_is_bag_accessible )
//		_bag.write(_str_metadata_topic, _scan_metadata.header.stamp, _scan_metadata) ;
////	ROS_INFO(" SCAN metadata is set @ R position <%f %f %f> \n",
////			_scan_metadata.rpose.position.x,
////			_scan_metadata.rpose.position.y,
////			_scan_metadata.rpose.position.z);
//}
//
//void departFlagCallBack( const std_msgs::BoolConstPtr& depart_msg )
//{
//	// set bag file name
//	time_t currentTime = time(0);
//	tm* currentDate = localtime(&currentTime);
//	char filename[256] = {0};
//
//	//strcpy(filename, "C:/Users/Admin/Documents/MATLAB/datafile");
//	strcat(filename, fmt("bag_%04d-%02d.%02d-%02d-%02d-%02d_%05d",
//			currentDate->tm_year+1900, currentDate->tm_mon+1, currentDate->tm_mday, currentDate->tm_hour, currentDate->tm_min, currentDate->tm_sec, _bag_file_cnt).c_str());
//
//	ros::Time ctime = ros::Time::now() ;
//	_str_bagfile = _str_bagfile_path + "/" + string(filename) + ".bag" ;
//
//	// robot begins to move
//	std_msgs::Bool data = *depart_msg ;
//ROS_INFO("Got departure msg %s",  (data.data == true) ? "TRUE" : "FALSE" );
//
//	if(data.data)
//	{
//		const std::unique_lock<mutex> lock(mutex_bag) ;
//		_b_is_bag_accessible = true ;
//	}
//	else
//	{
//		const std::unique_lock<mutex> lock(mutex_bag) ;
//		_b_is_bag_accessible = false ;
//		_bag.close();
//	}
//
//	if(_b_is_bag_accessible)
//	{
//		// write bag file
////		if( _bag.isOpen() )
////		{
////			ROS_ERROR("_bag is open for some reason. I'm closing it... \n");
////			_bag.close();
////		}
//		ROS_ASSERT( _bag.isOpen() == false );
//		ROS_INFO("Opening the bag file # < %d > ", _bag_file_cnt++);
//		_bag.open(_str_bagfile.c_str(), rosbag::bagmode::Write) ;
//	}
//	else // The bag should not be accessible
//	{
//		ROS_INFO("Closing the bag file \n");
//		{
//			_bag.close();
//		}
//	}
//}
//
//
//void arrivalCallBack( const std_msgs::Int8ConstPtr& arrival_msg )
//{
//	// robot stops
//	std_msgs::Int8 data = *arrival_msg ;
//
//	{
//		const std::unique_lock<mutex> lock(mutex_bag) ;
//		_b_is_bag_accessible = false ;
//	}
//
//	ROS_INFO("Robot finished its motion with status: < %d >. Closing the bag file...", data.data);
//
//	ROS_ASSERT( _bag.isOpen() ) ;
//	_bag.close() ;
//
////	if(_bag.isOpen())
////		_bag.close() ;
////	else
////		ROS_ERROR("_bag is not open for some reason.. so skipping the closing process \n");
//}
//
//
//
//int main(int argc, char** argv)
//{
//	ros::init(argc, argv, "navdata_collector");
//	ros::NodeHandle nh;
//	ros::NodeHandle private_nh("~");
//
//	ros::WallTime start_, end_;
//	ROS_INFO("args: %s %s %s\n", argv[0], argv[1], argv[2]);
//
//	string str_worldframe_id, str_robotframe_id, str_scan_topic, str_twist_topic,  str_twiststamped_topic,
//			str_robotpose_topic, str_rgb_topic, str_depth_topic ;
//
//	nh.getParam("/navdata_collector/world_frame_id", str_worldframe_id);
//	nh.getParam("/navdata_collector/base_frame_id", str_robotframe_id);
//
//	nh.getParam("/navdata_collector/metadata_topc", _str_metadata_topic) ;
//
//	nh.getParam("/navdata_collector/scan_topic", str_scan_topic);
//	nh.getParam("/navdata_collector/twist_topic", str_twist_topic);
//	nh.getParam("/navdata_collector/twiststamped_topic", str_twiststamped_topic);
//	nh.getParam("/navdata_collector/robotpose_topic", str_robotpose_topic);
//	nh.getParam("/navdata_collector/rgb_topic", str_rgb_topic);
//	nh.getParam("/navdata_collector/depth_topic", str_depth_topic);
//
//	nh.getParam("/navdata_collector/bagfile_path", _str_bagfile_path);
//
//	message_filters::Subscriber<sensor_msgs::Image> mf_rgbSub(nh, str_rgb_topic, 1) ;
//	message_filters::Subscriber<sensor_msgs::Image> mf_depthSub(nh, str_depth_topic, 1)  ;
//	message_filters::Subscriber<sensor_msgs::LaserScan> mf_scanSub(nh, "scan", 1);
//	message_filters::Subscriber<nav_msgs::OccupancyGrid> mf_mapSub(nh, "map", 1);
//	message_filters::Subscriber<geometry_msgs::TwistStamped> mf_velSub(nh, "robot_twist_stamped", 1) ;
//	message_filters::Subscriber<geometry_msgs::PoseStamped> mf_poseSub(nh, "base_pose", 1)  ;
//
//	ros::Subscriber arrivalmsgSub 	= nh.subscribe( "arrival_status", 1, &arrivalCallBack);
//	ros::Subscriber departmsgSub 	= nh.subscribe( "departure_flag", 1, &departFlagCallBack );
//
////	ROS_INFO("rgb topic name: %s\n", mstr_rgb_topic.c_str()) ;
////	ROS_INFO("depth topic name: %s\n", mstr_depth_topic.c_str()) ;
////	ROS_INFO("robot pose topic: %s\n", mstr_robotpose_topic.c_str());
//
//	ros::Publisher _syncScandataPub, _syncRGBDdataPub;
//	ros::Publisher _robotVelPub  = nh.advertise<geometry_msgs::TwistStamped>("robot_twist_stamped", 1);
//	ros::Publisher _robotPosePub = nh.advertise<geometry_msgs::PoseStamped>("base_pose", 1 );
//
//	ros::Subscriber _robotTwistSub = nh.subscribe("/former_base_controller/cmd_vel", 1, twistReceiveCallBack); // kmHan
//
//	tf::TransformListener _listener;
//
//
//	typedef sync_policies::ApproximateTime
//			<sensor_msgs::LaserScan, nav_msgs::OccupancyGrid, geometry_msgs::TwistStamped, geometry_msgs::PoseStamped> ApproxScanTimeSyncPolicy;
//	typedef Synchronizer<ApproxScanTimeSyncPolicy> Scan_Sync;
//
//	Scan_Sync scan_sync( ApproxScanTimeSyncPolicy(10), mf_scanSub, mf_mapSub, mf_velSub, mf_poseSub );
//	scan_sync.registerCallback(boost::bind(&ScanMetaDataCallBack, _1, _2, _3, _4));
//	_syncScandataPub	= nh.advertise<navdata_collector::scan_metadata>(_str_metadata_topic, 1);
//
//	ros::Rate rate(10);
//	while (ros::ok() && _bag_file_cnt < 500)
//	{
//		tf::StampedTransform map2baselink;
//		try{
//		  _listener.lookupTransform(str_worldframe_id, str_robotframe_id,
//								   ros::Time(0), map2baselink);
//		}
//		catch (tf::TransformException &ex) {
//		  ROS_ERROR("%s",ex.what());
//		  ros::Duration(1.0).sleep();
//		}
//
//		geometry_msgs::PoseStamped outPose;
//		outPose.pose.position.x = map2baselink.getOrigin().x();
//		outPose.pose.position.y = map2baselink.getOrigin().y();
//		outPose.pose.position.z = 0.f;
//		outPose.header.frame_id = str_worldframe_id;
//		outPose.header.stamp = ros::Time::now() ;
//
//		_robotPosePub.publish(outPose) ;
//		_robotVelPub.publish(_outvel) ;
//
//		_syncScandataPub.publish(_scan_metadata);
//
//		ros::spinOnce() ;
//		rate.sleep();
//	}
//}
//

using namespace navdata_collector;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "navdata_collector");
	const ros::NodeHandle nh;
	const ros::NodeHandle private_nh("~");

	ros::WallTime start_, end_;
	ROS_INFO("args: %s %s %s\n", argv[0], argv[1], argv[2]);

	NavDataCollector oDataCollector(private_nh, nh);

	ros::Rate rate(10);

	while( ros::ok() ) //& !oDataCollector.isDone()  )
	{
		oDataCollector.publishRobotPose();
		oDataCollector.publishRobotVel() ;
		ros::spinOnce();
		rate.sleep() ;
	}
}

