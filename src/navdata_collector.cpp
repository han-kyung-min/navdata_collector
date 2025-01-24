/*
 * navdata_collector.cpp
 *
 *  Created on: Jan 6, 2025
 *      Author: hankm
 */


#include "navdata_collector.hpp"

namespace navdata_collector
{

NavDataCollector::NavDataCollector(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_):
m_nh_private(private_nh_),
m_nh(nh_),
mn_bagfile_cnt(0), mn_max_num_bagfiles(1000),
mstr_twist_topic("/former_base_controller/cmd_vel"), mstr_scan_topic("scan"), mstr_twiststamped_topic("robot_twist_stamped"),
mstr_odom_topic("/former_base_controller/odom"), mstr_odom_filtered_topic("/odometry/filtered"),
mb_navdata_collection_is_completed(false)
{
	m_nh.param("/navdata_collector/world_frame_id", mstr_worldframe_id, std::string("map"));
	m_nh.param("/navdata_collector/robot_frame_id", mstr_robotframe_id, std::string("base_link"));

	m_nh.param("/navdata_collector/robotpose_topic", mstr_robotpose_topic, std::string(""));
	m_nh.param("/navdata_collector/twist_topic", mstr_twist_topic, mstr_twist_topic);
	m_nh.param("/navdata_collector/twiststamped_topic", mstr_twiststamped_topic, mstr_twiststamped_topic);
	m_nh.param("/navdata_collector/max_num_bagfiles", mn_max_num_bagfiles, mn_max_num_bagfiles);

	m_nh.getParam("/navdata_collector/bagfile_root_path", mstr_bagfile_path); // root file path

//	time_t start_time = time(0);
//	tm* start_date = localtime(&start_time);
//	char filename[256] = {0};
//	strcat(filename, fmt("%04d-%02d-%02d-%02d",
//			start_date->tm_year+1900, start_date->tm_mon+1, start_date->tm_mday, start_date->tm_hour).c_str());
//	mstr_bagfile_path = mstr_bagfile_path + "/" + string(filename) ;

//	m_arrivalmsgSub 	= m_nh.subscribe( "arrival_status", 1, &NavDataCollector::arrivalCallBack, this);
//	m_departmsgSub 		= m_nh.subscribe( "departure_flag", 5, &NavDataCollector::departFlagCallBack, this );
	m_doneSub			= m_nh.subscribe( "data_collection_is_completed", 1, &NavDataCollector::doneCallBack, this ) ;

	m_initdonePub = m_nh.advertise<std_msgs::Bool>("navdata_collector_is_initialized", 1);

	m_robotTwistSub = m_nh.subscribe(mstr_twist_topic, 1, &NavDataCollector::twistReceiveCallBack, this); // kmHan
	m_robotposePub = m_nh.advertise<geometry_msgs::PoseStamped>(mstr_robotpose_topic, 1);
	m_robotVelPub  = m_nh.advertise<geometry_msgs::TwistStamped>(mstr_twiststamped_topic, 1);

	waitForCompMetadata( ) ;

	std_msgs::Bool bmsg_ok ;
	bmsg_ok.data = true ;
	m_initdonePub.publish(bmsg_ok) ;
}

NavDataCollector::~NavDataCollector()
{

}

bool NavDataCollector::waitForCompMetadata( )
{
//	while (true)
//	{
//		if( ros::topic::waitForMessage<sensor_msgs::Image>(mstr_rgb_topic, m_nh, ros::Duration(1.0) )  )
//		{
//			ROS_INFO("got %s msg \n", mstr_rgb_topic.c_str());
//			break ;
//		}
//		else
//			ROS_WARN("Waitining for the %s msg \n", mstr_rgb_topic.c_str());
//	}
//
//	while (true)
//	{
//		if( ros::topic::waitForMessage<sensor_msgs::Image>(mstr_depth_topic, m_nh, ros::Duration(1.0) )  )
//		{
//			ROS_INFO("got %s msg \n", mstr_depth_topic.c_str());
//			break ;
//		}
//		else
//			ROS_WARN("Waitining for the %s msg \n", mstr_depth_topic.c_str());
//	}

	while (true)
	{
		if( ros::topic::waitForMessage<sensor_msgs::LaserScan>(mstr_scan_topic, m_nh, ros::Duration(1.0) )  )
		{
			ROS_INFO("got %s msg \n", mstr_scan_topic.c_str());
			break ;
		}
		else
			ROS_WARN("Waitining for the %s msg \n", mstr_scan_topic.c_str());
	}

//	while (true)
//	{
//		if( ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(mstr_map_topic, m_nh, ros::Duration(1.0))  )
//		{
//			ROS_INFO("got map msg \n");
//			break ;
//		}
//		else
//			ROS_WARN("Waiting for map msg \n");
//	}

	while (true)
	{
		tf::StampedTransform map2baselink;
		try{
		  m_listener.lookupTransform(mstr_worldframe_id, mstr_robotframe_id,
								   ros::Time(0), map2baselink);
		  ROS_INFO("got pose tf msg \n");
		  break ;
		}
		catch (tf::TransformException &ex) {
		  ROS_ERROR("%s",ex.what());
		  ros::Duration(1.0).sleep();
		}
	}

//	while (true)
//	{
//		if( ros::topic::waitForMessage<nav_msgs::Odometry>(mstr_odom_topic, m_nh, ros::Duration(1.0) )  )
//		{
//			ROS_INFO("got %s msg \n", mstr_odom_topic.c_str());
//			break ;
//		}
//		else
//			ROS_WARN("Waitining for the %s msg \n", mstr_odom_topic.c_str());
//	}
//
//	while (true)
//	{
//		if( ros::topic::waitForMessage<nav_msgs::Odometry>(mstr_odom_filtered_topic, m_nh, ros::Duration(1.0) )  )
//		{
//			ROS_INFO("got %s msg \n", mstr_odom_filtered_topic.c_str());
//			break ;
//		}
//		else
//			ROS_WARN("Waitining for the %s msg \n", mstr_odom_filtered_topic.c_str());
//	}

	return true;
}


void NavDataCollector::twistReceiveCallBack(  const geometry_msgs::TwistConstPtr& msg  )
{
//	ROS_INFO("got twist msg \n");
	geometry_msgs::TwistStamped outvel ;
	m_twiststamped.twist = *msg ;
	m_twiststamped.header.stamp = ros::Time::now() ;
}

void NavDataCollector::publishRobotPose()
{
	tf::StampedTransform map2baselink;
	try{
	  m_listener.lookupTransform(mstr_worldframe_id, mstr_robotframe_id,
							   ros::Time(0), map2baselink);
	}
	catch (tf::TransformException &ex) {
	  ROS_ERROR("%s",ex.what());
	  ros::Duration(1.0).sleep();
	}

	geometry_msgs::PoseStamped outPose;
	outPose.pose.position.x = map2baselink.getOrigin().x();
	outPose.pose.position.y = map2baselink.getOrigin().y();
	outPose.pose.position.z = 0.f;
	outPose.header.frame_id = mstr_worldframe_id;
	outPose.header.stamp = ros::Time::now() ;

	m_robotposePub.publish(outPose);
}

void NavDataCollector::publishRobotVel( )
{
	m_robotVelPub.publish(m_twiststamped);
}

//void NavDataCollector::departFlagCallBack( const std_msgs::BoolConstPtr& depart_msg )
//{
//	// set bag file name
//	time_t current_time = time(0);
//	tm* current_date = localtime(&current_time);
//	char filename[256] = {0};
//
//	//strcpy(filename, "C:/Users/Admin/Documents/MATLAB/datafile");
//	strcat(filename, fmt("bag_%04d-%02d-%02d-%02d-%02d-%02d_%05d",
//			current_date->tm_year+1900, current_date->tm_mon+1, current_date->tm_mday,
//			current_date->tm_hour, current_date->tm_min, current_date->tm_sec, mn_bagfile_cnt).c_str());
//
//	ros::Time ctime = ros::Time::now() ;
//	mstr_bagfile = mstr_bagfile_path + "/" + string(filename) + ".bag" ;
//
//	// robot begins to move
//	std_msgs::Bool data = *depart_msg ;
//ROS_INFO("@NavDataCollector Got departure msg %s",  (data.data == true) ? "TRUE" : "FALSE" );
//
//	if(data.data)
//	{
//		const std::unique_lock<mutex> lock(mutex_bag) ;
//		mb_is_bag_accessible = true ;
//	}
//	else
//	{
//		ROS_INFO("@NavdataCollector Got False departure msg. Closing the bag file \n");
//		{
//			const std::unique_lock<mutex> lock(mutex_bag) ;
//			mb_is_bag_accessible = false ;
//		}
//		m_bag.close();
//		return;
//	}
//
//	ROS_ASSERT( m_bag.isOpen() == false );
//	ROS_INFO("Opening <%d>th bag < %s > ", mn_bagfile_cnt, mstr_bagfile.c_str());
//	m_bag.open(mstr_bagfile.c_str(), rosbag::bagmode::Write) ;
//	mn_bagfile_cnt++ ;
//}


void NavDataCollector::doneCallBack( const std_msgs::BoolConstPtr& done_msg )
{
	if( (*done_msg).data == true )
		mb_navdata_collection_is_completed = true;
}


}
