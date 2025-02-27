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
mstr_rgb_topic("/camera/color/image_raw"), mstr_depth_topic("/camera/depth/image_rect_raw"),
mstr_twist_topic("/former_base_controller/cmd_vel"), mstr_scan_topic("scan"), mstr_twiststamped_topic("robot_twist_stamped"),
mstr_odom_topic("/former_base_controller/odom"), mstr_odom_filtered_topic("/odometry/filtered"),
mb_navdata_collection_is_completed(false)
{
	m_nh.param("/navdata_collector/world_frame_id", mstr_worldframe_id, std::string("map"));
	m_nh.param("/navdata_collector/robot_frame_id", mstr_robotframe_id, std::string("base_link"));

	m_nh.param("/navdata_collector/rgb_topic", mstr_rgb_topic, mstr_rgb_topic) ;
	m_nh.param("/navdata_collector/depth_topic", mstr_depth_topic, mstr_depth_topic) ;
	m_nh.param("/navdata_collector/robotpose_topic", mstr_robotpose_topic, std::string(""));
	m_nh.param("/navdata_collector/twist_topic", mstr_twist_topic, mstr_twist_topic);
	m_nh.param("/navdata_collector/twiststamped_topic", mstr_twiststamped_topic, mstr_twiststamped_topic);
	m_nh.param("/navdata_collector/max_num_bagfiles", mn_max_num_bagfiles, mn_max_num_bagfiles);
	m_nh.param("/navdata_collector/rgbd_topic", mstr_rgbd_topic, string(""));

	m_nh.getParam("/navdata_collector/bagfile_root_path", mstr_bagfile_path); // root file path

	m_mf_rgbSub.subscribe(m_nh, mstr_rgb_topic, 1) ;
	m_mf_depthSub.subscribe(m_nh, mstr_depth_topic, 1)  ;

	m_doneSub			= m_nh.subscribe( "data_collection_is_completed", 1, &NavDataCollector::doneCallBack, this ) ;

	m_initdonePub = m_nh.advertise<std_msgs::Bool>("navdata_collector_is_initialized", 1);

	m_robotTwistSub = m_nh.subscribe(mstr_twist_topic, 1, &NavDataCollector::twistReceiveCallBack, this); // kmHan
	m_robotposePub = m_nh.advertise<geometry_msgs::PoseStamped>(mstr_robotpose_topic, 1);
	m_robotVelPub  = m_nh.advertise<geometry_msgs::TwistStamped>(mstr_twiststamped_topic, 1);

	waitForCompMetadata( ) ;

	m_rgbd_sync.reset(new RGBD_Sync(ApproxRGBDTimeSyncPolicy(10), m_mf_rgbSub, m_mf_depthSub) );
	m_rgbd_sync->registerCallback(boost::bind(&NavDataCollector::RGBDCallBack, this, _1, _2));
	m_syncdataPub	= m_nh.advertise<navdata_collector::rgbd>(mstr_rgbd_topic, 1);

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
////
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

	m_robotVelPub.publish(m_twiststamped);
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


void NavDataCollector::RGBDCallBack( const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg)
{
	double rgb_time = static_cast<double>( (*rgb_msg).header.stamp.sec ) + static_cast<double>( (*rgb_msg).header.stamp.nsec ) * 10e-9   ;
	double depth_time = static_cast<double>( (*depth_msg).header.stamp.sec ) + static_cast<double>( (*depth_msg).header.stamp.nsec ) * 10e-9   ;
	double ftimediff = fabs( rgb_time - depth_time) ;

	ROS_INFO("RGBD msg is set @ timediff: %f (s)\n", ftimediff );
	m_rgbd.header 	= (*rgb_msg).header ;
	m_rgbd.rgb 	= *rgb_msg ;
	m_rgbd.depth	= *depth_msg;
	m_syncdataPub.publish(m_rgbd);
}

void NavDataCollector::doneCallBack( const std_msgs::BoolConstPtr& done_msg )
{
	if( (*done_msg).data == true )
		mb_navdata_collection_is_completed = true;
}


}
