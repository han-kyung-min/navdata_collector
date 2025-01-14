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
mn_data_cnt(0), mstr_twist_topic("/former_base_controller/cmd_vel"), mstr_scan_topic("scan")
{
	m_nh.param("/navdata_collector/world_frame_id", mstr_worldframe_id, std::string("map"));
	m_nh.param("/navdata_collector/robot_frame_id", mstr_robotframe_id, std::string("base_link"));

	m_nh.param("/navdata_collector/rgb_topic", mstr_rgb_topic, string("image_rect_color") );
	m_nh.param("/navdata_collector/depth_topic", mstr_depth_topic, string(""));
	m_nh.param("/navdata_collector/scan_topic", mstr_scan_topic, mstr_scan_topic);

	m_nh.param("/navdata_collector/robotpose_topic", mstr_robotpose_topic, std::string(""));
	m_nh.param("/navdata_collector/twist_topic", mstr_twist_topic, mstr_twist_topic);
	m_nh.param("/navdata_collector/twiststamped_topic", mstr_twiststamped_topic, string("robot_twist_stamped"));
	m_nh.param("/navdata_collector/metadata_topc", mstr_metadata_topic, string(""));


	message_filters::Subscriber<sensor_msgs::Image> m_mf_rgbSub(m_nh, mstr_rgb_topic, 1) ;
	message_filters::Subscriber<sensor_msgs::Image> m_mf_depthSub(m_nh, mstr_depth_topic, 1)  ;
	message_filters::Subscriber<sensor_msgs::LaserScan> m_mf_scanSub(m_nh, mstr_scan_topic, 1);
	message_filters::Subscriber<geometry_msgs::TwistStamped> m_mf_velSub(m_nh, mstr_twiststamped_topic, 1) ;
	message_filters::Subscriber<geometry_msgs::PoseStamped> m_mf_poseSub(m_nh, mstr_robotpose_topic, 1)  ;


//	ROS_INFO("rgb topic name: %s\n", mstr_rgb_topic.c_str()) ;
//	ROS_INFO("depth topic name: %s\n", mstr_depth_topic.c_str()) ;
//	ROS_INFO("robot pose topic: %s\n", mstr_robotpose_topic.c_str());

	// pub
	static map<string, int> metadata_id;
	metadata_id["rgbd_metadata"] = METDATA_TYPE::RGBD;
	metadata_id["scan_metadata"] = METDATA_TYPE::SCAN;
	metadata_id["comp_metadata"] = METDATA_TYPE::COMPLETE;

//	m_scan_sync.reset(new Scan_Sync(ApproxScanTimeSyncPolicy(10), m_mf_scanSub, m_mf_velSub, m_mf_poseSub) );
//	m_scan_sync->registerCallback(boost::bind(&NavDataCollector::ScanMetaDataCallBack, this, _1, _2, _3));
//	m_syncdataPub	= m_nh.advertise<navdata_collector::scan_metadata>(mstr_metadata_topic, 10);

	ROS_INFO("%s %d \n", mstr_metadata_topic.c_str(), metadata_id[ mstr_metadata_topic.c_str() ] ) ;

	switch ( metadata_id[ mstr_metadata_topic.c_str() ] )
	{
		case METDATA_TYPE::RGBD:
		{
			m_rgbd_sync.reset(new RGBD_Sync(ApproxRGBDTimeSyncPolicy(10), m_mf_rgbSub, m_mf_depthSub, m_mf_velSub, m_mf_poseSub) );
			m_rgbd_sync->registerCallback(boost::bind(&NavDataCollector::RGBDMetaDataCallBack, this, _1, _2, _3, _4));
			m_syncdataPub	= m_nh.advertise<navdata_collector::scan_metadata>(mstr_metadata_topic, 1);

			ROS_INFO(" RGBD-Metadata type collection is requested \n");
			break;
		}
		case METDATA_TYPE::SCAN:
		{
			m_scan_sync.reset(new Scan_Sync(ApproxScanTimeSyncPolicy(10), m_mf_scanSub, m_mf_velSub, m_mf_poseSub) );
			m_scan_sync->registerCallback(boost::bind(&NavDataCollector::ScanMetaDataCallBack, this, _1, _2, _3));
			m_syncdataPub	= m_nh.advertise<navdata_collector::scan_metadata>(mstr_metadata_topic, 1);

			ROS_INFO(" Scan-Metadata type collection is requested \n");
			break;
		}
		case METDATA_TYPE::COMPLETE:
		{
			  ROS_ERROR(" We don't know what is the complete dataset yet \n");
			  break;
		}

		default: ROS_ERROR("Invalid metdata type \n");
	}

	m_robotTwistSub = m_nh.subscribe(mstr_twist_topic, 1, &NavDataCollector::twistReceiveCallBack, this); // kmHan
	m_robotposePub = m_nh.advertise<geometry_msgs::PoseStamped>(mstr_robotpose_topic, 1);
	m_robotVelPub  = m_nh.advertise<geometry_msgs::TwistStamped>(mstr_twiststamped_topic, 1);
}

NavDataCollector::~NavDataCollector()
{

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


void NavDataCollector::RGBDMetaDataCallBack( const sensor_msgs::ImageConstPtr& rgb_msg,
						const sensor_msgs::ImageConstPtr& depth_msg,
						const geometry_msgs::TwistStampedConstPtr& vel_msg,
						const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
	ROS_INFO("rgbd msgs set\n");
	m_rgbd_metadata.header 	= (*pose_msg).header ;
	m_rgbd_metadata.rgb 	= *rgb_msg ;
	m_rgbd_metadata.depth	= *depth_msg;
	m_rgbd_metadata.cmd_vel	= (*vel_msg).twist ;
	m_rgbd_metadata.rpose	= (*pose_msg).pose ;

	m_syncdataPub.publish(m_rgbd_metadata);
	mn_data_cnt++;
	ROS_INFO("<%lu> th RGBD metadata has published @ R position <%f %f %f> \n",
			mn_data_cnt,
			(*pose_msg).pose.position.x,
			(*pose_msg).pose.position.y,
			(*pose_msg).pose.position.z);
}

void NavDataCollector::ScanMetaDataCallBack( 	const sensor_msgs::LaserScanConstPtr& scan_msg,
												const geometry_msgs::TwistStampedConstPtr& vel_msg,
												const geometry_msgs::PoseStamped::ConstPtr& pose_msg )
{
	ROS_INFO("got Scan msgs set\n");
	m_scan_metadata.header 	= (*scan_msg).header ;
	m_scan_metadata.cmd_vel 	= (*vel_msg).twist ;
	m_scan_metadata.scan	= *scan_msg;
	m_scan_metadata.rpose	= (*pose_msg).pose ;

	m_syncdataPub.publish(m_scan_metadata);
	mn_data_cnt++;
	ROS_INFO("<%lu> th SCAN metadata has published @ R position <%f %f %f> \n",
			mn_data_cnt,
			(*pose_msg).pose.position.x,
			(*pose_msg).pose.position.y,
			(*pose_msg).pose.position.z);
}

}
