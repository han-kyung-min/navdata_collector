/*
 * navdata_collector.cpp
 *
 *  Created on: Jan 6, 2025
 *      Author: hankm
 */


#include "navdata_collector.hpp"

namespace navdata
{

NavDataCollector::NavDataCollector(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_):
m_nh_private(private_nh_),
m_nh(nh_),
mn_data_cnt(0)
{
	m_nh.param("/navdata_collector/rgb_topic", mstr_rgb_topic);
	m_nh.param("/navdata_collector/depth_topic", mstr_depth_topic);
	m_nh.param("/navdata_collector/world_frame_id", mstr_worldframe_id, std::string("map"));
	m_nh.param("/navdata_collector/robot_frame_id", mstr_robotframe_id, std::string("base_link"));
	message_filters::Subscriber<sensor_msgs::Image> m_mf_rgbSub(m_nh, mstr_rgb_topic, 1) ;
	message_filters::Subscriber<sensor_msgs::Image>m_mf_depthSub(m_nh, mstr_depth_topic, 1)  ;
	message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> m_mf_poseSub(m_nh, mstr_robotpose_topic, 1)  ;
	m_sync.reset(new Sync(ApproxTimeSyncPolicy(10), m_mf_rgbSub, m_mf_depthSub, m_mf_poseSub) );
	m_sync->registerCallback(boost::bind(&NavDataCollector::metadataCallBack, this, _1, _2, _3));
	// pub
	m_syncdataPub = m_nh.advertise<navdata_collector::metadata>(mstr_metadata_topic, 10);
}

NavDataCollector::~NavDataCollector()
{

}

void NavDataCollector::metadataCallBack( const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg,
						const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
	ROS_INFO("got msgs \n");
	m_metadata.header 	= (*pose_msg).header ;
	m_metadata.rgb 		= *rgb_msg ;
	m_metadata.depth	= *depth_msg;
	m_metadata.rpose	= (*pose_msg).pose ;

	m_syncdataPub.publish(m_metadata);
	mn_data_cnt++;
	ROS_INFO("<%lu> th metadata is published @ R position <%f %f %f> \n",
			mn_data_cnt,
			(*pose_msg).pose.pose.position.x,
			(*pose_msg).pose.pose.position.y,
			(*pose_msg).pose.pose.position.z);
}


}
