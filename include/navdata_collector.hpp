/*
 * navdata_collector.hpp
 *
 *  Created on: Jan 6, 2025
 *      Author: hankm
 */

#ifndef INCLUDE_NAVDATA_COLLECTOR_HPP_
#define INCLUDE_NAVDATA_COLLECTOR_HPP_

#include <ros/ros.h>
#include <ros/console.h>
//#include <cv_bridge/cv_bridge.h>

#include <move_base/move_base.h>
#include <move_base_msgs/MoveBaseAction.h>

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

#include "navdata_collector/rgbd.h"
#include <cstdarg>

namespace navdata_collector
{

using namespace std;
using namespace message_filters;

typedef enum{	FREE 	= 0,
				UNKNOWN	= 127,
				OCCUPIED= 255, // is moving but needs to be stopped
			} OCC_LABEL ;


class NavDataCollector
{
public:
	NavDataCollector(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_);
	virtual ~NavDataCollector();

	void doneCallBack( const std_msgs::BoolConstPtr& done_msg ) ;

	void twistReceiveCallBack( const geometry_msgs::TwistConstPtr& msg ) ;
	void publishRobotPose(  ) ;
	//void publishRobotVel( );

	void departFlagCallBack( const std_msgs::BoolConstPtr& depart_msg ) ;

	bool waitForCompMetadata( ) ;

	void RGBDCallBack( const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg) ;
	//void generateGridmapFromCostmap( );
	inline bool isDone() const {  return ( mn_bagfile_cnt > mn_max_num_bagfiles || mb_navdata_collection_is_completed ); }

	std::string fmt(const std::string& fmt, ...)
	{
	    int size = 200;
	    std::string str;
	    va_list ap;
	    while (1)
	    {
	        str.resize(size);
	        va_start(ap, fmt);
	        int n = vsnprintf((char*)str.c_str(), size, fmt.c_str(), ap);
	        va_end(ap);
	        if (n > -1 && n < size)
	        {
	            str.resize(n);
	            return str;
	        }
	        if (n > -1)
	            size = n + 1;
	        else
	            size *= 2;
	    }
	    return str;
	}

//	geometry_msgs::PoseStamped GetCurrRobotPose ( )
//	{
//		tf::StampedTransform map2baselink;
//		try{
//		  m_listener.lookupTransform(mstr_worldframe_id, mstr_robotframe_id,
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
//		outPose.header.frame_id = mstr_worldframe_id;
//
//		return outPose;
//	}

private:

	ros::NodeHandle m_nh;
	ros::NodeHandle m_nh_private;
	ros::Subscriber	m_robotPoseSub,  m_robotTwistSub, m_currGoalSub, m_doneSub  ;

	navdata_collector::rgbd m_rgbd ;

	typedef sync_policies::ApproximateTime
			<sensor_msgs::Image, sensor_msgs::Image> ApproxRGBDTimeSyncPolicy;
	typedef sync_policies::ExactTime
			<sensor_msgs::Image, sensor_msgs::Image> ExactRGBDTimeSyncPolicy;

	typedef Synchronizer<ExactRGBDTimeSyncPolicy> RGBD_Sync;
	boost::shared_ptr<RGBD_Sync> m_rgbd_sync;
	message_filters::Subscriber<sensor_msgs::Image> m_mf_rgbSub ;
	message_filters::Subscriber<sensor_msgs::Image> m_mf_depthSub ;

//	ros::Subscriber m_departmsgSub;
	ros::Publisher m_robotposePub, m_robotVelPub ;
	ros::Publisher m_initdonePub ;
	ros::Publisher m_syncdataPub ;

	string mstr_rgb_topic, mstr_depth_topic, mstr_scan_topic, mstr_map_topic, mstr_metadata_topic ;
	string mstr_twist_topic, mstr_twiststamped_topic, mstr_robotpose_topic, mstr_odom_topic, mstr_odom_filtered_topic, mstr_rgbd_topic ;
	string mstr_worldframe_id, mstr_robotframe_id ;

	tf::TransformListener m_listener;
	geometry_msgs::TwistStamped m_twiststamped ;

	int32_t mn_bagfile_cnt, mn_max_num_bagfiles ;
	string mstr_bagfile, mstr_bagfile_path ;

	bool mb_navdata_collection_is_completed ;

};


}






#endif /* INCLUDE_NAVDATA_COLLECTOR_HPP_ */
