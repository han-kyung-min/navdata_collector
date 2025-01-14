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

#include "navdata_collector/scan_metadata.h"
#include "navdata_collector/rgbd_metadata.h"
//#include "navdata_collector/comp_metadata.h"

namespace navdata_collector
{

using namespace std;
using namespace message_filters;
//typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ApproxTimeSyncPolicy;
//typedef message_filters::Synchronizer<ApproxTimeSyncPolicy> Sync;
//typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> ExactTimeSyncPolicy;

typedef enum{	FREE 	= 0,
				UNKNOWN	= 127,
				OCCUPIED= 255, // is moving but needs to be stopped
			} OCC_LABEL ;

typedef enum {RGBD=1, SCAN, COMPLETE} METDATA_TYPE;

class NavDataCollector
{
public:
	NavDataCollector(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_);
	virtual ~NavDataCollector();

	//void globalCostmapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg) ;
	void TestMetaDataCallBack( 	const sensor_msgs::LaserScanConstPtr& scan_msg,
								const nav_msgs::OdometryConstPtr& odom_msg) ;

	void ScanMetaDataCallBack( 	const sensor_msgs::LaserScanConstPtr& scan_msg,
								const geometry_msgs::TwistStampedConstPtr& vel_msg,
								const geometry_msgs::PoseStamped::ConstPtr& pose_msg ) ;

	void RGBDMetaDataCallBack( 	const sensor_msgs::ImageConstPtr& rgb_msg,
								const sensor_msgs::ImageConstPtr& depth_msg,
								const geometry_msgs::TwistStampedConstPtr& vel_msg,
								const geometry_msgs::PoseStamped::ConstPtr& pose_msg);

//	void metadataCallBack( 	const sensor_msgs::LaserScanConstPtr& scan_msg,
//							const geometry_msgs::TwistConstPtr& vel_msg,
//							const geometry_msgs::PoseStamped::ConstPtr& pose_msg ) ;
	void twistReceiveCallBack( const geometry_msgs::TwistConstPtr& msg ) ;
	void publishRobotPose(  ) ;
	void publishRobotVel( );

	//void generateGridmapFromCostmap( );
	inline bool isDone() const {  return mn_data_cnt > 1000 ; }

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

	ros::Subscriber	m_robotPoseSub,  m_robotTwistSub  ;

	message_filters::Subscriber<sensor_msgs::Image> m_mf_rgbSub ;
	message_filters::Subscriber<sensor_msgs::Image> m_mf_depthSub ;
	message_filters::Subscriber<sensor_msgs::LaserScan> m_mf_scanSub ;

	message_filters::Subscriber<geometry_msgs::PoseStamped> m_mf_poseSub ;
	message_filters::Subscriber<geometry_msgs::TwistStamped> m_mf_velSub ;


	typedef sync_policies::ApproximateTime
			<sensor_msgs::Image, sensor_msgs::Image, geometry_msgs::TwistStamped, geometry_msgs::PoseStamped> ApproxRGBDTimeSyncPolicy;
	typedef Synchronizer<ApproxRGBDTimeSyncPolicy> RGBD_Sync;

	typedef sync_policies::ApproximateTime
			<sensor_msgs::LaserScan, geometry_msgs::TwistStamped, geometry_msgs::PoseStamped> ApproxScanTimeSyncPolicy;
	typedef Synchronizer<ApproxScanTimeSyncPolicy> Scan_Sync;


	boost::shared_ptr<RGBD_Sync> m_rgbd_sync;
	boost::shared_ptr<Scan_Sync> m_scan_sync;

//	message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, geometry_msgs::PoseWithCovariance> m_sync;
//	message_filters::Synchronizer<ApproxTimeSyncPolicy> m_sync ;

	ros::Publisher m_syncdataPub;
	ros::Publisher m_robotposePub, m_robotVelPub ;

	string mstr_rgb_topic, mstr_depth_topic, mstr_robotpose_topic, mstr_scan_topic, mstr_metadata_topic ;
	string mstr_twist_topic, mstr_twiststamped_topic ;
	string mstr_worldframe_id, mstr_robotframe_id ;

	navdata_collector::rgbd_metadata m_rgbd_metadata ;
	navdata_collector::scan_metadata m_scan_metadata ;
	int64_t mn_data_cnt;
	tf::TransformListener m_listener;

	geometry_msgs::TwistStamped m_twiststamped ;

	//tf::MessageFilter<geometry_msgs::PoseStamped> m_tf_filter;
};


}






#endif /* INCLUDE_NAVDATA_COLLECTOR_HPP_ */
