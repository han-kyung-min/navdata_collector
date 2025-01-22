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

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "navdata_collector/scan_metadata.h"
#include "navdata_collector/rgbd_metadata.h"
#include "navdata_collector/comp_metadata.h"
//#include "navdata_collector/comp_metadata.h"
#include <cstdarg>

namespace navdata_collector
{

using namespace std;
using namespace message_filters;

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

	void ScanMetaDataCallBack( 	const sensor_msgs::LaserScanConstPtr& scan_msg,
								const nav_msgs::OccupancyGridConstPtr& map_msg,
								const geometry_msgs::TwistStampedConstPtr& vel_msg,
								const geometry_msgs::PoseStampedConstPtr& pose_msg );

	void RGBDMetaDataCallBack( 	const sensor_msgs::ImageConstPtr& rgb_msg,
								const sensor_msgs::ImageConstPtr& depth_msg,
								const geometry_msgs::TwistStampedConstPtr& vel_msg,
								const geometry_msgs::PoseStamped::ConstPtr& pose_msg,
								const nav_msgs::Odometry::ConstPtr& odom_msg,
								const nav_msgs::Odometry::ConstPtr& odom_f_msg);

	void CompMetaDataCallBack( 	const sensor_msgs::ImageConstPtr& rgb_msg,
								const sensor_msgs::ImageConstPtr& depth_msg,
								const sensor_msgs::LaserScanConstPtr& scan_msg,
								const nav_msgs::OccupancyGridConstPtr& map_msg,
								const geometry_msgs::TwistStampedConstPtr& vel_msg,
								const geometry_msgs::PoseStamped::ConstPtr& pose_msg,
								const nav_msgs::Odometry::ConstPtr& odom_msg,
								const nav_msgs::Odometry::ConstPtr& odom_f_msg);

	void doneCallBack( const std_msgs::BoolConstPtr& done_msg ) ;

	void twistReceiveCallBack( const geometry_msgs::TwistConstPtr& msg ) ;
	void publishRobotPose(  ) ;
	void publishRobotVel( );

	void departFlagCallBack( const std_msgs::BoolConstPtr& depart_msg ) ;
	void arrivalCallBack( const std_msgs::Int8ConstPtr& arrival_msg ) ;

	bool waitForScanMetadata( ) ;
	bool waitForRGBDMetadata( ) ;
	bool waitForCompMetadata( ) ;

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

	message_filters::Subscriber<sensor_msgs::Image> m_mf_rgbSub ;
	message_filters::Subscriber<sensor_msgs::Image> m_mf_depthSub ;
	message_filters::Subscriber<sensor_msgs::LaserScan> m_mf_scanSub ;
	message_filters::Subscriber<nav_msgs::OccupancyGrid> m_mf_mapSub ;

	message_filters::Subscriber<geometry_msgs::PoseStamped > m_mf_poseSub ;
	message_filters::Subscriber<geometry_msgs::TwistStamped> m_mf_velSub ;

	message_filters::Subscriber<nav_msgs::Odometry> m_mf_odomSub ;
	message_filters::Subscriber<nav_msgs::Odometry> m_mf_odomFilteredSub ;

	ros::Subscriber m_arrivalmsgSub, m_departmsgSub;

	typedef sync_policies::ApproximateTime
			<sensor_msgs::Image, sensor_msgs::Image, geometry_msgs::TwistStamped, geometry_msgs::PoseStamped,
			nav_msgs::Odometry, nav_msgs::Odometry> ApproxRGBDTimeSyncPolicy;
	typedef Synchronizer<ApproxRGBDTimeSyncPolicy> RGBD_Sync;

	typedef sync_policies::ApproximateTime
			<sensor_msgs::LaserScan, nav_msgs::OccupancyGrid, geometry_msgs::TwistStamped, geometry_msgs::PoseStamped> ApproxScanTimeSyncPolicy;
	typedef Synchronizer<ApproxScanTimeSyncPolicy> Scan_Sync;

	typedef sync_policies::ApproximateTime
			<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::LaserScan, nav_msgs::OccupancyGrid,
			geometry_msgs::TwistStamped, geometry_msgs::PoseStamped,
			nav_msgs::Odometry, nav_msgs::Odometry>
			ApproxCompTimeSyncPolicy;
	typedef Synchronizer<ApproxCompTimeSyncPolicy> Comp_Sync;

	boost::shared_ptr<RGBD_Sync> m_rgbd_sync;
	boost::shared_ptr<Scan_Sync> m_scan_sync;
	boost::shared_ptr<Comp_Sync> m_comp_sync;

	ros::Publisher m_syncdataPub;
	ros::Publisher m_robotposePub, m_robotVelPub ;
	ros::Publisher m_initdonePub ;

	string mstr_rgb_topic, mstr_depth_topic, mstr_scan_topic, mstr_map_topic, mstr_metadata_topic ;
	string mstr_twist_topic, mstr_twiststamped_topic, mstr_robotpose_topic, mstr_odom_topic, mstr_odom_filtered_topic ;
	string mstr_worldframe_id, mstr_robotframe_id ;

	navdata_collector::rgbd_metadata m_rgbd_metadata ;
	navdata_collector::scan_metadata m_scan_metadata ;
	navdata_collector::comp_metadata m_comp_metadata ;

	uint64_t mu_data_cnt;
	tf::TransformListener m_listener;
	geometry_msgs::TwistStamped m_twiststamped ;

	rosbag::Bag m_bag;

	int32_t mn_bagfile_cnt, mn_max_num_bagfiles ;
	string mstr_bagfile, mstr_bagfile_path ;
	bool mb_is_bag_accessible ;
	bool mb_navdata_collection_is_completed ;

	std::mutex mutex_bag;
};


}






#endif /* INCLUDE_NAVDATA_COLLECTOR_HPP_ */
