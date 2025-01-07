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
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/action_client.h>

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

#include "motion_generator.hpp"
#include "navdata_collector/metadata.h"

namespace navdata
{

using namespace std;
using namespace message_filters;
//typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ApproxTimeSyncPolicy;
//typedef message_filters::Synchronizer<ApproxTimeSyncPolicy> Sync;
//typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> ExactTimeSyncPolicy;

class NavDataCollector
{
public:
	NavDataCollector(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_);
	virtual ~NavDataCollector();

	//void globalCostmapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg) ;
	void metadataCallBack( const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg,
			const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg) ;
	void robotPoseCallBack( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg ) ;
	void doneCB( const actionlib::SimpleClientGoalState& state ) ;

	//void generateGridmapFromCostmap( );
	inline bool isDone() const {  return mn_data_cnt > 1000 ; }

private:

	ros::NodeHandle m_nh;
	ros::NodeHandle m_nh_private;

	ros::Subscriber	m_rgbSub, m_depthSub, m_robotPoseSub ;

	message_filters::Subscriber<sensor_msgs::Image> m_mf_rgbSub ;
	message_filters::Subscriber<sensor_msgs::Image> m_mf_depthSub ;
	message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> m_mf_poseSub ;

	typedef sync_policies::ApproximateTime
			<sensor_msgs::Image, sensor_msgs::Image, geometry_msgs::PoseWithCovarianceStamped> ApproxTimeSyncPolicy;
	typedef Synchronizer<ApproxTimeSyncPolicy> Sync;
	boost::shared_ptr<Sync> m_sync;

//	message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, geometry_msgs::PoseWithCovariance> m_sync;
//	message_filters::Synchronizer<ApproxTimeSyncPolicy> m_sync ;

	ros::Publisher m_syncdataPub;

	string mstr_rgb_topic, mstr_depth_topic, mstr_robotpose_topic, mstr_metadata_topic ;
	string mstr_worldframe_id, mstr_robotframe_id ;

	navdata_collector::metadata m_metadata ;
	int64_t mn_data_cnt;
};


}






#endif /* INCLUDE_NAVDATA_COLLECTOR_HPP_ */
