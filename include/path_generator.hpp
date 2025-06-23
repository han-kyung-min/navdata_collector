/*********************************************************************
Copyright 2025 The Ewha Womans University.
All Rights Reserved.
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE
Permission to use, copy, modify OR distribute this software and its
documentation for educational, research and non-profit purposes, without
fee, and without a written agreement is hereby granted, provided that the
above copyright notice and the following three paragraphs appear in all
copies.

IN NO EVENT SHALL THE EWHA WOMANS UNIVERSITY BE
LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE
USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE EWHA WOMANS UNIVERSITY
BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.

THE EWHA WOMANS UNIVERSITY SPECIFICALLY DISCLAIM ANY
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE
PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND THE EWHA WOMANS UNIVERSITY
HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT,
UPDATES, ENHANCEMENTS, OR MODIFICATIONS.


The authors may be contacted via:


Mail:        Young J. Kim, Kyung Min Han
             Computer Graphics Lab                       
             Department of Computer Science and Engineering
             Ewha Womans University
             11-1 Daehyun-Dong Seodaemun-gu, Seoul, Korea 120-750


EMail:       hkm@ewha.ac.kr
*/

#ifndef INCLUDE_PATH_GENERATOR_HPP_
#define INCLUDE_PATH_GENERATOR_HPP_

#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>

#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "navdata_collector.hpp"

#include <move_base/move_base.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/action_client.h>
#include "ros/service_client.h"

#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

#define GLOBAL_MAP_XOFFSET (480) // location of (lower left corner of GM)
#define GLOBAL_MAP_YOFFSET (480)

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> SimpleMoveBaseClient;

namespace navdata_collector
{

using namespace std;

typedef enum{	ROBOT_IS_NOT_MOVING 	= -1,
				ROBOT_IS_READY_TO_MOVE	= 0,
				FORCE_TO_STOP    		= 1, // is moving but needs to be stopped
				ROBOT_IS_MOVING  		= 2
			} ROBOT_STATE ;


static const char *robot_state[] =
	  { "ROBOT_IS_NOT_MOVING", "ROBOT_IS_READY_TO_MOVE", "FORCE_TO_STOP", "ROBOT_IS_MOVING" };

enum PrevExpState {
  ABORTED = 0,
  SUCCEEDED
};
static const char *prevstate_str[] =
        { "ABORTED", "SUCCEEDED" };

class PathGenerator
{
public:
	PathGenerator(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_);
	virtual ~PathGenerator();

	geometry_msgs::PoseStamped GetCurrRobotPose ( )
	{
		tf::StampedTransform map2baselink;
		try{
		  m_listener.lookupTransform(mstr_worldframe_id, mstr_baseframe_id,
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
		outPose.header.stamp = ros::Time::now();

		return outPose;
	}

	geometry_msgs::PoseWithCovarianceStamped StampedPoseCovfromSE2( const float& x, const float& y, const float& yaw_radian )
	{
		geometry_msgs::PoseWithCovarianceStamped outPose ;
		outPose.pose.pose.position.x = x ;
		outPose.pose.pose.position.y = y ;

		float c[3] = {0,};
		float s[3] = {0,};
		c[0] = cos(yaw_radian/2) ;
		c[1] = cos(0) ;
		c[2] = cos(0) ;
		s[0] = sin(yaw_radian/2) ;
		s[1] = sin(0) ;
		s[2] = sin(0) ;

		float qout[4] = {0,};
		qout[0] = c[0]*c[1]*c[2] + s[0]*s[1]*s[2];
		qout[1] = c[0]*c[1]*s[2] - s[0]*s[1]*c[2];
		qout[2] = c[0]*s[1]*c[2] + s[0]*c[1]*s[2];
		qout[3] = s[0]*c[1]*c[2] - c[0]*s[1]*s[2];

		outPose.pose.pose.orientation.w = qout[0] ;
		outPose.pose.pose.orientation.x = qout[1] ;
		outPose.pose.pose.orientation.y = qout[2] ;
		outPose.pose.pose.orientation.z = qout[3] ;

		outPose.header.frame_id = mstr_worldframe_id ;
		outPose.header.stamp = ros::Time::now() ;

		return outPose;
	}

	void clusterToThreeLabels( cv::Mat& uImage  )
	{
		cv::Mat uUnkn = uImage.clone();
		cv::threshold( uUnkn, uUnkn, 187, 255, cv::THRESH_TOZERO_INV ); 	// 187 ~ 255 --> 0
		cv::threshold( uUnkn, uUnkn, 67,  255, cv::THRESH_TOZERO ); 		// 0 ~ 66 	--> 0
		cv::threshold( uUnkn, uUnkn, 0, OCC_LABEL::UNKNOWN, cv::THRESH_BINARY) ;// 67 ~ 187  --> 127 (unknown)

		cv::Mat uOcc = uImage.clone();
		cv::threshold( uOcc, uOcc, 128, OCC_LABEL::OCCUPIED, cv::THRESH_BINARY ); // 187 ~ 255 --> 255
		uImage = uOcc + uUnkn ;
#ifdef SAVE_DEBUG_IMAGES
		cv::imwrite("/home/hankm/catkin_ws/src/frontier_detector/launch/uImage.png",  uImage);
		cv::imwrite("/home/hankm/catkin_ws/src/frontier_detector/launch/occ.png",  uOcc);
		cv::imwrite("/home/hankm/catkin_ws/src/frontier_detector/launch/unkn.png", uUnkn);
#endif
	}

	cv::Point world2gridmap( cv::Point2f world_pos );
	cv::Point2f gridmap2world( cv::Point gm_pos );

	//void globalCostmapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg) ;
	void generateGridmapFromCostmap( ) ;
	void initGlobalmapimgs( const cv::Rect& roi, const nav_msgs::OccupancyGrid& globalcostmap ) ;
	bool isMapValid( ) ;
	bool isDone() { return mb_data_collection_is_done; } ;

	void robotVelCallBack( const geometry_msgs::Twist::ConstPtr& msg ) ;

	void publishArrivalStatus( ) ;
	void publishDepartureFlag( );

	void doneCB( const actionlib::SimpleClientGoalState& state ) ;
	void moveRobotCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg ) ;
	void mapdataCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) ;
	void globalCostmapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg);

	int sampleNextGoal( const cv::Mat& gridmap_img, const cv::Point& rpos_gm, cv::Point& targetgoal_gm );
	int moveToNextGoal();

// vis

protected:
	ros::NodeHandle m_nh;
	ros::NodeHandle m_nh_private;

	// action server
	SimpleMoveBaseClient m_move_client ;
	ros::ServiceClient m_makeplan_client;

// subscriber
	ros::Subscriber 	m_globalCostmapSub, m_currGoalSub, m_globalplanSub, m_mapframedataSub,
						m_is_navcollector_inialized ;

// publisher
	ros::Publisher 		m_markercandPub, m_makergoalPub,
						m_currentgoalPub, m_donePub, m_departureFlagPub, m_arrivalStatusPub ;

	string mstr_worldframe_id, mstr_baseframe_id ;

// gridmap / costmap
	nav_msgs::OccupancyGrid m_gridmap ;
	uint32_t mu_cmheight, mu_cmwidth, mu_gmheight, mu_gmwidth ;
	nav_msgs::OccupancyGrid m_globalcostmap ;
	costmap_2d::Costmap2D* mpo_costmap;
	uint8_t* mp_cost_translation_table;
	int mn_cols, mn_rows, mn_roi_origx, mn_roi_origy, mn_globalmap_width, mn_globalmap_height,
		mn_globalcostmap_xoffset, mn_globalcostmap_yoffset; //mn_globalmap_centx, mn_globalmap_centy, mn_globalmap_xc_, mn_globalmap_yc_  ;

	cv::Mat mcvu_costmapimg, mcvu_costmapimg_ds, mcvu_globalmapimg, mcvu_globalmapimg_ds ;

	int mn_occupancy_thr, mn_obstacle_cost_thr, mf_min_targetdist_meter ;
	int mn_scale, mn_numpyrdownsample ;
    //cv::Rect m_roi_active;

// robot state
	geometry_msgs::Twist m_robotvel ;
	PrevExpState me_prev_exploration_state ;
	ROBOT_STATE me_robotstate ;
	geometry_msgs::PoseStamped m_prev_robot_pose, m_targetgoal ;
	cv::Point m_rpos_gm ;
	geometry_msgs::PoseStamped m_rpos_world ;
	geometry_msgs::PoseWithCovarianceStamped m_move_robot_goal ;

	tf::TransformListener m_listener;

// timer / debug
	double mf_totalcallbacktime_msec ;
	int32_t mn_tot_nav_time, mn_start_nav_time, mn_max_nav_time ;
	bool mb_data_collection_is_done ;
	string mstr_debugpath ;
	int32_t mn_mapcallcnt ;

// lock
	std::mutex mutex_robot_state;
	std::mutex mutex_gridmap;
	std::mutex mutex_costmap;
	std::mutex mutex_timing_profile;
	std::mutex mutex_currgoal ;
};

}

#endif /* INCLUDE_PATH_GENERATOR_HPP_ */
