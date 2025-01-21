/*********************************************************************
Copyright 2024 The Ewha Womans University.
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


Mail:        Y. J. Kim, Kyung Min Han
             Computer Graphics Lab
             Department of Computer Science and Engineering
             Ewha Womans University
             11-1 Daehyun-Dong Seodaemun-gu, Seoul, Korea 120-750


Phone:       +82-2-3277-6798


EMail:       kimy@ewha.ac.kr
             hankm@ewha.ac.kr
*/


#include "viz_helper.hpp"

namespace navdata_collector
{

VizHelper::VizHelper(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_):
m_nh_private(private_nh_),
m_nh(nh_),
mb_navdata_collection_is_done(false)
{
	m_nh.getParam("/navdata_collector/debug_data_save_path", m_str_debugpath);

	m_nh.param("/navdata_collector/global_width",  mn_globalmap_width, 	2048) ;
	m_nh.param("/navdata_collector/global_height", mn_globalmap_height,	2048) ;

	m_nh.param("/navdata_collector/frame_id", mstr_world_frame_id, std::string("map"));
	m_nh.param("/move_base/global_costmap/resolution", mf_resolution, 0.05f) ;
//	m_nh.param("move_base/global_costmap/robot_radius", mf_robot_radius, 0.12); // 0.3 for fetch

//	m_vizDataSub  	= m_nh.subscribe("viz_data", 1, &VizHelper::vizCallback, this); // kmHan
	m_targetGoalSub = m_nh.subscribe("curr_goalpose", 1, &VizHelper::targetGoalCallback, this);
	m_doneSub		= m_nh.subscribe("data_collection_is_completed", 1, &VizHelper::doneCallback, this);

	//m_frontier_region_markers = SetVizMarker( 0, visualization_msgs::Marker::ADD, 0.f, 0.f, 0.f, m_worldFrameId, 1.f, 0.f, 0.f, 1.f, 0.1 );
	//m_frontier_region_markers.type = visualization_msgs::Marker::POINTS;

	m_goalcand_markers = visualization_msgs::MarkerArray() ;
	m_markerGoalPub = m_nh.advertise<visualization_msgs::Marker>("curr_goal_marker",10);
//	m_markerGoalCandPub = m_nh.advertise<visualization_msgs::MarkerArray>("global_frontier_point_markers", 10);
//	mn_FrontierID = 1;
//	mn_global_FrontierID = 1;
}

VizHelper::~VizHelper()
{
}

void VizHelper::targetGoalCallback( const geometry_msgs::PoseWithCovarianceStamped& targetgoal )
{
	m_targetgoal_marker.points.clear();
	m_targetgoal_marker = SetVizMarker( -1, visualization_msgs::Marker::ADD, targetgoal.pose.pose.position.x, targetgoal.pose.pose.position.y, 0.f,
			mstr_world_frame_id,	1.f, 0.f, 1.f, 1.f, (float)TARGET_MARKER_SIZE );
	m_markerGoalPub.publish(m_targetgoal_marker); // for viz
}

void VizHelper::publishGoalPointMarker( const geometry_msgs::Pose& targetgoal )
{
	m_targetgoal_marker.points.clear();
	m_targetgoal_marker = SetVizMarker( -1, visualization_msgs::Marker::ADD, targetgoal.position.x, targetgoal.position.y, 0.f,
			mstr_world_frame_id,	1.f, 0.f, 1.f, 1.f, (float)TARGET_MARKER_SIZE );
	m_markerGoalPub.publish(m_targetgoal_marker); // for viz
}


void VizHelper::doneCallback( const std_msgs::Bool::ConstPtr& msg  )
{
	if( msg->data  == true)
		mb_navdata_collection_is_done = true;
}


}

