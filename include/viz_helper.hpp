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


Mail:        Kyung Min Han
             Computer Graphics Lab                       
             Department of Computer Science and Engineering
             Ewha Womans University
             11-1 Daehyun-Dong Seodaemun-gu, Seoul, Korea 120-750


EMail:       hkm@ewha.ac.kr
*/

#ifndef INCLUDE_VIZ_HELPER_HPP_
#define INCLUDE_VIZ_HELPER_HPP_

#include <ros/console.h>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include "octomap_server/mapframedata.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>

#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point32.h"
#include "std_msgs/Bool.h"
//#include "neuro_explorer/VizDataStamped.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <tf/transform_listener.h>
#include <algorithm>
#include <random>
#include <unordered_set>
#include <boost/format.hpp>
#include <fstream>


#define FRONTIER_MARKER_SIZE (0.4)
#define TARGET_MARKER_SIZE (0.5)
#define UNREACHABLE_MARKER_SIZE (0.4)

namespace navdata_collector
{

using namespace std;

class VizHelper
{
public:

	VizHelper(){};
	VizHelper(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_);
	virtual ~VizHelper();

	void removeGoalPointMarker();
//	void setLocalFrontierPointMarkers(  const geometry_msgs::Point32& target_goal, const vector<geometry_msgs::Point32>& vo_localfpts_gm ) ;

	void publishGoalPointMarker( const geometry_msgs::Pose& targetgoal );
	void publishAll() ;

	void targetGoalCallback( const geometry_msgs::PoseWithCovarianceStamped& targetgoal );
	void doneCallback( const std_msgs::Bool::ConstPtr& msg  );
	bool isDone() const {return mb_navdata_collection_is_done; };

	visualization_msgs::Marker SetVizMarker( int32_t ID, int32_t action, float fx, float fy, float fz=0, string frame_id = "map",
			float fR=1.f, float fG=0.f, const float fB=1.f, const float falpha=1.f,	 float fscale=0.5)
	{
		visualization_msgs::Marker  viz_marker;
		viz_marker.header.frame_id= frame_id;
		viz_marker.header.stamp=ros::Time(0);
		viz_marker.ns= "markers";
		viz_marker.id = ID;
		viz_marker.action = action ;

		viz_marker.type = visualization_msgs::Marker::CUBE ;

		viz_marker.scale.x= fscale;
		viz_marker.scale.y= fscale;
		viz_marker.scale.z= fscale;

		viz_marker.color.r = fR;
		viz_marker.color.g = fG;
		viz_marker.color.b = fB;
		viz_marker.color.a= falpha;
		viz_marker.lifetime = ros::Duration();

		viz_marker.pose.position.x = fx;
		viz_marker.pose.position.y = fy;
		viz_marker.pose.position.z = fz;
		viz_marker.pose.orientation.w =1.0;

		return viz_marker;
	}

private:
	ros::NodeHandle m_nh;
	ros::NodeHandle m_nh_private;

	ros::Subscriber m_vizDataSub, m_doneSub, m_targetGoalSub;

	ros::Publisher  m_markerGoalPub, m_markerGoalCandPub;

	visualization_msgs::Marker  m_targetgoal_marker ;
	visualization_msgs::MarkerArray m_goalcand_markers ;

	int32_t mn_GoalCandID ;

	nav_msgs::OccupancyGrid m_gridmap;
	nav_msgs::OccupancyGrid m_globalcostmap ;

	geometry_msgs::Point32 m_rpos_w ; // (w.r.t world)

	std::string mstr_world_frame_id;
	std::string mstr_base_frmae_id ;

	string m_str_debugpath ;
	string m_str_inputparams ;
	cv::FileStorage m_fs;

	int mn_scale;
	int mn_globalmap_width, mn_globalmap_height, mn_globalmap_centx, mn_globalmap_centy ; // global
	//int mn_activemap_width, mn_activemap_height ;
	float mf_resolution ;

	int mn_cols, mn_rows, mn_orig_x_wrt_cent, mn_orig_y_wrt_cent ;
	bool mb_navdata_collection_is_done ;

	vector<cv::Point> m_frontiers;
	uint32_t mu_cmheight, mu_cmwidth, mu_gmheight, mu_gmwidth ;
//	geometry_msgs::Point32 m_targetgoal_w, m_optimal_targetgoal_w ; // actual goal /  optimal goal

};


}



#endif /* INCLUDE_VIZ_HELPER_HPP_ */
