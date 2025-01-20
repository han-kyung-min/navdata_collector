/*
 * path_generator.cpp
 *
 *  Created on: Jan 8, 2025
 *      Author: hankm
 */

#include "path_generator.hpp"

namespace navdata_collector
{

PathGenerator::PathGenerator(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_):
m_nh_private(private_nh_),
m_nh(nh_),
mp_cost_translation_table(NULL),
m_move_client("move_base", true),
mb_data_collection_is_done(false),
mstr_worldframe_id("map"), mstr_baseframe_id("base_link"),
mn_numpyrdownsample(0), mn_occupancy_thr(50), mn_obstacle_cost_thr(24), mn_mapcallcnt(0),  mf_min_targetdist_meter(8.0),
mn_tot_nav_time(0), mn_max_nav_time(3600),
mn_globalmap_width(2048), mn_globalmap_height(2048)
{
// params
	m_nh.getParam("/navdata_collector/debug_data_save_path", mstr_debugpath);
	m_nh.param("/navdata_collector/max_nav_time", mn_max_nav_time, 3600); // in sec
	m_nh.param("/navdata_collector/global_width", mn_globalmap_width, 2048) ;
	m_nh.param("/navdata_collector/global_height",mn_globalmap_height,2048) ;
	m_nh.param("/navdata_collector/occupancy_thr", mn_occupancy_thr, mn_occupancy_thr);
	m_nh.param("/navdata_collector/obstacle_cost_thr", mn_obstacle_cost_thr, mn_obstacle_cost_thr);
	m_nh.param("/navdata_collector/min_targetdist_meter", mf_min_targetdist_meter, mf_min_targetdist_meter);
	m_nh.param("/navdata_collector/world_frame_id", mstr_worldframe_id) ;
	m_nh.param("/navdata_collector/base_frame_id", mstr_baseframe_id);
// subscriber
	m_currGoalSub 		= m_nh.subscribe("curr_goalpose",1 , &PathGenerator::moveRobotCallback, this) ; // kmHan
	m_globalCostmapSub 	= m_nh.subscribe("move_base/global_costmap/costmap", 1, &PathGenerator::globalCostmapCallBack, this );
	m_mapframedataSub  	= m_nh.subscribe("move_base/global_costmap/costmap", 1, &PathGenerator::mapdataCallback, this); // kmHan

//	m_is_navcollector_inialized = m_nh.subscribe("navdata_collector_is_initialized", 1, callback, tracked_object, transport_hints)
// publisher
	m_currentgoalPub = m_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("curr_goalpose", 10);
	m_departureFlagPub = m_nh.advertise<std_msgs::Bool>("departure_flag", 1) ;
	m_arrivalStatusPub = m_nh.advertise<std_msgs::Int8>("arrival_status", 1);

	mcvu_costmapimg     = cv::Mat(mn_globalmap_height, mn_globalmap_width, CV_8U, cv::Scalar(255));
	mcvu_globalmapimg   = cv::Mat(mn_globalmap_height, mn_globalmap_width, CV_8U, cv::Scalar(127));
	mn_scale = pow(2, mn_numpyrdownsample);


// check if targetdist_thr sufficient for mapsize
	// check mapsize
	// isMapValid ()

	while(!m_move_client.waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	ROS_INFO("move_base action server is up");

	mpo_costmap = new costmap_2d::Costmap2D();
	if (mp_cost_translation_table == NULL)
	{
		mp_cost_translation_table = new uint8_t[101];
		// special values:
		mp_cost_translation_table[0] = 0;  // NO obstacle
		mp_cost_translation_table[99] = 253;  // INSCRIBED obstacle
		mp_cost_translation_table[100] = 254;  // LETHAL obstacle
//		mp_cost_translation_table[-1] = 255;  // UNKNOWN

		// regular cost values scale the range 1 to 252 (inclusive) to fit
		// into 1 to 98 (inclusive).
		for (int i = 1; i < 99; i++)
		{
			mp_cost_translation_table[ i ] = uint8_t( ((i-1)*251 -1 )/97+1 );
		}
	}

    uint32_t start_time = ros::Time::now().sec ;
    mn_start_nav_time = static_cast<int>(start_time) ;

	while (true)
	{
		if( ros::topic::waitForMessage<std_msgs::Bool>("navdata_collector_is_initialized", m_nh, ros::Duration(1.0) )  )
		{
			ROS_INFO("Navdata_collector initialized \n");
			break ;
		}
		else
			ROS_WARN("@PathGenerator Cannot start path_generator b/c navdata_collector has not been initilized yet \n" );
	}

}

PathGenerator::~PathGenerator()
{
	delete [] mp_cost_translation_table;
}


void PathGenerator::globalCostmapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	//ROS_INFO("@globalCostmapCallBack \n");
	const std::unique_lock<mutex> lock(mutex_costmap);
//ROS_INFO("cm callback is called \n");
	m_globalcostmap = *msg ;
	mu_cmheight = m_globalcostmap.info.height ;
	mu_cmwidth = m_globalcostmap.info.width ;
}

void PathGenerator::generateGridmapFromCostmap( )
{
	m_gridmap = m_globalcostmap ;
	m_gridmap.info = m_globalcostmap.info ;

	int gmheight = m_globalcostmap.info.height ;
	int gmwidth = m_globalcostmap.info.width ;

	for( int ii =0 ; ii < gmheight; ii++)
	{
		for( int jj = 0; jj < gmwidth; jj++)
		{
			int8_t obs_cost  = m_globalcostmap.data[ ii * gmwidth + jj] ;

			if ( obs_cost < 0) // if unknown
			{
				m_gridmap.data[ ii*gmwidth + jj ] = -1 ;
			}
			else if( obs_cost > 97 ) // mp_cost_translation_table[51:98] : 130~252 : possibly circumscribed ~ inscribed
			{
				m_gridmap.data[ ii*gmwidth + jj] = 100 ;
			}
			else
			{
				m_gridmap.data[ ii*gmwidth + jj] = 0 ;
			}
		}
	}
	//ROS_INFO("cmap: ox oy: %f %f \n W H: %d %d", m_globalcostmap.info.origin.position.x, m_globalcostmap.info.origin.position.y, m_globalcostmap.info.width, m_globalcostmap.info.height);
	//ROS_INFO("gmap: ox oy: %f %f \n W H: %d %d", m_gridmap.info.origin.position.x, m_gridmap.info.origin.position.y, m_gridmap.info.width, m_gridmap.info.height);
}

cv::Point2f PathGenerator::gridmap2world( cv::Point img_pt_roi  )
{
	float fgx =  static_cast<float>(img_pt_roi.x) * m_gridmap.info.resolution + m_gridmap.info.origin.position.x  ;
	float fgy =  static_cast<float>(img_pt_roi.y) * m_gridmap.info.resolution + m_gridmap.info.origin.position.y  ;

	return cv::Point2f( fgx, fgy );
}

cv::Point PathGenerator::world2gridmap( cv::Point2f grid_pt)
{
	float fx = (grid_pt.x - m_gridmap.info.origin.position.x) / m_gridmap.info.resolution ;
	float fy = (grid_pt.y - m_gridmap.info.origin.position.y) / m_gridmap.info.resolution ;

	return cv::Point( (int)fx, (int)fy );
}

//void PathGenerator::publishArrival( )
//{
//
////	if( mb_return_home ) // return to home position
////		moveToHome();
//
////    double favg_callback_time = mf_totalcallbacktime_msec / (double)(mn_mapcallcnt) ;
////	double favg_planning_time = mf_totalplanningtime_msec / (double)(mn_mapcallcnt) ;
////
////	ROS_INFO("total callback time (sec) %f \n", mf_totalcallbacktime_msec / 1000 );
////	ROS_INFO("total planning time (sec) %f \n", mf_totalplanningtime_msec / 1000 );
////	ROS_INFO("avg callback time (msec) %f \n", favg_callback_time  );
////	ROS_INFO("avg planning time (msec) %f \n", favg_planning_time  );
////
////	m_frontierpoint_markers = visualization_msgs::MarkerArray() ;
////	m_markerfrontierPub.publish(m_frontierpoint_markers);
////	m_targetgoal_marker = visualization_msgs::Marker() ;
////	m_makergoalPub.publish(m_targetgoal_marker); // for viz
//
//	ROS_INFO("The exploration task is done... publishing -done- msg" );
//	std_msgs::Bool arrived;
//	arrived.data = true;
//	m_donePub.publish( arrived );
//
//	ros::spinOnce();
//}


void PathGenerator::doneCB( const actionlib::SimpleClientGoalState& state )
{
//    ROS_INFO("@DONECB: simpleClientGoalState [%s]", state.toString().c_str());
    if (m_move_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
         // do something as goal was reached
    	ROS_INFO("Touch down  \n");
		{
			const std::unique_lock<mutex> lock(mutex_robot_state) ;
			me_robotstate = ROBOT_STATE::ROBOT_IS_NOT_MOVING ;
		}
		me_prev_exploration_state = SUCCEEDED ;

		std_msgs::Int8 status ;
		status.data = 1 ;
		m_arrivalStatusPub.publish( status ) ;
    }
    else if (m_move_client.getState() == actionlib::SimpleClientGoalState::ABORTED)
    {
        // do something as goal was canceled
    	ROS_ERROR("MoveBase() has failed to reach the goal. The target goal point has aboarted ... \n");
		{
			const std::unique_lock<mutex> lock(mutex_robot_state) ;
			me_robotstate = ROBOT_STATE::ROBOT_IS_NOT_MOVING ;
		}
		me_prev_exploration_state = ABORTED ;

		std_msgs::Int8 status ;
		status.data = 0 ;
		m_arrivalStatusPub.publish( status ) ;
    }
    else
    {
    	ROS_ERROR("doneCB() received an unknown state %d \n", m_move_client.getState());
    	exit(-1);
    }
}

void PathGenerator::moveRobotCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg )
{
// call actionlib
// robot is ready to move
	ROS_INFO("@moveRobotCallback Robot is < %s > \n ",  robot_state[me_robotstate+1] );

	if( me_robotstate >= ROBOT_STATE::FORCE_TO_STOP   )
		return;

	geometry_msgs::PoseWithCovarianceStamped goalpose = *msg ;
	{
		const std::unique_lock<mutex> lock(mutex_robot_state) ;
		me_robotstate = ROBOT_STATE::ROBOT_IS_MOVING ;
	}

	ROS_INFO("@moveRobotCallback received a plan\n");

	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = mstr_worldframe_id; //m_baseFrameId ;
	goal.target_pose.header.stamp = ros::Time::now() ;

//	geometry_msgs::PoseWithCovarianceStamped goalpose = // m_pathplan.poses.back() ;

	goal.target_pose.pose.position.x = goalpose.pose.pose.position.x ;
	goal.target_pose.pose.position.y = goalpose.pose.pose.position.y ;
	goal.target_pose.pose.orientation.w = goalpose.pose.pose.orientation.w ;

	ROS_INFO("curr robot pose is @              <%f %f> \n", m_rpos_world.pose.position.x, m_rpos_world.pose.position.y );
	ROS_INFO("new destination target is set to  <%f %f> \n", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y );

	// publish goal to Rviz
	//m_VizHelper.publishGoalPointMarker( goalpose.pose );

// inspect the path
////////////////////////////////////////////////////////////////////////////////////////////
ROS_INFO("+++++++++++++++++++++++++ @moveRobotCallback, sending a goal +++++++++++++++++++++++++++++++++++++\n");

// publish departure flag
	std_msgs::Bool bflag_true ;
	bflag_true.data = true ;
	m_departureFlagPub.publish(bflag_true) ;
	m_move_client.sendGoal(goal, boost::bind(&PathGenerator::doneCB, this, _1), SimpleMoveBaseClient::SimpleActiveCallback() ) ;
ROS_INFO("+++++++++++++++++++++++++ @moveRobotCallback, a goal is sent +++++++++++++++++++++++++++++++++++++\n");
	m_move_client.waitForResult();
ROS_INFO("+++++++++++++++++++++++++ @moveRobotCallback, Nav to goal is completed   +++++++++++++++++++++++++\n");

	std_msgs::Bool bflag_false ;
	bflag_false.data = false ;
	m_departureFlagPub.publish(bflag_false) ;
ROS_INFO("sent FALSE depart msg from path_generator \n");
// publish result flag

}

void PathGenerator::robotVelCallBack( const geometry_msgs::Twist::ConstPtr& msg )
{
	m_robotvel = *msg ;
}

int PathGenerator::sampleNextGoal( const cv::Mat& costmap_img, const cv::Point& rpos_gm, cv::Point& targetgoal_gm  )
{
// select random point based on EUC distance
	int nwidth = costmap_img.cols ;
	int nheight = costmap_img.rows ;
	int nrx = rpos_gm.x ;
	int nry = rpos_gm.y ;
	vector<int> vecandlist ;
	for (int nrow =0 ; nrow < nheight; nrow++)
	{
		for( int ncol=0; ncol < nwidth; ncol++)
		{
			int dist_sq = ( nrx - ncol ) * ( nrx - ncol ) + (nry - nrow ) * (nry - nrow ) ;
			double fdist = std::sqrt( static_cast<double>(dist_sq) ) ;
			int nidx = nrow * nwidth + ncol ;
			uint32_t ucost = costmap_img.data[ nidx ] ;
			if( fdist > mf_min_targetdist_meter && ucost < (uint8_t)mn_obstacle_cost_thr )
			{
				vecandlist.push_back(nidx);
			}
		}
	}

	if( vecandlist.size() == 0)
	{
		ROS_WARN("Cannot sample any target pt from the current robot and map config \n");
		return 0;
	}

	int randidx = std::rand() % ( vecandlist.size() -1 ) ;

	ROS_ASSERT( randidx < vecandlist.size() ) ;
//	ROS_INFO("randidx %d   size  %d \n", randidx, vecandlist.size());
	int nsampleidx = vecandlist[randidx] ;

	int nsampled_ty = (int) ( nsampleidx / nwidth ) ;
	int nsampled_tx = nsampleidx % nwidth ;

	targetgoal_gm.x = nsampled_tx ;
	targetgoal_gm.y = nsampled_ty ;

	double fdist2target_sq = 	(double)( nrx - nsampled_tx ) *  (double)( nrx - nsampled_tx ) +
								(double)( nry - nsampled_ty ) *  (double)( nry - nsampled_ty ) ;
	double fdist2target = std::sqrt(  fdist2target_sq  ) ;
	ROS_INFO("Sampled a new goal @(%f %f) of cost < %d > which is < %f > away from the robot \n",
			targetgoal_gm.x, targetgoal_gm.y, costmap_img.data[nsampleidx], fdist2target);

	return 1 ;
}

void PathGenerator::initGlobalmapimgs( const cv::Rect& roi, const nav_msgs::OccupancyGrid& globalcostmap )
{
	mcvu_globalmapimg.setTo(OCC_LABEL::UNKNOWN);
	int cmwidth = roi.width ;
	int cmheight= roi.height;
	int nox = roi.x ;
	int noy = roi.y ;

	for( int ii =0 ; ii < cmheight; ii++)
	{
		for( int jj = 0; jj < cmwidth; jj++)
		{
			int8_t occupancy = m_gridmap.data[ ii * cmwidth + jj ]; // dynamic gridmap size
			int8_t obs_cost  = globalcostmap.data[ ii * cmwidth + jj] ;
			int y_ = (noy + ii) ;
			int x_ = (nox + jj) ;

			if ( occupancy < 0 && obs_cost < 0)
			{
				mcvu_globalmapimg.data[ y_ * mn_globalmap_width + x_ ] = static_cast<uchar>(OCC_LABEL::UNKNOWN) ;
			}
			else if( occupancy >= 0 && occupancy < mn_occupancy_thr && obs_cost < 98) // mp_cost_translation_table[51:98] : 130~252 : possibly circumscribed ~ inscribed
			{
				mcvu_globalmapimg.data[ y_ * mn_globalmap_width + x_ ] = static_cast<uchar>(OCC_LABEL::FREE) ;
			}
			else
			{
				mcvu_globalmapimg.data[ y_ * mn_globalmap_width + x_ ] = static_cast<uchar>(OCC_LABEL::OCCUPIED) ;
			}
		}
	}
//ROS_INFO("mcvu_globalmapimg has been processed %d %d \n", mcvu_globalmapimg.rows, mcvu_globalmapimg.cols );
// process costmap
mcvu_costmapimg.setTo(OCC_LABEL::OCCUPIED);
ROS_ASSERT(cmwidth == globalcostmap.info.width );
ROS_ASSERT(cmheight == globalcostmap.info.height );

	for( int ii =0 ; ii < cmheight; ii++)
	{
		for( int jj = 0; jj < cmwidth; jj++)
		{
			int8_t val  = globalcostmap.data[ ii * cmwidth + jj] ;
//ROS_INFO("val: %d ", val);
			int y_ = (noy + ii) ;
			int x_ = (nox + jj) ;
			mcvu_costmapimg.data[ y_ * mn_globalmap_width + x_ ] = val < 0 ? 255 : mp_cost_translation_table[val];
		}
	}
}

// mapcallback for static mapsize. i.e) map is prebuilt based on SLAM
// We are not doing exploration of UNKNOWN env here
void PathGenerator::mapdataCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) //const octomap_server::mapframedata& msg )
{
	if( mn_tot_nav_time > mn_max_nav_time)
	{
		ROS_INFO("Max data collection time has been reached. Closing this node \n");
		mb_data_collection_is_done = true;
		return;
	}

ROS_INFO("********** \t start mapdata callback routine \t ********** \n");
ros::WallTime	mapCallStartTime = ros::WallTime::now();
mn_tot_nav_time = static_cast<int>(ros::Time::now().sec) - mn_start_nav_time;

	if(m_robotvel.linear.x == 0 && m_robotvel.angular.z == 0 ) // robot is physically stopped
	{
		const std::unique_lock<mutex> lock(mutex_robot_state) ;
		me_robotstate = ROBOT_STATE::ROBOT_IS_NOT_MOVING;
	}

//	nav_msgs::OccupancyGrid globalcostmap;
	m_globalcostmap = *msg ;
	const nav_msgs::OccupancyGrid globalcostmap = m_globalcostmap;

//ROS_INFO("got costmap size of %d %d \n", m_globalcostmap.info.height, m_globalcostmap.info.width );
	generateGridmapFromCostmap();

	float cmresolution=globalcostmap.info.resolution;
	float gmresolution= cmresolution ;
	float cmstartx=globalcostmap.info.origin.position.x;
	float cmstarty=globalcostmap.info.origin.position.y;
	uint32_t cmwidth =globalcostmap.info.width;
	uint32_t cmheight=globalcostmap.info.height;
	std::vector<signed char> cmdata, gmdata;

	cmdata = globalcostmap.data;
	gmdata = m_gridmap.data;

// print map info
//ROS_INFO("map info @ curr callback==>   ox oy width height: %f %f %d %d\n", cmstartx, cmstarty, cmwidth, cmheight);

// Set the cent of the map as the init world coordinate (0,0). This point could be the init robot position (x, y) w/ in real world experiment.
// cv::Point Offset = compute_rpose_wrt_maporig() ;
	m_rpos_world = GetCurrRobotPose( );
	cv::Point world_orig_in_gm = world2gridmap( cv::Point2f( 0.f, 0.f ) ) ; // (0,0) w.r.t the gmap orig (left, top). That is, (0,0)w --> (x,y)gm. e.g.) (288,0) in WG

/*******************************************************************************************************************************************************************
// coord convention
 * gridmap copied to a ROI of globalmap, then globalmap is downsampled to globalmap_ds
 * Thus, gridmap_ds \in globalmap_ds and active_map \in globalmap_ds.
 * active map (512 x 512) is downsampled!!
 * globalmap_ds (2048x2048) is a downsampled globalmap (4096x4096)!!
 * Thus, am, gmds, and glob_ds are defined in the same scale.
 *
 * Unify their scale first before bring them into the same coord frame.
	Globalmap (.),  Gridmap (+)
	.............................................................
	.............................................................
	.............................++++++++++++++++++++++++........
	.............................++++++++++++++++++++++++........
	.............................+++++++++++++++++++++)++........
	.............................++++++++++++++++++++++++........
	........................(gmox,gmoy)++++++++++++++++++........
	.............................................................
	........................{wox(gcx), woy(gcy)}.................
	.............................................................
	.............................................................
	.............................................................
	.............................................................
	.............................................................
	.............................................................
	(gox, goy)...................................................

	global map orig: (gox, goy)
	(wox, woy) is on top of the globamap cent:  (gcx, gcy)
	g : globalmap, l : localmap, gm: gridmap
	gridmap belongs to globalmap
*******************************************************************************************************************************************************************/

	int nglobalmap_xc_ = mn_globalmap_width / 2 - world_orig_in_gm.x; // offset free global cent x (equivalent to globalmap_width / 2 or mn_globalmap_centx if the world is at (0,0) gm )
	int nglobalmap_yc_ = mn_globalmap_height/ 2 - world_orig_in_gm.y; // offset free global cent y
	cv::Point rpos_gm = world2gridmap( cv::Point2f( m_rpos_world.pose.position.x, m_rpos_world.pose.position.y ) ) ;
	cv::Point rpos_gm_in_gcm = cv::Point( nglobalmap_xc_ + rpos_gm.x, nglobalmap_yc_ + rpos_gm.y ) ; // rpos w.r.t (offset free) globalmap's centroid
	int nox_roi = static_cast<int>( nglobalmap_xc_ ) ;
	int noy_roi = static_cast<int>( nglobalmap_yc_ ) ;
	cv::Rect roi_active = cv::Rect( nox_roi, noy_roi, cmwidth, cmheight ); //cmwidth, cmheight );

	initGlobalmapimgs( roi_active, globalcostmap );

	if( mn_numpyrdownsample > 0)
	{
		// be careful here... using pyrDown() interpolates occ and free, making the boarder area (0 and 127) to be 127/2 !!
		// 127 represents an occupied cell !!!
		for(int iter=0; iter < mn_numpyrdownsample; iter++ )
		{
			int nrows = mcvu_costmapimg.rows; // % 2 == 0 ? img.rows : img.rows + 1 ;
			int ncols = mcvu_costmapimg.cols; // % 2 == 0 ? img.cols : img.cols + 1 ;
			//ROS_INFO("sizes orig: %d %d ds: %d %d \n", img_.rows, img_.cols, nrows/2, ncols/2 );
			pyrDown(mcvu_costmapimg, mcvu_costmapimg_ds, cv::Size( ncols/2, nrows/2 ) );
			//clusterToThreeLabels( mcvu_globalmapimg_ds );
			roi_active = cv::Rect(roi_active.x / mn_scale, roi_active.y / mn_scale, roi_active.width / mn_scale, roi_active.y / mn_scale) ;
		}
	}
	else
	{
		mcvu_costmapimg_ds = mcvu_costmapimg.clone();
	}

// ROS_INFO("offset free global map's cent: %d %d \n", mn_globalmap_xc_, mn_globalmap_yc_) ;
// ROS_INFO("active roi (costmap) size: %d %d %d %d \n", nox_roi, noy_roi, cmwidth, cmheight );

	cv::Mat cm_active = mcvu_costmapimg_ds( roi_active ); // partial (on reconstructing map) refers to local map
	cv::Mat costmap_img = cm_active.clone() ;

// ROS_INFO("Locate Next Goal Pos in costmap : %d %d \n", costmap_img.rows, costmap_img.cols);
//////////////////////////////////////////////////////////////////////////////////////
// 						Identify Next Goal Pos in Curr Gridmap						//
//////////////////////////////////////////////////////////////////////////////////////
	cv::Point target_gm ;
	if( sampleNextGoal( costmap_img, rpos_gm, target_gm ) )
	{
		{
			const std::unique_lock<mutex> lock(mutex_robot_state) ;
			me_robotstate = ROBOT_STATE::ROBOT_IS_READY_TO_MOVE;
		}
		///////////////////////////////////////////////////////////////////////////////////////////////////////
		// Publishing Goal Pose... moveRobotCallback() takes the control until it confirms the robot's arrival
		///////////////////////////////////////////////////////////////////////////////////////////////////////
		m_prev_robot_pose = m_rpos_world;
		// convert gm to world coord
		cv::Point2f target_world = gridmap2world( target_gm );
		geometry_msgs::PoseWithCovarianceStamped targetGoal = StampedPoseCovfromSE2( target_world.x, target_world.y, 0.f );
		m_currentgoalPub.publish(targetGoal) ;	// publishing this msg instantiates moveRobotCallback()...
	}
	else
	{
		ROS_WARN("For some reason, sampleNextGoal() has failed to find a next goal \n");
	}

//cv::imwrite("/media/results/navdata_collector/globalmap.png", mcvu_globalmapimg);
//cv::cvtColor(mcvu_costmapimg_ds, mcvu_costmapimg_ds, CV_GRAY2RGB);
//cv::circle(mcvu_costmapimg_ds, rpos_gm_in_gcm, 8, CV_RGB(0,255,255), -1, CV_8U, 0);
//cv::Point target_gcm = cv::Point(target_gm.x + nglobalmap_xc_ , target_gm.y +  nglobalmap_yc_ ) ;
//cv::circle(mcvu_costmapimg_ds, target_gcm, 8, CV_RGB(255,0,255), -1, CV_8U, 0);
//cv::rectangle(mcvu_costmapimg_ds, cv::Point(nglobalmap_xc_, nglobalmap_yc_), cv::Point(nglobalmap_xc_ + cmwidth, nglobalmap_yc_+cmheight), CV_RGB(0,0,255), 5, CV_8U, 0);
//cv::circle(mcvu_costmapimg_ds, cv::Point(nglobalmap_xc_, nglobalmap_yc_), 12, CV_RGB(255,0,0), 1, CV_8U, 0);
//cv::circle(mcvu_costmapimg_ds, cv::Point(mn_globalmap_width / 2, mn_globalmap_height / 2), 8, CV_RGB(0,255,0), -1, CV_8U, 0);
//cv::imwrite("/media/results/navdata_collector/mcvu_costmapimg_ds.png", mcvu_costmapimg_ds);
//cv::cvtColor(costmap_img, costmap_img, CV_GRAY2RGB);
//cv::circle(costmap_img, rpos_gm, 8, CV_RGB(0,255,255), -1, CV_8U, 0);
//cv::circle(costmap_img, target_gm, 8, CV_RGB(255,255,0),  -1, CV_8U, 0);
//cv::imwrite("/media/results/navdata_collector/gridmap_img.png", costmap_img);

	ros::WallTime mapCallEndTime = ros::WallTime::now();
	double mapcallback_time = (mapCallEndTime - mapCallStartTime).toNSec() * 1e-6;
	ROS_INFO("\n "
			 " ********************************************************************************************* \n "
			 "     mapDataCallback exec time (ms): %f \n "
			 " ********************************************************************************************* \n "
			, mapcallback_time);

	ROS_INFO("\n********** End of mapdata callback routine \t ********** \n");
	ROS_INFO("Tot nav time spent %d (s)\n", mn_tot_nav_time);

		//saveDNNData( img_frontiers_offset, start, best_goal, best_plan, ROI_OFFSET, roi ) ;

	// for timing
	mf_totalcallbacktime_msec += mapcallback_time ;

	mn_mapcallcnt++;

}


}

