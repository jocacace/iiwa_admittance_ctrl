	#include "stiffness_calculation.h"

void load_param( double & p, double def, string name ) {
	ros::NodeHandle n_param("~");
	if( !n_param.getParam( name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}

void load_param( string & p, string def, string name ) {
	ros::NodeHandle n_param("~");
	if( !n_param.getParam( name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}

void load_param( int & p, int def, string name ) {
	ros::NodeHandle n_param("~");
	if( !n_param.getParam( name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}

void load_param( bool & p, bool def, string name ) {
	ros::NodeHandle n_param("~");
	if( !n_param.getParam( name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}


visualization_msgs::Marker create_marker(Vector<3> pi, Vector<3> pf, float rgba[4] ) {


	visualization_msgs::Marker arrow;
	arrow.header.frame_id =  "/world";
	arrow.ns = "eef_path";
	arrow.action = visualization_msgs::Marker::ADD;
	

	arrow.id = 2;

	arrow.type = visualization_msgs::Marker::ARROW;

	arrow.scale.x = 0.01;
	arrow.scale.y = 0.01;
	arrow.scale.z = 0.05;
	
	arrow.color.r = rgba[0];
	arrow.color.g = rgba[1];
	arrow.color.b = rgba[2];
	arrow.color.a = rgba[3];

	geometry_msgs::Point p;
	p.x = pi[0];
	p.y = pi[1];
	p.z = pi[2];
	arrow.points.push_back(p);
	
	p.x = pf[0];
	p.y = pf[1];
	p.z = pf[2];
	arrow.points.push_back(p);
	
	return arrow;
	 

}


variable_stiffness::variable_stiffness() {


	_eef_p_direction_sub 				= _nh.subscribe( "/iiwa/eef_p_direction", 0, &variable_stiffness::pdir_cb, this );
	_eef_h_direction_sub 				= _nh.subscribe( "/iiwa/eef_h_direction", 0, &variable_stiffness::hdir_cb, this );
	_x_sub 											= _nh.subscribe( "/iiwa/eef_x", 0, &variable_stiffness::eef_x_cb, this );
	_dx_pub 										= _nh.subscribe( "/iiwa/eef_dx", 0, &variable_stiffness::eef_dx_cb, this );
	_cartesian_position_sub			= _nh.subscribe( "/iiwa/state/CartesianPose", 0, &variable_stiffness::cartesian_position_cb, this);
	_trajectory_size_sub				= _nh.subscribe ("/iiwa/trajectory_size", 0, &variable_stiffness::trajectory_size_cb, this);
	_trajectory_deviation_sub 	= _nh.subscribe ("/iiwa/trajectory_deviation", 0, &variable_stiffness::trajectory_deviation_cb, this);
	_mis_dp_sub 								= _nh.subscribe ("/iiwa/state/CartesianVel", 0, &variable_stiffness::mis_dp_cb, this);
	_c_sub 											= _nh.subscribe ("/nn_classification/class", 0, &variable_stiffness::c_cb, this);
	_xd_sub											= _nh.subscribe ("/iiwa/xd", 0, &variable_stiffness::xd_cb, this );
	_c_start_sub 								= _nh.subscribe ("/nn_classification/start_class", 0, &variable_stiffness::c_start_cb, this);
	_trajectory_streaming_sub 	= _nh.subscribe ("/iiwa/streaming_trajectory", 0, &variable_stiffness::streaming_trajectory_cb, this);

	_listen_f_pub								= _nh.advertise<std_msgs::Bool>("/iiwa/listen_force", 0);
	_halt_robot_pub							= _nh.advertise<std_msgs::Bool>("/iiwa/halt", 0 );
	_k_pub 											= _nh.advertise<std_msgs::Float64>("/iiwa/stiffness", 0);
	_replan_pub 								= _nh.advertise<std_msgs::Bool>("/iiwa/replan", 0);
	_marker_hdir_pub    				= _nh.advertise<visualization_msgs::Marker>("/iiwa/hdir", 0);
	_marker_pdir_pub    				= _nh.advertise<visualization_msgs::Marker>("/iiwa/pdir", 0);
	_wp_pub											= _nh.advertise<geometry_msgs::Pose>("/iiwa/next_wp", 0);
	_slow_wp_pub 								= _nh.advertise<geometry_msgs::Pose>("/iiwa/slow_wp", 0);
	_marker_sphere_pub					= _nh.advertise<visualization_msgs::Marker>("/iiwa/workspace", 0);
	_cpoint_pub									= _nh.advertise<visualization_msgs::Marker>("/iiwa/c_point", 0);
	_strat_traj_pub 						= _nh.advertise<std_msgs::Bool>("/iiwa/start_traj", 0 );
	_attraction_point_pub 			= _nh.advertise<geometry_msgs::Point>("/iiwa/attraction_point", 0 );		

	_hdir = _pdir = Zeros;
	Vector<3> p0;		
	
	/*
	p0[0] = 0.48;
	p0[1] = 0.11;
	p0[2] = 0.45;
	_wp_stack.push_back(p0); //LR

	
	p0[0] = 0.485;
	p0[1] = -0.11;
	p0[2] = 0.45;
	_wp_stack.push_back(p0); //UR
	*/
	
	p0[0] = 0.30;
	p0[1] = -0.28;
	p0[2] = 0.6;
	_wp_stack.push_back(p0); //LR

	
	p0[0] = 0.6;
	p0[1] = -0.28;
	p0[2] = 0.6;
	_wp_stack.push_back(p0); //UR
	
	p0[0] = 0.29;
	p0[1] = 0.28;
	p0[2] = 0.6;
	_wp_stack.push_back(p0); //LL

	p0[0] = 0.6;
	p0[1] = 0.28;
	p0[2] = 0.6	;
	_wp_stack.push_back(p0); //UL
	
	_wp_index = 0;

	load_param( _stream_trajectory, false, "stream_trajectory");
	load_param( _ws_size, 0.5, "ws_size" );
	load_param( _k_param, 150, "k");
	load_param( _check_sspace, 0.1, "check_sspace"); //in cms

	_starting_point = Zeros;
	_classification = "C_TYPE_NO_CONTACT";
	_start_classification = "C_TYPE_NO_CONTACT";
	_print_ws = false;
	_wp_passed = false;
	_is_streaming_traj = false;
}

void variable_stiffness::c_cb( std_msgs::String c ) {
	_classification = c.data;
}

void variable_stiffness::mis_dp_cb( geometry_msgs::Twist mis_dp_) {
	_mis_dp = makeVector( mis_dp_.linear.x, mis_dp_.linear.y, mis_dp_.linear.x );
}

void variable_stiffness::trajectory_size_cb( std_msgs::Int32 tsize_ ) {
	_trajectory_size = tsize_.data;
}

void variable_stiffness::trajectory_deviation_cb( std_msgs::Float64 deviation_d ) {
	_t_deviation = deviation_d.data;
}

void variable_stiffness::xd_cb( geometry_msgs::Pose xd ) {
	_xd.slice<0,3>() = makeVector( xd.position.x, xd.position.y, xd.position.z );
}


void variable_stiffness::c_start_cb( std_msgs::String c ) {
	_start_classification = c.data;
}

void variable_stiffness::streaming_trajectory_cb( std_msgs::Bool streaming ) {
	_is_streaming_traj = streaming.data;

}

void variable_stiffness::print_ws() {

	ros::Rate r(2);
	visualization_msgs::Marker marker;
	
	while( ros::ok() ) {

		marker.header.frame_id = "world";
		marker.header.stamp = ros::Time();
		marker.ns = "workspace";
		marker.id = 3;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = _xd[0];
		marker.pose.position.y = _xd[1];
		marker.pose.position.z = _xd[2];
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = _ws_size;
		marker.scale.y = _ws_size;
		marker.scale.z = _ws_size;
		marker.color.a = 0.3; // Don't forget to set the alpha!
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
		marker.lifetime = ros::Duration(0.5);


		if(_print_ws)
			_marker_sphere_pub.publish( marker );

		r.sleep();
	}
}

void variable_stiffness::print_cpoint(Vector<3> p) {
	
	visualization_msgs::Marker point;
	point.header.frame_id = "/world";
	point.ns = "eef_path";
	point.action = visualization_msgs::Marker::ADD;	
	point.pose.orientation.w = 1.0;

	point.id = 10;
	
	point.type = visualization_msgs::Marker::POINTS;

	point.scale.x = 0.02; //_check_sspace * 2.0;
	point.scale.y = 0.02; //_check_sspace * 2.0;
	point.scale.z = 0.02; //_check_sspace * 2.0;
	point.pose.orientation.w = 1.0;
	point.color.g = 1.0f;
	point.color.a = 1.0;
	
	geometry_msgs::Point pt;
	pt.x = p[0];
	pt.y = p[1];
	pt.z = p[2];
	point.points.push_back(pt);
	
	
	_cpoint_pub.publish( point );

	
}

void variable_stiffness::find_closest_point() {
	
	cout << "find_closest_point!!!"<< endl;

	ros::Rate r(100);

	Vector<3> direction;
	direction = ( _wp_stack[_wp_index] - _starting_point);
	direction /= norm( direction );
	Vector<3> cpoint = Zeros;
	geometry_msgs::Point attraction_point;
	
	while(ros::ok()) {
		
		bool maj = false;
		Vector<3> cpoint = _starting_point;
		cout << "while" << endl;		
		if( _is_streaming_traj ) {

			cout << "_is_streaming_traj" << endl;
			float dist = 0;
			float pdist = 1000;
			int step = 0;
			Vector<3> cpoint = _starting_point;
			
			while( !maj && ( norm( cpoint - _wp_stack[_wp_index]) >= _check_sspace) ) {				
				
				dist = norm( cpoint - _eef_pose.slice<0,3>() );				
							
				if( pdist > dist ) {
					maj = true;
					continue;
				} 
				else {			
					cpoint += direction*_check_sspace*(step++);				
				}
				pdist = dist;	
			
			}
			if( maj ) {
				cout << norm( cpoint - _eef_pose.slice<0,3>()) << endl;
				

				print_cpoint( cpoint);
				attraction_point.x = cpoint[0];	
				attraction_point.y = cpoint[1];
				attraction_point.z = cpoint[2];
				
				//if( _classification == "C_TYPE_CONTACT_DEVIATION" ) {
					_attraction_point_pub.publish( attraction_point );					
				//}

			}
		}
		r.sleep();
	}
}


void variable_stiffness::manage_wp( Vector<3> p ) {

	bool reached = false;

	ros::Rate r(10);

	//---Start new navigation
	geometry_msgs::Pose wp;
	wp.position.x = p[0];
	wp.position.y = p[1];
	wp.position.z = p[2];
	_wp_pub.publish( wp );
	//---

	std_msgs::Bool streaming_start;
	streaming_start.data = false;
	/* -- -RE-ADD this!!
	sleep(1);
	//---Check if need to start the traj (considering the human touch	)
	while( !_is_streaming_traj ) {
		if( _start_classification == "C_TYPE_CONTACT_CONCORDE" ) {
			_strat_traj_pub.publish( streaming_start );
		}
		r.sleep();
	}
	*/


	std_msgs::Float64 d;
	float d_prec;
	float dist_p0;
	float dist_p1;
	float verse = 100; 
	bool first = true;
	std_msgs::Bool listen_f;	
	std_msgs::Bool halt_robot;
	halt_robot.data = false;
	listen_f.data = true;
	float delta = 0.01;
	int vel_red_seq = 0;
	
	boost::thread find_closest_point_t( &variable_stiffness::find_closest_point, this);
	_starting_point = _eef_pose.slice<0,3>();

	while( !reached ) {	
		
		float d1 = norm( p - _eef_pose.slice<0,3>() );
		
		if( first ) {
			//dist_p1 = dist_p0;
			dist_p1 = d1;
			d_prec = d1;
		}
		verse = ( dist_p1 - dist_p0  );
		
		//cout << "d1: " << d1 << endl;
		//Behaviour decisioner
		if(  ( d1 < 0.03 ) )	   {					
			reached = true;
			_print_ws = false;
		} // Wp reached, session complete
		else if( ( norm( _mis_dp ) > 0.02 ) && (d_prec < d1) && ( d1 < 0.05 )) { //considera il verso di movimento?
			halt_robot.data = true;
			_halt_robot_pub.publish( halt_robot );
			reached = true;
			_wp_passed = true;
			_print_ws = false;
		} // Wp passed, session complete
		else {
			/*
			if( _classification == "C_TYPE_CONTACT_CONCORDE" ) {
				d.data = 0.0;		
				_print_ws = false;
				listen_f.data = true;		
				vel_red_seq = 0;		

				//Start Traj!
			}
			else if( _classification == "C_TYPE_CONTACT_OPPOSITE" ) {
				
				d.data = 0.0;		
				_print_ws = false;
				listen_f.data = false;				
				
				if( vel_red_seq++ == 3 ) {
					_slow_wp_pub.publish( wp );
				}
			}
			else */if( _classification == "C_TYPE_CONTACT_DEVIATION" ) {
				_print_ws = true;
				d.data = _k_param;		
				/*
				if( (  (_t_deviation - delta >  _ws_size )  ) || ( ( _t_deviation + delta ) > (_ws_size ) ) ) {
					_wp_pub.publish( wp );	
					vel_red_seq = 0;
					//listen_f.data = false;	
				} // Out from WS! Replan!!
				else {
					listen_f.data = true;				
					vel_red_seq = 0;
				}
				*/
			}
			/**/
			/*
			else if( _trajectory_size  == 0 ) {
				_wp_pub.publish( wp );
				vel_red_seq = 0;			
			} //Trajectory finished without reach the WP		
			*/
			/*
			else {
				listen_f.data = true;				
				_print_ws = false;
			}
			*/
		}
		
		dist_p1 = dist_p0;	
		d_prec = d1;
		
		_k_pub.publish( d );	

		_listen_f_pub.publish( listen_f );	
		first = false;
		
		r.sleep();
	}
}

void variable_stiffness::stream_trajectory() {

	ros::Rate r(10);
	sleep(3);

	//start with wp[0]
	while(ros::ok()) {

		cout << "_wp_index: " << _wp_index << endl;

		//if ( !_wp_passed ) {
			manage_wp( _wp_stack[_wp_index] );
			
			//sleep(1);	

			if( _wp_index++ == _wp_stack.size()-1 ) 
				_wp_index = 0;
		//}
		r.sleep();
	}
}


void variable_stiffness::cartesian_position_cb( geometry_msgs::PoseStamped eef_c_pos ) {
	_eef_pose.slice<0,3>() = makeVector( eef_c_pos.pose.position.x, eef_c_pos.pose.position.y, eef_c_pos.pose.position.z );
}



void variable_stiffness::pdir_cb( geometry_msgs::Vector3 pdir ) {
	_pdir = makeVector( pdir.x, pdir.y, pdir.z );

	float rgba[4];
	rgba[0] = 1;
	rgba[1] = 0;
	rgba[2] = 0;
	rgba[3] = 1;
	
	Vector<3> pdir_th = Zeros;
	pdir_th = _pdir * 0.2;
	if ( norm ( _pdir ) > 0.0 ) {

		//publish pdir marker
		visualization_msgs::Marker parrow = create_marker( _x_eef, pdir_th + _x_eef, rgba);
		_marker_pdir_pub.publish( parrow );
	}
	else {
		visualization_msgs::Marker parrow = create_marker( _pdir, _pdir, rgba );
		_marker_pdir_pub.publish( parrow );	
	}
}

void variable_stiffness::hdir_cb( geometry_msgs::Vector3 hdir ) {

	_hdir = makeVector( hdir.x, hdir.y, hdir.z );	
	
	float rgba[4];
	rgba[0] = 0;
	rgba[1] = 1;
	rgba[2] = 0;
	rgba[3] = 1;	

	Vector<3> hdir_th = Zeros;
	hdir_th = _hdir * 0.2;
	if ( norm ( _hdir ) > 0.0 ) {

		//publish pdir marker
		visualization_msgs::Marker harrow = create_marker( _x_eef, hdir_th + _x_eef, rgba);
		_marker_hdir_pub.publish( harrow );
	}
	else {
		visualization_msgs::Marker harrow = create_marker( _hdir, _hdir, rgba );
		_marker_hdir_pub.publish( harrow );	
	}

}


void variable_stiffness::eef_x_cb( geometry_msgs::Pose pose ) {
	_x_eef = makeVector( pose.position.x, pose.position.y, pose.position.z );
}

void variable_stiffness::eef_dx_cb( geometry_msgs::Twist dpose ) {
	_dx_eef = makeVector( dpose.linear.x, dpose.linear.y, dpose.linear.z );
}



void variable_stiffness::calc_direction ( ) {

	ros::Rate r(200);
	float cos_a = 0.0;
	float a = 0;
	while(ros::ok()) {

		if( norm( _pdir) > 0 && norm(_hdir ) > 0 ) {			
			cos_a = (_pdir * _hdir ) / (norm(_pdir) * norm(_hdir ));
			a = acos( cos_a );
		}
		else {
			cos_a = 0;
			a = 0.0;
		}
		//cout << "cosa: " << a*180.0/M_PI << endl;


	//	cout << "h: "<< _hdir << endl;
	//	cout << "p: " << _pdir << endl;

		r.sleep();
	}


}

void variable_stiffness::run () {

	if( _stream_trajectory ) {
		boost::thread stream_trajectory_t( &variable_stiffness::stream_trajectory, this );

	}

	//boost::thread calc_direction_t( &variable_stiffness::calc_direction, this );
	boost::thread print_ws_t( &variable_stiffness::print_ws, this );
	ros::spin();

}



int main( int argc, char** argv ) {

	ros::init( argc, argv, "stiffness_calculation");
	variable_stiffness va;
	va.run();

	return 0;
}