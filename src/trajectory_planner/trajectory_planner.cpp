#include "../admittance_ctrl.h"

using namespace std;
using namespace TooN;



void iiwa_admittance_ctrl::create_marker(Vector<3> pi, Vector<3> pf ) {



	visualization_msgs::Marker line, point;
	line.header.frame_id = point.header.frame_id = "/world";
	line.ns = point.ns = "eef_path";
	point.action = visualization_msgs::Marker::ADD;
	line.action = visualization_msgs::Marker::ADD;
	line.pose.orientation.w = point.pose.orientation.w = 1.0;

	point.id = 0;
	line.id = 1;

	point.type = visualization_msgs::Marker::POINTS;
	line.type = visualization_msgs::Marker::LINE_STRIP;

	point.scale.x = 0.05;
	point.scale.y = 0.05;
	point.scale.z = 0.05;
	point.pose.orientation.w = 1.0;
	line.scale.x = 0.01;
	point.color.b = 1.0f;
	point.color.a = 1.0;
	line.color.r = 1.0;
	line.color.a = 1.0; 


	geometry_msgs::Point p;
	p.x = pi[0];
	p.y = pi[1];
	p.z = pi[2];
	line.points.push_back(p);
	point.points.push_back(p);

	p.x = pf[0];
	p.y = pf[1];
	p.z = pf[2];
	line.points.push_back(p);
	point.points.push_back(p);
	
	_marker_line_pub.publish(point);
	_marker_line_pub.publish(line);
	
	 

}

void iiwa_admittance_ctrl::monitor_wp_pd(Matrix<4> pf) {

	
	ros::Rate r(_rate );	//Streaming Rate

	int i=0;
	_halt = false;

	Vector<3> ppi;							//Initial position
	Vector<3> ppf;							//Position to reach
	Vector<3> target_point;			//Final point of the trajectory

	//---Wait start trajectory
	ROS_INFO("Wait to start a new trajectory");
	while( _wait_to_start ) {
		ppi = _xc.slice<0,3>();
		ppf = makeVector(pf[0][3], pf[1][3], pf[2][3]);
		_dxd_start.slice<0,3>() = (ppf - ppi);			
		_streaming_state.data = false;

		r.sleep();
	}
	ROS_INFO("Trajectory start!!");
	//---

	ppi = _xc.slice<0,3>();
	ppf = makeVector(pf[0][3], pf[1][3], pf[2][3]);	
	target_point = makeVector(pf[0][3], pf[1][3], pf[2][3]);	//at start ppf =  target point
	float ep = norm( target_point.slice<0,2>() - ppi.slice<0,2>() );										//Ep: distance from target point
	
	Vector<3> v, vp;
	Vector<3> va, vap;
	Vector<3> vt, vtp;
	float factor_t;
	float factor_a;
	
	_pv = (target_point - ppi);	//Motion direction to reach target point from initial point
	
	vt = vtp = Zeros;

	while( ep > 0.04 ) {
		//---TODO: choose here the behaviour!!!
		
		ppi = _xc.slice<0,3>();

		if( _classification == "C_TYPE_CONTACT_DEVIATION_C" && _attraction_point_ready ){		
			ppf = _apoint; 
			va = _kp_a*(ppf - ppi) + _kd_a*((va - vap) / _Tc );
			factor_a = 1.5;
			factor_t = 1.5;
		}
		else if ( _classification == "C_TYPE_CONTACT_DEVIATION_O" && _attraction_point_ready ){			
			ppf = _apoint; 
			va = _kp_a*(ppf - ppi) + _kd_a*((va - vap) / _Tc );
			factor_a = 3.0;
			factor_t = 0.0;
		}
		else if(  _classification == "C_TYPE_CONTACT_OPPOSITE" ) {
			factor_a = 0.0;
			factor_t = 0.0;
		} //opposite
		else if ( _classification == "C_TYPE_CONTACT_CONCORDE" ) {
			va = Zeros;
			factor_a = 0.0;
			factor_t = 1.0;
		}
		else {
			va = Zeros;
			factor_a = 0.0;
			factor_t = 0.8;	
		}
		
		_attraction_point_ready = false;		

		vap = va;
		
		ep = norm( target_point.slice<0,2>() - ppi.slice<0,2>() );
		vt = _kp_wp*(target_point - ppi) + _kd_wp*((vt - vtp) / _Tc );
		vtp = vt;

		v = factor_a*va + factor_t*vt;	
		if ( norm(v) > _max_vel ) 
			v *= _max_vel;

		_dxd.slice<0,3>() = v;
		_xd.slice<0,3>() += v*_Tc;

		_trajectory_streaming = true;
		_streaming_state.data = true;		

		r.sleep();		
	}
	_dxd = Zeros; // ! Be careful to this

	_trajectory_streaming = false;
	_streaming_state.data = false;		

}

void iiwa_admittance_ctrl::monitor_wp_traj(vector< POSE > x, vector< TWIST > dx, vector< DTWIST > ddx ) {
	//_wait_to_start = true;

	std_msgs::Int32 t_size_d;
	ros::Rate r(_rate );

	int i=0;
	_halt = false;


	Vector<3> ppi;
	Vector<3> ppf;

	ROS_INFO("Wait to start a new trajectory");
	while( _wait_to_start ) {

		ppi = makeVector(x[i].p[0],  x[i].p[1],  x[i].p[2]);
		ppf = makeVector(x[x.size()-1].p[0], x[x.size()-1].p[1], x[x.size()-1].p[2]);
		_dxd_start.slice<0,3>() = (ppf - ppi);
		
		_streaming_state.data = false;

		r.sleep();
	}
	ROS_INFO("Trajectory start!!");

	ofstream planned_traj;
	planned_traj.open("/tmp/planned_traj.txt");
		
	while( i<x.size()-1 && !_new_wp  && !_halt && !_new_slow_wp ) {

		_xd = makeVector( x[i].p[0],  x[i].p[1],  x[i].p[2],  x[i].q[0], x[i].q[1], x[i].q[2], x[i].q[3] );	
		_dxd = makeVector( dx[i].dp[0],  dx[i].dp[1],  dx[i].dp[2],  dx[i].w[0], dx[i].w[1], dx[i].w[2] );
		_ddxd = makeVector( ddx[i].ddp[0],  ddx[i].ddp[1],  ddx[i].ddp[2],  ddx[i].dw[0], ddx[i].dw[1], ddx[i].dw[2] ); 
		
		_t_size = x.size() - i;
		t_size_d.data = _t_size;
		_trajectory_size_pub.publish( t_size_d );
		
		//cout << "Direction in final: " << (ppf - ppi) << endl;
		i++;
		_trajectory_streaming = true;
		_streaming_state.data = true;		

		//cout << "orientaiton: " << _xd.slice<2,4>() << endl;
		planned_traj << _xd << endl;

		r.sleep();
	}

	planned_traj.close();

	_dxd = Zeros; // ! Be careful to this

	_trajectory_streaming = false;
	_streaming_state.data = false;		

}

void iiwa_admittance_ctrl::trajectory_planner_client() {
	
	_motion_planner = new motion_planner();
	_motion_planner->Init( _Tc );	
	_motion_planner->set_params( _vp_max, _ap_max, _jp_max, _sp_max,  _vo_max, _ao_max, _jo_max, _so_max );
	

	Matrix<4> Hi = Zeros;
	Matrix<4> Hf = Zeros;
	vector< Matrix<4> > Hfs;
	vector< POSE > x;
	vector< TWIST > dx;	
	vector< DTWIST > ddx;	
	
	Vector<3, float> rpy;
	helper::eulerFromQuat( makeVector( _mis_o[0], _mis_o[1], _mis_o[2], _mis_o[3]), rpy[0], rpy[1], rpy[2]  );
	cout << rpy << endl;
	rpy[2] += M_PI / 2.0;
	Matrix<3> R_temp = helper::rpyToMat( rpy[0], rpy[1], rpy[2] );
	Vector<4> final_quat = helper::MatToQuat( R_temp );

	std_msgs::Int32 t_size_d;
	

	while( ros::ok() ) {
		if ( _new_wp || _new_slow_wp) {

			cout  << _new_wp << " | | " << _new_slow_wp << endl;
			_trajectory_streaming = false;
			_streaming_state.data = false;
		
			if( _trajectory_streaming ) {
				_th_force = true;
			} //in replanning we stop forces?

			_new_wp = false;
			_new_slow_wp = false;
			if( _to_wait_to_start )
				_wait_to_start = true;
			else
				_wait_to_start = false;
			
			Hfs.clear();
			x.clear();
			dx.clear();
			Hi.slice<0,0,3,3>() = helper::QuatToMat( makeVector( _mis_o[0], _mis_o[1], _mis_o[2], _mis_o[3]));	
			
			
			Hi[0][3] = _mis_p[0];
			Hi[1][3] = _mis_p[1];
			Hi[2][3] = _mis_p[2];
			Hi[3][3] = 1.0;
			
			//Hf.slice<0,0,3,3>() = helper::QuatToMat( makeVector( _mis_o[0], _mis_o[1], _mis_o[2], _mis_o[3]));
			//Hf.slice<0,0,3,3>() = helper::QuatToMat( final_quat );
			cout << "Initial quaternion: " << helper::MatToQuat( Hi.slice<0,0,3,3>() ) << endl;
			//Hf.slice<0,0,3,3>() = helper::QuatToMat( makeVector( 1, 0, 0, 0));
			Hf.slice<0,0,3,3>() = helper::QuatToMat( makeVector( 0, 0, 1, 0));
			cout << "Final quaternion: " << helper::MatToQuat( Hf.slice<0,0,3,3>() ) << endl;

			Hf[0][3] = _wp[0];
			Hf[1][3] = _wp[1];
			Hf[2][3] = _wp[2];
			Hf[3][3] = 1.0;


			//Go to next wp
			create_marker( makeVector(Hi[0][3], Hi[1][3], Hi[2][3] ), makeVector(Hf[0][3], Hf[1][3], Hf[2][3] ) );					

			if( _wp_monitor_type == "traj" ) {

				cout << "Hi, Hf: " << Hi << endl << Hf << endl;
				_motion_planner->move_linear(Hi, Hf, _planning_vel, 
					_ap_max, _jp_max, _sp_max,  _vo_max, _ao_max, _jo_max, _so_max, 5.0, x, dx, ddx );				

				
				Matrix<4> t = Hf;
				t[0][3] = ( Hi[0][3] + Hf[0][3] ) / 2.0;
				t[1][3] = ( Hi[1][3] + Hf[1][3] ) / 2.0;
				t[2][3] = 0.5; //( Hi[2][3] + Hf[2][3] ) / 2.0;
				Hfs.push_back(t);
				Hfs.push_back(Hf);
				//_motion_planner->move_linear_wps(Hi, Hfs, 0.1, 0.15, x, dx );

				
				monitor_wp_traj (x, dx, ddx);
				sleep(1);
			}
			else if( _wp_monitor_type == "pd" ) {
				cout << "before to: monitor_wp_pd" << endl;
				monitor_wp_pd(Hf);
			}
			

		}
		else {
			
			_streaming_state.data = false;
			
			usleep(0.1*1e6);
			_t_size = 0;
			t_size_d.data = _t_size;
			_trajectory_size_pub.publish( t_size_d );
		}
	}	
}


