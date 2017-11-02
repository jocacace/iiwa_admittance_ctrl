/*
 * Copyright (C) 2017, Jonathan Cacace and Prisma Lab
 * Email id : jonathan.cacace@gmail.com
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
*/

#include "admittance_ctrl.h"

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


admittance_ctrl::admittance_ctrl() {

	//---Admittance control gains
	load_param( _mp, 10.0, "mp");
	load_param( _dp, 255.0, "dp");
	load_param( _kp, 500.0, "kp");
	load_param( _mo, 1.0, "mo");
	load_param( _do, 50.0, "do");
	load_param( _ko, 0.0, "ko");

	load_param( _bb_z_min, 0.35, "bb_z_min");
	//---

	load_param( _goto_initial_pos, false, "goto_initial_pos");
	load_param( _x_i, 0.0, "x_i");
	load_param( _y_i, 0.0, "y_i");
	load_param( _z_i, 0.0, "z_i");

	_initial_pos = makeVector( _x_i, _y_i, _z_i );
	if( _goto_initial_pos && norm( _initial_pos ) == 0.0 ) {
		ROS_ERROR( "Initial position not correctly initialized!");
		exit(1);
	}

	load_param(_fts_topic_sub, "fts_data", "fts_data");
	load_param( _rate, 100, "rate");
	load_param( _topic_ctrl, "/iiwa/cartesian_ctrl", "topic_ctrl");
	
	_cartesian_position_pub = _nh.advertise< geometry_msgs::PoseStamped >(_topic_ctrl.c_str(), 0 );
	_x_pub 										= _nh.advertise< geometry_msgs::Pose >("/iiwa/xc", 0);
	_xd_pub 									= _nh.advertise< geometry_msgs::Pose > ("/iiwa/xd", 0);
	_dx_pub 									= _nh.advertise< geometry_msgs::Twist >("/iiwa/eef_dx", 0);
	_eef_h_direction_pub 			= _nh.advertise< geometry_msgs::Vector3 >("/iiwa/eef_h_direction", 0);
	//---

	//---Input
	_cartesian_position_sub 	= _nh.subscribe("/iiwa/state/CartesianPose", 0, &admittance_ctrl::cartesian_position_cb, this);
	_joint_position_sub 			= _nh.subscribe("/iiwa/state/JointPosition", 0, &admittance_ctrl::joint_position_cb, this);
	_fts_sub 									= _nh.subscribe(_fts_topic_sub.c_str(), 0, &admittance_ctrl::fts_cb, this);	
	_enable_k_sub 						= _nh.subscribe("/iiwa/enable_stiffness", 0, &admittance_ctrl::enable_stiffness_cb, this );
	_dcmd_sub 								= _nh.subscribe("/cartesian_commad", 0, &admittance_ctrl::cartesian_cmd_cb, this);	
	//---

	//---Init
	_Tc = 1.0/float(_rate);

	_Mp = makeVector(_mp, _mp, _mp );
	_Dp = makeVector(_dp, _dp, _dp );
	_Kp = makeVector(_kp, _kp, _kp );

	_xd = Zeros;
	_dxd = _ddxd = Zeros;

	_iiwa = new LBRiiwa();
	_first_cpose = false;
	//---
	
	_Tbe = Zeros;
}

void admittance_ctrl::joint_position_cb( iiwa_msgs::JointPosition jpose ) {	
	_mis_q = makeVector( jpose.position.a1, jpose.position.a2, jpose.position.a3, jpose.position.a4, jpose.position.a5, jpose.position.a6, jpose.position.a7 );	
} //eef pose

void admittance_ctrl::enable_stiffness_cb( std_msgs::Bool enable_ ) {
	_enable_stiffness = enable_.data;
}	//Enable/Disable stiffness


void admittance_ctrl::fts_cb( geometry_msgs::Wrench wrench ) {
	
	Vector<3> S = makeVector(0, 0, -0.10 );
	Vector<3> f = makeVector ( wrench.force.x, wrench.force.y, wrench.force.z );
	Vector<3> t = makeVector ( wrench.torque.x, wrench.torque.y, wrench.torque.z );

	_f = f;
	_t = t;	

	_f = _Tbe.slice<0,0,3,3>()*_f;	
	_t = robohelper::Skew( S )*_Tbe.slice<0,0,3,3>()*_f + _Tbe.slice<0,0,3,3>()*_t;
		
}

void admittance_ctrl::cartesian_cmd_cb( ctrl_msgs::desired_command dcmd ) {
	_xd = makeVector( dcmd.xd.position.x, dcmd.xd.position.y, dcmd.xd.position.z, dcmd.xd.orientation.w, dcmd.xd.orientation.x, dcmd.xd.orientation.y, dcmd.xd.orientation.z);
	_dxd = makeVector( dcmd.dxd.linear.x, dcmd.dxd.linear.y, dcmd.dxd.linear.z, dcmd.dxd.angular.x, dcmd.dxd.angular.y, dcmd.dxd.angular.z);
	_ddxd = makeVector( dcmd.ddxd.linear.x, dcmd.ddxd.linear.y, dcmd.ddxd.linear.z, dcmd.ddxd.angular.x, dcmd.ddxd.angular.y, dcmd.ddxd.angular.z);
}

void admittance_ctrl::cartesian_position_cb( geometry_msgs::PoseStamped pos ) {
	_mis_p = makeVector( pos.pose.position.x, pos.pose.position.y, pos.pose.position.z );
	_mis_o = makeVector( pos.pose.orientation.w, pos.pose.orientation.x, pos.pose.orientation.y, pos.pose.orientation.z );	
	_first_cpose = true;
}

void admittance_ctrl::publish_cmd () {

	ros::Rate r(_rate);
	geometry_msgs::PoseStamped _cmd_pose;

	while(!_first_cpose)
		usleep(0.1*1e6);

	_cp = _mis_p;
	_co = _mis_o;

	while( ros::ok() ) {

		_cmd_pose.pose.position.x = _cp[0];
		_cmd_pose.pose.position.y = _cp[1];

		//if( _cp[2] > _bb_z_min )
			_cmd_pose.pose.position.z = _cp[2];
		//else
		//	_cmd_pose.pose.position.z = _bb_z_min;

		_cmd_pose.pose.orientation.w = _co[0];
		_cmd_pose.pose.orientation.x = _co[1];
		_cmd_pose.pose.orientation.y = _co[2];
		_cmd_pose.pose.orientation.z = _co[3];

		_cartesian_position_pub.publish( _cmd_pose );
		
		r.sleep();
	}
}

Vector<3> diag_gain(Vector<3> K, Vector<3> v) {

	Vector<3> Kv;
	for(int i=0; i<3; i++ ) {
		Kv[i] = K[i]*v[i];
	}

	return Kv;
}

void admittance_ctrl::ctrl() {

	ros::Rate r(_rate);
 
	static tf::TransformBroadcaster br;
  tf::Transform transform; 

	while(!_first_cpose) {	
		usleep(0.1*1e6);
	} //Wait first end effector cartesian pose
	ROS_INFO("First end effector cartesian pose");

	_ddxd = Zeros;
	_dxd = Zeros;
	_xd = Zeros;
	_dxc = Zeros;
	_xc = Zeros;
	

	ROS_INFO("Be careful! Going to initial position");
	if( _goto_initial_pos ) {
		_cp = _initial_pos;		
		while( norm( _cp - _mis_p) > 0.01 ) {
			_cp = _initial_pos;
			usleep(0.1*1e6);
		}
	}


	_xd = makeVector( _mis_p[0], _mis_p[1], _mis_p[2], _mis_o[0], _mis_o[1], _mis_o[2], _mis_o[3] );
	_dxd = _ddxd = Zeros;
	_xc = _xd;
	_ddxc = Zeros;

	float eta_cd;
	Matrix<3> I 			= Identity;
	Matrix<3> Rwd 		= Zeros;
	Matrix<3> Rwc 		= Zeros;
	Matrix<3> Rcd 		= Zeros;
	Vector<4> qcd 		= Zeros;
	Vector<3> eps_cd 	= Zeros;
	Matrix<3> Kop 		= Zeros;	
	Vector<3> Dw_dc 	= Zeros;
	Vector<3> Ddw_dc 	= Zeros;
	Vector<3> dwc 		= Zeros;
	Vector<3> wc 			= Zeros;
	Matrix<3> Do 			= Zeros;
	Matrix<3> Mo 			= Zeros;
	Matrix<3> Ko 			= Zeros;
	Vector<4> dq 			= Zeros;
	
	Vector<3> av 			= Zeros; //admittance velocity?
	geometry_msgs::Vector3 eef_h_d;


	Fill( Do ) = _do, 0, 0,	
								0, _do, 0,
								0, 0, _do;

	Fill( Mo ) = _mo, 0, 0,	
								0, _mo, 0,
								0, 0, _mo;

	Fill( Ko ) = _ko, 0, 0,	
								0, _ko, 0,
								0, 0, _ko;

	Matrix<3> invMo;						
	Fill( invMo ) = 1.0/_mo, 0, 0,	
									0, 1.0/_mo, 0,
									0, 0, 1.0/_mo;
		
	geometry_msgs::Pose xc_data;
	geometry_msgs::Twist dxc_data;
	geometry_msgs::Pose xd_data;

	while( ros::ok() ) {

	
		_Tbe = _iiwa->dirkin(_mis_q);		


		if( _enable_stiffness ) 
			_Kp = makeVector(_kp, _kp, _kp );
		else
			_Kp = Zeros;

		//---Position
		Vector<3> ddpc = diag_gain( makeVector( 1.0 / _Mp[0], 1.0 / _Mp[1], 1.0 / _Mp[2] ), 
											 diag_gain( _Mp, makeVector(_ddxd[0],_ddxd[1],_ddxd[2]) ) + 
											 diag_gain( _Dp, makeVector(_dxd[0],_dxd[1],_dxd[2]) - makeVector(_dxc[0],_dxc[1],_dxc[2])) +
											 diag_gain( _Kp, makeVector(_xd[0],_xd[1],_xd[2]) - makeVector(_xc[0],_xc[1],_xc[2])) +
											 makeVector( _f[0], _f[1], _f[2] ) );

		
		
		_ddxc.slice<0,3>() = makeVector( ddpc[0], ddpc[1], ddpc[2] );
		_dxc.slice<0,3>() += _ddxc.slice<0,3>()*_Tc;
		//---
		//---Orientation	
		_t[0] = 0.0;
		_t[1] = 0.0;
		Rwd = robohelper::QuatToMat( _xd.slice<3, 4>() );
		Rwc = robohelper::QuatToMat( _xc.slice<3, 4>() );		
		Rcd = Rwc.T()*Rwd;
		qcd = robohelper::MatToQuat( Rcd );
		eta_cd = qcd[0];
		eps_cd = qcd.slice<1,3>();
		Kop = 2.0*( eta_cd*Identity -robohelper::Skew( eps_cd ) ).T() * Ko;
		Dw_dc = Rwc.T() * ( _dxd.slice<3,3>() - _dxc.slice<3,3>() );
		Ddw_dc = invMo * ( (Rwc.T() * _t) - ( Do * Dw_dc) - (Kop * eps_cd ) );
		dwc =  _ddxd.slice<3,3>() - Rwc * Ddw_dc + robohelper::Skew( _dxc.slice<3,3>()).T()  * ( _dxd.slice<3,3>() - _dxc.slice<3,3>() ) ;		
		wc += dwc*_Tc;	
		_dxc.slice<3,3>() = wc;
		dq = robohelper::w2dq( wc, _xc.slice<3,4>() );		
		//---

		//Update Orientation 		
		_xc.slice<3,4>() += dq*_Tc;						
		_xc.slice<3,4>() = _xc.slice<3,4>()  / norm( _xc.slice<3,4>()  );
		//Update Position
		_xc.slice<0,3>() += makeVector( _dxc[0], _dxc[1], _dxc[2] )*_Tc;		
	
		if( _xc[2] < _bb_z_min )
			_xc[2] = _bb_z_min;

 		//Send control data		
		_cp = makeVector( _xc[0], _xc[1], _xc[2] );
		_co = makeVector( _xc[3], _xc[4], _xc[5], _xc[6]  );

		//---Compliancy data
		xc_data.position.x = _xc[0];
		xc_data.position.y = _xc[1];
		xc_data.position.z = _xc[2];
		_x_pub.publish( xc_data );
		
		dxc_data.linear.x = _dxc[0];
		dxc_data.linear.y = _dxc[1];
		dxc_data.linear.z = _dxc[2];		
		_dx_pub.publish( dxc_data );

		xd_data.position.x = _xd[0];
		xd_data.position.y = _xd[1];
		xd_data.position.z = _xd[2];
		_xd_pub.publish( xd_data );
		//---

		//---Tf
		Vector<4> quat = makeVector( _xc[3], _xc[4], _xc[5], _xc[6]);		
	  transform.setOrigin( tf::Vector3( _xc[0], _xc[1], _xc[2] ) );
		tf::Quaternion qr(quat[1], quat[2], quat[3], quat[0] );
		transform.setRotation(qr);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "XC"));

		Vector<4> quat_xd = makeVector( _xd[3], _xd[4], _xd[5], _xd[6]);		
	  transform.setOrigin( tf::Vector3( _xd[0], _xd[1], _xd[2] ) );
		tf::Quaternion qr_xd(quat_xd[1], quat_xd[2], quat_xd[3], quat_xd[0] );
		transform.setRotation(qr_xd);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "XD"));
		//---

		if( norm(_f) > 0.0 ) {
			av = makeVector( _dxc[0] - _dxd[0], _dxc[1] - _dxd[1], _dxc[2] - _dxd[2] );
			if( norm(av) > 0 )
				av /= norm(av);
		}
		else
			av = Zeros;

		eef_h_d.x = av[0];
		eef_h_d.y = av[1];
		eef_h_d.z = av[2];
		_eef_h_direction_pub.publish( eef_h_d );


  	r.sleep();

	}
}


void admittance_ctrl::run() {

	boost::thread publish_cmd_t( &admittance_ctrl::publish_cmd, this );		//Stream cmd
	boost::thread admittance_ctrl_t( &admittance_ctrl::ctrl, this );	//calculate admittance
	
	ros::spin();
}


int main( int argc, char** argv ) {
	
	ros::init( argc, argv, "admtt_ctrl");
	admittance_ctrl admtt_ctrl;
	admtt_ctrl.run();
	return 0;
}

