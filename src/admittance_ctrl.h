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

#include "ros/ros.h"
#include "robohelper/robohelper.hpp"
#include "TooN/TooN.h"
#include "TooN/LU.h"
#include "boost/thread.hpp"
#include "iiwa_kinematic/LBRiiwa.h"
#include "iiwa_msgs/JointPosition.h"
#include "ctrl_msgs/desired_command.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Wrench.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

using namespace std;
using namespace TooN;

class admittance_ctrl {

public:
	admittance_ctrl();
	
	void run();		
	void cartesian_position_cb( geometry_msgs::PoseStamped pos );	
	void publish_cmd();
	void ctrl();
	void fts_cb( geometry_msgs::Wrench wrench );
	void joint_position_cb( iiwa_msgs::JointPosition jpose );
	void enable_stiffness_cb( std_msgs::Bool k_ );
	void cartesian_cmd_cb( ctrl_msgs::desired_command dcmd );

private:

	ros::NodeHandle _nh;
	ros::Subscriber _cartesian_position_sub;
	ros::Subscriber _fts_sub;
	ros::Subscriber _joint_position_sub;	
	ros::Subscriber _enable_k_sub;
	ros::Subscriber _dcmd_sub;
	ros::Publisher  _cartesian_position_pub;	
	ros::Publisher  _x_pub;
	ros::Publisher  _dx_pub;
	ros::Publisher  _xd_pub;	
	ros::Publisher	_eef_h_direction_pub;
	
	Vector<3> _mis_p;
	Vector<4> _mis_o;
	Vector<3> _cp;
	Vector<4> _co;

	bool _first_cpose;
	

	//---params
	int _rate;
	double _Tc;
	string _fts_topic_sub;	
	string _topic_ctrl;
	double _bb_x_min;
	double _bb_y_min;
	double _bb_z_min;

	bool   _goto_initial_pos;
	double _x_i;
	double _y_i;
	double _z_i;
	Vector<3> _initial_pos;

	//---input
	Vector<7> _xd;
	Vector<6> _dxd;
	Vector<6> _ddxd;
	//---

	//---output
	Vector<7> _xc;
	Vector<6> _dxc;
	Vector<6> _ddxc;
	//---

	Vector<7> _mis_q;

	Vector<3> _f;
	Vector<3> _t;
	Vector<3> _Mp, _Dp, _Kp;	
	
	double _mp, _dp, _kp;
	double _mo, _do, _ko;

	bool _initial_position;	
	bool _enable_stiffness;

	LBRiiwa *_iiwa;
		
	Matrix<4> _Tbe;
};



	
