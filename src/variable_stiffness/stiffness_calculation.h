#include "ros/ros.h"
#include "TooN/TooN.h"
#include "helper/helper.h"
#include "sensor_msgs/JointState.h"
#include "boost/thread.hpp"
#include "geometry_msgs/Wrench.h"
#include "sensor_msgs/JointState.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include "geometry_msgs/WrenchStamped.h"
#include "iiwa_kinematic/LBRiiwa.h"
#include "TooN/LU.h"
#include "6d_motion_planner/6d_motion_planner.h"
#include "iiwa_msgs/JointPosition.h"
#include "visualization_msgs/Marker.h"
#include <iostream>
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

using namespace std;
using namespace TooN;

class variable_stiffness {

	public: 
		variable_stiffness();
		void run();
		void pdir_cb( geometry_msgs::Vector3 pdir );
		void hdir_cb( geometry_msgs::Vector3 pdir );
		void eef_x_cb( geometry_msgs::Pose pose );
		void eef_dx_cb( geometry_msgs::Twist dpose );
		void calc_direction();
		void cartesian_position_cb( geometry_msgs::PoseStamped eef_c_pos );
		void stream_trajectory();
		void manage_wp(Vector<3> p );
		void trajectory_size_cb( std_msgs::Int32 tsize_cb );
		void trajectory_deviation_cb( std_msgs::Float64 deviation_d );
		void mis_dp_cb( geometry_msgs::Twist mis_dp_);
		void c_cb( std_msgs::String c );
		void print_ws();
		void print_cpoint(Vector<3> p);
		void xd_cb( geometry_msgs::Pose xd );
		void c_start_cb( std_msgs::String c );
		void streaming_trajectory_cb( std_msgs::Bool streaming );
		void find_closest_point();
		void emg_norm_cb( std_msgs::Float64 emg_);
		
	private:

		ros::NodeHandle _nh;

		ros::Subscriber _eef_p_direction_sub;
		ros::Subscriber _eef_h_direction_sub;
		ros::Subscriber _x_sub;
		ros::Subscriber _dx_pub;
		ros::Subscriber _cartesian_position_sub;
		ros::Subscriber _trajectory_size_sub;
		ros::Subscriber _trajectory_deviation_sub;
		ros::Subscriber _mis_dp_sub;
		ros::Subscriber _c_sub;
		ros::Publisher  _marker_sphere_pub;
		ros::Subscriber _xd_sub;
		ros::Subscriber _c_start_sub;
		ros::Subscriber _trajectory_streaming_sub;
		ros::Subscriber _emg_norm_sub;

		ros::Publisher _k_pub;
		ros::Publisher _replan_pub;
		ros::Publisher _marker_hdir_pub;
		ros::Publisher _marker_pdir_pub;
		ros::Publisher _wp_pub;
		ros::Publisher _slow_wp_pub;
		ros::Publisher _listen_f_pub;
		ros::Publisher _halt_robot_pub;
		ros::Publisher _strat_traj_pub;
		ros::Publisher _cpoint_pub;
		ros::Publisher _attraction_point_pub;
		ros::Publisher _dist_from_traj_pub;
		string _classification;
		string _start_classification;
		bool _is_streaming_traj;

		double _k_param;
		bool _stream_trajectory;
		int _trajectory_size;
		Vector<3> _pdir;
		Vector<3> _hdir;
		Vector<3> _x_eef;


		vector< Vector<3> > _wp_stack;
		Vector<7> _eef_pose;
		Vector<7> _xd;
		
		Vector<3> _dx_eef;
		Vector<3> _mis_dp;
		Vector<3> _starting_point;
		double     _check_sspace;
		int _wp_index;
		double _ws_size;
		float _t_deviation;


		bool _wp_passed;
		bool _print_ws;
		
};

