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
#include "std_msgs/Int32MultiArray.h"
#include <fstream>    
#include <ros/package.h>
#include "std_msgs/String.h"

using namespace std;
using namespace TooN;

#define C_TYPE_NO_CONTACT               "0"
#define C_TYPE_CONTACT_CONCORDE         "1"
#define C_TYPE_CONTACT_OPPOSITE         "2"
#define C_TYPE_CONTACT_DEVIATION        "3"

class cooperative_validation {
	public:

		cooperative_validation();
		~cooperative_validation();
		void emg_cb(std_msgs::Int32MultiArray emg );		
		void pose_cb( geometry_msgs::PoseStamped pose_ );
		void class_cb( std_msgs::String class_ );
  	void hdir_cb( geometry_msgs::Vector3 hdir );
  	void pdir_cb( geometry_msgs::Vector3 hdir );
		void angle_deviation_calc();
		void force_cb( geometry_msgs::WrenchStamped force_);
		void Init();
		void run();
		

	private:

		ros::NodeHandle _nh;

		ros::Subscriber _emg_sub;
		ros::Subscriber _pose_sub;
		ros::Subscriber _class_sub;
		ros::Subscriber _force_sub;
		ros::Subscriber  _eef_p_direction_sub;
		ros::Subscriber  _eef_h_direction_sub;
		
		//File
  	std::ofstream 		_emg_file;
  	std::ofstream     _pose_file;
  	std::ofstream     _class_file;
		std::ofstream     _angle_file;
		std::ofstream     _force_file;


		Vector<3> _pdir;
		Vector<3> _hdir;
		Vector<3> _force;

  
  	Vector<8> _emg;
  	string session;
};