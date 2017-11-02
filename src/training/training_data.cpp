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
#include <ros/package.h>
#include "svm.h"

using namespace std;
using namespace TooN;


#define C_TYPE_NO_CONTACT 							"1"
#define C_TYPE_CONTACT_CONCORDE					"2"
#define C_TYPE_CONTACT_OPPOSITE					"3"
#define C_TYPE_CONTACT_DEVIATION 				"4"

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

void load_param( int & p, bool def, string name ) {
	ros::NodeHandle n_param("~");
	if( !n_param.getParam( name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}

string get_c_name(int c) {
	if( c == 1 )
		return "C_TYPE_NO_CONTACT";
	if( c == 2 )
		return "C_TYPE_CONTACT_CONCORDE";
	if( c == 3 )
		return "C_TYPE_CONTACT_OPPOSITE";
	else if( c == 4 ) 
		return "C_TYPE_CONTACT_DEVIATION";
}

class training_data {
	public:
		training_data();
		void run();
		void hdir_cb( geometry_msgs::Vector3 hdir );
		void pdir_cb( geometry_msgs::Vector3 pdir );
		void force_cb( geometry_msgs::WrenchStamped force_);
		void data();
		void eef_x_cb( geometry_msgs::Pose pose );
		void test_prediction();


	private:
		ros::NodeHandle _nh;
		ros::Subscriber _eef_p_direction_sub;
		ros::Subscriber _eef_h_direction_sub;
		ros::Subscriber _force_sub;
		ros::Publisher _marker_hdir_pub;
		ros::Publisher _marker_pdir_pub;
		Vector<3> _pdir;
		Vector<3> _hdir;
		Vector<3> _force;
		Vector<3> _x_eef;

		std::ofstream _norm_force_file;	  
	  std::ofstream _force_file;
	  std::ofstream _direction_file;
	  std::ofstream _complete_file;
	  std::ofstream _dirforce_file;
	  
	  std::ofstream _norm_force_file_normal;	  
	  std::ofstream _force_file_normal;
	  std::ofstream _direction_file_normal;
	  std::ofstream _complete_file_normal;
	  std::ofstream _dirforce_file_normal;
	  
	  string _dir_path;
  	int _rate;
};


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

training_data::training_data() {
	_eef_p_direction_sub 				= _nh.subscribe( "/iiwa/eef_p_direction", 0, &training_data::pdir_cb, this );
	_eef_h_direction_sub 				= _nh.subscribe( "/iiwa/eef_h_direction", 0, &training_data::hdir_cb, this );
	_force_sub									= _nh.subscribe( "/netft_data/filt", 0, &training_data::force_cb, this );
	_marker_hdir_pub    				= _nh.advertise<visualization_msgs::Marker>("/iiwa/hdir", 10);
	_marker_pdir_pub    				= _nh.advertise<visualization_msgs::Marker>("/iiwa/pdir", 10);

  _dir_path = ros::package::getPath("iiwa_admittance_ctrl");


  load_param( _rate, 100, "rate" );
}

void training_data::force_cb( geometry_msgs::WrenchStamped force_) {
	_force = makeVector( force_.wrench.force.x, force_.wrench.force.y, force_.wrench.force.z );
}

void training_data::pdir_cb( geometry_msgs::Vector3 pdir ) {
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

void training_data::hdir_cb( geometry_msgs::Vector3 hdir ) {
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

void training_data::eef_x_cb( geometry_msgs::Pose pose ) {
	_x_eef = makeVector( pose.position.x, pose.position.y, pose.position.z );
}

void training_data::data() {

	ros::Rate r(_rate);

	string force_file_name;
	string direction_file_name;
	string complete_file_name;
	string norm_force_file_name;
	string dirforce_file_name;

	
	string line;
	string contact;
	
	cout << "Insert data type: " << endl;
	cout << "1 - TRAIN" << endl;
	cout << "2 - TEST" << endl;
	getline(cin, line);	
	string prefix = "";
	if( line == "1") 
		prefix = "train_";
	else if( line == "2" )
		prefix = "test_";

	cout << "Insert contact type: " << endl;
	cout << "1 - no contact" << endl;
	cout << "2 - contact_concorde" << endl;
	cout << "3 - contact_opposite" << endl;
	cout << "4 - contact_deviation" << endl;
	getline(cin, line);	

	force_file_name = prefix + "force";
	force_file_name = _dir_path + "/ts/" + force_file_name;
	direction_file_name = prefix + "direction";
	direction_file_name = _dir_path + "/ts/" + direction_file_name;
	complete_file_name = _dir_path + "/ts/" + prefix +  "complete";
	norm_force_file_name= _dir_path + "/ts/"  + prefix +  "norm_force";
	dirforce_file_name = _dir_path + "/ts/"  + prefix +  "dirforce";

	if( line == "1" )
		contact = C_TYPE_NO_CONTACT;
	else if( line == "2" )
		contact = C_TYPE_CONTACT_CONCORDE;
	else if ( line == "3" )
		contact = C_TYPE_CONTACT_OPPOSITE;
	else if ( line == "4" )
		contact = C_TYPE_CONTACT_DEVIATION;


	_norm_force_file.open (  helper::string2char( norm_force_file_name ), std::ofstream::out | std::ofstream::app);
	_force_file.open (  helper::string2char( force_file_name ), std::ofstream::out | std::ofstream::app);
	_direction_file.open (  helper::string2char( direction_file_name ), std::ofstream::out | std::ofstream::app);
	_complete_file.open (  helper::string2char( complete_file_name ), std::ofstream::out | std::ofstream::app);
	_dirforce_file.open (  helper::string2char( dirforce_file_name ), std::ofstream::out | std::ofstream::app);

	_norm_force_file_normal.open (  helper::string2char( norm_force_file_name + "_normal" ), std::ofstream::out | std::ofstream::app);
	_force_file_normal.open (  helper::string2char( force_file_name + "_normal" ), std::ofstream::out | std::ofstream::app);
	_direction_file_normal.open (  helper::string2char( direction_file_name + "_normal" ), std::ofstream::out | std::ofstream::app);
	_complete_file_normal.open (  helper::string2char( complete_file_name + "_normal" ), std::ofstream::out | std::ofstream::app);
	_dirforce_file_normal.open (  helper::string2char( dirforce_file_name + "_normal" ), std::ofstream::out | std::ofstream::app);

	float angle;
	Vector<3> axis;

	Vector<3> nforce;
	while( ros::ok() ) {
		if( norm( _pdir) && norm( _hdir ) > 0.0 && norm(_force) > 0.0 ) {	

			angle = atan2( norm( _pdir ^ _hdir ), _pdir * _hdir);
			//cout << "angle: " << angle<< endl;

			nforce = _force;
			nforce = nforce / norm(nforce);

			/*
			_norm_force_file << contact << " 1:" << norm( _force ) << endl;
			_direction_file << contact << " 1:" << angle << endl;
			_force_file << contact <<  " 1:" << nforce[0] << " 2:" << nforce[1] << " 3:" << nforce[2] << " 4:" << norm( _force ) << endl;
			_complete_file << contact <<  " 1:" << nforce[0] << " 2:" << nforce[1] << " 3:" << nforce[2] << " 4:" << norm( _force ) << " 5:" << angle << endl;
			_dirforce_file << contact << " 1:" << norm(_force) << " 2:" << angle << endl;
			*/
			
			_norm_force_file_normal << norm( _force ) << " " << contact << endl;
			_direction_file_normal  << angle << " " << contact << endl;
			_force_file_normal      << nforce[0] << " " << nforce[1] << " " << nforce[2] << " " << norm( _force ) << " " << contact << endl;
			_complete_file_normal   << nforce[0] << " " << nforce[1] << " " << nforce[2] << " " << norm( _force ) << " " << angle << " " << contact << endl;
			_dirforce_file_normal   << norm(_force) << " " << angle <<  " " << contact << endl;


		}
		else {
			angle = 0.0;
		}
		r.sleep();
	}
	

}


void training_data::run() {

	boost::thread data_t( &training_data::data,  this );
	ros::spin();

}

void training_data::test_prediction() {

}


int main( int argc, char** argv ) {

	ros::init( argc, argv, "learning_data");

	training_data td;
	td.run();

	return 0;
}