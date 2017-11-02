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

#define TRAIN_TEST						 	2

#define C_TYPE_NO_CONTACT 			"1"
#define C_TYPE_CONTACT					"2"
#define C_TYPE_OPPOSITE					"3"


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
		return "no contact";
	if( c == 2 )
		return "contact";
	if( c == 3 )
		return "opposite";
}

class classification_data {
	public:
		classification_data();
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

classification_data::classification_data() {
	_eef_p_direction_sub 				= _nh.subscribe( "/iiwa/eef_p_direction", 0, &classification_data::pdir_cb, this );
	_eef_h_direction_sub 				= _nh.subscribe( "/iiwa/eef_h_direction", 0, &classification_data::hdir_cb, this );
	_force_sub									= _nh.subscribe( "/netft_data/filt", 0, &classification_data::force_cb, this );
	_marker_hdir_pub    				= _nh.advertise<visualization_msgs::Marker>("/iiwa/hdir", 10);
	_marker_pdir_pub    				= _nh.advertise<visualization_msgs::Marker>("/iiwa/pdir", 10);

  _dir_path = ros::package::getPath("iiwa_admittance_ctrl");


  load_param( _rate, 100, "rate" );
}

void classification_data::force_cb( geometry_msgs::WrenchStamped force_) {
	_force = makeVector( force_.wrench.force.x, force_.wrench.force.y, force_.wrench.force.z );
}

void classification_data::pdir_cb( geometry_msgs::Vector3 pdir ) {
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

void classification_data::hdir_cb( geometry_msgs::Vector3 hdir ) {
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

void classification_data::eef_x_cb( geometry_msgs::Pose pose ) {
	_x_eef = makeVector( pose.position.x, pose.position.y, pose.position.z );
}

void classification_data::data() {
	cout << "data!!" << endl;	
	ros::Rate r(_rate);

	string line;
	cout << "Insert classification type: " << endl;
	cout << "1 - Dirforce" << endl;
	
	getline(cin, line);
	int n = 0;
	if( line == "1" ) {
 		n = 2;
	}


	float angle;

	struct svm_model* model;
	if((model=svm_load_model( "/home/jcacace/ros_ws/src/my/IIWA/iiwa_admittance_ctrl/ts/model" ))==0) {
		printf("can't open model file\n");
		exit(1);
	}
	else
		cout << "Loaded!" << endl;

	//int n = 5;
	struct svm_node *x = (struct svm_node *) malloc((n+1)*sizeof(struct svm_node));

	int svm_type=svm_get_svm_type(model);
	int nr_class=svm_get_nr_class(model);
	Vector<3> nforce;

	while(ros::ok() ) {
		if( norm( _hdir ) > 0.0 && norm(_force) > 0.0 ) {			
			nforce = _force;
			nforce /= norm( nforce );

			if( line == "1" ) {

				angle = atan2( norm( _pdir ^ _hdir ), _pdir * _hdir);			
				x[0].index = 0; 
				x[0].value = norm(_force); 
				x[1].index = 1; 
				x[1].value = angle; 
		
			} 

		
			x[n].index = -1;
			cout << svm_predict(model,x) << endl;
			//cout << get_c_name( svm_predict(model,x) ) << endl;
		}		
		r.sleep();
	
	}
}


void classification_data::run() {

	boost::thread data_t( &classification_data::data,  this );
	ros::spin();

}

void classification_data::test_prediction() {

}


int main( int argc, char** argv ) {

	ros::init( argc, argv, "learning_data");

	classification_data td;
	td.run();

	return 0;
}