#include "validation.h"
	
using namespace std;
using namespace TooN;

cooperative_validation::cooperative_validation() {

	_emg_sub 		= _nh.subscribe("/MYO/myo_emg", 0, &cooperative_validation::emg_cb, this);
	_pose_sub   = _nh.subscribe("/iiwa/state/CartesianPose", 0, &cooperative_validation::pose_cb, this);
	_class_sub  = _nh.subscribe("/nn_classification/class", 0, &cooperative_validation::class_cb, this);

	_eef_p_direction_sub        = _nh.subscribe( "/iiwa/eef_p_direction", 0, &cooperative_validation::pdir_cb, this );
  _eef_h_direction_sub        = _nh.subscribe( "/iiwa/eef_h_direction", 0, &cooperative_validation::hdir_cb, this );
  _force_sub                  = _nh.subscribe( "/netft_data/filt", 0, &cooperative_validation::force_cb, this );


}

cooperative_validation::~cooperative_validation() {
	_emg_file.close();
	_pose_file.close();
	_class_file.close();
	_angle_file.close();
	_force_file.close();
}

void cooperative_validation::pose_cb( geometry_msgs::PoseStamped pose_ ) {
	_pose_file << pose_.pose.position.x << " " <<  pose_.pose.position.y << " " <<  pose_.pose.position.z << " " << 
								pose_.pose.orientation.w << " " <<  pose_.pose.orientation.x << " " <<  pose_.pose.orientation.y << " " << pose_.pose.orientation.z << endl;

}

void cooperative_validation::pdir_cb( geometry_msgs::Vector3 pdir ) {
  _pdir = makeVector( pdir.x, pdir.y, pdir.z );
}

void cooperative_validation::hdir_cb( geometry_msgs::Vector3 hdir ) {
  _hdir = makeVector( hdir.x, hdir.y, hdir.z ); 
}


void cooperative_validation::class_cb(std_msgs::String class_ ) {

	if ( class_.data == "C_TYPE_NO_CONTACT" )
		_class_file << C_TYPE_NO_CONTACT << endl; // " " << class_.data << endl;
	else if ( class_.data == "C_TYPE_CONTACT_CONCORDE" )
		_class_file << C_TYPE_CONTACT_CONCORDE << endl; //<< " " << class_.data << endl;
	else if ( class_.data == "C_TYPE_CONTACT_OPPOSITE" )
		_class_file << C_TYPE_CONTACT_OPPOSITE << endl; //<< " " << class_.data << endl;
	else if ( class_.data == "C_TYPE_CONTACT_DEVIATION" )
		_class_file << C_TYPE_CONTACT_DEVIATION << endl; //.<< " " << class_.data << endl;
}

void cooperative_validation::Init() {
	string line;

	cout << "1 - Admittance" << endl;
	cout << "2 - Shared" << endl;

	getline( cin, line );

	if( line == "1" ) 
		session = "admittance";
	else if( line == "2" ) 
		session = "shared";


	string pkg_path = ros::package::getPath("iiwa_admittance_ctrl");
	pkg_path += "/validation/";

	_emg_file.open( helper::string2char( pkg_path + session + "_emg.txt" ));
	_pose_file.open( helper::string2char( pkg_path + session + "_pose.txt" ));
	_class_file.open(helper::string2char( pkg_path + session + "_class.txt" ));
	_force_file.open(helper::string2char( pkg_path + session + "_force.txt" ));
	_angle_file.open(helper::string2char( pkg_path + session + "_angle.txt" ));

}


void cooperative_validation::emg_cb( std_msgs::Int32MultiArray emg ) {

	_emg = makeVector( emg.data[0], emg.data[1], emg.data[2], emg.data[3], emg.data[4], emg.data[5], emg.data[6], emg.data[7] );
	_emg_file << _emg << " " << norm(_emg) << endl;
}

void cooperative_validation::force_cb( geometry_msgs::WrenchStamped force_) {
  _force = makeVector( force_.wrench.force.x, force_.wrench.force.y, force_.wrench.force.z );
  _force_file << _force << " " << norm(_force) << endl;
}


void cooperative_validation::angle_deviation_calc() {

	ros::Rate r(100);
	float angle;
	while( ros::ok() ) {
	  if( norm( _pdir) && norm( _hdir ) > 0.0 && norm(_force) > 0.0 ) { 
  	  angle = atan2( norm( _pdir ^ _hdir ), _pdir * _hdir);
  	  _angle_file << angle << endl;
  	}
  	r.sleep();
	}


}

void cooperative_validation::run() {
	Init();
	
	boost::thread angle_deviation_calc_t( &cooperative_validation::angle_deviation_calc, this );
	ros::spin();
}



int main ( int argc, char** argv ) {

	ros::init(argc, argv, "cooperative_validation");
	cooperative_validation cv;
	cv.run();

	return 0;
}