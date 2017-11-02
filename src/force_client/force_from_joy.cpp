#include "ros/ros.h"
#include "TooN/TooN.h"
#include "helper/helper.h"
#include "boost/thread.hpp"
#include "geometry_msgs/Wrench.h"
#include "sensor_msgs/Joy.h"

using namespace std;
using namespace TooN;

class force_from_joy {
public:
	force_from_joy();
	void joy_cb( sensor_msgs::Joy joy );
	void run();
	void generate_force();
private:
	ros::NodeHandle _nh;
	ros::Subscriber _joy_sub;
	ros::Publisher _force_pub;
};

force_from_joy::force_from_joy() {
	_joy_sub = _nh.subscribe( "/joy", 0, &force_from_joy::joy_cb, this);
	_force_pub = _nh.advertise<geometry_msgs::Wrench>("/ft", 0);
}

void force_from_joy::joy_cb( sensor_msgs::Joy joy ) {

}


void force_from_joy::generate_force() {


	ros::Rate r(50);
	geometry_msgs::Wrench ft;
	while(ros::ok()) {


		_force_pub.publish( ft );
		r.sleep();
	}


}

void force_from_joy::run() {
	boost::thread generate_force_t( &force_from_joy::generate_force, this);
	ros::spin();
}

int main( int argc, char** argv ) {

	ros::init( argc, argv, "force_from_joy" );
	force_from_joy ffj;
	ffj.run();

	return 0;
}
