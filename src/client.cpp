#include "ros/ros.h"
#include <inttypes.h>
#include "lvxConvert.cpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "livox_to_ply");

	ros::NodeHandle n;

	if(argc < 4){
		ROS_FATAL("Supplied with too few arguments (%d instead of 3)! Quitting", argc-1);
		return 1;
	}

	LvxConvert *conv = new LvxConvert(std::string(argv[1]));
	
	ros::Subscriber laser_sub = n.subscribe(std::string(argv[2]), 1000, &LvxConvert::processPointCloudMessageCallback, conv);
	
	ros::Subscriber xsens_sub = n.subscribe(std::string(argv[3]), 1000, &LvxConvert::processOrientationMessageCallback, conv);

	ros::spin();

	conv->~LvxConvert();

	return 0;
}
