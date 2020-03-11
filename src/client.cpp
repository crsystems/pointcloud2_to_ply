#include "ros/ros.h"
#include <inttypes.h>
#include "lvxConvert.cpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "livox_to_ply");

	ros::NodeHandle n;

	if(argc < 3){
		ROS_FATAL("Supplied with too few arguments (%d instead of 2)! Quitting", argc-1);
		return 1;
	}

	LvxConvert *conv = new LvxConvert(std::string(argv[1]));
	
	ros::Subscriber sub = n.subscribe(std::string(argv[2]), 1000, &LvxConvert::processPointCloudMessageCallback, conv);

	ros::spin();

	conv->~LvxConvert();

	return 0;
}
