#include "ros/ros.h"
#include <inttypes.h>
#include "lvxConvert.cpp"


int main(int argc, char** argv)
{
	ros::init(argc, argv, "livox_to_ply");

	ros::NodeHandle n;

	if(argc < 5){
		ROS_FATAL("Supplied with too few arguments (%d instead of 4)! Quitting", argc-1);
		return 1;
	}

	LvxConvert *conv = new LvxConvert(std::string(argv[1]), argv[4]);
	
	conv->start(&n, argv[2], argv[3]);

	ros::spin();

	conv->~LvxConvert();

	return 0;
}
