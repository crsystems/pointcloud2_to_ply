#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/QuaternionStamped.h"
#include <cstring>
#include <iostream>

double first[4], current[4];

void testDriftCallbackImu(sensor_msgs::Imu::ConstPtr geom)
{
	if(first[0] == 0.0 && first[1] == 0.0 && first[2] == 0.0 && first[3] == 0){
		first[0] = (double) geom->orientation.w;
		first[1] = (double) geom->orientation.x;
		first[2] = (double) geom->orientation.y;
		first[3] = (double) geom->orientation.z;
	} else {
		double diff[8];
		diff[0] = first[0] - (double) geom->orientation.w;
		diff[1] = first[1] - (double) geom->orientation.x;
		diff[2] = first[2] - (double) geom->orientation.y;
		diff[3]	= first[3] - (double) geom->orientation.z;
		diff[4] = current[0] - (double) geom->orientation.w;
		diff[5] = current[1] - (double) geom->orientation.x;
		diff[6]	= current[2] - (double) geom->orientation.y;
		diff[7] = current[3] - (double) geom->orientation.z;

		current[0] = (double) geom->orientation.w;
		current[1] = (double) geom->orientation.x;
		current[2] = (double) geom->orientation.y;
		current[3] = (double) geom->orientation.z;

		// Human readable for ROS and stdout
		ROS_INFO("Diff to start: %f, %f, %f, %f --- Diff to last: %f, %f, %f, %f", diff[0], diff[1], diff[2], diff[3], diff[4], diff[5], diff[6], diff[7]);
		
		// And the easy to parse version to stderr
		std::cerr << diff[0] << "," << diff[1] << "," << diff[2] << "," << diff[3] << ",";
		std::cerr << diff[4] << "," << diff[5] << "," << diff[6] << "," << diff[7] << "\n";
	}
}

void testDriftCallbackQuaternion(geometry_msgs::QuaternionStamped::ConstPtr geom)
{
	if(first[0] == 0.0 && first[1] == 0.0 && first[2] == 0.0 && first[3] == 0){
		first[0] = (double) geom->quaternion.w;
		first[1] = (double) geom->quaternion.x;
		first[2] = (double) geom->quaternion.y;
		first[3] = (double) geom->quaternion.z;
	} else {
		double diff[8];
		diff[0] = first[0] - (double) geom->quaternion.w;
		diff[1] = first[1] - (double) geom->quaternion.x;
		diff[2] = first[2] - (double) geom->quaternion.y;
		diff[3]	= first[3] - (double) geom->quaternion.z;
		diff[4] = current[0] - (double) geom->quaternion.w;
		diff[5] = current[1] - (double) geom->quaternion.x;
		diff[6]	= current[2] - (double) geom->quaternion.y;
		diff[7] = current[3] - (double) geom->quaternion.z;

		current[0] = (double) geom->quaternion.w;
		current[1] = (double) geom->quaternion.x;
		current[2] = (double) geom->quaternion.y;
		current[3] = (double) geom->quaternion.z;

		// Human readable format for ROS and stdout
		ROS_INFO("Diff to start: %f, %f, %f, %f --- Diff to last: %f, %f, %f, %f", diff[0], diff[1], diff[2], diff[3], diff[4], diff[5], diff[6], diff[7]);
		
		// And the easy to parse version to stderr
		std::cerr << diff[0] << "," << diff[1] << "," << diff[2] << "," << diff[3] << ",";
		std::cerr << diff[4] << "," << diff[5] << "," << diff[6] << "," << diff[7] << "\n";
	}
}

int main(int argc, char **argv)
{
	memset(first, 0, 4*sizeof(double));
	memset(current, 0, 4*sizeof(double));

	ros::init(argc, argv, "driftCheck");

	ros::NodeHandle n;

	if(argc < 2){
		ROS_FATAL("Supplied with too few arguments. Quitting");
		return 1;
	}

	ros::Subscriber xsens_sub;
	
	if(strcmp(argv[1], "/imu/data") == 0){
		ROS_INFO("Using /imu/data topic");
		xsens_sub = n.subscribe("/imu/data", 1000, &testDriftCallbackImu);
	}else{
		ROS_INFO("Using /filter/quaternion topic");
		xsens_sub = n.subscribe("/filter/quaternion", 1000, &testDriftCallbackQuaternion);
	}

	ros::spin();

	return 0;
}
