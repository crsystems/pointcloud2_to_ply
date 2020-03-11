#include <iostream>
#include <fstream>
#include "sensor_msgs/PointCloud2.h"
#include "ros/ros.h"

class LvxConvert {
	public:
		//LvxConvert(){}

		LvxConvert(std::string s)
		{
			finalFileHandle.open(s);
			tmpFileHandle.open("/tmp/livox_convert_tmp", std::fstream::in | std::fstream::out | std::fstream::trunc);
		}

		~LvxConvert()
		{

			writeHeader();

			tmpFileHandle.seekg(0, tmpFileHandle.beg);
			finalFileHandle << tmpFileHandle.rdbuf();

			if(tmpFileHandle.is_open()){
				tmpFileHandle.close();
			}
			if(finalFileHandle.is_open()){
				finalFileHandle.close();
			}
		}

		void writeHeader()
		{
			finalFileHandle << "ply\nformat ascii 1.0\nelement vertex " << numPoints << "\n";
			finalFileHandle << "property float32 x\nproperty float32 y\nproperty float32 z\n";
			finalFileHandle << "property float32 intensity\nend_header\n";
		}

		unsigned long int getNumberPoints()
		{
			return numPoints;
		}

		void processPointCloudMessageCallback (const sensor_msgs::PointCloud2::ConstPtr pcloud)
		{
			ROS_INFO("Processing %u points in this package", (unsigned int) pcloud->width);

			int step = (int) pcloud->point_step;
			unsigned int points = (unsigned int) pcloud->width;

			float *dat = (float *) pcloud->data.data();

			for(unsigned int i = 0; i < points; i++){
				float *coord = dat + i*step/4;
				tmpFileHandle << *coord << " " << *(coord + 1) << " " << *(coord + 2) << " " << *(coord + 3) << "\n";
				numPoints++;
			}
		}


	private:
		unsigned long int numPoints = 0;
		std::fstream   tmpFileHandle;
		std::ofstream  finalFileHandle;
};
