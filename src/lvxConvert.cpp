#include <iostream>
#include <fstream>
#include "sensor_msgs/PointCloud2.h"
#include "ros/ros.h"

class LvxConvert {
	public:
		LvxConvert(std::string s)
		{
			numPoints = 0;
			infoNumber = 1;

			finalFileHandle.open(s.c_str());
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
			if(numPoints >= infoNumber*100000){
				ROS_INFO("[livox_to_ply] Processed %u points", (unsigned int) infoNumber*100000);
				infoNumber++;
			}

			int step = (int) pcloud->point_step;
			unsigned int points = (unsigned int) pcloud->width;

			char *dat = (char *) pcloud->data.data();

			for(unsigned int i = 0; i < points; i++){
				float *coord = (float *) (dat + i*step);
				tmpFileHandle << *coord << " " << *(coord + 1) << " " << *(coord + 2) << " " << *(coord + 3) << "\n";
				numPoints++;
			}
		}


	private:
		int infoNumber;
		unsigned long int numPoints;
		std::fstream   tmpFileHandle;
		std::ofstream  finalFileHandle;
};
