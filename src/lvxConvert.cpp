#include <iostream>
#include <fstream>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "ros/ros.h"

#include "atomicOrientation.cpp"

class LvxConvert {
	public:
		LvxConvert(std::string s)
		{
			numPoints = 0;
			infoNumber = 1;

			ornt = new AtomicOrientation();

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

		void inline performRotation(float *src, float* dst)
		{
			// Disable orientation correction for debugging
			dst[0] = src[0];
			dst[1] = src[1];
			dst[2] = src[2];

			/*
			// Hamilton multiplication method
			double r[4];
			double qp[4];
			double vq[4];

			ornt->getOrientation(r);

			// Streamlined Hamilton multiplication for p*q^-1
			qp[0] = -(r[1]*((double) src[0])) - (r[2]*((double) src[1])) - (r[3]*((double) src[2]));
			qp[1] = (r[0]*((double) src[0])) + (r[2]*((double) src[2])) - (r[3]*((double) src[1]));
			qp[2] = (r[0]*((double) src[1])) - (r[1]*((double) src[2])) + (r[3]*((double) src[0]));
			qp[3] = (r[0]*((double) src[2])) + (r[1]*((double) src[1])) - (r[2]*((double) src[0]));

			// Normal Hamilton multiplication
			vq[0] = (qp[0]*r[0]) + (qp[1]*r[1]) + (qp[2]*r[2]) + (qp[3]*r[3]);
			vq[1] = -(qp[0]*r[1]) + (qp[1]*r[0]) - (qp[2]*r[3]) + (qp[3]*r[2]);
			vq[2] = -(qp[0]*r[2]) + (qp[1]*r[3]) + (qp[2]*r[0]) - (qp[3]*r[1]);
			vq[3] = -(qp[0]*r[3]) - (qp[1]*r[2]) + (qp[2]*r[1]) + (qp[3]*r[0]);

			dst[0] = (float) vq[1];
			dst[1] = (float) vq[2];
			dst[2] = (float) vq[3];
			*/

			/*
			// Rotation matrix variant
			double q[4];
			ornt->getOrientation(q);

			// Ugly inlined rotation matrix
			dst[0] = (float) ((1.0 - (2.0*(q[2]*q[2] + q[3]*q[3])))*((double) src[0]) + (2.0*(q[1]*q[2] - q[3]*q[0]))*((double) src[1]) + (2.0*(q[1]*q[3] + q[2]*q[0]))*((double) src[2]));
			dst[1] = (float) ((2.0*(q[1]*q[2] + q[3]*q[0]))*((double) src[0]) + (1.0 - (2.0*(q[1]*q[1] + q[3]*q[3])))*((double) src[1]) + (2.0*(q[2]*q[3] - q[1]*q[0]))*((double) src[2]));
			dst[2] = (float) ((2.0*(q[1]*q[3] - q[2]*q[0]))*((double) src[0]) + (2.0*(q[2]*q[3] + q[1]*q[0]))*((double) src[1]) + (1.0 - (2.0*(q[1]*q[1] + q[2]*q[2])))*((double) src[2]));
			*/

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
				float src[3], dst[3];

				src[0] = *coord;
				src[1] = *(coord + 1);
				src[2] = *(coord + 2);

				performRotation(src, dst);

				tmpFileHandle << dst[0] << " " << dst[1] << " " << dst[2] << " " << *(coord + 3) << "\n";

				//tmpFileHandle << *coord << " " << *(coord + 1) << " " << *(coord + 2) << " " << *(coord + 3) << "\n";
				
				numPoints++;
			}
		}

		
		void processOrientationMessageCallback (const geometry_msgs::QuaternionStamped::ConstPtr geom)
		{
			//ROS_INFO("Orientation x value is : %f", (double) geom->quaternion.x);
			
			double quaternion[4];
			quaternion[0] = (double) geom->quaternion.w;
			quaternion[1] = (double) geom->quaternion.x;
			quaternion[2] = (double) geom->quaternion.y;
			quaternion[3] = (double) geom->quaternion.z;

			ornt->setOrientation(quaternion);
		}

		/*
		void processOrientationMessageCallback (const sensor_msgs::Imu::ConstPtr geom)
		{
			//ROS_INFO("Orientation x value is : %f", (double) geom->quaternion.x);
			
			double quaternion[4];
			quaternion[0] = (double) geom->orientation.w;
			quaternion[1] = (double) geom->orientation.x;
			quaternion[2] = (double) geom->orientation.y;
			quaternion[3] = (double) geom->orientation.z;

			ornt->setOrientation(quaternion);
		}
		*/

	private:
		int infoNumber;
		unsigned long int numPoints;
		std::fstream   tmpFileHandle;
		std::ofstream  finalFileHandle;

		AtomicOrientation *ornt;
};
