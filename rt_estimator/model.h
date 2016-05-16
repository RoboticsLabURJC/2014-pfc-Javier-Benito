#ifndef RT_ESTIMATOR_MODEL_H
#define RT_ESTIMATOR_MODEL_H


#include <iostream>
#include <jderobot/pointcloud.h>
#include <Eigen/Dense>
//#include <Eigen/SVD>
#include <random>

//PCL
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#define RT_ROWS 4
#define RT_COLUMNS 4
#define NUM_POINTS 60

namespace rt_estimator {
	class Model {
	public:
		Model();
		virtual ~Model();

		float getX(int index);
		float getY(int index);
		float getZ(int index);
		int getRed(int index);
		int getGreen(int index);
		int getBlue(int index);
		int getSize();
		bool isNull();
		bool isFinal();

		void createPointCloud();
		void calculateNewPointCloudxRT();
		void estimateRT();
		void addGaussianNoise();

	private:
		bool are_points_estimates;
		bool is_final;

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud_estimate;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud2;


		Eigen::Matrix4f RT;
	}; //class
}//namespace

#endif //RT_ESTIMATOR_MODEL_H
