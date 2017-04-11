#include "model.h"

namespace rt_estimator {

	Model::Model() {
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGBA>);
			pc->points.resize(NUM_POINTS);
			this->pointCloud = pc;
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc2(new pcl::PointCloud<pcl::PointXYZRGBA>);
			pc2->points.resize(NUM_POINTS*2);
			this->pointCloud_estimate = pc2;
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc3(new pcl::PointCloud<pcl::PointXYZRGBA>);
			pc3->points.resize(NUM_POINTS);
			this->pointCloud2 = pc3;

			this->are_points_estimates = false;
			this->is_final = false;
	}

	Model::~Model() {}

	float Model::getX(int index){
		if (!this->are_points_estimates)
			return this->pointCloud->points[index].x;
		else
			return this->pointCloud_estimate->points[index].x;
	}

	float Model::getY(int index){
		if (!this->are_points_estimates)
			return this->pointCloud->points[index].y;
		else
			return this->pointCloud_estimate->points[index].y;
	}

	float Model::getZ(int index){
		if (!this->are_points_estimates)
			return this->pointCloud->points[index].z;
		else
			return this->pointCloud_estimate->points[index].z;
	}

	int Model::getRed(int index){
		if (!this->is_final)
			return this->pointCloud->points[index].r;
		else
			return this->pointCloud_estimate->points[index].r;

	}

	int Model::getGreen(int index){
		if (!this->is_final)
			return this->pointCloud->points[index].g;
		else
			return this->pointCloud_estimate->points[index].g;
	}

	int Model::getBlue(int index){
		if (!this->is_final)
			return this->pointCloud->points[index].b;
		else
			return this->pointCloud_estimate->points[index].b;
	}

	int Model::getSize() {
		if (!this->is_final)
			return this->pointCloud->points.size();
		else
			return 2*this->pointCloud->points.size();
	}

	bool Model::isFinal() {
		return this->is_final;
	}

	void Model::createPointCloud() {

		for(int i=0; i<NUM_POINTS; i+=4){
			/*this->pointCloud->points[i].x = 0;
			this->pointCloud->points[i].y = i*50;
			this->pointCloud->points[i].z = i*50;
			this->pointCloud->points[i].r = 150;
			this->pointCloud->points[i].g = 80;
			this->pointCloud->points[i].b = 25;
			this->pointCloud->points[i+1].x = 0;
			this->pointCloud->points[i+1].y = (i+1)*50;
			this->pointCloud->points[i+1].z = i*50;
			this->pointCloud->points[i+1].r = 200;
			this->pointCloud->points[i+1].g = 150;
			this->pointCloud->points[i+1].b = 200;

			this->pointCloud->points[i+2].x = 0;
			this->pointCloud->points[i+2].y = (i+2)*50;
			this->pointCloud->points[i+2].z = i*50;
			this->pointCloud->points[i+2].r = 80;
			this->pointCloud->points[i+2].g = 110;
			this->pointCloud->points[i+2].b = 0;*/

			this->pointCloud->points[i].x = i*25;
			this->pointCloud->points[i].y = 200+0;
			this->pointCloud->points[i].z = 200+0;
			this->pointCloud->points[i].r = 225;
			this->pointCloud->points[i].g = 25;
			this->pointCloud->points[i].b = 25;

			this->pointCloud->points[i+1].x = 0;
			this->pointCloud->points[i+1].y = 200+i*25;
			this->pointCloud->points[i+1].z = 200+0;
			this->pointCloud->points[i+1].r = 25;
			this->pointCloud->points[i+1].g = 225;
			this->pointCloud->points[i+1].b = 25;

			this->pointCloud->points[i+2].x = 0;
			this->pointCloud->points[i+2].y = 200+0;
			this->pointCloud->points[i+2].z = 200+i*25;
			this->pointCloud->points[i+2].r = 25;
			this->pointCloud->points[i+2].g = 25;
			this->pointCloud->points[i+2].b = 225;

			this->pointCloud->points[i+3].x = i*25;
			this->pointCloud->points[i+3].y = i*50;
			this->pointCloud->points[i+3].z = i*75;
			this->pointCloud->points[i+3].r = 75;
			this->pointCloud->points[i+3].g = 75;
			this->pointCloud->points[i+3].b = 75;
		}

		this->are_points_estimates = false;
		this->is_final = false;
	}

	void Model::calculateNewPointCloudxRT() {

		//Eigen::MatrixXf RT(RT_ROWS, RT_COLUMNS);
		/*this->RT << 1, 0.5, 1.5, 0.5,
					1, -1.5, 1, -1.5,
					0.5, 1.5, 0.5, 1,
					0, 0, 0, 1;*/
		/*RT(0,0) = 1;
		RT(0,1) = 0.5;
		RT(0,2) = 1.5;
		RT(0,3) = 0.5;
		RT(1,0) = 1;
		RT(1,1) = -1.5;
		RT(1,2) = 1;
		RT(1,3) = -1.5;
		RT(2,0) = 0.5;
		RT(2,1) = 1.5;
		RT(2,2) = 0.5;
		RT(2,3) = 1;
		RT(3,0) = 0;
		RT(3,1) = 0;
		RT(3,2) = 0;
		RT(3,3) = 1;*/
		this->RT << 0.6, 0.7, -0.5, 1,
					-0.7, 0.7, 0, -500,
					0.5, 0, 1, 1,
					0, 0, 0, 1;

		std::cout << "The RT Matrix is: \n" << "[" << RT << "]" << std::endl;

		Eigen::Vector4f points_ref_1;
		Eigen::Vector4f points_ref_2;
		for(int i=0; i<NUM_POINTS; i++){
			points_ref_1(0) = this->pointCloud->points[i].x;
			points_ref_1(1) = this->pointCloud->points[i].y;
			points_ref_1(2) = this->pointCloud->points[i].z;
			points_ref_1(3) = 1;

			points_ref_2 = RT*points_ref_1;

			this->pointCloud_estimate->points[i].x = points_ref_2(0);
			this->pointCloud_estimate->points[i].y = points_ref_2(1);
			this->pointCloud_estimate->points[i].z = points_ref_2(2);
			this->pointCloud2->points[i].x = points_ref_2(0);
			this->pointCloud2->points[i].y = points_ref_2(1);
			this->pointCloud2->points[i].z = points_ref_2(2);
		}




		this->are_points_estimates = true;
	}

	void Model::estimateRT() {
		// Ax = b -> mx + b = y
		// points_ref_1*RT = points_ref_2

		/*Eigen::MatrixXf RT_estimate(RT_ROWS, RT_);
		Eigen::VectorXf b(_n_points);

		for (int i=0; i<_n_points; i++) {
			A(i, 0) = _points_noise[i].x;
			A(i, 1) = 1;
			b(i) = _points_noise[i].y;
		}
		Eigen::Vector2f solution = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
		std::cout << "The line solved using the SVD decomposition is:\n\t y = (" << solution(0) << ")x + " << solution(1) << std::endl;

		// y = mx + b
		for (int i=0; i<RESOLUTION; i++) {
			_points_solution[i].x = i;
			_points_solution[i].y = solution(0)*_points_solution[i].x + solution(1);
		}
		_is_solved = true;*/
		/////////////////////////////////////////////////////

		Eigen::MatrixXf points_ref_1(NUM_POINTS, 4);
		Eigen::MatrixXf points_ref_2(NUM_POINTS, 4);

		for (int i=0; i<NUM_POINTS; i++) {
			points_ref_1(i,0) = this->pointCloud->points[i].x;
			points_ref_1(i,1) = this->pointCloud->points[i].y;
			points_ref_1(i,2) = this->pointCloud->points[i].z;
			points_ref_1(i,3) = 1;
			points_ref_2(i,0) = this->pointCloud_estimate->points[i].x;
			points_ref_2(i,1) = this->pointCloud_estimate->points[i].y;
			points_ref_2(i,2) = this->pointCloud_estimate->points[i].z;
			points_ref_2(i,3) = 1;
		}

		//std::cout << "buuu: \n" << points_ref_1 << " --------- " << points_ref_2 << std::endl;

		//JacobiSVD<MatrixXf> svd(points_ref_1, Eigen::ComputeThinU | Eigen::ComputeThinV);
		//std::cout << "The estimate RT Matrix is: \n" << svd.solve(points_ref_2) << std::endl;

		Eigen::Matrix4f RT_estimate = points_ref_1.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(points_ref_2).transpose();

		std::cout << "The estimate RT Matrix is: \n" << "[" << RT_estimate << "]" << std::endl;

		Eigen::Vector4f points_ref_1_aux;
		Eigen::Vector4f points_ref_2_aux;
		for(int i=0; i<NUM_POINTS; i++){
			points_ref_2_aux(0) = this->pointCloud2->points[i].x;
			points_ref_2_aux(1) = this->pointCloud2->points[i].y;
			points_ref_2_aux(2) = this->pointCloud2->points[i].z;
			points_ref_2_aux(3) = 1;

			points_ref_1_aux = RT_estimate.inverse()*points_ref_2_aux;
			//std::cout << "asdfasdfasdfasdftrix is: \n" << points_ref_2_aux << std::endl;
			this->pointCloud_estimate->points[i].x = points_ref_1_aux(0);
			this->pointCloud_estimate->points[i].y = points_ref_1_aux(1);
			this->pointCloud_estimate->points[i].z = points_ref_1_aux(2);
		}

		for(int i=0; i<NUM_POINTS; i++){
			this->pointCloud_estimate->points[i+NUM_POINTS].x = this->pointCloud->points[i].x;
			this->pointCloud_estimate->points[i+NUM_POINTS].y = this->pointCloud->points[i].y;
			this->pointCloud_estimate->points[i+NUM_POINTS].z = this->pointCloud->points[i].z;
			this->pointCloud_estimate->points[i].r = this->pointCloud->points[i].r;
			this->pointCloud_estimate->points[i].g = this->pointCloud->points[i].g;
			this->pointCloud_estimate->points[i].b = this->pointCloud->points[i].b;
			this->pointCloud_estimate->points[i+NUM_POINTS].r = (int)0.3*this->pointCloud->points[i].r;
			this->pointCloud_estimate->points[i+NUM_POINTS].g = (int)0.3*this->pointCloud->points[i].g;
			this->pointCloud_estimate->points[i+NUM_POINTS].b = (int)0.3*this->pointCloud->points[i].b;
		}

		this->is_final = true;
	}

	void Model::addGaussianNoise() {
		std::default_random_engine generator(time(0));
		std::normal_distribution<float> distribution(0.0,10.0);
		for (int i=0; i<NUM_POINTS; i++) {
			this->pointCloud_estimate->points[i].x = this->pointCloud2->points[i].x + distribution(generator);
			this->pointCloud_estimate->points[i].y = this->pointCloud2->points[i].y + distribution(generator);
			this->pointCloud_estimate->points[i].z = this->pointCloud2->points[i].z + distribution(generator);
		}
	}
}
