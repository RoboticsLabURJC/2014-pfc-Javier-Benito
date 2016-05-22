#include "model.h"

#define w 400

//Shared memory class storage of shared resources of the component

namespace real_rt_estimator {

	Model::Model() {
		//pthread_mutex_t controlImgRGB = PTHREAD_MUTEX_INITIALIZER;
		//pthread_mutex_t controlImgDEPTH = PTHREAD_MUTEX_INITIALIZER;

		/*xmlReader(&TPHcamaraDepth, "camDepth");
		TPHcamaraDepth.position.H=1;
		TPHcamaraDepth.foa.H=1;
		update_camera_matrix(&TPHcamaraDepth);
		display_camerainfo(TPHcamaraDepth);*/
		this->RT_final << 1, 0, 0, 0,
							0, 1, 0, 0,
							0, 0, 1, 0,
							0, 0, 0, 1;
		std::cout << this->RT_final;


		this->mypro= new real_rt_estimator::myprogeo();
		char c_null[0];
		this->mypro->load_cam(c_null,0,320, 240);

		//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc2(new pcl::PointCloud<pcl::PointXYZRGBA>);
		//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc_aux2(new pcl::PointCloud<pcl::PointXYZRGBA>);
		//this->pc = pc2;
		//this->pc_converted = pc_aux2;

		this->sift_points = 0;

		//this->pc->points.resize(0);


		//this->pc_converted->points.resize(0);
	}

	Model::~Model() {}

	std::vector<jderobot::RGBPoint> Model::get_pc() {
		return this->pc;
	}

	std::vector<jderobot::RGBPoint> Model::get_pc_converted() {
		return this->pc_converted;
	}

	cv::Mat Model::getImageCameraRGB() {
		pthread_mutex_lock(&this->controlImgRGB);
		cv::Mat result = this->imageRGB.clone();
		pthread_mutex_unlock(&this->controlImgRGB);
		return result;
	}
	cv::Mat Model::getImageCameraRGBAux(){
		//pthread_mutex_lock(&this->controlImgRGB);
		cv::Mat result = this->imageRGB_aux.clone();
		//pthread_mutex_unlock(&this->controlImgRGB);
		return result;
	}
	cv::Mat Model::getImageCameraDEPTH() {
		pthread_mutex_lock(&this->controlImgRGB);
		cv::Mat result = this->imageDEPTH.clone();
		pthread_mutex_unlock(&this->controlImgRGB);
		return result;
	}
	cv::Mat Model::getImageCameraDEPTHAux() {
		//pthread_mutex_lock(&this->controlImgDEPTH);
		cv::Mat result = this->imageDEPTH_aux.clone();
		//pthread_mutex_unlock(&this->controlImgDEPTH);
		return result;
	}
	cv::Mat Model::getImageCameraMatches() {
		pthread_mutex_lock(&this->controlMatches);
		cv::Mat result = this->imageMatches.clone();
		pthread_mutex_unlock(&this->controlMatches);
		return result;
	}

	void Model::createImageRGB(cv::Mat data) {
		this->dataRGB = data;
		pthread_mutex_lock(&this->controlImgRGB);
		imageRGB.create(data.rows, data.cols, CV_8UC3);
		pthread_mutex_unlock(&this->controlImgRGB);
		imageRGB_aux.create(data.rows, data.cols, CV_8UC3);
	}

	void Model::createImageRGBAux(cv::Mat data) {
		//pthread_mutex_lock(&this->controlImgRGB);
		imageRGB_aux.create(data.rows, data.cols, CV_8UC3);
		//pthread_mutex_unlock(&this->controlImgRGB);
	}

	void Model::createImageDEPTH(cv::Mat data) {
		this->dataDEPTH = data;
		pthread_mutex_lock(&this->controlImgRGB);
		imageDEPTH.create(data.rows, data.cols, CV_8UC3);
		pthread_mutex_unlock(&this->controlImgRGB);
		imageDEPTH_aux.create(data.rows, data.cols, CV_8UC3);
	}

	void Model::createImageDEPTHAux(cv::Mat data) {
		//pthread_mutex_lock(&this->controlImgDEPTH);
		imageDEPTH_aux.create(data.rows, data.cols, CV_8UC3);
		//pthread_mutex_unlock(&this->controlImgDEPTH);
	}

    /*void Model::createEmptyImageRGB() {
		pthread_mutex_lock(&this->controlGui);
		imageRGB.create(w, w, CV_8UC3);
		pthread_mutex_unlock(&this->controlGui);
    }

	void Model::createEmptyImageRGB() {
		pthread_mutex_lock(&this->controlGui2);
		imageDEPTH.create(w, w, CV_8UC3);
		pthread_mutex_unlock(&this->controlGui2);
    }*/

	void Model::updateImageRGB(cv::Mat data){
		this->updateImageRGBAux(this->dataRGB);
		pthread_mutex_lock(&this->controlImgRGB);
		memcpy((unsigned char *) imageRGB.data ,&(data.data), imageRGB.cols*imageRGB.rows * 3);
		pthread_mutex_unlock(&this->controlImgRGB);
		this->dataRGB = data;
	}

	void Model::updateImageRGBAux(cv::Mat data){
		//pthread_mutex_lock(&this->controlImgRGB);
		memcpy((unsigned char *) imageRGB_aux.data ,&(data.data), imageRGB_aux.cols*imageRGB_aux.rows * 3);
		//pthread_mutex_unlock(&this->controlImgRGB);
	}

	void Model::updateImageDEPTH(cv::Mat data){
		this->updateImageDEPTHAux(this->dataDEPTH);
		pthread_mutex_lock(&this->controlImgRGB);
		memcpy((unsigned char *) imageDEPTH.data ,&(data.data), imageDEPTH.cols*imageDEPTH.rows * 3);
		pthread_mutex_unlock(&this->controlImgRGB);
		this->dataDEPTH = data;
	}

	void Model::updateImageDEPTHAux(cv::Mat data){
		//pthread_mutex_lock(&this->controlImgDEPTH);
		memcpy((unsigned char *) imageDEPTH_aux.data ,&(data.data), imageDEPTH_aux.cols*imageDEPTH_aux.rows * 3);
		//pthread_mutex_unlock(&this->controlImgDEPTH);
	}
// 	cv::Mat Model::getImage() {
// 		pthread_mutex_lock(&this->controlGui);
// 		cv::Mat result = image1.clone();
// 		pthread_mutex_unlock(&this->controlGui);
// 		return result;
// 	}

	bool Model::sortByDistance(const Model::myMatch &lhs, const Model::myMatch &rhs) {
		return ((lhs.matchDistance + lhs.matchAprox) < (rhs.matchDistance + rhs.matchAprox));
	}

	jderobot::RGBPoint Model::getPoints3D(int x, int y, cv::Mat* imgRGB, cv::Mat* imgDepth) {

		int width = imgDepth->cols;
		int height = imgDepth->rows;

		std::cout <<  "TAMAÑOnto x! " << width << height << std::endl;
		//float module;
		//float ux,uy,uz;

		pcl::PointXYZRGBA points;

		int realDepthDist = ((0 << 24)|(0 << 16)|(imgDepth->data[3*x+imgDepth->rows*y+1]<<8)|(imgDepth->data[3*x+imgDepth->rows*y+2]));

		std::cout <<  "Mejor punto x! " << x << std::endl;
		std::cout <<  "Mejor punto y! " << y << std::endl;
		std::cout <<  "Distandica best! " << realDepthDist << std::endl;

		/* Defining auxiliar points*/
		//HPoint2D auxPoint2DCam1;
		//HPoint3D auxPoint3DCam1;

		float d = (float)realDepthDist;
		float xp,yp,zp,camx,camy,camz;
		mypro->mybackproject(x, y, &xp, &yp, &zp, &camx, &camy, &camz,0);
		//vector unitario
		float modulo;
		float c1x, c1y, c1z;
		float fx,fy,fz;
		float fmod;
		float t;
		float Fx,Fy,Fz;
		float ux,uy,uz;

		modulo = sqrt(1/(((camx-xp)*(camx-xp))+((camy-yp)*(camy-yp))+((camz-zp)*(camz-zp))));
						mypro->mygetcamerafoa(&c1x, &c1y, &c1z, 0);

		fmod = sqrt(1/(((camx-c1x)*(camx-c1x))+((camy-c1y)*(camy-c1y))+((camz-c1z)*(camz-c1z))));
		fx = (c1x - camx)*fmod;
		fy = (c1y - camy)*fmod;
		fz = (c1z - camz) * fmod;
		ux = (xp-camx)*modulo;
		uy = (yp-camy)*modulo;
		uz = (zp-camz)*modulo;

		Fx= d*fx + camx;
		Fy= d*fy + camy;
		Fz= d*fz + camz;

		// Calculamos el punto real
		t = (-(fx*camx) + (fx*Fx) - (fy*camy) + (fy*Fy) - (fz*camz) + (fz*Fz))/((fx*ux) + (fy*uy) + (fz*uz));
		jderobot::RGBPoint p;

		p.r=(int)imgRGB->data[3*x+imgRGB->rows*y];
		p.g=(int)imgRGB->data[3*x+imgRGB->rows*y+1];
		p.b=(int)imgRGB->data[3*x+imgRGB->rows*y+2];
		p.x=t*ux+camx;
		p.y=t*uy+camy;
		p.z=t*uz+camz;

		std::cout <<  "punto en todas las dimensiones! " << p.x << ", " << p.y << ", " << p.z << std::endl;

		return p;

	}

	int Model::doSiftAndGetPoints() {

		// GetTems
		pthread_mutex_lock(&this->controlImgRGB);
			this->temp_imageRGB = this->imageRGB.clone();
		pthread_mutex_unlock(&this->controlImgRGB);
		this->temp_imageRGB_aux = this->imageRGB_aux.clone();

		pthread_mutex_lock(&this->controlImgRGB);
			this->temp_imageDEPTH = this->imageDEPTH.clone();
		pthread_mutex_unlock(&this->controlImgRGB);
		this->temp_imageDEPTH_aux = this->imageDEPTH_aux.clone();

		//Función doSIFT
		std::vector<cv::KeyPoint> keypoints1, keypoints2;
		cv::Mat descriptors1, descriptors2;
		cv::SiftDescriptorExtractor extractor;
		cv::Mat inputGray1, inputGray2;

		cv::cvtColor(this->temp_imageRGB, inputGray1, CV_BGR2GRAY);
		cv::cvtColor(this->temp_imageRGB_aux, inputGray2, CV_BGR2GRAY);

		cv::SiftFeatureDetector siftdet;
		siftdet.detect(inputGray1, keypoints1);
		siftdet.detect(inputGray2, keypoints2);

		extractor.compute(inputGray1, keypoints1, descriptors1);
		extractor.compute(inputGray2, keypoints2, descriptors2);

		cv::BruteForceMatcher<cv::L2<float> > matcher;
		std::vector<cv::DMatch> matches;

		matcher.match(descriptors1, descriptors2, matches);

		//cv::Mat imgMatches;
		//cv::drawMatches(this->temp_imageRGB, keypoints1, this->temp_imageRGB_aux, keypoints2, matches, imgMatches);
		//this->imageMatches = imgMatches;

		/*pthread_mutex_lock(&this->controlImgRGB);
		cv::drawKeypoints(this->imageRGB, keypoints1, this->imageRGB);
		pthread_mutex_unlock(&this->controlImgRGB);

		pthread_mutex_lock(&this->controlImgRGB);
		cv::drawKeypoints(this->imageRGB_aux, keypoints2, this->imageRGB_aux);
		pthread_mutex_unlock(&this->controlImgRGB);*/

		if (matches.size() != 0) {
			/*std::cout <<  "puntos de la kp1 " << keypoints1.size() << std::endl;
			std::cout <<  "puntos de la kp2 " << keypoints2.size() << std::endl;
			std::cout <<  "puntos de los maches " << matches.size() << std::endl;*/

			//Model::myMatch[matches.size()] myMatches = { };

			//For save the matches
			this->myMatches.resize(matches.size());
			this->pc.resize(0);

				for(int i=0; i<((int)matches.size()); i++){

					////////////////////////////////////////////////////////////////////////////////////////////////
					// Save the point cloud...
					this->pc.push_back(getPoints3D(keypoints1[i].pt.x, keypoints1[i].pt.y, &this->temp_imageRGB, &this->temp_imageDEPTH));
					////////////////////////////////////////////////////////////////////////////////////////////////

					if (matches[i].distance < 100) {
						/*int bu1 = ((int) matches[i].queryIdx);
						int bu2 = ((int) matches[i].trainIdx);

						std::cout <<  "" << std::endl;
						std::cout <<  "[Match: " << i <<  "]" << std::endl;
						std::cout <<  "match.distance: " << matches[i].distance << std::endl;
						std::cout << " Img point 1:" << keypoints1[bu1].pt << std::endl;
						std::cout << " Img point 2:" << keypoints2[bu2].pt << std::endl;
						float resul = (abs(keypoints1[bu1].pt.x - keypoints2[bu2].pt.x) + abs(keypoints1[bu1].pt.y - keypoints2[bu2].pt.y));
						std::cout << "My distance (aproximity): " << resul << std::endl;*/
						//std::cout <<  "imgIdx " << matches[i].imgIdx << std::endl;
						//std::cout <<  "queryIdx " << matches[i].queryIdx << std::endl;
						//std::cout <<  "trainIdx " << matches[i].trainIdx << std::endl;
					}
					this->myMatches[i].matchNum = i;
					this->myMatches[i].matchDistance = matches[i].distance;
					this->myMatches[i].matchAprox = (abs(keypoints1[((int) matches[i].queryIdx)].pt.x - keypoints2[((int) matches[i].trainIdx)].pt.x) + abs(keypoints1[((int) matches[i].queryIdx)].pt.y - keypoints2[((int) matches[i].trainIdx)].pt.y));
				}



			//Sort new match vector, best first
			std::sort(this->myMatches.begin(), this->myMatches.end(), sortByDistance);

			/*for(int i=0; i<this->myMatches.size(); i++){
				std::cout <<  i << std::endl;
				std::cout <<  "matchNUM " << this->myMatches[i].matchNum << std::endl;
				std::cout <<  "matchDistance " << this->myMatches[i].matchDistance << std::endl;
				std::cout <<  "matchAprox " << this->myMatches[i].matchAprox << std::endl;
			}*/






			int bestMatch = this->myMatches[0].matchNum;

			std::cout <<  "matchNUM " << this->myMatches[0].matchNum << std::endl;
			std::cout <<  "best match " << bestMatch << std::endl;
			std::cout <<  "best match 1" << keypoints1[matches[bestMatch].queryIdx].pt.x << std::endl;
			std::cout <<  "best match 2" << keypoints1[matches[bestMatch].queryIdx].pt.y << std::endl;

			// Save X best points to Points Cloud
			int numBestPoints = N_ESTIMATOR_POINTS;
			if ((int)matches.size() < numBestPoints) {
				numBestPoints = matches.size();
			}
			int x_1, y_1, x_2, y_2;

			this->v_rgbp.resize(0);
			this->v_rgbp_aux.resize(0);

			std::vector<cv::DMatch> matches_aux;
			matches_aux.resize(0);

			for (int i=0; i<numBestPoints; i++) {

				int bests_m = this->myMatches[i].matchNum;
				x_1 = (int)(keypoints1[matches[bests_m].queryIdx].pt.x+0.5f);
				y_1 = (int)(keypoints1[matches[bests_m].queryIdx].pt.y+0.5f);
				x_2 = (int)(keypoints2[matches[bests_m].trainIdx].pt.x+0.5f);
				y_2 = (int)(keypoints2[matches[bests_m].trainIdx].pt.y+0.5f);
				this->v_rgbp.push_back(getPoints3D(x_1, y_1, &this->temp_imageRGB, &this->temp_imageDEPTH));
				this->v_rgbp_aux.push_back(getPoints3D(x_2, y_2, &this->temp_imageRGB_aux, &this->temp_imageDEPTH_aux));

				////////////////////////////////////////////////////////
				// Draw the chosen points of interest
				//std::vector<cv::KeyPoint> keypoints1_aux, keypoints2_aux;
				//keypoints1_aux.resize(0);
				//keypoints2_aux.resize(0);

				matches_aux.push_back(matches[bests_m]);
			}
			cv::Mat imgMatches;
			cv::drawMatches(this->temp_imageRGB, keypoints1, this->temp_imageRGB_aux, keypoints2, matches_aux, imgMatches);
			this->imageMatches = imgMatches;
			/////////////////////////////////////////////////////////////

			std::cout << "tamaño puntos: " << v_rgbp.size() << " y " << v_rgbp_aux.size() << std::endl;
			//this->pc->points = v_rgbp;
			//this->pc_aux->points = v_rgbp_aux;

			return 1;
		} else {
			return 0;
		}
	}

	void Model::estimateRT() {

		// TODO: Comprobar número mínimo de puntos!!!!!!!!!

		int num_points_for_RT = v_rgbp.size();
		Eigen::MatrixXf points_ref_1(num_points_for_RT, 4);
		Eigen::MatrixXf points_ref_2(num_points_for_RT, 4);

		for (int i=0; i<num_points_for_RT; i++) {
			points_ref_1(i,0) = v_rgbp[i].x;
			points_ref_1(i,1) = v_rgbp[i].y;
			points_ref_1(i,2) = v_rgbp[i].z;
			points_ref_1(i,3) = 1;
			points_ref_2(i,0) = v_rgbp_aux[i].x;
			points_ref_2(i,1) = v_rgbp_aux[i].y;
			points_ref_2(i,2) = v_rgbp_aux[i].z;
			points_ref_2(i,3) = 1;
		}

		//std::cout << "buuu: \n" << points_ref_1 << " --------- " << points_ref_2 << std::endl;

		//JacobiSVD<MatrixXf> svd(points_ref_1, Eigen::ComputeThinU | Eigen::ComputeThinV);
		//std::cout << "The estimate RT Matrix is: \n" << svd.solve(points_ref_2) << std::endl;

		Eigen::Matrix4f RT_estimate = points_ref_1.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(points_ref_2).transpose();

		std::cout << "The estimate RT Matrix is: \n" << "[" << RT_estimate << "]" << std::endl;
		this->RT_final = RT_estimate * this->RT_final;

		std::cout << "The FINAL RT Matrix is: \n" << "[" << RT_final << "]" << std::endl;

		Eigen::Vector4f points_ref_1_aux;
		Eigen::Vector4f points_ref_2_aux;

		this->pc_converted.resize(num_points_for_RT);

		for(int i=0; i<num_points_for_RT; i++){
			points_ref_2_aux(0) = this->pc[i].x;
			points_ref_2_aux(1) = this->pc[i].y;
			points_ref_2_aux(2) = this->pc[i].z;
			points_ref_2_aux(3) = 1;

			points_ref_1_aux = RT_final.inverse()*points_ref_2_aux;
			//std::cout << "asdfasdfasdfasdftrix is: \n" << points_ref_2_aux << std::endl;
			this->pc_converted[i].x = points_ref_1_aux(0);
			this->pc_converted[i].y = points_ref_1_aux(1);
			this->pc_converted[i].z = points_ref_1_aux(2);
			this->pc_converted[i].r = this->pc[i].r;
			this->pc_converted[i].g = this->pc[i].g;
			this->pc_converted[i].b = this->pc[i].b;
		}

		/*
		for(int i=0; i<num_points_for_RT; i++){
			this->pointCloud_estimate->points[i+num_points_for_RT].x = this->pointCloud->points[i].x;
			this->pointCloud_estimate->points[i+num_points_for_RT].y = this->pointCloud->points[i].y;
			this->pointCloud_estimate->points[i+num_points_for_RT].z = this->pointCloud->points[i].z;
			this->pointCloud_estimate->points[i].r = this->pointCloud->points[i].r;
			this->pointCloud_estimate->points[i].g = this->pointCloud->points[i].g;
			this->pointCloud_estimate->points[i].b = this->pointCloud->points[i].b;
			this->pointCloud_estimate->points[i+num_points_for_RT].r = (int)0.3*this->pointCloud->points[i].r;
			this->pointCloud_estimate->points[i+num_points_for_RT].g = (int)0.3*this->pointCloud->points[i].g;
			this->pointCloud_estimate->points[i+num_points_for_RT].b = (int)0.3*this->pointCloud->points[i].b;
		}*/

		//this->is_final = true;
	}





}
