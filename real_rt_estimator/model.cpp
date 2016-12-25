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


		this->_firstIteration = true;
		this->iterationCloud = 0;


		this->isChangeImageAux = true;

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

		this->RT_final << 1, 0, 0, 0,
							0, 1, 0, 0,
							0, 0, 1, 0,
							0, 0, 0, 1;
		std::cout << this->RT_final;


		// Set camera TODO: COger de la configuración de progeo
		this->pc_camera.resize(0);
		this->p_aux.x=0;
		this->p_aux.y=0;
		this->p_aux.z=0;
		this->pc_camera.push_back(this->p_aux);
		this->p_aux.x=0;
		this->p_aux.y=1000;
		this->p_aux.z=0;
		this->pc_camera.push_back(this->p_aux);
		this->p_aux.x=-200;
		this->p_aux.y=500;
		this->p_aux.z=-200;
		this->pc_camera.push_back(this->p_aux);
		this->p_aux.x=-200;
		this->p_aux.y=500;
		this->p_aux.z=200;
		this->pc_camera.push_back(this->p_aux);
		this->p_aux.x=200;
		this->p_aux.y=500;
		this->p_aux.z=200;
		this->pc_camera.push_back(this->p_aux);
		this->p_aux.x=200;
		this->p_aux.y=500;
		this->p_aux.z=-200;
		this->pc_camera.push_back(this->p_aux);

	}

	Model::~Model() {}

	// Get Methods
	std::vector<jderobot::RGBPoint> Model::get_pc() {
		return this->pc;
	}
	std::vector<jderobot::RGBPoint> Model::get_pc_converted() {
		//pthread_mutex_lock(&this->controlPcConverted);
		return this->pc_converted;
		//pthread_mutex_unlock(&this->controlPcConverted);
	}

	std::vector<jderobot::RGBPoint> Model::get_pc_camera() {
		return this->pc_camera;
	}

	std::vector<jderobot::RGBPoint> Model::get_pc_camera_converted() {
		return this->pc_camera_converted;
	}

	cv::Mat Model::getImageCameraRGB() {
		pthread_mutex_lock(&this->controlImgRGB);
		cv::Mat result;
		this->imageRGB.copyTo(result);
		pthread_mutex_unlock(&this->controlImgRGB);
		return result;
	}
	cv::Mat Model::getImageCameraRGBAux(){
		pthread_mutex_lock(&this->controlImgRGB);
		cv::Mat result;
		this->imageRGB_aux.copyTo(result);
		pthread_mutex_unlock(&this->controlImgRGB);
		return result;
	}
	cv::Mat Model::getImageCameraDEPTH() {
		pthread_mutex_lock(&this->controlImgRGB);
		cv::Mat result;
		this->imageDEPTH.copyTo(result);
		pthread_mutex_unlock(&this->controlImgRGB);
		return result;
	}
	cv::Mat Model::getImageCameraDEPTHAux() {
		pthread_mutex_lock(&this->controlImgRGB);
		cv::Mat result;
		this->imageDEPTH_aux.copyTo(result);
		pthread_mutex_unlock(&this->controlImgRGB);
		return result;
	}
	cv::Mat Model::getImageCameraMatches() {
		pthread_mutex_lock(&this->controlMatches);
		cv::Mat result;
		this->imageMatches.copyTo(result);
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
		pthread_mutex_lock(&this->controlImgRGB);
		/*if (this->isChangeImageAux) {
			this->updateImageRGBAux(this->dataRGB);
		}*/
		data.copyTo(imageRGB); // FIXME: Evitar copyTo
		//memcpy((unsigned char *) imageRGB.data ,&(data.data), imageRGB.cols*imageRGB.rows * 3);

		//this->dataRGB = data;
		pthread_mutex_unlock(&this->controlImgRGB);
	}

	void Model::updateImageRGBAux(cv::Mat data){
		//pthread_mutex_lock(&this->controlImgRGB);
		data.copyTo(imageRGB_aux);
		//memcpy((unsigned char *) imageRGB_aux.data ,&(data.data), imageRGB_aux.cols*imageRGB_aux.rows * 3);
		//pthread_mutex_unlock(&this->controlImgRGB);
	}

	void Model::updateImageDEPTH(cv::Mat data){
		pthread_mutex_lock(&this->controlImgRGB);
		/*if (this->isChangeImageAux) {
			this->updateImageDEPTHAux(this->dataDEPTH);
			this->isChangeImageAux = false;
		}*/
		data.copyTo(imageDEPTH);
		//memcpy((unsigned char *) imageDEPTH.data ,&(data.data), imageDEPTH.cols*imageDEPTH.rows * 3);
		//this->dataDEPTH = data;
		pthread_mutex_unlock(&this->controlImgRGB);
	}

	void Model::updateImageDEPTHAux(cv::Mat data){
		//pthread_mutex_lock(&this->controlImgDEPTH);
		data.copyTo(imageDEPTH_aux);
		//memcpy((unsigned char *) imageDEPTH_aux.data ,&(data.data), imageDEPTH_aux.cols*imageDEPTH_aux.rows * 3);
		//pthread_mutex_unlock(&this->controlImgDEPTH);
	}
// 	cv::Mat Model::getImage() {
// 		pthread_mutex_lock(&this->controlGui);
// 		cv::Mat result = image1.clone();
// 		pthread_mutex_unlock(&this->controlGui);
// 		return result;
// 	}


	void Model::changeImageAux() {
		this->isChangeImageAux = true;
	}

	bool Model::sortByDistance(const Model::myMatch &lhs, const Model::myMatch &rhs) {
		return ((lhs.matchDistance + lhs.matchAprox) < (rhs.matchDistance + rhs.matchAprox));
		//return ((lhs.matchDistance) < (rhs.matchDistance));
	}

	jderobot::RGBPoint Model::getPoints3D(int x, int y, cv::Mat* imgRGB, cv::Mat* imgDepth) {

		int width = imgDepth->cols;
		int height = imgDepth->rows;

		//std::cout <<  "TAMAÑOnto x! " << width << height << std::endl;
		//std::cout <<  imgDepth->data  << std::endl;
		//float module;
		//float ux,uy,uz;

		pcl::PointXYZRGBA points;

		//std::cout <<  "lol" << std::endl;
		//std::cout << imgDepth->data[3*x+imgDepth->rows*y+1] << std::endl;
		//std::cout << imgDepth->data[3*x+imgDepth->rows*y+1] << std::endl;
		//std::cout << (3*x+imgDepth->rows*y+2) << std::endl;

		int realDepthDist = ((0 << 24)|(0 << 16)|(imgDepth->data[3*x+imgDepth->rows*y+1]<<8)|(imgDepth->data[3*x+imgDepth->rows*y+2]));

		//std::cout <<  "Mejor punto x! " << x << std::endl;
		//std::cout <<  "Mejor punto y! " << y << std::endl;
		//std::cout <<  "Distandica best! " << realDepthDist << std::endl;

		/* Defining auxiliar points*/
		//HPoint2D auxPoint2DCam1;
		//HPoint3D auxPoint3DCam1;
		float d = (float)realDepthDist;
		//std::cout <<  d << std::endl;
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

		//std::cout <<  "MODULO " << modulo << std::endl;

		fmod = sqrt(1/(((camx-c1x)*(camx-c1x))+((camy-c1y)*(camy-c1y))+((camz-c1z)*(camz-c1z))));
		fx = (c1x - camx)*fmod;
		fy = (c1y - camy)*fmod;
		fz = (c1z - camz)*fmod;
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

		//std::cout <<  "coloeres dimensiones! " << p.r << ", " << p.g << ", " << p.b << std::endl;
		//std::cout <<  "punto en todas las dimensiones! " << p.x << ", " << p.y << ", " << p.z << std::endl;

		//for(int i=0; i<(3*width*width); i++) {
		//	int realDepthDist = ((0 << 24)|(0 << 16)|(imgDepth->data[i+1]<<8)|(imgDepth->data[i+2]));
		//	std::cout << realDepthDist << std::endl;
		//}
		//std::terminate();
		/*if (p.x == 0) {
			std::terminate();
		}*/

		return p;

	}

	bool Model::isBorderPoint(int x, int y, cv::Mat* imgDepth) {
		//int width = imgDepth->cols;
		int height = imgDepth->rows;

		//std::cout << x <<  "asdf" << y << std::endl;

		int d_aux = 0;
		int distance = 0;

		// 5 pixel window
		std::cout <<  "-------- Window 3x3 --------" << std::endl;
		for(int i=-1; i<2; i++) {
			for(int j=-1; j<2; j++) {
				if (((x+i) >= 0) && ((y+j) >= 0)) {
					int d = ((0 << 24)|(0 << 16)|(imgDepth->data[3*(x+i)+height*(y+j)+1]<<8)|(imgDepth->data[3*(x+i)+height*(y+j)+2]));
					std::cout << d << std::endl;
					if (d != 0) {
						if (d_aux != 0) {
							distance += abs(d - d_aux);
						}
						d_aux = d;
					} else {
						distance += 1000;
					}
				} else {
					//std::cout <<  "borde de la imagen" << std::endl;
				}
			}
		}
		std::cout <<  "Typical deviation ¿?: " << distance << std::endl;
 		std::cout <<  "-------------------------" << std::endl;
		if (distance < 3000) {
			return false;
		} else {
			return true;
		}
	}

	int Model::doSiftAndGetPoints() {

		// GetTems
		/*pthread_mutex_lock(&this->controlImgRGB);
			this->imageRGB.copyTo(this->temp_imageRGB);
			this->imageRGB_aux.copyTo(this->temp_imageRGB_aux);
			this->imageDEPTH.copyTo(this->temp_imageDEPTH);
			this->imageDEPTH_aux.copyTo(this->temp_imageDEPTH_aux);
		pthread_mutex_unlock(&this->controlImgRGB);*/


		if (!_firstIteration) {
			std::cout <<  "SEGUNDA 22222 ITERACIÓN" << std::endl;

			// Local vars
			std::vector<cv::KeyPoint> keypoints1, keypoints2;
			cv::Mat descriptors_n, descriptors_n_aux;
			cv::Mat inputGray1, inputGray2;
			cv::SiftDescriptorExtractor extractor;

			//inputGray1 = cv::imread( this->imageRGB, CV_LOAD_IMAGE_GRAYSCALE );
			//inputGray2 = cv::imread( this->imageRGB_aux, CV_LOAD_IMAGE_GRAYSCALE );
			cv::cvtColor(this->imageRGB, inputGray1, CV_BGR2GRAY);
			cv::cvtColor(this->imageRGB_aux, inputGray2, CV_BGR2GRAY);

			// SURF Detector
		  //int minHessian = 400;
		  //cv::SurfFeatureDetector surfdet(minHessian);

			// Sift
			cv::SiftFeatureDetector siftdet;

			siftdet.detect(inputGray1, keypoints1);
			siftdet.detect(inputGray2, keypoints2);
			extractor.compute(inputGray1, keypoints1, descriptors_n);
			extractor.compute(inputGray2, keypoints2, descriptors_n_aux);

			//descriptors_n.copyTo(descriptors_n_aux);

			// matcher
			cv::BruteForceMatcher<cv::L2<float> > matcher;
			//cv::FlannBasedMatcher matcher;
			std::vector<cv::DMatch> matches;
			matcher.match(descriptors_n, descriptors_n_aux, matches);
			//matcher.knnMatch(descriptors_n, descriptors_n_aux, 20, matches);

			this->sift_points = matches.size();

			//std::cout <<  "KEYPOINTS1 --> " << keypoints1.size() << std::endl;
			//std::cout <<  "KEYPOINTS2 --> " << keypoints2.size() << std::endl;

			if (matches.size() != 0) { // FIXME
			std::cout <<  "MATCHES SIZE!!!!! !=0 --> " << matches.size() << std::endl;

				//Model::myMatch[matches.size()] myMatches = { };

				//For save the matches
				this->myMatches.resize(matches.size());
				this->pc.resize(0);

					for(int i=0; i<((int)matches.size()); i++){

						////////////////////////////////////////////////////////////////////////////////////////////////
						// Save the point cloud...

						this->pc.push_back(getPoints3D(keypoints1[i].pt.x, keypoints1[i].pt.y, &this->imageRGB, &this->imageDEPTH));

						////////////////////////////////////////////////////////////////////////////////////////////////
						std::cout <<  "match.distance: " << matches[i].distance << std::endl;
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

				//std::cout <<  "matchNUM " << this->myMatches[0].matchNum << std::endl;
				//std::cout <<  "best match " << bestMatch << std::endl;
				//std::cout <<  "best match 1" << keypoints1[matches[bestMatch].queryIdx].pt.x << std::endl;
				//std::cout <<  "best match 2" << keypoints1[matches[bestMatch].queryIdx].pt.y << std::endl;

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

				std::cout <<  "WEEEEEEEEEEEEEEEEEEEEE---------------->" << numBestPoints << std::endl;

				int count = 0;
				for (int i=0; i<numBestPoints; i++) {

					int bests_m = this->myMatches[i].matchNum;
					x_1 = (int)(keypoints1[matches[bests_m].queryIdx].pt.x);//+0.5f);
					y_1 = (int)(keypoints1[matches[bests_m].queryIdx].pt.y);//+0.5f);
					x_2 = (int)(keypoints2[matches[bests_m].trainIdx].pt.x);//+0.5f);
					y_2 = (int)(keypoints2[matches[bests_m].trainIdx].pt.y);//+0.5f);

					//std::cout <<  "Entramos funcion" << std::endl;
					//std::cout <<  x_1 << ", " << y_1 << ", " << x_2 << ", " << y_2 << std::endl;
					jderobot::RGBPoint p1 = getPoints3D(x_1, y_1, &this->imageRGB, &this->imageDEPTH);
					jderobot::RGBPoint p2 = getPoints3D(x_2, y_2, &this->imageRGB_aux, &this->imageDEPTH_aux);

					if (!isBorderPoint(x_1, y_1, &this->imageDEPTH) && !isBorderPoint(x_2, y_2, &this->imageDEPTH_aux)) {
						if (p1.z != 0 && p2.z != 0) {
							std::cout <<  "Puntos calculados ----------" <<  x_1 << ", " << y_1 << ", " << x_2 << ", " << y_2 << std::endl;
							std::cout <<  p1.x << ", " << p1.y << ", " <<  p1.z << std::endl;
							std::cout <<  p2.x << ", " << p2.y << ", " <<  p2.z << std::endl;
							this->v_rgbp.push_back(p1);
							this->v_rgbp_aux.push_back(p2);
							count++;
						}
					} else {
						std::cout << "es border point" << std::endl;
					}


					//this->v_rgbp.push_back(getPoints3D(x_1, y_1, &this->temp_imageRGB, &this->temp_imageDEPTH));
					//std::cout <<  "Entramos funcion2" << std::endl;
					//this->v_rgbp_aux.push_back(getPoints3D(x_2, y_2, &this->temp_imageRGB_aux, &this->temp_imageDEPTH_aux));


					////////////////////////////////////////////////////////
					// Draw the chosen points of interest
					//std::vector<cv::KeyPoint> keypoints1_aux, keypoints2_aux;
					//keypoints1_aux.resize(0);
					//keypoints2_aux.resize(0);
					matches_aux.push_back(matches[bests_m]);
				}

				cv::Mat imgMatches;
				cv::drawMatches(this->imageRGB, keypoints1, this->imageRGB_aux, keypoints2, matches_aux, imgMatches);
				this->imageMatches = imgMatches;

				/////////////////////////////////////////////////////////////

				//std::cout << "tamaño puntos: " << v_rgbp.size() << " y " << v_rgbp_aux.size() << std::endl;
				//this->pc->points = v_rgbp;
				//this->pc_aux->points = v_rgbp_aux;

				pthread_mutex_unlock(&this->controlPcConverted);
				if (count < 5) {
					return 0;
				} else {
					return 1;
				}
			} else {
				return 0;
			}
		} else { // First iteration
			std::cout <<  "PRIMERA ITERACIÓN 1111111111" << std::endl;
			// FIXME: LOCK
			this->imageRGB.copyTo(this->imageRGB_aux);
			this->imageDEPTH.copyTo(this->imageDEPTH_aux);
			_firstIteration = false;
			return 0;
		}
	}

	void Model::estimateRT() {
		pthread_mutex_lock(&this->controlPcConverted);








		// TODO: Comprobar número mínimo de puntos!!!!!!!!!


		int num_points_for_RT = v_rgbp.size();
		std::cout << "The points number for RT calculation is: \n" <<  (num_points_for_RT-1) << std::endl;

		Eigen::MatrixXf points_ref_1(num_points_for_RT, 4);
		//Eigen::MatrixXf points_ref_2(num_points_for_RT, 4);
		Eigen::Vector4f points_ref_2;
		Eigen::Vector4f points_ref_2_ant;
		Eigen::MatrixXf points_ref_2_world(num_points_for_RT, 4);
		std::cout << "The FINAL RT Matrix is: \n" << "[" << this->RT_final << "]" << std::endl;
		for (int i=0; i<num_points_for_RT; i++) {
			points_ref_1(i,0) = v_rgbp[i].x;
			points_ref_1(i,1) = v_rgbp[i].y;
			points_ref_1(i,2) = v_rgbp[i].z;
			points_ref_1(i,3) = 1;
			points_ref_2(0) = v_rgbp_aux[i].x;
			points_ref_2(1) = v_rgbp_aux[i].y;
			points_ref_2(2) = v_rgbp_aux[i].z;
			points_ref_2(3) = 1;
			points_ref_2_ant = this->RT_final.inverse()*points_ref_2;
			points_ref_2_world(i,0) = points_ref_2_ant(0);
			points_ref_2_world(i,1) = points_ref_2_ant(1);
			points_ref_2_world(i,2) = points_ref_2_ant(2);
			points_ref_2_world(i,3) = 1;
			std::cout << "v_rgbp_aux[i]" << std::endl;
			std::cout << v_rgbp[i].x << ", " << v_rgbp[i].y << ", " << v_rgbp[i].z << std::endl;
			std::cout << v_rgbp_aux[i].x << ", " << v_rgbp_aux[i].y << ", " << v_rgbp_aux[i].z << std::endl;
			std::cout << "Magia y... " << std::endl;
			std::cout << points_ref_2_ant(0) << ", " << points_ref_2_ant(1) << ", " << points_ref_2_ant(2) << std::endl;
		}


		/*if (this->first) {
			this->RT_final << 1, 0, 0, 0,
								0, 1, 0, 0,
								0, 0, 1, 0,
								0, 0, 0, 1;
			std::cout << this->RT_final;

			Eigen::Vector4f points_ref_1_aux;
			Eigen::Vector4f points_ref_2_aux;

			this->pc_converted.resize(num_points_for_RT);

			for(int i=0; i<num_points_for_RT; i++){
				points_ref_2_aux(0) = this->pc[i].x;
				points_ref_2_aux(1) = this->pc[i].y;
				points_ref_2_aux(2) = this->pc[i].z;
				points_ref_2_aux(3) = 1;

				Eigen::Vector4f points_ref_1_aux;
				Eigen::Vector4f points_ref_2_aux;

				points_ref_1_aux = RT_final.inverse()*points_ref_2_aux;
				//points_ref_1_aux = 2*points_ref_2_aux;
				//std::cout << "asdfasdfasdfasdftrix is: \n" << points_ref_2_aux << std::endl;
				this->pc_converted[i].x = points_ref_1_aux(0);
				this->pc_converted[i].y = points_ref_1_aux(1);
				this->pc_converted[i].z = points_ref_1_aux(2);
				this->pc_converted[i].r = this->pc[i].r;
				this->pc_converted[i].g = this->pc[i].g;
				this->pc_converted[i].b = this->pc[i].b;
			}
			this->first = false;
		} else {*/


			//std::cout << "buuu: \n" << points_ref_1 << " --------- " << points_ref_2 << std::endl;

			//JacobiSVD<MatrixXf> svd(points_ref_1, Eigen::ComputeThinU | Eigen::ComputeThinV);
			//std::cout << "The estimate RT Matrix is: \n" << svd.solve(points_ref_2) << std::endl;


			Eigen::Matrix4f RT_estimate = points_ref_1.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(points_ref_2_world).transpose();

			//Eigen::Matrix4f RT_estimate = RT_final;

			std::cout << "The estimate RT Matrix is: \n" << "[" << RT_estimate << "]" << std::endl;

			//this->RT_final = this->RT_final*RT_estimate;

			//std::cout << "The FINAL RT Matrix is: \n" << "[" << RT_final << "]" << std::endl;

			Eigen::Vector4f points_ref_1_aux;
			Eigen::Vector4f points_ref_2_aux;

			this->pc_converted.resize(num_points_for_RT);

			for(int i=0; i<num_points_for_RT; i++){
				points_ref_2_aux(0) = points_ref_2_world(i,0);
				points_ref_2_aux(1) = points_ref_2_world(i,1);
				points_ref_2_aux(2) = points_ref_2_world(i,2);
				points_ref_2_aux(3) = 1;

				points_ref_1_aux = RT_estimate.inverse()*points_ref_2_aux;
				//points_ref_1_aux = RT_final.inverse()*points_ref_2_aux;
				//points_ref_1_aux = RT_final*points_ref_2_aux;
				//std::cout << "asdfasdfasdfasdftrix is: \n" << points_ref_2_aux << std::endl;
				this->pc_converted[i].x = points_ref_1_aux(0);
				this->pc_converted[i].y = points_ref_1_aux(1);
				this->pc_converted[i].z = points_ref_1_aux(2);
				//this->pc_converted[i].r = this->pc[i].r;
				//this->pc_converted[i].g = this->pc[i].g;
				//this->pc_converted[i].b = this->pc[i].b;
				std::cout << "ANTES DE CONVERTIR: \n" << points_ref_2_world(i,0) << ", " << points_ref_2_world(i,1) << ", " << points_ref_2_world(i,2) << std::endl;
				std::cout << "FINAL >>>>>>>: \n" << this->pc_converted[i].x << ", " << this->pc_converted[i].y << ", " << this->pc_converted[i].z << std::endl;
				if (this->iterationCloud == 0) {
					this->pc_converted[i].r = 255;
					this->pc_converted[i].g = 0;
					this->pc_converted[i].b = 0;
				}
				if (this->iterationCloud == 1) {
					this->pc_converted[i].r = 0;
					this->pc_converted[i].g = 255;
					this->pc_converted[i].b = 0;
				}
				if (this->iterationCloud == 2) {
					this->pc_converted[i].r = 0;
					this->pc_converted[i].g = 0;
					this->pc_converted[i].b = 255;
				}
				if (this->iterationCloud == 3) {
					this->pc_converted[i].r = 255;
					this->pc_converted[i].g = 255;
					this->pc_converted[i].b = 0;
				}
				if (this->iterationCloud == 4) {
					this->pc_converted[i].r = 0;
					this->pc_converted[i].g = 255;
					this->pc_converted[i].b = 255;
				}
				if (this->iterationCloud == 5) {
					this->pc_converted[i].r = 255;
					this->pc_converted[i].g = 0;
					this->pc_converted[i].b = 255;
				}
				if (this->iterationCloud == 6) {
					this->pc_converted[i].r = 255;
					this->pc_converted[i].g = 255;
					this->pc_converted[i].b = 255;
				}

			}
			this->iterationCloud++;

			// Camera camera converted
			moveCamera(RT_estimate);
			std::cout << "ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ: \n" << std::endl;
			for(int i=0; i<16; i++) {
				this->RT_final(i) = RT_estimate(i);
			}
		//}

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
		pthread_mutex_unlock(&this->controlPcConverted);

		// Save image n to n-1 if sucess estimate
		// FIXME: LOCK
		this->imageRGB.copyTo(this->imageRGB_aux);
		this->imageDEPTH.copyTo(this->imageDEPTH_aux);
	}

	bool Model::isEstimated() {
		return (this->iterationCloud > 0);
	}

	void Model::RotateXAxis() {
		Eigen::Matrix4f RT_estimate;
		RT_estimate << 1, 0, 			 0, 			 0,
									 0, cos(30), -sin(30), 0,
									 0, sin(30), cos(30),  0,
									 0, 0, 			 0, 			 1;
		moveCamera(RT_estimate);
	}

	void Model::RotateYAxis() {
		Eigen::Matrix4f RT_estimate;
		RT_estimate << cos(30),  0, sin(30), 0,
									 0, 			 1, 0, 			 0,
									 -sin(30), 0, cos(30), 0,
									 0, 			 0, 0, 			 1;
		moveCamera(RT_estimate);
	}

	void Model::moveDownRT() {
		Eigen::Matrix4f RT_estimate;
		RT_estimate << 1, 0, 0, -10,
									 0, 1, 0, 0,
									 0, 0, 1, -10,
									 0, 0, 0, 1;
	  moveCamera(RT_estimate);
	}

	void Model::moveUpRT() {
		Eigen::Matrix4f RT_estimate;
		RT_estimate << 1, 0, 0, 10,
									 0, 1, 0, 0,
									 0, 0, 1, 10,
									 0, 0, 0, 1;
		moveCamera(RT_estimate);
	}

	void Model::moveCamera(Eigen::Matrix4f RT_estimate) {
		//std::cout << "The estimate RT Matrix is: \n" << "[" << RT_estimate << "]" << std::endl;

		//this->RT_final = this->RT_final*RT_estimate;

		//std::cout << "The FINAL RT Matrix is: \n" << "[" << RT_final << "]" << std::endl;

		Eigen::Vector4f points_ref_1_aux;
		Eigen::Vector4f points_ref_2_aux;

		// Camera camera converted
		this->pc_camera_converted.resize(pc_camera.size());
		for(int i=0; i<pc_camera.size(); i++) {
			points_ref_2_aux(0) = this->pc_camera[i].x;
			points_ref_2_aux(1) = this->pc_camera[i].y;
			points_ref_2_aux(2) = this->pc_camera[i].z;
			points_ref_2_aux(3) = 1;

			points_ref_1_aux = RT_estimate.inverse()*points_ref_2_aux;

			this->pc_camera_converted[i].x = points_ref_1_aux(0);
			this->pc_camera_converted[i].y = points_ref_1_aux(1);
			this->pc_camera_converted[i].z = points_ref_1_aux(2);
		}
	}
}
