#ifndef REAL_RT_ESTIMATOR_CONTROL_H
#define REAL_RT_ESTIMATOR_CONTROL_H

//ICE
#include <iostream>
#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>

#include "../model.h"
//#include "../parallelIce/cameraClient.h"

//INTERFACES
//#include <jderobot/camera.h>
//#include <jderobot/pose3d.h>

//Opencv
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/algorithm/string.hpp>

#include <parallelIce/cameraClient.h>
#include "easyiceconfig/EasyIce.h"

namespace real_rt_estimator {

class Control {
	public:

		Control(Ice::CommunicatorPtr ic, real_rt_estimator::Model* sm);	//constructor
		~Control();
		void update();

    void updateImage();
		void calculatePoints(cv::String mode, cv::String filter);
		void calculateMatching(cv::String mode, cv::String filter, int percentagePoints);
		void estimateMatrix(cv::String mode);

		void automaticModeOn(cv::String detectionModeAux, cv::String detectionFilterModeAux,
                         cv::String matchingModeAux, cv::String matchingFilterModeAux, int percentagePointsAux,
                         cv::String estimateFilterModeAux);

    bool isImageUpdated();
		bool isCalculationPointsDone();
		bool isCalculationMatchingDone();
		bool isCalculationEstimateRTDone();

    void printTimes();

	private:

		Model* sm;	//Shared memory

		Ice::CommunicatorPtr ic;
		jderobot::CameraPrx cprxRGB;
		jderobot::CameraPrx cprxDEPTH;

		//jderobot::cameraClient* camDEPTH;
	};
}

#endif // REAL_RT_ESTIMATOR_CONTROL_H
