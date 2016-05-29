#ifndef SENSORS_H
#define SENSORS_H

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
		void update();

	private:

		Model* sm;	//Shared memory

		Ice::CommunicatorPtr ic;
		jderobot::CameraPrx cprxRGB;
		jderobot::CameraPrx cprxDEPTH;


		//jderobot::cameraClient* camDEPTH;
	};
}

#endif // SENSORS_H
