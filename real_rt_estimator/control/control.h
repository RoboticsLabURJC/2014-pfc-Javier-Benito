#ifndef SENSORS_H
#define SENSORS_H

//ICE
#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>

#include "../model.h"

//INTERFACES
#include <jderobot/camera.h>

//Opencv
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/algorithm/string.hpp>

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
	};
}

#endif // SENSORS_H
 
