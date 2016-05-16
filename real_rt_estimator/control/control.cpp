#include "control.h"


namespace real_rt_estimator {

bool cameraRGBOn = false;
bool cameraDEPTHOn = false;

Control::Control(Ice::CommunicatorPtr ic, Model* sm) {
    /*Obtaining the configuration file (*.cfg) properties such as ports and IP's*/
    this->ic = ic;
    this->sm = sm;

    Ice::PropertiesPtr prop = ic->getProperties();

    /*Checking if the property has value and the creation of the proxy is possible. If the property is not set or has no value
       it will be set as "miss" and then we will know that we cannot create a proxy with the camera (or other sensors/actuators)*/
    std::string camRGB = prop->getPropertyWithDefault("real_rt_estimator.CameraRGB.Proxy", "miss");
    if (!boost::iequals(camRGB , "miss")) {

		/*Creation of a proxy to connect with cameraServer*/
		Ice::ObjectPrx baseRGB = ic->propertyToProxy("real_rt_estimator.CameraRGB.Proxy");
		if (0==baseRGB)
			throw "Could not create proxy";
		/*cast to CameraPrx*/
		this->cprxRGB = jderobot::CameraPrx::checkedCast(baseRGB);
		if (0==cprxRGB)
			throw "Invalid proxy";

		cameraRGBOn = true;

		/*Get the image data from the camera proxy*/
		jderobot::ImageDataPtr data = cprxRGB->getImageData();

		/*Create the first image obtained from the camera and stores in the shared memory*/
		this->sm->createImageRGB(data);
	std::cout <<  "aaaaaaaaAAAAAAAAAA 1" << std::endl;
    } else {
		cameraRGBOn = false; 
		/*Create an empty image if there is no camera connected*/
		//this->sm->createEmptyImageRGB();
		std::cout << "No camera RGB connected." << std::endl;
    }
    
	std::string camDEPTH = prop->getPropertyWithDefault("real_rt_estimator.CameraDEPTH.Proxy", "miss");
    if (!boost::iequals(camDEPTH , "miss")) {

		/*Creation of a proxy to connect with cameraServer*/
		Ice::ObjectPrx baseDEPTH = ic->propertyToProxy("real_rt_estimator.CameraDEPTH.Proxy");
		if (0==baseDEPTH)
			throw "Could not create proxy";
		/*cast to CameraPrx*/
		this->cprxDEPTH = jderobot::CameraPrx::checkedCast(baseDEPTH);
		if (0==cprxDEPTH)
			throw "Invalid proxy";

		cameraDEPTHOn = true;
std::cout <<  "aaaaaaaaAAAAAAAAAA 3" << std::endl;
		/*Get the image data from the camera proxy*/
		jderobot::ImageDataPtr data = cprxDEPTH->getImageData();
std::cout <<  "aaaaaaaaAAAAAAAAAA 35" << std::endl;
		/*Create the first image obtained from the camera and stores in the shared memory*/
		this->sm->createImageDEPTH(data);
	std::cout <<  "aaaaaaaaAAAAAAAAAA 4" << std::endl;
    } else {
		cameraDEPTHOn = false; 
		/*Create an empty image if there is no camera connected*/
 		//this->sm->createEmptyImageDEPTH();
		std::cout << "No camera DEPTH connected" << std::endl;
    }
    
}

	void Control::update() {
		
		if(cameraRGBOn) { //TODO: Sincronizar imagen RGB y DEPTTH, poniendo algún tipo de espera a las 2??
			//Get de data from the camera and stores de image in the shared memory periodically (see threadcontrol)
			jderobot::ImageDataPtr data = cprxRGB->getImageData();
			this->sm->updateImageRGB(data);
		}
		if(cameraDEPTHOn) {
			//Get de data from the camera and stores de image in the shared memory periodically (see threadcontrol)
			jderobot::ImageDataPtr data = cprxDEPTH->getImageData();
			this->sm->updateImageDEPTH(data);
		}
	}

}
 
