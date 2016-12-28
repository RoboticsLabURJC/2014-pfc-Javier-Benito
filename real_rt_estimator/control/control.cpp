#include "control.h"


namespace real_rt_estimator {

  cv::String detectionMode;
  cv::String detectionFilterMode;

  bool cameraRGBOn = false;
  bool cameraDEPTHOn = false;
  bool calculatePointsOn = false;
  bool estimateMatrixOn = false;
  bool correctEstimate = false;

  bool estimatePointsDone = false;
  bool estimateMatrixDone = false;

  jderobot::cameraClient* camRGB=NULL;
  jderobot::cameraClient* camDEPTH=NULL;

  Control::Control(Ice::CommunicatorPtr ic, Model* sm) {
    /*Obtaining the configuration file (*.cfg) properties such as ports and IP's*/
    this->ic = ic;
    this->sm = sm;

    Ice::PropertiesPtr prop = ic->getProperties();

    // *** camRGB ***
    /*Checking if the property has value and the creation of the proxy is possible. If the property is not set or has no value
       it will be set as "miss" and then we will know that we cannot create a proxy with the camera (or other sensors/actuators)*/
    std::string camRGBCfg = prop->getPropertyWithDefault("real_rt_estimator.CameraRGB.Proxy", "miss");
    if (!boost::iequals(camRGBCfg , "miss")) {

    	/*Creation of a proxy to connect with cameraServer*/
    	Ice::ObjectPrx baseRGB = ic->propertyToProxy("real_rt_estimator.CameraRGB.Proxy");
    	if (0==baseRGB)
    		throw "Could not create proxy";

      // parallelIce for camRGB
      camRGB = new jderobot::cameraClient(ic, "real_rt_estimator.CameraRGB.");
    	if (camRGB != NULL){
    		cameraRGBOn = true;
    		camRGB->start();
    		//create_gui=true;
      } else {
        cameraRGBOn = false;
    	  throw "Failed to load RGB Camera";
      }

      cv::Size rgbSize(0,0);
      cv::Mat data;
      camRGB->getImage(data,true);
      rgbSize=data.size();

      /*Create the first image obtained from the camera and stores in the shared memory*/
      this->sm->createImageRGB(data);

    } else {
      cameraRGBOn = false;
    	/*Create an empty image if there is no camera connected*/
    	//this->sm->createEmptyImageRGB();
    	std::cout << "No camera RGB connected." << std::endl;
    }

    // *** camDEPTH ***
    std::string camDEPTHCfg = prop->getPropertyWithDefault("real_rt_estimator.CameraDEPTH.Proxy", "miss");
    if (!boost::iequals(camDEPTHCfg , "miss")) {

      /*Creation of a proxy to connect with cameraServer*/
    	Ice::ObjectPrx baseDEPTH = ic->propertyToProxy("real_rt_estimator.CameraDEPTH.Proxy");
    	if (0==baseDEPTH)
    		throw "Could not create proxy";

      // parallelIce for camDEPTH
      camDEPTH = new jderobot::cameraClient(ic, "real_rt_estimator.CameraDEPTH.");
      if (camDEPTH != NULL){
        cameraDEPTHOn = true;
        camDEPTH->start();
        //create_gui=true;
      } else {
        cameraDEPTHOn = false;
        throw "Failed to load DEPTH Camera";
      }

      cv::Size depthSize(0,0);
      cv::Mat dataDEPTH;
      camDEPTH->getImage(dataDEPTH,true);
      depthSize=dataDEPTH.size();

      /*Create the first image obtained from the camera and stores in the shared memory*/
      this->sm->createImageDEPTH(dataDEPTH);

    } else {
      cameraDEPTHOn = false;
      /*Create an empty image if there is no camera connected*/
      //this->sm->createEmptyImageDEPTH();
      std::cout << "No camera DEPTH connected" << std::endl;
    }


  }

  void Control::update() {
    // FIXME: SE ACTUALIZAN A LA VEZ LA IMAGEN RGB Y PROFUNDIDAD?? PROBLEMAS CONCURRENCIA??
    if(cameraRGBOn) {
  	  //Get de data from the camera and stores de image in the shared memory periodically (see threadcontrol)
  		//jderobot::ImageDataPtr data = cprxRGB->getImageData();
      cv::Mat data;
  		camRGB->getImage(data,true);
  		cv::Size rgbSize = data.size();
  		this->sm->updateImageRGB(data);
    }
    if(cameraDEPTHOn) {
      //Get de data from the camera and stores de image in the shared memory periodically (see threadcontrol)
      cv::Mat dataDEPTH;
  		camDEPTH->getImage(dataDEPTH,true);
  		cv::Size depthSize = dataDEPTH.size();
      this->sm->updateImageDEPTH(dataDEPTH);
    }

    if (calculatePointsOn) {
      if (this->sm->calculatePoints(detectionMode, detectionFilterMode)) {
        calculatePointsOn = false;
        correctEstimate = true;
        estimatePointsDone = true;
      }


        //this->sm->putPointCloud();
    }

    if (estimateMatrixOn && correctEstimate) {
      //std::cout << "ESTIMATE" << std::endl;
      this->sm->estimateRT();
      //std::cout << "ESTIMATE FIN" << std::endl;
      correctEstimate = false;
      estimateMatrixOn = false;
      estimateMatrixDone = true;
    }

  }

  void Control::calculatePoints(cv::String mode, cv::String filter) {
    detectionMode = mode;
    detectionFilterMode = filter;
    calculatePointsOn = true;
  }

  void Control::estimateMatrix() {
    estimateMatrixOn = true;
  }

  bool Control::isEstimatePointsDone() {
    if (estimatePointsDone) {
      estimatePointsDone = false;
      return true;
    } else {
      return false;
    }
  }

  bool Control::isEstimateMatrixDone() {
    if (estimateMatrixDone) {
      estimateMatrixDone = false;
      return true;
    } else {
      return false;
    }
  }

}
