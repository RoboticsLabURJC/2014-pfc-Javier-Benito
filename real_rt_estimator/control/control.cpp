#include "control.h"


namespace real_rt_estimator {

  cv::String detectionMode = "sift"; // FIXME: Incluir por el .conf
  cv::String detectionFilterMode;
  cv::String matchingMode = "bruteforce";
  cv::String matchingFilterMode;

  bool cameraRGBOn = false;
  bool cameraDEPTHOn = false;
  bool updateImageOn = false;
  bool calculatePointsOn = false;
  bool estimateMatrixOn = false;
  bool calculateMatchingOn = false;

  bool correctUpdateImages = false;
  bool correctPointsCalculation = false;
  bool correctMatchingCalculation = false;

  bool updateImageDone = false;
  bool calculationPointsDone = false;
  bool calculationMatchingDone = false;
  bool calculationEstimateRTDone = false;
  bool calculationFail = false;

  bool automaticMode = false;
  bool firstTime = true;
  bool processDone = true;

  int percentagePoints = 20;

  struct timeval it, ft;
  double updateImagesTime = 0.0;
  int updateImagesCount = 0;
  double calculatePointsTime = 0.0;
  int calculatePointsCount = 0;
  double calculateMatchingTime = 0.0;
  int calculateMatchingCount = 0;
  double estimateRTTime = 0.0;
  int estimateRTCount = 0;

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

    //Let's check if the user want to show the gui or not. This setting must be in the .cfg file
    std::string gui = prop->getPropertyWithDefault("real_rt_estimator.Gui", "miss");
      if (!boost::iequals(gui , "miss") && !boost::iequals(gui, "OFF")) {
    } else {
      std::cout << "No Gui mode -> automatic control mode" << std::endl;
      automaticMode = true;
    }


  }

  Control::~Control() {
    std::cout << "Exit control" << std::endl;
  }

  void Control::update() {
    if(cameraRGBOn) {
  	  //Get de data from the camera and stores de image in the shared memory periodically (see threadcontrol)
  		//jderobot::ImageDataPtr data = cprxRGB->getImageData();
      cv::Mat data;
  		camRGB->getImage(data,true);
  		//cv::Size rgbSize = data.size();
  		this->sm->updateImageRGB(data);
    }
    if(cameraDEPTHOn) {
      //Get de data from the camera and stores de image in the shared memory periodically (see threadcontrol)
      cv::Mat dataDEPTH;
  		camDEPTH->getImage(dataDEPTH,true);
  		//cv::Size depthSize = dataDEPTH.size();
      this->sm->updateImageDEPTH(dataDEPTH);
    }

    ////////////////////////////////////////////////////////////////////////////
    if (automaticMode && processDone) {
      processDone = false;
      if (firstTime) {
        this->sm->updateImages();
        firstTime = false;
      }
      updateImageOn = true;
      calculatePointsOn = true;
      calculateMatchingOn = true;
      estimateMatrixOn = true;
    }



    if (updateImageOn) {
      //////////////////////////
      gettimeofday(&it, NULL);
      this->sm->updateImages();
      gettimeofday(&ft, NULL);
      double t = (ft.tv_sec - it.tv_sec)*1000 + (ft.tv_usec - it.tv_usec)/1000.0;
      updateImagesTime+= t;
      updateImagesCount++;
      //////////////////////////
      updateImageOn = false;
      updateImageDone = true;
      correctUpdateImages = true;
    }
    if (calculatePointsOn && correctUpdateImages) {
      //////////////////////////
      gettimeofday(&it, NULL);
      if (this->sm->calculatePoints(detectionMode, detectionFilterMode)) {
        gettimeofday(&ft, NULL);
        double t = (ft.tv_sec - it.tv_sec)*1000 + (ft.tv_usec - it.tv_usec)/1000.0;
        calculatePointsTime+= t;
        calculatePointsCount++;
        //////////////////////////
        calculatePointsOn = false;
        correctPointsCalculation = true;
        calculationPointsDone = true;
        correctUpdateImages = false;
      } else {
        correctPointsCalculation = false;
      }
    }
    if (calculateMatchingOn && correctPointsCalculation) {
      //////////////////////////
      gettimeofday(&it, NULL);
      if (this->sm->calculateMatching(matchingMode, matchingFilterMode, percentagePoints)) {
        gettimeofday(&ft, NULL);
        double t = (ft.tv_sec - it.tv_sec)*1000 + (ft.tv_usec - it.tv_usec)/1000.0;
        calculateMatchingTime+= t;
        calculateMatchingCount++;
        //////////////////////////
        calculateMatchingOn = false;
        correctMatchingCalculation = true;
        calculationMatchingDone = true;
        correctPointsCalculation = false;
      } else {
        correctMatchingCalculation = false;
      }
    }
    if (estimateMatrixOn && correctMatchingCalculation) {
      //////////////////////////
      gettimeofday(&it, NULL);
      if (this->sm->estimateRT()) {
        gettimeofday(&ft, NULL);
        double t = (ft.tv_sec - it.tv_sec)*1000 + (ft.tv_usec - it.tv_usec)/1000.0;
        estimateRTTime+= t;
        estimateRTCount++;
        //////////////////////////
        estimateMatrixOn = false;
        calculationEstimateRTDone = true;
        correctMatchingCalculation = false;
        processDone = true;
      }
    }
    ////////////////////////////////////////////////////////////////////////////
  }

  void Control::updateImage() {
    updateImageOn = true;
  }

  void Control::calculatePoints(cv::String mode, cv::String filter) {
    detectionMode = mode;
    detectionFilterMode = filter;
    calculatePointsOn = true;
  }

  void Control::calculateMatching(cv::String mode, cv::String filter, int percentage) {
    matchingMode = mode;
    matchingFilterMode = filter;
    percentagePoints = percentage;
    calculateMatchingOn = true;
  }

  void Control::estimateMatrix() {
    estimateMatrixOn = true;
  }

  void Control::automaticModeOn() {
    automaticMode = true;
  }

  bool Control::isImageUpdated() {
    if (updateImageDone) {
      updateImageDone = false;
      return true;
    } else {
      return false;
    }
  }

  bool Control::isCalculationPointsDone() {
    if (calculationPointsDone) {
      calculationPointsDone = false;
      return true;
    } else {
      return false;
    }
  }

  bool Control::isCalculationMatchingDone() {
    if (calculationMatchingDone) {
      calculationMatchingDone = false;
      return true;
    } else {
      return false;
    }
  }

  bool Control::isCalculationEstimateRTDone() {
    if (calculationEstimateRTDone) {
      calculationEstimateRTDone = false;
      return true;
    } else {
      return false;
    }
  }

  void Control::printTimes() {
    std::cout << "Average times:" << std::endl;
    std::cout << "- Update images: " << updateImagesTime/updateImagesCount << " ms" << std::endl;
    std::cout << "- Points detection: " << calculatePointsTime/calculatePointsCount << " ms" << std::endl;
    std::cout << "- Points matching: " << calculateMatchingTime/calculateMatchingCount << " ms" << std::endl;
    std::cout << "- Estimate matrix: " << estimateRTTime/estimateRTCount << " ms" << std::endl;
    std::cout << "-----------------------------" << std::endl;
    double total = updateImagesTime/updateImagesCount + calculatePointsTime/calculatePointsCount +
                   calculateMatchingTime/calculateMatchingCount + estimateRTTime/estimateRTCount;
    std::cout << "- TOTAL: " << total << " ms" << std::endl;
  }

}
