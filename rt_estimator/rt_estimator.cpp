#include <iostream>
#include <cv.h>
#include <highgui.h>

#include "gui.h"
#include "model.h"


int main(int argc, char* argv[]) {
	int status = 0;
	rt_estimator::Gui* gui;

	try {
		rt_estimator::Model *model = new rt_estimator::Model();
		model->createPointCloud();
		
		gui = new rt_estimator::Gui(model);
		
		while (gui->isVisible()){
			gui->display();
			usleep(100000);
		}
	
	} catch (const char* msg) {
		std::cerr << msg << std::endl;
		status = 1;
	}

	return status;
}