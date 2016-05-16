 
#ifndef THREADCONTROL_H
#define THREADCONTROL_H

#include <iostream>
#include <sys/time.h>

#include "control.h"
#include "../model.h"

#define cycle_control 20 //miliseconds

namespace real_rt_estimator {
	class ThreadControl {
	public:
		ThreadControl(Ice::CommunicatorPtr ic, Model* sm);
		void start();

	private:
		Control* control;
	};
}

#endif // THREADCONTROL_H
