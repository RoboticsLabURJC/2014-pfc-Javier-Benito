
#ifndef THREADCONTROL_H
#define THREADCONTROL_H

#include <iostream>
#include <sys/time.h>

#include "control.h"
#include "../model.h"

#define cycle_control 500 //miliseconds

namespace real_rt_estimator {
	class ThreadControl {
	public:
		ThreadControl(Ice::CommunicatorPtr ic, Model* sm);
		void start();
    void printTimes();

	private:
		Control* control;
	};
}

#endif // THREADCONTROL_H
