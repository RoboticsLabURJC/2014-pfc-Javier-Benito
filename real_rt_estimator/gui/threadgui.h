#ifndef THREADGUI_H
#define THREADGUI_H

#include <iostream>
#include <sys/time.h>

#include "gui.h"

#include "../model.h"

#define cycle_gui 500 //miliseconds

namespace real_rt_estimator {
class ThreadGui
{
public:
    ThreadGui(Model* sm);
    void start();

private:
    Gui* gui;
};
}
#endif // THREADGUI_H
