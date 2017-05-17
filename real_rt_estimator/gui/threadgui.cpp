
#include "threadgui.h"

namespace real_rt_estimator {
ThreadGui::ThreadGui(Model* sm)
{
    //New instace of Gui with the shared memory object.
    gui = new Gui(sm);
    gui->display();

}

void ThreadGui::start()
{
    //Calculates the refreshing time of the gui.
    struct timeval a, b, c;
    long totalc, totalb, totala;
    long diff;

    while (true) {
        gettimeofday(&a, NULL);
        totala = a.tv_sec * 1000000 + a.tv_usec;

        //updates the gui (image shown in this component)
        gui->display();

        gettimeofday(&b, NULL);
        totalb = b.tv_sec * 1000000 + b.tv_usec;
        diff = (totalb - totala) / 1000;

        std::cout << "time gui 1: " << diff << std::endl;

        if (diff < 0 || diff > cycle_gui)
            diff = cycle_gui;
        else
            diff = cycle_gui - diff;


        /*Sleep Algorithm*/
        usleep(diff * 1000);
        if (diff < 33)
            usleep(33 * 1000);

        gettimeofday(&c, NULL);
        totalc = c.tv_sec * 1000000 + c.tv_usec;
        diff = (totalc - totala) / 1000;

        std::cout << "time gui 2: " << diff << std::endl;
        std::cout << "#fps: " << (1000/diff) << std::endl;
    }
}

}
