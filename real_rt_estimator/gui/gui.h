
#ifndef REAL_RT_ESTIMATOR_GUI_H
#define REAL_RT_ESTIMATOR_GUI_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <gtk-2.0/gtk/gtk.h>
#include <gtk-2.0/gdk/gdk.h>
#include <gtkmm-2.4/gtkmm.h>
#include <gtkmm/drawingarea.h>
#include <gdkmm/pixbuf.h>
#include <libglademm.h>
#include <math.h>
#include <string>
#include <iostream>
#include <sstream>

#include <cv.h>
#include <jderobot/pointcloud.h>
#include <gtkmm.h>
#include <IceUtil/Thread.h>
#include <IceUtil/Time.h>
#include <pthread.h>

#include <sys/time.h>

#include "../model.h"
#include "../control/control.h"
#include "../drawarea.h"


namespace real_rt_estimator {

    class Gui {
    public:

      Gui(Model* sm);
      virtual ~Gui();

      void display();

      cv::Mat image_rgb_aux;
      cv::Mat image_rgb;
      cv::Mat image_depth_aux;
      cv::Mat image_depth;

    private:

  		Model* sm;
      Control* ctrl;

      bool processDone;
      bool firstProcess;

      Gtk::Main gtkmain;
      Glib::RefPtr<Gnome::Glade::Xml> refXml;
      std::string gladepath;

      // Windows
      Gtk::Window *secondarywindow;

      // Cameras
      Gtk::Image *gtk_image_rgb;
      Gtk::Image *gtk_image_rgb_aux;
  		Gtk::Image *gtk_image_depth;
      Gtk::Image *gtk_image_depth_aux;

      // Button
      Gtk::Button *button_update;
      Gtk::Button *button_detection;
      Gtk::Button *button_matching;
      Gtk::Button *button_estimate;
      //Gtk::Button *w_button1;
      //Gtk::Button *w_button2;
      //Gtk::Button *w_button3;
      //Gtk::Button *w_button4;

      Gtk::VScale* percentage_points;
      Gtk::CheckButton * button_sift;
      Gtk::CheckButton * button_surf;
      Gtk::CheckButton * button_bruteforce;
      Gtk::CheckButton * button_flann;

  		//DrawArea* world1;
  		DrawArea* world;
  		std::string worldpath;

      // Private Methods
      void setCamara(const cv::Mat image, int id);
  		void ShowImage();

      void estimatePoints();
      void estimateCurrentRT();
      void moveRT1();
      void moveRT2();
      void moveRT3();
      void moveRT4();

      // Button Methods
      void updateImages();

  		void putPointCloud();
  		void putCamera();



    }; //class
}//namespace
#endif //REAL_RT_ESTIMATOR_GUI_H
