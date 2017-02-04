
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

      // Buttons
      Gtk::Button *button_update;
      Gtk::Button *button_detection;
      Gtk::Button *button_matching;
      Gtk::Button *button_estimate;

      //Gtk::Button *w_button1;
      //Gtk::Button *w_button2;
      //Gtk::Button *w_button3;
      //Gtk::Button *w_button4;

      // VScale bar
      Gtk::VScale* percentage_points;

      // Detection check buttons
      Gtk::CheckButton * button_sift;
      Gtk::CheckButton * button_surf;
      Gtk::CheckButton * button_borderline;
      Gtk::CheckButton * button_bruteforce;
      Gtk::CheckButton * button_flann;
      Gtk::CheckButton * button_correlation;
      Gtk::CheckButton * button_outstanding;

      Gtk::ToggleButton * button_automatic;

  		//DrawArea* world1;
  		DrawArea* world;
  		std::string worldpath;

      // Private Methods
      void setCamara(const cv::Mat image, int id);
  		void ShowImage();

      //void estimatePoints();
      void estimateCurrentRT();
      void moveRT1();
      void moveRT2();
      void moveRT3();
      void moveRT4();

      // Button Methods
      void updateImages();
      void detectionPoints();
      void matchingPoints();

  		void putPointCloud();
  		void putCamera();


      //Checks if the button has been clicked
      void button_sift_clicked();
      void button_surf_clicked();
      void button_borderline_clicked();
      void button_bruteforce_clicked();
      void button_flann_clicked();
      void button_correlation_clicked();
      void button_outstanding_clicked();
      void button_automatic_clicked();

      // Checkbox control
      int sift_box;
      int surf_box;
      int borderline_box;
      int bruteforce_box;
      int flann_box;
      int correlation_box;
      int outstanding_box;
      int automatic_mode;

      bool first_time;
      bool finish_cycle;

    }; //class
}//namespace
#endif //REAL_RT_ESTIMATOR_GUI_H
