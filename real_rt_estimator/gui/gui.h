
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
#include <Eigen/Dense>
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
      //virtual ~Gui();
      ~Gui();

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
      Gtk::CheckButton * button_outstanding;
      Gtk::CheckButton * button_ransac;

      Gtk::ToggleButton * button_automatic;

      Gtk::Label* p_matching;
      Gtk::Label* p_total;
      Gtk::Label* m00;
      Gtk::Label* m01;
      Gtk::Label* m02;
      Gtk::Label* m03;
      Gtk::Label* m10;
      Gtk::Label* m11;
      Gtk::Label* m12;
      Gtk::Label* m13;
      Gtk::Label* m20;
      Gtk::Label* m21;
      Gtk::Label* m22;
      Gtk::Label* m23;
      Gtk::Label* m30;
      Gtk::Label* m31;
      Gtk::Label* m32;
      Gtk::Label* m33;
      Gtk::Label* euclidean_err;
      Gtk::Label* reprojection_err;

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
      void button_outstanding_clicked();
      void button_ransac_clicked();
      void button_automatic_clicked();

      // Checkbox control
      int sift_box;
      int surf_box;
      int borderline_box;
      int bruteforce_box;
      int flann_box;
      int outstanding_box;
      int ransac_box;
      int automatic_mode;

      bool first_time;
      bool finish_cycle;
      bool first_draw;
      jderobot::RGBPoint prev_pos;

    }; //class
}//namespace
#endif //REAL_RT_ESTIMATOR_GUI_H
