#ifndef RT_ESTIMATOR_GUI_H
#define RT_ESTIMATOR_GUI_H

#include <iostream>
#include <gtkmm.h>
#include <libglademm.h>
#include <cv.h>
#include <string>
#include <jderobot/pointcloud.h>

#include <IceUtil/Thread.h>
#include <IceUtil/Time.h>
#include <pthread.h>

#include "model.h"
#include "drawarea.h"

namespace rt_estimator {

	class Gui {
	public:
		Gui(Model *model);
		virtual ~Gui();
		
		bool isVisible();
		void display();
		
		void putPointCloud();
	private:
		Model *model;
		Model *model_rt;
		Model *model_estimate;
		
		// Glade objects
		Gtk::Main gtkmain;
		Glib::RefPtr<Gnome::Glade::Xml> refXml;
		std::string gladepath;
		
		Gtk::Window* gtk_window;
		
		Gtk::Button *gtk_button1;
		Gtk::Button *gtk_button2;
		Gtk::Button *gtk_button3;
		Gtk::Button *gtk_button4;
		
		//DrawArea* world1;
		DrawArea* world;
		std::string worldpath;
		
		// Methods
		void on_clicked_button1();
		void on_clicked_button2();
		void on_clicked_button3();
		void on_clicked_button4();
		
	}; //class
}//namespace

#endif //RT_ESTIMATOR_GUI_H