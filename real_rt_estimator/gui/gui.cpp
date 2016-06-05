#include "gui.h"

namespace real_rt_estimator {

    Gui::Gui(Model* sm) : gtkmain(0, 0) {

        this->sm = sm;

		/*Init OpenGL*/
		if(!Gtk::GL::init_check(NULL, NULL)) {
			std::cerr << "Couldn't initialize GL\n";
			std::exit(1);
		}

        std::cout << "Loading glade... \n";

        refXml = Gnome::Glade::Xml::create("./real_rt_estimator.glade");
        //Get widgets
        refXml->get_widget("secondarywindow", secondarywindow);
        // Camera images
        refXml->get_widget("image1", gtk_image1);
	      refXml->get_widget("image2", gtk_image2);
        refXml->get_widget("image3", gtk_image3);

		//opengl world
		refXml->get_widget_derived("drawingarea1",world);

		//world->setCamerasResolution(640,480);
		this->worldpath=std::string("./config/world.cfg");
		world->readFile(this->worldpath);
		world->draw_kinect_points = true;
		world->draw_kinect_with_color = true;

        secondarywindow->show();

		std::cout << "Done." << std::endl;
    }

    Gui::~Gui() {

    }

    void Gui::ShowImage() {

  		this->image1 = this->sm->getImageCameraRGB();
  		this->image2 = this->sm->getImageCameraRGBAux();

  		int done = 0;

  		struct timeval t_ini, t_fin;
  		long total_ini, total_fin;
  		long diff;
  		gettimeofday(&t_ini, NULL);

  		total_ini = t_ini.tv_sec * 1000000 + t_ini.tv_usec;
   			if (this->sm->doSiftAndGetPoints()) {
  				done = 1;
  				this->sm->estimateRT();
  			}

  		gettimeofday(&t_fin, NULL);
  		total_fin = t_fin.tv_sec * 1000000 + t_fin.tv_usec;

  		diff = (total_fin - total_ini) / 1000;;
  		std::cout <<  "Tiempo procesado-> " << diff << " ms" << std::endl;

  		this->image3 = this->sm->getImageCameraMatches();
      setCamara(this->image1, 1);
  		setCamara(this->image2, 2);
  		setCamara(this->image3, 3);

  		if (done) {
  			this->putPointCloud();
  			this->putCamera();
  		}
    }

    void Gui::display() {
        ShowImage();
        while (gtkmain.events_pending())
            gtkmain.iteration();
    }

	void Gui::putPointCloud() {
		//this->world->clear_points();
		//if (this->model->isFinal()) {
			std::vector<jderobot::RGBPoint> p = this->sm->get_pc_converted();
			for (int i = 0; i < (int)p.size(); i++){
		 		this->world->add_kinect_point(p[i].x,
											  p[i].y,
											  p[i].z,
		 									  p[i].r,
		 									  p[i].g,
											  p[i].b);
		 	}
		//}
	}

	void Gui::putCamera() {
		//this->world->clear_points();
		//if (this->model->isFinal()) {
			//std::vector<jderobot::RGBPoint> p = this->sm->get_pc();
			//for (int i = 0; i < (int)p.size(); i++){
		 		this->world->add_camera_line(0,
											 0,
											 0,
		 									 75,
		 									 75,
											 75);
		 	//}
		//}
	}

    // First parameter is the widget which will show the image and the id indicates which widget is. This is useful when we have
    // two cameras and we want to choose which one will offer us the image.
    void Gui::setCamara(const cv::Mat image, int id) {
        // Set image
        Glib::RefPtr<Gdk::Pixbuf> imgBuff = Gdk::Pixbuf::create_from_data((const guint8*) image.data,
                Gdk::COLORSPACE_RGB,
                false,
                8,
                image.cols,
                image.rows,
                image.step);
        switch (id) {
			case 1:
				gtk_image1->clear();
				gtk_image1->set(imgBuff);
			case 2:
				gtk_image2->clear();
				gtk_image2->set(imgBuff);
			case 3:
				gtk_image3->clear();
				gtk_image3->set(imgBuff);
        }
    }


	// PRIVATE buttons
	/*void Gui::on_clicked_button1() {
		std::cout << "Resetting... " << std::endl;
		model->calculateNewPointCloudxRT();
		std::cout << "Done." << std::endl;
	}*/

} // namespace
