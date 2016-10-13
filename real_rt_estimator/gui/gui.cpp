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

    //Button
    refXml->get_widget("button_estimate_rt", w_button_estimate_rt);
    refXml->get_widget("button1", w_button1);
    refXml->get_widget("button2", w_button2);
    refXml->get_widget("button3", w_button3);
    refXml->get_widget("button4", w_button4);

    w_button_estimate_rt->signal_clicked().connect(sigc::mem_fun(this,&Gui::estimateCurrentRT));
    w_button1->signal_clicked().connect(sigc::mem_fun(this,&Gui::moveRT1));
    w_button2->signal_clicked().connect(sigc::mem_fun(this,&Gui::moveRT2));
    w_button3->signal_clicked().connect(sigc::mem_fun(this,&Gui::moveRT3));
    w_button4->signal_clicked().connect(sigc::mem_fun(this,&Gui::moveRT4));

		//opengl world
		refXml->get_widget_derived("drawingarea1",world);

		//world->setCamerasResolution(640,480);
		this->worldpath=std::string("./config/world.cfg");
		world->readFile(this->worldpath);
		world->draw_kinect_points = true;
		world->draw_kinect_with_color = true;

        secondarywindow->show();

    this->processDone = false;
    this->firstProcess = true;

		std::cout << "Done." << std::endl;
    }

    Gui::~Gui() {

    }

    void Gui::ShowImage() {

  		this->image1 = this->sm->getImageCameraRGB();
      this->image2 = this->sm->getImageCameraRGBAux();


      if (this->sm->doSiftAndGetPoints()) {
        this->processDone = true;
      }


      setCamara(this->image1, 1);
      setCamara(this->image2, 2);
      setCamara(this->image3, 3);

    }

    void Gui::estimateCurrentRT() {
      //this->image2 = this->sm->getImageCameraRGBAux();

      struct timeval t_ini, t_fin;
      long total_ini, total_fin;
      long diff;
      gettimeofday(&t_ini, NULL);

      total_ini = t_ini.tv_sec * 1000000 + t_ini.tv_usec;

      if (this->processDone) {
        this->sm->estimateRT();
        this->putPointCloud();
      }

      gettimeofday(&t_fin, NULL);
      total_fin = t_fin.tv_sec * 1000000 + t_fin.tv_usec;

      diff = (total_fin - total_ini) / 1000;;
      std::cout <<  "Tiempo procesado-> " << diff << " ms" << std::endl;

      this->image3 = this->sm->getImageCameraMatches();


      this->sm->changeImageAux();

      //if (done) {
      //  this->putPointCloud();
        //this->putCamera();
      //}
    }

    void Gui::moveRT1() {
      this->sm->RotateXAxis();
      this->putPointCloud();
    }
    void Gui::moveRT2() {
      this->sm->RotateYAxis();
      this->putPointCloud();
    }
    void Gui::moveRT3() {
      this->sm->moveDownRT();
      this->putPointCloud();
    }
    void Gui::moveRT4() {
      this->sm->moveUpRT();
      this->putPointCloud();
    }

    void Gui::display() {
        ShowImage();
        while (gtkmain.events_pending())
            gtkmain.iteration();
    }

	void Gui::putPointCloud() {

      // Dibujamos la cámara
      this->world->clear_camera_lines();
      std::vector<jderobot::RGBPoint> line = this->sm->get_pc_camera_converted();
      for (int i = 1; i < (int)line.size(); i++){
        this->world->add_camera_line(
                        line[0].x,
                        line[0].y,
                        line[0].z,
                        line[i].x,
                        line[i].y,
                        line[i].z);
      }
      this->world->add_camera_line(
                      line[1].x,
                      line[1].y,
                      line[1].z,
                      line[0].x,
                      line[0].y,
                      line[0].z);
      for (int i = 3; i < (int)line.size(); i++){
        this->world->add_camera_line(
                        line[i-1].x,
                        line[i-1].y,
                        line[i-1].z,
                        line[i].x,
                        line[i].y,
                        line[i].z);
      }
      this->world->add_camera_line(
                      line[line.size()-1].x,
                      line[line.size()-1].y,
                      line[line.size()-1].z,
                      line[2].x,
                      line[2].y,
                      line[2].z);


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
