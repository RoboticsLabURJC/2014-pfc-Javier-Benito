#include "gui.h"

namespace real_rt_estimator {

    Gui::Gui(Model* sm) : gtkmain(0, 0) {

    this->sm = sm;

    // Checkbuttons value
    sift_box=1;
    surf_box=0;
    borderline_box=0;

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
    refXml->get_widget("image_rgb", gtk_image_rgb);
    refXml->get_widget("image_rgb_aux", gtk_image_rgb_aux);
    refXml->get_widget("image_depth", gtk_image_depth);
    refXml->get_widget("image_depth_aux", gtk_image_depth_aux);

    //Buttons
    refXml->get_widget("button_update", button_update);
    refXml->get_widget("button_update", button_detection);
    refXml->get_widget("button_update", button_matching);
    refXml->get_widget("button_estimate", button_estimate);

    //refXml->get_widget("button1", w_button1);
    //refXml->get_widget("button2", w_button2);
    //refXml->get_widget("button3", w_button3);
    //refXml->get_widget("button4", w_button4);

    // Checkbuttons
    refXml->get_widget("button_sift", button_sift);
    refXml->get_widget("button_surf", button_surf);
    refXml->get_widget("button_borderline", button_borderline);

    refXml->get_widget("button_bruteforce", button_bruteforce);
    refXml->get_widget("button_flann", button_flann);

    refXml->get_widget("percentage_points", percentage_points);

    button_update->signal_clicked().connect(sigc::mem_fun(this,&Gui::updateImages));
    button_detection->signal_clicked().connect(sigc::mem_fun(this,&Gui::detectionPoints));
    button_estimate->signal_clicked().connect(sigc::mem_fun(this,&Gui::estimateCurrentRT));
    //w_button1->signal_clicked().connect(sigc::mem_fun(this,&Gui::moveRT1));
    //w_button2->signal_clicked().connect(sigc::mem_fun(this,&Gui::moveRT2));
    //w_button3->signal_clicked().connect(sigc::mem_fun(this,&Gui::moveRT3));
    //w_button4->signal_clicked().connect(sigc::mem_fun(this,&Gui::moveRT4));


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
      //std::cout << "show image" << std::endl;
      //this->image2 = this->sm->getImageCameraRGB();
      //std::cout << "2" << std::endl;
      //setCamara(this->image2, 2);
      //std::cout << "3" << std::endl;
      /*
  		this->image_rgb = this->sm->getImageCameraRGB();
      this->image2 = this->sm->getImageCameraRGBAux();


      if (this->sm->doSiftAndGetPoints()) {
        this->processDone = true;
      }


      setCamara(this->image_rgb, 1);
      setCamara(this->image2, 2);
      setCamara(this->image_depth, 3);
      */
      if (this->ctrl->isEstimatePointsDone()) {
        this->image_rgb = this->sm->getImageCameraRGBAux();
        setCamara(this->image_rgb, 1);

        this->image_depth = this->sm->getImageCameraRGBMatches();
        setCamara(this->image_depth, 3);

        std::cout << "5" << std::endl;
      }
      if (this->ctrl->isEstimateMatrixDone()) {

        this->putPointCloud();
        std::cout << "6" << std::endl;
      }

      std::cout << "7" << std::endl;
    }

    /*void Gui::estimatePoints() {
      this->ctrl->estimatePoints();
    }*/

    void Gui::estimateCurrentRT() {
      //std::cout << "2" << std::endl;
      this->ctrl->estimateMatrix();

      //std::cout << "3" << std::endl;

      //std::cout << "4" << std::endl;
      /*if (this->sm->doSiftAndGetPoints()) {
        this->processDone = true;
      }*/








      //this->image2 = this->sm->getImageCameraRGBAux();
      /*
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
      std::cout <<  "Tiempo procesado-> " << diff << " ms" << std::endl;*/



      //std::cout << "5" << std::endl;
      //this->sm->changeImageAux();

      //if (done) {
      //  this->putPointCloud();
        //this->putCamera();
      //}


      //std::cout << "6" << std::endl;
      /*if (this->sm->isEstimated()) {
        this->image_depth = this->sm->getImageCameraRGBMatches();
        setCamara(this->image_depth, 3);
        this->putPointCloud();
      }*/
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
        //ShowImage();
        while (gtkmain.events_pending())
            gtkmain.iteration();
    }


  void Gui::updateImages() {
    this->sm->updateGuiImages();
    this->image_rgb_aux = this->sm->getImageCameraRGBAux();
    setCamara(this->image_rgb_aux, 1);
    this->image_rgb = this->sm->getImageCameraRGB();
    setCamara(this->image_rgb, 2);
    this->image_depth_aux = this->sm->getImageCameraDEPTHAux();
    setCamara(this->image_depth_aux, 3);
    this->image_depth = this->sm->getImageCameraDEPTH();
    setCamara(this->image_depth, 4);
  }

  void Gui::detectionPoints() {
    cv::String detectionMode;
    if (sift_box) {
      detectionMode = "sift";
    } else if (surf_box) {
      detectionMode = "surf";
    }

    // Filters
    cv::String filterMode;
    if (borderline_box) {
      filterMode = "borderline";
    }

    this->ctrl->calculatePoints(detectionMode, filterMode);


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
  				gtk_image_rgb_aux->clear();
  				gtk_image_rgb_aux->set(imgBuff);
          break;
  			case 2:
          gtk_image_rgb->clear();
  				gtk_image_rgb->set(imgBuff);
          break;
        case 3:
          gtk_image_depth_aux->clear();
          gtk_image_depth_aux->set(imgBuff);
          break;
        case 4:
          gtk_image_depth->clear();
          gtk_image_depth->set(imgBuff);
          break;
      }
    }


	// PRIVATE buttons
  void Gui::button_sift_clicked() {
		if(sift_box)
			sift_box = 0;
		else
			sift_box = 1;
  }
  void Gui::button_surf_clicked() {
    if(surf_box)
      surf_box = 0;
    else
      surf_box = 1;
  }
  void Gui::button_borderline_clicked() {
    if(borderline_box)
      borderline_box = 0;
    else
      borderline_box = 1;
  }

} // namespace
