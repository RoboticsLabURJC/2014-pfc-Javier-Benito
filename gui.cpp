#include "gui.h"

namespace rt_estimator {
	
    Gui::Gui (Model *model) : gtkmain(0, 0) { 
		this->model = model;
		
		/*Init OpenGL*/
		if(!Gtk::GL::init_check(NULL, NULL)) {
			std::cerr << "Couldn't initialize GL\n";
			std::exit(1);
		}

		std::cout << "Loading glade...\n";
		//refXml = Gnome::Glade::Xml::create("./rt_estimator.glade");
		this->gladepath=std::string("./rt_estimator.glade");
		refXml = Gnome::Glade::Xml::create(this->gladepath);
		
		//Get widgets
		//refXml->get_widget("window1", gtk_window);
		refXml->get_widget("window1",gtk_window);
		
		refXml->get_widget("button1", gtk_button1);
		refXml->get_widget("button2", gtk_button2);
		refXml->get_widget("button3", gtk_button3);
		refXml->get_widget("button4", gtk_button4);
		
		//opengl world
		refXml->get_widget_derived("drawingarea1",world);
		//refXml->get_widget_derived("drawingarea1",world);
		//refXml->get_widget_derived("drawingarea2",world2);

		//world->setCamerasResolution(640,480);
		this->worldpath=std::string("./config/world.cfg");
		world->readFile(this->worldpath);
		world->draw_kinect_points = true;
		world->draw_kinect_with_color = true;

		gtk_button1->signal_clicked().connect(sigc::mem_fun(this,&Gui::on_clicked_button1));
		gtk_button2->signal_clicked().connect(sigc::mem_fun(this,&Gui::on_clicked_button2));
		gtk_button3->signal_clicked().connect(sigc::mem_fun(this,&Gui::on_clicked_button3));
		gtk_button4->signal_clicked().connect(sigc::mem_fun(this,&Gui::on_clicked_button4));
		
		gtk_window->show();
		
		std::cout << "Done." << std::endl;
		
		this->putPointCloud();
	}
	
	Gui::~Gui() {}

	bool Gui::isVisible() {
		return gtk_window->is_visible();
	}
	
	void Gui::display() {
		this->putPointCloud();
		while (gtkmain.events_pending())
			gtkmain.iteration();
	}
	
	void Gui::putPointCloud() {
		this->world->clear_points();

		//if (this->model->isFinal()) {
			for (int i = 0; i < model->getSize(); i++){
		 		this->world->add_kinect_point(this->model->getX(i),
											  this->model->getY(i),
											  this->model->getZ(i),
		 									  this->model->getRed(i),
		 									  this->model->getGreen(i),
											  this->model->getBlue(i));
		 	}
		//}
	}
	
	
// PRIVATE buttons
	
	void Gui::on_clicked_button1() {
		std::cout << "Calculating new point cloud... " << std::endl;
		model->calculateNewPointCloudxRT();
		std::cout << "Done." << std::endl;
	}
	void Gui::on_clicked_button2() {
		std::cout << "Estimating the RT known from the beginning... " << std::endl;
		model->estimateRT();
		std::cout << "Done." << std::endl;
	}
	void Gui::on_clicked_button3() {
		std::cout << "Adding Gaussian Noise... ";
		this->model->addGaussianNoise();
		std::cout << "Done." << std::endl;
	}
	void Gui::on_clicked_button4() {
		std::cout << "Reseting... ";
		this->model->createPointCloud();
		std::cout << "Done." << std::endl;
	}
	
} // namespace
