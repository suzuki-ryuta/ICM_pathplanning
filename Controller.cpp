#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>

#include "Controller.h"
#include "Rectangle.h"
#include "LShape.h"
#include "Triangle.h"
#include "TShape.h"

namespace bp = boost::property_tree;
Controller* Controller::instance = nullptr;


Shape* Controller::shape_create()
{
	std::ofstream log("../ICM_Log/icm.log", std::ios::app);
	bp::ptree pt;
	read_ini("config/ProblemDefine.ini", pt);

	boost::optional<int> carrier = pt.get_optional<int>("object.shape");
	int sh = carrier.get();
	if(sh == 1){
		log << "Object is Rectangle\n";
		return new Rectangle;
	}
	if(sh == 2){
		log << "Object is L-Shape\n";
		return new LShape;
	}
	if(sh == 3){
		log << "Object is Triangle\n";
		return new Triangle;
	}
	if(sh == 4){
		log << "Object is T-Shape\n";
		return new TShape;
	}

	assert(true);
	return 0;
}


Controller::Controller()
	:robot(), shape(), wall()
{
	robot = new Robot();
	shape = shape_create();
	wall = new Wall();
}


Controller::~Controller()
{
	delete robot;
	delete wall;
}


Controller* Controller::get_instance()
{
	if(instance == nullptr){
		instance = new Controller();
	}
	return instance;
}


void Controller::robot_update(Node newnode)
{
	robot->update(newnode);
}

void Controller::shape_update(State3D st)
{
	shape->update(st);
}


bool Controller::RintersectR()
{
	return robot->rr_intersect();
}


bool Controller::RintersectR(Node newnode)
{
	robot->update(newnode);
	return robot->rr_intersect();
}


bool Controller::RintersectW(Node newnode)
{
	robot->update(newnode);
	if(robot->intersect(wall->getter()))return true;
	return false;
}


bool Controller::RintersectS(Node newnode, State3D st)
{
	robot->update(newnode);
	shape->update(st);
//	if(robot->intersect(shape->get_square()))return true;
	return shape->intersect_robot(robot);
}

bool Controller::RintersectS() //対象物とロボットの干渉
{
//	return robot->intersect(shape->get_square());
	return shape->intersect_robot(robot);
}

bool Controller::WintersectS() //壁と対象物の干渉
{
//	return wall->intersect(shape->get_square());
	return shape->intersect_wall(wall);
}

bool Controller::RintersectL(int index)
{
	if(robot->intersect(robot->get_link(index).get_square()))	return true;
	return false;
}

bool Controller::LintersectS(int index)
{
//	if(robot->get_link(index).intersect(shape->get_square()))	return true;
//	return false;
	return shape->intersect(robot->get_link(index).get_square());
}
