#include "TaskSet.h"

#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/json_parser.hpp>

using namespace boost::property_tree;

void TaskSet::set_all()
{
	ptree pt;

	double th_s1, th_s2, th_s3, th_s4, th_s5, th_s6;
	std::cout << "Input initial state" << std::endl;
	std::cout << "th1: ";	std::cin >> th_s1;
	std::cout << "th2: ";	std::cin >> th_s2;
	std::cout << "th3: ";	std::cin >> th_s3;
	std::cout << "th4: ";	std::cin >> th_s4;
	std::cout << "th5: ";	std::cin >> th_s5;
	std::cout << "th6: ";	std::cin >> th_s6;
	std::cout << std::endl;

	double th_g1, th_g2, th_g3, th_g4, th_g5, th_g6;
	std::cout << "Input final state" << std::endl;
	std::cout << "th1: ";	std::cin >> th_g1;
	std::cout << "th2: ";	std::cin >> th_g2;
	std::cout << "th3: ";	std::cin >> th_g3;
	std::cout << "th4: ";	std::cin >> th_g4;
	std::cout << "th5: ";	std::cin >> th_g5;
	std::cout << "th6: ";	std::cin >> th_g6;
	std::cout << std::endl;

	int shape = 0, maxi = 4;
	do {
		std::cout << "Choose object:" << std::endl;
		std::cout << "(Rectangle -> 1 , LShape -> 2 , Triangle -> 3, TShape -> 4): ";

		std::cin >> shape;
	} while (shape <= 0 || shape > maxi);
	pt.put("object.shape", shape);

	int x, y, th = 0;
	int e = 0;
	std::cout << "Input goal condition" << std::endl;
	std::cout << "x_goal: ";	std::cin >> x;
	std::cout << "y_goal: ";	std::cin >> y;
	std::cout << "th_goal: ";	std::cin >> th;
	std::cout << "Input threshold: ";	std::cin >> e;

	pt.put("start.th1", th_s1);
	pt.put("start.th2", th_s2);
	pt.put("start.th3", th_s3);
	pt.put("start.th4", th_s4);
	pt.put("start.th5", th_s5);
	pt.put("start.th6", th_s6);
	pt.put("finish.th1", th_g1);
	pt.put("finish.th2", th_g2);
	pt.put("finish.th3", th_g3);
	pt.put("finish.th4", th_g4);
	pt.put("finish.th5", th_g5);
	pt.put("finish.th6", th_g6);
	pt.put("goal.coordx", x);
	pt.put("goal.coordy", y);
	pt.put("goal.coordt", th);
	pt.put("goal.epsilon", e);

	write_ini("config/ProblemDefine.ini", pt);
	std::cout << std::endl;
}


void TaskSet::set_shape()
{
	ptree problem;
	read_ini("config/ProblemDefine.ini", problem);

	int shape = 0, maxi = 4;
	do {
		std::cout << "Choose object:" << std::endl;
		std::cout << "(Rectangle -> 1 , LShape -> 2 , Triangle -> 3, TShape -> 4): ";

		std::cin >> shape;
	} while (shape <= 0 || shape > maxi);
	problem.put("object.shape", shape);

	boost::optional<double> th = problem.get_optional<double>("start.th1");
	problem.put("start.th1", th.get());
	th = problem.get_optional<double>("start.th2");
	problem.put("start.th2", th.get());
	th = problem.get_optional<double>("start.th3");
	problem.put("start.th3", th.get());
	th = problem.get_optional<double>("start.th4");
	problem.put("start.th4", th.get());
	th = problem.get_optional<double>("start.th5");
	problem.put("start.th5", th.get());
	th = problem.get_optional<double>("start.th6");
	problem.put("start.th6", th.get());

	th = problem.get_optional<double>("finish.th1");
	problem.put("finish.th1", th.get());
	th = problem.get_optional<double>("finish.th2");
	problem.put("finish.th2", th.get());
	th = problem.get_optional<double>("finish.th3");
	problem.put("finish.th3", th.get());
	th = problem.get_optional<double>("finish.th4");
	problem.put("finish.th4", th.get());
	th = problem.get_optional<double>("finish.th5");
	problem.put("finish.th5", th.get());
	th = problem.get_optional<double>("finish.th6");
	problem.put("finish.th6", th.get());

	boost::optional<int> x, y, z;
	x = problem.get_optional<int>("goal.coordx");
	y = problem.get_optional<int>("goal.coordy");
	z = problem.get_optional<int>("goal.coordt");
	problem.put("goal.coordx", x.get());
	problem.put("goal.coordy", y.get());
	problem.put("goal.coordt", z.get());

	boost::optional<int> epsilon = problem.get_optional<int>("goal.epsilon");
	problem.put("goal.epsilon", epsilon.get());

	write_ini("config/ProblemDefine.ini", problem);
	std::cout << std::endl;
}


void TaskSet::set_robotangle()
{
	ptree problem;
	read_ini("config/ProblemDefine.ini", problem);

	boost::optional<int> shape = problem.get_optional<int>("object.shape");
	problem.put("object.shape", shape.get());

	double th_s1, th_s2, th_s3, th_s4, th_s5, th_s6;
	std::cout << "Input initial state" << std::endl;
	std::cout << "th1: ";	std::cin >> th_s1;
	std::cout << "th2: ";	std::cin >> th_s2;
	std::cout << "th3: ";	std::cin >> th_s3;
	std::cout << "th4: ";	std::cin >> th_s4;
	std::cout << "th5: ";	std::cin >> th_s5;
	std::cout << "th6: ";	std::cin >> th_s6;
	std::cout << std::endl;

	double th_g1, th_g2, th_g3, th_g4, th_g5, th_g6;
	std::cout << "Input final state" << std::endl;
	std::cout << "th1: ";	std::cin >> th_g1;
	std::cout << "th2: ";	std::cin >> th_g2;
	std::cout << "th3: ";	std::cin >> th_g3;
	std::cout << "th4: ";	std::cin >> th_g4;
	std::cout << "th5: ";	std::cin >> th_g5;
	std::cout << "th6: ";	std::cin >> th_g6;
	std::cout << std::endl;

	boost::optional<int> x, y, z;
	x = problem.get_optional<int>("goal.coordx");
	y = problem.get_optional<int>("goal.coordy");
	z = problem.get_optional<int>("goal.coordt");
	problem.put("goal.coordx", x.get());
	problem.put("goal.coordy", y.get());
	problem.put("goal.coordt", z.get());

	boost::optional<int> epsilon = problem.get_optional<int>("goal.epsilon");
	problem.put("goal.epsilon", epsilon.get());

	write_ini("config/ProblemDefine.ini", problem);
	std::cout << std::endl;
}


void TaskSet::set_goal()
{
	ptree problem;
	read_ini("config/ProblemDefine.ini", problem);

	boost::optional<int> shape = problem.get_optional<int>("object.shape");
	problem.put("object.shape", shape.get());
	
	boost::optional<double> th = problem.get_optional<double>("start.th1");
	problem.put("start.th1", th.get());
	th = problem.get_optional<double>("start.th2");
	problem.put("start.th2", th.get());
	th = problem.get_optional<double>("start.th3");
	problem.put("start.th3", th.get());
	th = problem.get_optional<double>("start.th4");
	problem.put("start.th4", th.get());
	th = problem.get_optional<double>("start.th5");
	problem.put("start.th5", th.get());
	th = problem.get_optional<double>("start.th6");
	problem.put("start.th6", th.get());

	th = problem.get_optional<double>("finish.th1");
	problem.put("finish.th1", th.get());
	th = problem.get_optional<double>("finish.th2");
	problem.put("finish.th2", th.get());
	th = problem.get_optional<double>("finish.th3");
	problem.put("finish.th3", th.get());
	th = problem.get_optional<double>("finish.th4");
	problem.put("finish.th4", th.get());
	th = problem.get_optional<double>("finish.th5");
	problem.put("finish.th5", th.get());
	th = problem.get_optional<double>("finish.th6");
	problem.put("finish.th6", th.get());

	int x, y, phi = 0;
	int e = 0;
	std::cout << "Input goal condition" << std::endl;
	std::cout << "x_goal: ";	std::cin >> x;
	std::cout << "y_goal: ";	std::cin >> y;
	std::cout << "th_goal: ";	std::cin >> phi;
	std::cout << "Input threshold: ";	std::cin >> e;
	problem.put("goal.coordx", x);
	problem.put("goal.coordy", y);
	problem.put("goal.coordt", phi);
	problem.put("goal.epsilon", e);

	write_ini("config/ProblemDefine.ini", problem);
	std::cout << std::endl;
}


void TaskSet::space_config(int x, int y, int th)
{
	ptree pt;
	pt.put("range.x", x);
	pt.put("range.y", y);
	pt.put("range.th", th);

	pt.put("top.x", 400);
	pt.put("top.y", 500);
	pt.put("top.th", 360);

	pt.put("bottom.x", -400);
	pt.put("bottom.y", -50);
	pt.put("bottom.th", 0);

	write_ini("config/SpaceConfig.ini", pt);
}


void TaskSet::set_handtype()
{
	ptree pt;
	read_ini("config/RobotConfig.ini", pt);

	boost::optional<int> box = pt.get_optional<int>("size.Lh1");
	pt.put("size.Lh1", box.get());
	box = pt.get_optional<int>("size.Lh2");
	pt.put("size.Lh2", box.get());
	box = pt.get_optional<int>("size.Lh3");
	pt.put("size.Lh3", box.get());
	box = pt.get_optional<int>("size.Lh4");
	pt.put("size.Lh4", box.get());
	box = pt.get_optional<int>("size.Lw1");
	pt.put("size.Lw1", box.get());
	box = pt.get_optional<int>("size.Lw2");
	pt.put("size.Lw2", box.get());
	box = pt.get_optional<int>("size.Lw3");
	pt.put("size.Lw3", box.get());
	box = pt.get_optional<int>("size.Lw4");
	pt.put("size.Lw4", box.get());

	box = pt.get_optional<int>("size.Rh1");
	pt.put("size.Rh1", box.get());
	box = pt.get_optional<int>("size.Rh2");
	pt.put("size.Rh2", box.get());
	box = pt.get_optional<int>("size.Rh3");
	pt.put("size.Rh3", box.get());
	box = pt.get_optional<int>("size.Rh4");
	pt.put("size.Rh4", box.get());
	box = pt.get_optional<int>("size.Rw1");
	pt.put("size.Rw1", box.get());
	box = pt.get_optional<int>("size.Rw2");
	pt.put("size.Rw2", box.get());
	box = pt.get_optional<int>("size.Rw3");
	pt.put("size.Rw3", box.get());
	box = pt.get_optional<int>("size.Rw4");
	pt.put("size.Rw4", box.get());

	int htype = 0;
	do {
		std::cout << "Choose hand type:" << std::endl;
		std::cout << "(Parallel hand -> 1 , Opposite hand -> 2): ";
		std::cin >> htype;
	} while (htype <= 0 || htype > 2);
	if (htype == 1) {
		pt.put("origin.rx", 30.01);		pt.put("origin.ry", 0.0);

		pt.put("coordinate.r00", 1);	pt.put("coordinate.r01", 0);
		pt.put("coordinate.r10", 0);	pt.put("coordinate.r11", 1);
	}
	if (htype == 2) {
		pt.put("origin.rx", 30.01);		pt.put("origin.ry", 415.0);

		pt.put("coordinate.r00", -1);	pt.put("coordinate.r01", 0);
		pt.put("coordinate.r10", 0);	pt.put("coordinate.r11", -1);
	}

	write_ini("config/RobotConfig.ini", pt);
}

void TaskSet::set_discretization()
{
	ptree space;
	read_ini("config/SpaceConfig.ini", space);
	
	boost::optional<int> box = space.get_optional<int>("top.x");
	space.put("top.x", box.get());
	box = space.get_optional<int>("top.y");
	space.put("top.y", box.get());
	box = space.get_optional<int>("top.th");
	space.put("top.th", box.get());

	box = space.get_optional<int>("bottom.x");
	space.put("bottom.x", box.get());
	box = space.get_optional<int>("bottom.y");
	space.put("bottom.y", box.get());
	box = space.get_optional<int>("bottom.th");
	space.put("bottom.th", box.get());

	std::cout << "Set the discretized distance\n";
	int dx, dy, dphi;
	std::cout << "dx   -> ";	std::cin >> dx;
	std::cout << "dy   -> ";	std::cin >> dy;
	std::cout << "dphi -> ";	std::cin >> dphi;
	space.put("range.x", dx);
	space.put("range.y", dy);
	space.put("range.th", dphi);
}

void TaskSet::check()
{
	ptree robot, problem, object, space;
	read_ini("config/RobotConfig.ini", robot);
	read_ini("config/ProblemDefine.ini", problem);
	read_ini("config/ObjectParameter.ini", object);
	read_ini("config/SpaceConfig.ini", space);

	boost::optional<int> handtype = robot.get_optional<int>("coordinate.r00");
	if(handtype.get() == 1)		std::cout << "\'Parallel Hand\'" << std::endl;
	if(handtype.get() == -1)	std::cout << "\'Opposite Hand\'" << std::endl;

	boost::optional<double> th = problem.get_optional<double>("start.th1");
	std::cout << "start                : [" << th.get() << ", ";
	th = problem.get_optional<double>("start.th2");
	std::cout << th.get() << ", ";
	th = problem.get_optional<double>("start.th3");
	std::cout << th.get() << ", ";
	th = problem.get_optional<double>("start.th4");
	std::cout << th.get() << ", ";
	th = problem.get_optional<double>("start.th5");
	std::cout << th.get() << ", ";
	th = problem.get_optional<double>("start.th6");
	std::cout << th.get() << "]" << std::endl;

	th = problem.get_optional<double>("finish.th1");
	std::cout << "finish               : [" << th.get() << ", ";
	th = problem.get_optional<double>("finish.th2");
	std::cout << th.get() << ", ";
	th = problem.get_optional<double>("finish.th3");
	std::cout << th.get() << ", ";
	th = problem.get_optional<double>("finish.th4");
	std::cout << th.get() << ", ";
	th = problem.get_optional<double>("finish.th5");
	std::cout << th.get() << ", ";
	th = problem.get_optional<double>("finish.th6");
	std::cout << th.get() << "]" << std::endl;

	boost::optional<int> shape_index = problem.get_optional<int>("object.shape");
	std::cout << "Shape                : ";
	if(shape_index == 1)		std::cout << "Rectangle" << std::endl;
	else if(shape_index == 2)	std::cout << "L-Shape" << std::endl;
	else if(shape_index == 3)	std::cout << "Triangle" << std::endl;
	else if(shape_index == 4)	std::cout << "T-Shape" << std::endl;
	else						std::cout << "ERROR!" << std::endl;


	boost::optional<int> x, y, z;
	x = problem.get_optional<int>("goal.coordx");
	y = problem.get_optional<int>("goal.coordy");
	z = problem.get_optional<int>("goal.coordt");
	std::cout << "Target coordinate    : [" << x.get() << ", " << y.get() << ", " << z.get() << "]" << std::endl;

	boost::optional<int> epsilon = problem.get_optional<int>("goal.epsilon");
	std::cout << "Threshold epsilon    : " << epsilon.get() << std::endl;

	boost::optional<int> delta = space.get_optional<int>("range.x");
	std::cout << "Discretized distance : [" << delta.get();
	delta = space.get_optional<int>("range.y");
	std::cout << ", " << delta.get() << ", ";
	delta = space.get_optional<int>("range.th");
	std::cout << delta.get() << "]\n";
}




//void TaskSet::robotconfig()
//{
//	ptree pt;
//
//	int htype = 0;
//	do {
//		std::cout << "Choose hand type:" << std::endl;
//		std::cout << "(Parallel hand -> 1 , Opposite hand -> 2): ";
//		std::cin >> htype;
//	} while (htype <= 0 || htype > 2);
//	if (htype == 1) {
//		pt.put("origin.rx", 30.01);		pt.put("origin.ry", 0.0);
//
//		pt.put("coordinate.r00", 1);	pt.put("coordinate.r01", 0);
//		pt.put("coordinate.r10", 0);	pt.put("coordinate.r11", 1);
//	}
//	if (htype == 2) {
//		pt.put("origin.rx", 30.01);		pt.put("origin.ry", 415.0);
//
//		pt.put("coordinate.r00", -1);	pt.put("coordinate.r01", 0);
//		pt.put("coordinate.r10", 0);	pt.put("coordinate.r11", -1);
//	}
//
//	pt.put("size.Lh1", 38);		pt.put("size.Lh2", 130);
//	pt.put("size.Lh3", 130);	pt.put("size.Lh4", 90);
//	pt.put("size.Lw1", 60);		pt.put("size.Lw2", 35);
//	pt.put("size.Lw3", 35);		pt.put("size.Lw4", 35);
//	pt.put("size.Rh1", 38);		pt.put("size.Rh2", 130);
//	pt.put("size.Rh3", 130);	pt.put("size.Rh4", 90);
//	pt.put("size.Rw1", 60);		pt.put("size.Rw2", 35);
//	pt.put("size.Rw3", 35);		pt.put("size.Rw4", 35);
//
//	pt.put("origin.lx", -30.0);		pt.put("origin.ly", 0.0);
//	pt.put("coordinate.l00", 1);	pt.put("coordinate.l01", 0);
//	pt.put("coordinate.l10", 0);	pt.put("coordinate.l11", 1);
//
//	write_ini("config/RobotConfig.ini", pt);
//	std::cout << std::endl;
//}


//void TaskSet::object()
//{
//	ptree pt;
//
//	pt.put("Rectangle.long_side", 100);
//	pt.put("Rectangle.short_side", 70);
//	pt.put("Rectangle.symmetry", 180);
//
//	pt.put("LShape.long_side", 150);
//	pt.put("LShape.short_side", 100);
//	pt.put("LShape.long_pro", 40);
//	pt.put("LShape.short_pro", 40);
//	pt.put("LShape.symmetry", 360);
//
//	pt.put("Triangle.edge1", 150);
//	pt.put("Triangle.edge2", 150);
//	pt.put("Triangle.edge3", 150);
//	pt.put("Triangle.symmetry", 120);
//
//	pt.put("TShape.long_side", 150);
//	pt.put("TShape.short_side", 100);
//	pt.put("TShape.long_pro", 40);
//	pt.put("TShape.short_pro", 40);
//	pt.put("TShape.symmetry", 360);
//
//	write_ini("config/ObjectParameter.ini", pt);
//	std::cout << std::endl;
//}


