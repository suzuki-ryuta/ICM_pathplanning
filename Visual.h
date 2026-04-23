#pragma once

#include "Robot.h"
#include "Shape.h"
#include <string>
#include <fstream>

class Visual
{

public:
	static void robot_vis(Robot* robot)
	{
		std::string fn = "../robo.csv";
		std::ofstream ofs(fn);

		for (int i = 0; i < 8; ++i) {
			std::vector<Point2D> lp = robot->get_link(i).get_square().get_vertices();
			ofs << lp[0].x << "," << lp[0].y << std::endl;
			ofs << lp[1].x << "," << lp[1].y << std::endl;
			ofs << lp[2].x << "," << lp[2].y << std::endl;
			ofs << lp[3].x << "," << lp[3].y << std::endl; 
			ofs << lp[0].x << "," << lp[0].y << std::endl;
			ofs << std::endl;
		}
	}

	static void rs_vis(Robot* robot, Shape* shape)
	{
		std::string fn = "../visualize.csv";
		std::ofstream ofs(fn);

		for (int i = 0; i < 8; ++i) {
			std::vector<Point2D> lp = robot->get_link(i).get_square().get_vertices();
			ofs << lp[0].x << "," << lp[0].y << std::endl;
			ofs << lp[1].x << "," << lp[1].y << std::endl;
			ofs << lp[2].x << "," << lp[2].y << std::endl;
			ofs << lp[3].x << "," << lp[3].y << std::endl;
			ofs << lp[0].x << "," << lp[0].y << std::endl;
			ofs << std::endl;
		}

		std::vector<Point2D> sp = shape->get_poly().getter();
		for (int i = 0; i < (int)sp.size(); ++i) {
			ofs << sp[i].x << "," << sp[i].y << std::endl;
		}
		ofs << sp[0].x << "," << sp[0].y << std::endl;
		ofs << std::endl;
	}

};

