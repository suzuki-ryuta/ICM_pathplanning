#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>

#include "Robot.h"
#include "OneHand.h"

namespace bp = boost::property_tree;


Robot::Robot()
	:left(get_lhand_info()),
	right(get_rhand_info())
{
}


void Robot::update(Node joint_angle)
{
	left.update(joint_angle.get_langles());
	right.update(joint_angle.get_rangles());
}


bool Robot::rr_intersect()
{
	return right.rr_intersect(left);	// intersect -> true : Not intersect -> false
}


bool Robot::intersect(Square poly)
{
	if (left.intersect(poly))	return true;
	if (right.intersect(poly))	return true;
	return false;
}

bool Robot::intersect(MultiSquare poly)
{
	if (left.intersect(poly))	return true;
	if (right.intersect(poly))	return true;
	return false;
}

bool Robot::intersect(Triangulus poly)
{
	if (left.intersect(poly))	return true;
	if (right.intersect(poly))	return true;
	return false;
}


Link Robot::get_link(int dof)
{
	assert(dof < 8);
	if (dof < 4)	return left.get_link(dof);
	else            return right.get_link(dof - 4);
}


HandInfo get_lhand_info()
{
	bp::ptree pt;
	read_ini("config/RobotConfig.ini", pt);

	// Hand size setup	
	std::vector<int> Lheight(4);
	std::vector<int> Lwidth(4);

	boost::optional<int> carrier = pt.get_optional<int>("size.Lh1");
	Lheight[0] = carrier.get();
	carrier = pt.get_optional<int>("size.Lh2");
	Lheight[1] = carrier.get();
	carrier = pt.get_optional<int>("size.Lh3");
	Lheight[2] = carrier.get();
	carrier = pt.get_optional<int>("size.Lh4");
	Lheight[3] = carrier.get();

	carrier = pt.get_optional<int>("size.Lw1");
	Lwidth[0] = carrier.get();
	carrier = pt.get_optional<int>("size.Lw2");
	Lwidth[1] = carrier.get();
	carrier = pt.get_optional<int>("size.Lw3");
	Lwidth[2] = carrier.get();
	carrier = pt.get_optional<int>("size.Lw4");
	Lwidth[3] = carrier.get();


	// Hand origin setup
	boost::optional<float> carrier2 = pt.get_optional<float>("origin.lx");
	double x = carrier2.get();
	carrier2 = pt.get_optional<float>("origin.ly");
	double y = carrier2.get();
	Point2D lorg(x, y);


	// Hand coordinate setup
	// 		--        --
	// 		|  00  10  |
	//		|  01  11  |
	// 		--        --
	int p00 = 0, p01 = 0, p10 = 0, p11 = 0;
	carrier = pt.get_optional<int>("coordinate.l00");
	p00 = carrier.get();
	carrier = pt.get_optional<int>("coordinate.l01");
	p01 = carrier.get();
	carrier = pt.get_optional<int>("coordinate.l10");
	p10 = carrier.get();
	carrier = pt.get_optional<int>("coordinate.l11");
	p11 = carrier.get();
	Mat22<int> lmat(p00, p01, p10, p11);


	// Integrate Robot hand information
	HandInfo linfo = { Lheight, Lwidth, lorg, lmat };

	return linfo;
}

HandInfo get_rhand_info()
{
	bp::ptree pt;
	read_ini("config/RobotConfig.ini", pt);

	// Hand size setup	
	std::vector<int> Rheight(4);
	std::vector<int> Rwidth(4);

	boost::optional<int> carrier = pt.get_optional<int>("size.Rh1");
	Rheight[0] = carrier.get();
	carrier = pt.get_optional<int>("size.Rh2");
	Rheight[1] = carrier.get();
	carrier = pt.get_optional<int>("size.Rh3");
	Rheight[2] = carrier.get();
	carrier = pt.get_optional<int>("size.Rh4");
	Rheight[3] = carrier.get();

	carrier = pt.get_optional<int>("size.Rw1");
	Rwidth[0] = carrier.get();
	carrier = pt.get_optional<int>("size.Rw2");
	Rwidth[1] = carrier.get();
	carrier = pt.get_optional<int>("size.Rw3");
	Rwidth[2] = carrier.get();
	carrier = pt.get_optional<int>("size.Rw4");
	Rwidth[3] = carrier.get();


	// Hand origin setup
	boost::optional<float> carrier2 = pt.get_optional<float>("origin.rx");
	double x = carrier2.get();
	carrier2 = pt.get_optional<float>("origin.ry");
	double y = carrier2.get();
	Point2D rorg(x, y);

	// Hand coordinate setup
	// 		--        --
	// 		|  00  10  |
	//		|  01  11  |
	// 		--        --

	int p00 = 0, p01 = 0, p10 = 0, p11 = 0;
	carrier = pt.get_optional<int>("coordinate.r00");
	p00 = carrier.get();
	carrier = pt.get_optional<int>("coordinate.r01");
	p01 = carrier.get();
	carrier = pt.get_optional<int>("coordinate.r10");
	p10 = carrier.get();
	carrier = pt.get_optional<int>("coordinate.r11");
	p11 = carrier.get();
	Mat22<int> rmat(p00, p01, p10, p11);

	// Integrate Robot hand information
	HandInfo rinfo = { Rheight, Rwidth, rorg, rmat };

	return rinfo;
}
