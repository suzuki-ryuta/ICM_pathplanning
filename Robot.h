#pragma once

#include "OneHand.h"


class Robot {
private:
	OneHand left;
	OneHand right;

public:
	Robot();
	void update(Node joint_angle);

	bool rr_intersect();
	bool intersect(Square poly);
	bool intersect(MultiSquare poly);
	bool intersect(Triangulus poly);

	Link get_link(int dof);

};


HandInfo get_lhand_info();
HandInfo get_rhand_info();
