#pragma once

#include <vector>
#include "Link.h"
#include "Node.h"

struct HandInfo
{
	std::vector<int> height;
	std::vector<int> width;
	Point2D origin;
	Mat22<int> coord;
};

class OneHand
{
private:
	std::vector<Link> hand;
	const Point2D origin;

public:
	OneHand(HandInfo info);

	int hand_size();
	void update(HalfNode hn);
	bool opposite_intersect(int No, OneHand other);
	bool rr_intersect(OneHand other);
	bool intersect(Square poly);
	bool intersect(MultiSquare poly);
	bool intersect(Triangulus poly);
	bool simple_check(Square poly);
	// bool OneHand::simple_check(MultiSquare poly)

	Link get_link(int dof);
};


std::vector<Link> make_hand(HandInfo info);
