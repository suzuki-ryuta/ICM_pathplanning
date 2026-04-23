#include "OneHand.h"
#include "Square.h"


OneHand::OneHand(HandInfo info)
	:hand(make_hand(info)), origin(info.origin)
{
}


int OneHand::hand_size()
{
	return (int)hand.size();
}


void OneHand::update(HalfNode hn)
{
	hand[0].update(0.0, origin);
	hand[1].update(hn.get_element(0), hand[0].get_top());
	hand[2].update(hn.get_element(1), hand[1].get_top());
	hand[3].update(hn.get_element(2), hand[2].get_top());
}


bool OneHand::opposite_intersect(int No, OneHand other)
{
	for (int i = 0; i < other.hand_size(); i++) {
		if (hand[No].intersect(other.hand[i].get_square()))	return true;
	}
	return false;
}


bool OneHand::rr_intersect(OneHand other)
{
	for (int i = 0; i < (int)hand.size(); ++i) {
		if (opposite_intersect(i, other) == true)	return true;
	}
	return false;
}


bool OneHand::intersect(Square poly)
{
	if (!simple_check(poly))	return false;

	for (int i = 0; i < (int)hand.size(); ++i) {
		if (hand[i].intersect(poly))		return true;
	}

	return false;
}


bool OneHand::intersect(MultiSquare poly)
{
	for (int i = 0; i < (int)poly.size(); ++i) {
		if(intersect(poly.element(i)))	return true;
	}

	return false;
}


bool OneHand::intersect(Triangulus poly)
{
	//if (!simple_check(poly))	return false;

	for (int i = 0; i < (int)hand.size(); ++i) {
		if (hand[i].intersect(poly))		return true;
	}

	return false;
}


bool OneHand::simple_check(Square poly)		// Not intersect -> false
{
	for (int i = 0; i < (int)hand.size(); ++i) {
		double dist = distance(hand[i].get_center(), poly.get_center());
		if (dist < (hand[i].get_radius() + poly.get_radius()))
			return true;
	}

	return false;
}


//bool OneHand::simple_check(Triangulus poly)		// Not intersect -> false
//{
//	for (int i = 0; i < (int)hand.size(); ++i) {
//		double dist = distance(hand[i].get_center(), poly.get_center());
//		if (dist < (hand[i].get_radius() + poly.get_radius()))
//			return true;
//	}
//
//	return false;
//}

//bool OneHand::simple_check(MultiSquare poly)
//{
//	for (int i = 0; i < (int)poly.size(); ++i) {
//		if (simple_check(poly.element(i)))	return true;
//	}
//
//	return false;
//}


Link OneHand::get_link(int dof) { return hand[dof]; }


std::vector<Link> make_hand(HandInfo info)
{
	RecSize rs0(info.height[0], info.width[0]);
	Link L0(rs0, info.coord, info.origin);

	RecSize rs1(info.height[1], info.width[1]);
	Link L1(rs1, info.coord, L0.get_top());

	RecSize rs2(info.height[2], info.width[2]);
	Link L2(rs2, info.coord, L1.get_top());

	RecSize rs3(info.height[3], info.width[3]);
	Link L3(rs3, info.coord, L2.get_top());

	std::vector<Link> tmp;
	tmp.push_back(L0);
	tmp.push_back(L1);
	tmp.push_back(L2);
	tmp.push_back(L3);

	return tmp;
}
