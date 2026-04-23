#pragma once

#include <cassert>
#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>

#include "PointCloud.h"
#include "icmMath.h"


namespace bp = boost::property_tree;


int read_symangle(); //シンメトリーアングル
State3D read_top();
State3D read_bottom();
Vector3D<int> read_range();


class CSpaceConfig
{
private:
	State3D top, bottom;
	Vector3D<int> range;
	int symangle;
	int numx, numy, numth;

	CSpaceConfig();
	static CSpaceConfig* instance;

public:
	static CSpaceConfig* get_instance();

	State3D gettop(){return top;}
	State3D getbottom(){return bottom;}
	Vector3D<int> getrange(){return range;}
	int getsymangle(){return symangle;}
	int getnumx(){return numx;}
	int getnumy(){return numy;}
	int getnumth(){return numth;}
};


// To deal with whole discretization C-Space
struct CSpace
{
	std::vector<PointMark> elm;

	CSpace()
	{
		CSpaceConfig* cs = CSpaceConfig::get_instance();
		elm.resize(cs->getnumx() * cs->getnumy() * cs->getnumth());

		int debug = 0;
		for (int i = 0; i < cs->getnumx(); ++i) {
			for (int j = 0; j < cs->getnumy(); ++j) {
				for (int k = 0; k < cs->getnumth(); ++k) {
					State3D pos(cs->getbottom().x + cs->getrange().x * i,
						        cs->getbottom().y + cs->getrange().y * j,
						        cs->getbottom().th + cs->getrange().z * k);

					int index = i*cs->getnumy()*cs->getnumth() + j*cs->getnumth() + k;
					elm[index] = PointMark(pos, false);
					debug++;
				}
			}
		}
		assert(debug == cs->getnumx() * cs->getnumy() * cs->getnumth());
	}

	void init()
	{
		for(int i=0; i<(int)elm.size(); ++i){
			elm[i].mk = false;
		}
	}

	int size(){
		return (int)elm.size();
	}

	State3D get_pt(int i){
		return elm[i].pt;
	}

	void toFalse(int i)
	{
		elm[i].toFalse();
	}
	void toTrue(int i)
	{
		elm[i].toTrue();
	}
	int coord_to_index(State3D st)		//test later
	{
		CSpaceConfig* cs = CSpaceConfig::get_instance();
		int dx = (st.x - cs->getbottom().x) / cs->getrange().x;
		int dy = (st.y - cs->getbottom().y) / cs->getrange().y;
		int dth = (st.th - cs->getbottom().th) / cs->getrange().z;
		
		assert((st.x - cs->getbottom().x) % cs->getrange().x == 0);
		assert((st.y - cs->getbottom().y) % cs->getrange().y == 0);
		assert((st.th - cs->getbottom().th) % cs->getrange().z == 0);

		return dx * (cs->getnumy() * cs->getnumth()) + dy * (cs->getnumth()) + dth;
	}

	State3D index_to_coord(int index)	// test later
	{
		CSpaceConfig* cs = CSpaceConfig::get_instance();
		int x = index / (cs->getnumy() * cs->getnumth());
		int resx = index % (cs->getnumy() * cs->getnumth());
		int y = resx / (cs->getnumth());
		int th = resx % (cs->getnumth());

		return State3D(cs->getbottom().x  + x * cs->getrange().x
					 , cs->getbottom().y  + y * cs->getrange().y
					 , cs->getbottom().th + th* cs->getrange().z);
	}
};
