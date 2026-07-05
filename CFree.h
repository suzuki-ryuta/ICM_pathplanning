#pragma once
#include <stack>

#include "PointCloud.h"
#include "CSpace.h"
#include "Controller.h"
#include "CFreeICS.h"
#include "Profiler.h"

class CFO
{
public:
	virtual std::vector<PointCloud> extract(PointCloud prev, Node newnode) = 0;

	virtual ~CFO(){}

protected:
	CFO(){}
};


class RasterCFO : public CFO
{
private:
	std::vector<PointCloud> c_freeobj;

	void init()
	{
		c_freeobj.clear();
		c_freeobj.shrink_to_fit();
	}


public:
	RasterCFO(){};
	~RasterCFO(){};

	std::vector<PointCloud> extract(PointCloud prev, Node newnode){
		PROFILE_SCOPE("RasterCFO.extract.total");
		init();
		CFreeICS cfi(newnode);
		{
			PROFILE_SCOPE("RasterCFO.ICS_extract");
			c_freeobj = cfi.extract();
		}

		for(auto itr = c_freeobj.begin(); itr != c_freeobj.end();){
			bool has_overlap = false;
			{
				PROFILE_SCOPE("RasterCFO.cluster_overlap");
				has_overlap = prev.overlap(*itr);
			}
			if(has_overlap){
				++itr;	continue;
			}
			itr = c_freeobj.erase(itr);
		}

		return c_freeobj;
	}
};



class DfsCFO : public CFO
{
private:
	std::vector<PointCloud> c_dfs;
	std::vector<PointCloud> c_del;
	CSpace target;
	PointMarkCloud previous;
	bool cancel;	// if "true", stop explorartion. "false" in the begining.

	void init()
	{
		c_dfs.clear();
		c_dfs.shrink_to_fit();
		c_del.clear();
		c_del.shrink_to_fit();
		target.init();
		previous.init();
		cancel = false;
	}

public:
	DfsCFO()
		:c_dfs(), c_del(), target(), cancel(false)
	{
	}
	~DfsCFO(){}

	std::vector<PointCloud> extract(PointCloud prev, Node newnode)
	{
		PROFILE_SCOPE("DfsCFO.extract.total");
		init();
		{
			PROFILE_SCOPE("DfsCFO.previous_assign");
			previous.assign(prev);
		}
		Controller* controller = Controller::get_instance();
		{
			PROFILE_SCOPE("DfsCFO.robot_update");
			controller->robot_update(newnode);	
		}
		// robot condition is updated, below we can use 
		// no argument function in Controller.h like RintersectS().

		for (int i = 0; i < previous.size(); ++i) {
			if (previous.get_mk(i) == true)	continue;
			previous.toTrue(i);		// Change the state to "True", 
									// which means the point has been explored.

			target.init();
									
			{
				PROFILE_SCOPE("DfsCFO.seed_shape_update");
				controller->shape_update(previous.get_pt(i));
			}
			bool robot_hit = false;
			{
				PROFILE_SCOPE("DfsCFO.seed_RintersectS");
				robot_hit = controller->RintersectS();
			}
			if (robot_hit)	continue;
			bool wall_hit = false;
			{
				PROFILE_SCOPE("DfsCFO.seed_WintersectS");
				wall_hit = controller->WintersectS();
			}
			if (wall_hit)	continue;

			c_dfs.resize(c_dfs.size() + 1);
			int index = target.coord_to_index(previous.get_pt(i));
			target.toTrue(index);
			c_dfs.back().push(previous.get_pt(i));
			explore2(previous.get_pt(i));
		}

		return c_dfs;
	}

	bool preprocess(State3D& pt)
	{
		PROFILE_SCOPE("DfsCFO.preprocess");
		CSpaceConfig* cs = CSpaceConfig::get_instance();
		if(pt.th < 0)	            pt.th = pt.th + (cs->gettop().th + cs->getrange().z);
		if(pt.th > cs->gettop().th)	pt.th = pt.th - (cs->gettop().th + cs->getrange().z);
		if(pt.x < cs->getbottom().x || pt.x > cs->gettop().x) return false;
		if(pt.y < cs->getbottom().y || pt.y > cs->gettop().y) return false;
		if(pt.th < cs->getbottom().th || pt.th > cs->gettop().th) return false;

		int index = target.coord_to_index(pt);
		check_ifexist(pt);

		if (target.elm[index].mk == true)	return false;
		target.toTrue(index);
		return true;
	}

	void explore2(State3D point)
	{	
		PROFILE_SCOPE("DfsCFO.explore2.total");
		Controller* controller = Controller::get_instance();
		for (int i = 1; i <= 26; ++i) {
			std::stack<State3D> stack;
			State3D orig = move(point, i);
			assert(stack.size() == 0);

			if (!preprocess(orig))	continue;
			bool is_edge = false;
			{
				PROFILE_SCOPE("DfsCFO.edge_judge");
				is_edge = edge_judge(orig);
			}
			if (is_edge) {
				c_del.push_back(c_dfs.back());
				c_dfs.pop_back();
				return;
			}

			{
				PROFILE_SCOPE("DfsCFO.shape_update");
				controller->shape_update(orig);	// shape is updated
			}
			bool robot_hit = false;
			{
				PROFILE_SCOPE("DfsCFO.RintersectS");
				robot_hit = controller->RintersectS();
			}
			if (robot_hit)	continue;
			bool wall_hit = false;
			{
				PROFILE_SCOPE("DfsCFO.WintersectS");
				wall_hit = controller->WintersectS();
			}
			if (wall_hit)	continue;

			bool cdel_hit = false;
			{
				PROFILE_SCOPE("DfsCFO.cdel_cluster_overlap");
				cdel_hit = cdel_judge(orig);
			}
			if(cdel_hit){
				c_del.push_back(c_dfs.back());
				c_dfs.pop_back();
				return;
			}

			c_dfs.back().push(orig);
			stack.emplace(orig);

			while (stack.size() != 0)
			{
				State3D pt = stack.top();
				stack.pop();
				for (int i = 1; i <= 26; ++i) {
					State3D next = move(pt, i);

					if (!preprocess(next))	continue;
					bool next_is_edge = false;
					{
						PROFILE_SCOPE("DfsCFO.edge_judge");
						next_is_edge = edge_judge(next);
					}
					if (next_is_edge) {
						c_del.push_back(c_dfs.back());
						c_dfs.pop_back();
						return;
					}

					{
						PROFILE_SCOPE("DfsCFO.shape_update");
						controller->shape_update(next);	// shape is updated
					}
					bool next_robot_hit = false;
					{
						PROFILE_SCOPE("DfsCFO.RintersectS");
						next_robot_hit = controller->RintersectS();
					}
					if (next_robot_hit)	continue;
					bool next_wall_hit = false;
					{
						PROFILE_SCOPE("DfsCFO.WintersectS");
						next_wall_hit = controller->WintersectS();
					}
					if (next_wall_hit)	continue;

					bool next_cdel_hit = false;
					{
						PROFILE_SCOPE("DfsCFO.cdel_cluster_overlap");
						next_cdel_hit = cdel_judge(next);
					}
					if(next_cdel_hit){
						c_del.push_back(c_dfs.back());
						c_dfs.pop_back();
						return;
					}
					c_dfs.back().push(next);
					stack.emplace(next);
				}
			}
		}
	}

	State3D move(State3D pt, int dir)
	{
		CSpaceConfig* cs = CSpaceConfig::get_instance();

		if (dir == 1)		return State3D(pt.x - cs->getrange().x, pt.y - cs->getrange().y, pt.th - cs->getrange().z);
		if (dir == 2)		return State3D(pt.x,                    pt.y - cs->getrange().y, pt.th - cs->getrange().z);
		if (dir == 3)		return State3D(pt.x + cs->getrange().x, pt.y - cs->getrange().y, pt.th - cs->getrange().z);
		if (dir == 4)		return State3D(pt.x - cs->getrange().x, pt.y,                    pt.th - cs->getrange().z);
		if (dir == 5)		return State3D(pt.x,                    pt.y,                    pt.th - cs->getrange().z);
		if (dir == 6)		return State3D(pt.x + cs->getrange().x, pt.y,                    pt.th - cs->getrange().z);
		if (dir == 7)		return State3D(pt.x - cs->getrange().x, pt.y + cs->getrange().y, pt.th - cs->getrange().z);
		if (dir == 8)		return State3D(pt.x,                    pt.y + cs->getrange().y, pt.th - cs->getrange().z);
		if (dir == 9)		return State3D(pt.x + cs->getrange().x, pt.y + cs->getrange().y, pt.th - cs->getrange().z);

		if (dir == 10)		return State3D(pt.x - cs->getrange().x, pt.y - cs->getrange().y, pt.th);
		if (dir == 11)		return State3D(pt.x,                    pt.y - cs->getrange().y, pt.th);
		if (dir == 12)		return State3D(pt.x + cs->getrange().x, pt.y - cs->getrange().y, pt.th);
		if (dir == 13)		return State3D(pt.x - cs->getrange().x, pt.y,                    pt.th);
		if (dir == 14)		return State3D(pt.x + cs->getrange().x, pt.y,                    pt.th);
		if (dir == 15)		return State3D(pt.x - cs->getrange().x, pt.y + cs->getrange().y, pt.th);
		if (dir == 16)		return State3D(pt.x,                    pt.y + cs->getrange().y, pt.th);
		if (dir == 17)		return State3D(pt.x + cs->getrange().x, pt.y + cs->getrange().y, pt.th);

		if (dir == 18)		return State3D(pt.x - cs->getrange().x, pt.y - cs->getrange().y, pt.th + cs->getrange().z);
		if (dir == 19)		return State3D(pt.x,                    pt.y - cs->getrange().y, pt.th + cs->getrange().z);
		if (dir == 20)		return State3D(pt.x + cs->getrange().x, pt.y - cs->getrange().y, pt.th + cs->getrange().z);
		if (dir == 21)		return State3D(pt.x - cs->getrange().x, pt.y,                    pt.th + cs->getrange().z);
		if (dir == 22)		return State3D(pt.x,                    pt.y,                    pt.th + cs->getrange().z);
		if (dir == 23)		return State3D(pt.x + cs->getrange().x, pt.y,                    pt.th + cs->getrange().z);
		if (dir == 24)		return State3D(pt.x - cs->getrange().x, pt.y + cs->getrange().y, pt.th + cs->getrange().z);
		if (dir == 25)		return State3D(pt.x,                    pt.y + cs->getrange().y, pt.th + cs->getrange().z);
		if (dir == 26)		return State3D(pt.x + cs->getrange().x, pt.y + cs->getrange().y, pt.th + cs->getrange().z);

		assert(true);
		return State3D();
	}

	void check_ifexist(State3D st)	// If st exists, convert to "true" correspond PointMark of previous C_free_obj
	{
		PROFILE_SCOPE("DfsCFO.previous_overlap_check");
		for (int i = 0; i < (int)previous.size(); ++i) {
			if (previous.get_pt(i) == st) {
				previous.toTrue(i);
				return;
			}
		}
	}

	PointCloud get_cfree_obj()
	{
		return c_dfs[0];
	}

//	To judge whether 'st' is a part of c_del or not.
//	If 'st' is a part, return true
	bool cdel_judge(State3D st)
	{
		PROFILE_SCOPE("DfsCFO.cdel_judge.inner");
		for(int i=0; i<(int)c_del.size(); ++i){
			if(c_del[i].exist(st))	return true;
		}
		return false;
	}


};





