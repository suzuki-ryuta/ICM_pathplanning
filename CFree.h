#pragma once
#include <stack>

#include "PointCloud.h"
#include "CSpace.h"
#include "Controller.h"
#include "CFreeICS.h"

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
		init();
		CFreeICS cfi(newnode);
		c_freeobj = cfi.extract();

		for(auto itr = c_freeobj.begin(); itr != c_freeobj.end();){
			if(prev.overlap(*itr)){
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
	std::vector<int> previous_lookup;
	std::vector<int> previous_lookup_touched;
	std::vector<int> target_touched;
	bool cancel;	// if "true", stop explorartion. "false" in the begining.

	void init()
	{
		c_dfs.clear();
		c_dfs.shrink_to_fit();
		c_del.clear();
		c_del.shrink_to_fit();
		clear_target_marks();
		previous.init();
		clear_previous_lookup();
		cancel = false;
	}

	void clear_target_marks()
	{
		for (int index : target_touched) {
			target.toFalse(index);
		}
		target_touched.clear();
	}

	void mark_target(int index)
	{
		if (target.elm[index].mk == true) return;
		target.toTrue(index);
		target_touched.push_back(index);
	}

	void clear_previous_lookup()
	{
		for (int index : previous_lookup_touched) {
			previous_lookup[index] = -1;
		}
		previous_lookup_touched.clear();
	}

	void build_previous_lookup()
	{
		if ((int)previous_lookup.size() != target.size()) {
			previous_lookup.assign(target.size(), -1);
			previous_lookup_touched.clear();
		}
		if ((int)previous_lookup_touched.capacity() < previous.size()) {
			previous_lookup_touched.reserve(previous.size());
		}

		for (int i = 0; i < previous.size(); ++i) {
			int index = target.coord_to_index(previous.get_pt(i));
			if (previous_lookup[index] == -1) {
				previous_lookup_touched.push_back(index);
			}
			previous_lookup[index] = i;
		}
	}

public:
	DfsCFO()
		:c_dfs(), c_del(), target(), previous_lookup(), previous_lookup_touched(), target_touched(), cancel(false)
	{
	}
	~DfsCFO(){}

	std::vector<PointCloud> extract(PointCloud prev, Node newnode)
	{
		init();
		previous.assign(prev);
		build_previous_lookup();
		Controller* controller = Controller::get_instance();
		controller->robot_update(newnode);	
		// robot condition is updated, below we can use 
		// no argument function in Controller.h like RintersectS().

		for (int i = 0; i < previous.size(); ++i) {
			if (previous.get_mk(i) == true)	continue;
			previous.toTrue(i);		// Change the state to "True", 
									// which means the point has been explored.

			clear_target_marks();
									
			controller->shape_update(previous.get_pt(i));
			if (controller->RintersectS())	continue;
			if (controller->WintersectS())	continue;

			c_dfs.resize(c_dfs.size() + 1);
			int index = target.coord_to_index(previous.get_pt(i));
			mark_target(index);
			c_dfs.back().push(previous.get_pt(i));
			explore2(previous.get_pt(i));
		}

		return c_dfs;
	}

	bool preprocess(State3D& pt)
	{
		CSpaceConfig* cs = CSpaceConfig::get_instance();
		if(pt.th < 0)	            pt.th = pt.th + (cs->gettop().th + cs->getrange().z);
		if(pt.th > cs->gettop().th)	pt.th = pt.th - (cs->gettop().th + cs->getrange().z);
		if(pt.x < cs->getbottom().x || pt.x > cs->gettop().x) return false;
		if(pt.y < cs->getbottom().y || pt.y > cs->gettop().y) return false;
		if(pt.th < cs->getbottom().th || pt.th > cs->gettop().th) return false;

		int index = target.coord_to_index(pt);
		check_ifexist(index);

		if (target.elm[index].mk == true)	return false;
		mark_target(index);
		return true;
	}

	void explore2(State3D point)
	{	
		Controller* controller = Controller::get_instance();
		for (int i = 1; i <= 26; ++i) {
			std::stack<State3D> stack;
			State3D orig = move(point, i);
			assert(stack.size() == 0);

			if (!preprocess(orig))	continue;
			if (edge_judge(orig)) {
				c_del.push_back(c_dfs.back());
				c_dfs.pop_back();
				return;
			}

			controller->shape_update(orig);	// shape is updated
			if (controller->RintersectS())	continue;
			if (controller->WintersectS())	continue;

			if(cdel_judge(orig)){
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
					if (edge_judge(next)) {
						c_del.push_back(c_dfs.back());
						c_dfs.pop_back();
						return;
					}

					controller->shape_update(next);	// shape is updated
					if (controller->RintersectS())	continue;
					if (controller->WintersectS())	continue;

					if(cdel_judge(next)){
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

	void check_ifexist(int index)	// If st exists, convert to "true" correspond PointMark of previous C_free_obj
	{
		int previous_index = previous_lookup[index];
		if (previous_index != -1) {
			previous.toTrue(previous_index);
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
		for(int i=0; i<(int)c_del.size(); ++i){
			if(c_del[i].exist(st))	return true;
		}
		return false;
	}


};





