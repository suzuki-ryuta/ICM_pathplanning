#pragma once
#include <cassert>

#include "CSpace.h"
#include "Node.h"
#include "Controller.h"
#include "Labeling.h"


bool edge_judge(State3D pt);
bool contain_edge(PointCloud pc, int edge);


class CFreeICS
{
private:
	std::vector<PointCloud> c_free_ics;
	CSpace cspace;
	Node nownode;

public:
	CFreeICS(Node node)
		:c_free_ics(), cspace(), nownode(node){}

	std::vector<PointCloud> extract() //c_free_icsの抽出
	{
		Controller* controller = Controller::get_instance();
		CSpaceConfig* conf = CSpaceConfig::get_instance();	
		controller->robot_update(nownode);

		for(int i=0; i<cspace.size(); ++i){
			State3D pos = cspace.elm[i].pt; //th増加終了→y増加開始，y増加終了→x増加開始
			controller->shape_update(pos); //座標に基づいて対象物のデータを生成
			if(controller->RintersectS())	    cspace.toFalse(i); //ロボットと干渉する座標を排除
			else if(controller->WintersectS())	cspace.toFalse(i); //壁と干渉する座標を排除
			else                                cspace.toTrue(i); //残った座標をCfreeとして抽出．以降ICSを抽出
		}

		Labeling* label = new Labeling(cspace); //クラスター番号を管理するための配列を生成
		int clust = label->labeling3D(); //隣接する格子点を統合し，クラスターを生成．その後クラスターの数をカウント
		c_free_ics.resize(clust); //ICSのクラスタ数

		for (int ix = 0; ix < conf->getnumx(); ++ix)
			for (int iy = 0; iy < conf->getnumy(); ++iy)
				for (int ith = 0; ith < conf->getnumth(); ++ith) 
					distribute(label, ix, iy, ith); //格子点をクラスターに割り振る

		delete label;

		int exit_flag = 0;
		for(auto it=c_free_ics.begin(); it!=c_free_ics.end();){
			exit_flag = 0;
			for(int i=0; i<(*it).size(); ++i){
				if(edge_judge((*it).elm[i])){
					it = c_free_ics.erase(it); //Pfarを含むクラスタを弾く
					exit_flag = 1;
					break;
				}
			}
			if(exit_flag == 1)	continue;
			++it;
		}

		if(c_free_ics.size() == 0)	return c_free_ics;

		merge(conf->getsymangle());
		return c_free_ics;
	}

	void distribute(Labeling* lbel, int ix, int iy, int ith)
	{
		CSpaceConfig* conf = CSpaceConfig::get_instance();	
		State3D btm = conf->getbottom();
		Vector3D<int> rng = conf->getrange();
		int area = lbel->get_label(ix, iy, ith);
		if (area != 0)
			c_free_ics[area - 1].push(State3D(btm.x + rng.x*ix, btm.y + rng.y*iy, btm.th + rng.z*ith));
		
	}

	void merge(int edge)
	{
		std::vector<PointCloud> subject;
		std::vector<PointCloud> rest;

		for(const auto& e : c_free_ics){
			if(contain_edge(e, edge))	subject.push_back(e);
			else                        rest.push_back(e);
		}
	
		PCMerge mg(subject, edge);
		mg.merge();
	
		c_free_ics = mg.getter();
		c_free_ics.insert(c_free_ics.end(), rest.begin(), rest.end());

	}


};


