#pragma once
#include <cassert>
#include <cstdlib>

#include "CSpace.h"
#include "Node.h"
#include "Controller.h"
#include "Labeling.h"
#include "Profiler.h"


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
		PROFILE_SCOPE("ICS.extract.total");
		Controller* controller = Controller::get_instance();
		CSpaceConfig* conf = CSpaceConfig::get_instance();	
		{
			PROFILE_SCOPE("ICS.robot_update");
			controller->robot_update(nownode);
		}

		double shape_update_ms = 0.0;
		double robot_intersect_ms = 0.0;
		double wall_intersect_ms = 0.0;
		long long shape_update_calls = 0;
		long long robot_intersect_calls = 0;
		long long wall_intersect_calls = 0;
		const char* progress_env = std::getenv("ICS_PROFILE_PROGRESS_INTERVAL");
		const int progress_interval = progress_env ? std::atoi(progress_env) : 0;
		const auto scan_start = std::chrono::steady_clock::now();
		for(int i=0; i<cspace.size(); ++i){
			State3D pos = cspace.elm[i].pt; //th増加終了→y増加開始，y増加終了→x増加開始
			auto t0 = std::chrono::steady_clock::now();
			controller->shape_update(pos); //座標に基づいて対象物のデータを生成
			auto t1 = std::chrono::steady_clock::now();
			shape_update_ms += std::chrono::duration<double, std::milli>(t1 - t0).count();
			++shape_update_calls;
			bool robot_hit = false;
			t0 = std::chrono::steady_clock::now();
			robot_hit = controller->RintersectS();
			t1 = std::chrono::steady_clock::now();
			robot_intersect_ms += std::chrono::duration<double, std::milli>(t1 - t0).count();
			++robot_intersect_calls;
			if(robot_hit)	    cspace.toFalse(i); //ロボットと干渉する座標を排除
			else {
				bool wall_hit = false;
				t0 = std::chrono::steady_clock::now();
				wall_hit = controller->WintersectS();
				t1 = std::chrono::steady_clock::now();
				wall_intersect_ms += std::chrono::duration<double, std::milli>(t1 - t0).count();
				++wall_intersect_calls;
				if(wall_hit)	cspace.toFalse(i); //壁と干渉する座標を排除
				else            cspace.toTrue(i); //残った座標をCfreeとして抽出．以降ICSを抽出
			}
			if (progress_interval > 0 && (i + 1) % progress_interval == 0) {
				const auto now = std::chrono::steady_clock::now();
				const double scan_ms = std::chrono::duration<double, std::milli>(now - scan_start).count();
				const double shape_pct = scan_ms > 0.0 ? 100.0 * shape_update_ms / scan_ms : 0.0;
				const double robot_pct = scan_ms > 0.0 ? 100.0 * robot_intersect_ms / scan_ms : 0.0;
				const double wall_pct = scan_ms > 0.0 ? 100.0 * wall_intersect_ms / scan_ms : 0.0;
				std::cout << "[ICS-PROFILE] points=" << (i + 1) << "/" << cspace.size()
				          << " elapsed_ms=" << scan_ms
				          << " shape_update=" << shape_pct << "%"
				          << " RintersectS=" << robot_pct << "%"
				          << " WintersectS=" << wall_pct << "%"
				          << std::endl;
			}
		}
		ScopedProfiler::add("ICS.shape_update", shape_update_ms, shape_update_calls);
		ScopedProfiler::add("ICS.RintersectS", robot_intersect_ms, robot_intersect_calls);
		ScopedProfiler::add("ICS.WintersectS", wall_intersect_ms, wall_intersect_calls);

		Labeling* label = new Labeling(cspace); //クラスター番号を管理するための配列を生成
		int clust = 0;
		{
			PROFILE_SCOPE("ICS.labeling3D");
			clust = label->labeling3D(); //隣接する格子点を統合し，クラスターを生成．その後クラスターの数をカウント
		}
		c_free_ics.resize(clust); //ICSのクラスタ数

		{
			PROFILE_SCOPE("ICS.distribute");
			for (int ix = 0; ix < conf->getnumx(); ++ix)
				for (int iy = 0; iy < conf->getnumy(); ++iy)
					for (int ith = 0; ith < conf->getnumth(); ++ith) 
						distribute(label, ix, iy, ith); //格子点をクラスターに割り振る
		}

		delete label;

		int exit_flag = 0;
		{
			PROFILE_SCOPE("ICS.edge_cluster_filter");
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
		}

		if(c_free_ics.size() == 0)	return c_free_ics;

		{
			PROFILE_SCOPE("ICS.merge_theta_overlap");
			merge(conf->getsymangle());
		}
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


