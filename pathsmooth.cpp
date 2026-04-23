// // pathshortcut.cpp
// #include "pathshortcut.h"
// #include "Controller.h"
// #include "CFreeICS.h"
// #include "CFree.h"
// #include <random>
// #include <iostream>
// #include <cassert>

// PathShortcut::PathShortcut(const NodeList& input_path, int num_trials)
//     : path(input_path), trials(num_trials) {}

// bool PathShortcut::robot_update(const Node& newnode) {
//     Controller* controller = Controller::get_instance();
//     controller->robot_update(newnode);
//     return !(controller->RintersectR(newnode) || controller->RintersectW(newnode));
// }

// bool PathShortcut::caging_valid(const PointCloud& prev, const Node& now, const Node& next) {
//     DfsCFO dfs;
//     auto cfo_now = dfs.extract(prev, now);
//     if (cfo_now.size() != 1) return false;

//     auto cfo_next = dfs.extract(cfo_now[0], next);
//     return (cfo_next.size() == 1);
// }

// NodeList PathShortcut::shortcut() {
//     std::mt19937 rng(std::random_device{}());
//     NodeList result = path;

//     for (int trial = 0; trial < trials; ++trial) {
//         if (result.size() < 3) break;
//         std::uniform_int_distribution<> dist(0, result.size() - 1);
//         int i = dist(rng), j = dist(rng);
//         if (i == j) continue;
//         if (i > j) std::swap(i, j);
//         if (j - i < 2) continue;

//         Node n1 = result[i];
//         Node n2 = result[j];

//         // 線形補間
//         const int steps = 10;
//         std::vector<Node> interpolated;
//         bool valid = true;
//         PointCloud prev_cloud;

//         // 初期クラスタを取得
//         {
//             CFreeICS ics(n1);
//             auto init_cfree = ics.extract();
//             if (init_cfree.empty()) continue;

//             std::cout << "Select cluster index [0 - " << init_cfree.size() - 1 << "]: ";
//             int idx;
//             std::cin >> idx;
//             assert(0 <= idx && idx < (int)init_cfree.size());
//             prev_cloud = init_cfree[idx];
//         }

//         for (int k = 1; k < steps; ++k) {
//             double t = static_cast<double>(k) / steps;
//             Node mid = n1.interpolate(n2, t);

//             if (!robot_update(mid)) {
//                 valid = false;
//                 break;
//             }
//             if (!caging_valid(prev_cloud, mid, n2)) {
//                 valid = false;
//                 break;
//             }
//             interpolated.push_back(mid);
//         }

//         if (valid) {
//             result.elm.erase(result.elm.begin() + i + 1, result.elm.begin() + j);
//             result.elm.insert(result.elm.begin() + i + 1, interpolated.begin(), interpolated.end());
//         }
//     }

//     return result;
// }





// #include <vector>
// #include <random>
// #include "Node.h"
// #include "CFreeICS.h"
// #include "CFree.h"

// // path: RRTで得られたNode列
// void caging_shortcut(std::vector<Node>& path, int trials = 100) {
//     std::mt19937 rng(std::random_device{}());
//     std::uniform_int_distribution<> dist(0, path.size() - 1);

//     for (int t = 0; t < trials; ++t) {
//         if (path.size() < 3) break;                       // 最低 3 ノード必要
//         std::uniform_int_distribution<> dist(0, path.size() - 1);

//         int i = dist(rng);
//         int j = dist(rng);
//         if (i == j) continue;
//         if (i > j) std::swap(i, j);
//         if (j - i < 2) continue;
    
//   // 隣接点はスキップ

//         Node q1 = path[i];
//         Node q2 = path[j];

//         // 補間ノードを10個作る
//         const int steps = 10;
//         std::vector<Node> interpolated_nodes;
//         for (int k = 1; k < steps; ++k) {
//             double t = static_cast<double>(k) / steps;
//             Node interp = q1.interpolate(q2, t);  // 線形補間メソッドを実装必要
//             interpolated_nodes.push_back(interp);
//         }

//         bool valid = true;
//         for (const Node& node : interpolated_nodes) {
//             CFreeICS cics(node);
//             auto c_free_ics = cics.extract();  // C_(free_ICS) の取得

//             RasterCFO cfo;
//             auto c_free_obj = cfo.extract(path[i].getCloud(), node);

//             // ケージング条件の確認
//             if (c_free_obj.size() != 1) {
//                 valid = false;
//                 break;
//             }
//         }

//         if (valid) {
//             // 成功したら中間ノードを補間ノードで置き換える
//             path.erase(path.begin() + i + 1, path.begin() + j);
//             path.insert(path.begin() + i + 1, interpolated_nodes.begin(), interpolated_nodes.end());
//         }
//     }
// }




// #include <fstream>
// #include <sstream>

// #include "CFree.h"
// #include "Controller.h"
// #include "pathsmooth.h"
// #include "CFreeICS.h"

// #include <iostream>

// // コンストラクタ
// PathSmooth::PathSmooth(NodeList path)
//     : orig_path(path)
// {}

// // Controller でロボット状態を更新
// bool PathSmooth::robot_update(Node newnode) {
//     Controller* controller = Controller::get_instance();
//     if (!controller) return false;
//     controller->robot_update(newnode);
//     return true;
// }

// // スムージング処理
// NodeList PathSmooth::smooth() {
//     NodeList opt_path = orig_path;

//     if (opt_path.size() <= 2) return opt_path;

//     const int max_iter = 100;
//     for (int iter = 0; iter < max_iter; ++iter) {
//         for (int i = 1; i < opt_path.size() - 1; ++i) {
//             Node xi = opt_path[i];
//             Node xi_orig = orig_path[i];
//             Node xi_prev = opt_path[i - 1];
//             Node xi_next = opt_path[i + 1];

//             // 平滑化の式：αで元データとの距離を抑制し、βで曲率を抑制
//             Node term1 = xi - xi_orig;
//             Node term2 = (xi_prev + xi_next - xi * 2.0);

//             opt_path[i] = xi - term1 * alpha - term2 * beta;
//         }
//     }

//     return opt_path;
// }

// // パスの各点を標準出力に出す
// bool PathSmooth::debug() {
//     std::cout << "[Debug] Path size: " << orig_path.size() << "\n";
//     for (int i = 0; i < orig_path.size(); ++i) {
//         const Node& n = orig_path[i];
//         std::cout << "Node " << i << ": ";
//         for (int j = 0; j < Node::dof; ++j) {
//             std::cout << n.get_element(j);
//             if (j != Node::dof - 1) std::cout << ", ";
//         }
//         std::cout << std::endl;
//     }
//     return true;
// }





#include <fstream>
#include <sstream>

#include "CFree.h"
#include "Controller.h"
#include "pathsmooth.h"
#include "CFreeICS.h"

#include <iostream>

PathSmooth::PathSmooth(NodeList path)
	:orig_path(path)
{}


NodeList PathSmooth::smooth()
{
	NodeList opt_path = orig_path;

	Node ini = orig_path[0];
	CFreeICS ics(ini);

	for(int i=0; i<6;++i){
		std::cout << ini[i] << ", "; std::cout << std::endl;
	}
	std::vector<PointCloud> init_CFree = ics.extract();
	int num = 0;
	for (const auto& cls : init_CFree) {
		std::cout << num;	std::cout << ":" << cls.size() << std::endl;
		for (int i = 0; i < (int)cls.size(); ++i) {
			std::cout << "[";	std::cout << cls.get(i).x;	std::cout << ",";
			std::cout << cls.get(i).y;	std::cout << ",";	std::cout << cls.get(i).th;	std::cout << "],";
		//	if (i > 4)	break;
		}
		std::cout << std::endl << std::endl;
		++num;
	}

	std::cout << "Select the cluster:";
	int index = -1;
	std::cin >> index;
	assert(0 <= index && index < (int)init_CFree.size());

	int cnt = 0, iters_num = 10000;
//	double tolerance = 0.1;
	double error = 1.0;	// All value is OK except the value lower than 0.1
	
//	while(error > tolerance || cnt < iters_num){
	while(cnt < iters_num){
		PointCloud pre_cfo = init_CFree[index];
		error = 0.0;
		// initial and final point is fixed.
		for(int i=1; i<orig_path.size()-1; ++i){
			NodeList pre_path = opt_path;
			//opt_path[i] = opt_path[i] - alpha*(opt_path[i] - orig_path[i]);
			//opt_path[i] = opt_path[i] - beta*(2*opt_path[i] - opt_path[i-1] - opt_path[i+1]);
			for(int n=0; n<Node::dof; ++n){
				if(opt_path[i][n] > 90)		opt_path[i][n] = 90;
				if(opt_path[i][n] < -90)	opt_path[i][n] = -90;
			}
			std::cout << "before path: ";
			for(int elm=0; elm<Node::dof; ++elm){
				std::cout << pre_path[i][elm] << ", ";	std::cout << std::endl;
			}
			std::cout << "after  path: ";
			for(int elm=0; elm<Node::dof; ++elm){
				std::cout << opt_path[i][elm] << ", ";	std::cout << std::endl;
			}
			error += opt_path[i].norm(pre_path[i]);

			if(!robot_update(opt_path[i])){
				opt_path[i] = pre_path[i];
				break;
			}

			// caging_valid start
			DfsCFO dfs;
			std::vector<PointCloud> cfo_now = dfs.extract(pre_cfo, opt_path[i]);
			if((int)cfo_now.size() != 1){
				opt_path[i] = pre_path[i];
				std::vector<PointCloud> cfo_org = dfs.extract(pre_cfo, pre_path[i]);
				assert(cfo_org.size() == 1);
				pre_cfo = cfo_org[0];
				continue;
			}
			
			std::vector<PointCloud> cfo_aft = dfs.extract(cfo_now[0], opt_path[i+1]);
			if((int)cfo_aft.size() != 1){
				opt_path[i] = pre_path[i];
				std::vector<PointCloud> cfo_org = dfs.extract(pre_cfo, pre_path[i]);
				assert(cfo_org.size() == 1);
				pre_cfo = cfo_org[0];
				continue;
			}

			pre_cfo = cfo_now[0];
			// caging_valid end
		}
		std::cout << "Loop End" << std::endl;
		++cnt;
		std::cout << "No. " << cnt << std::endl;
		std::cout << "  diff: " << error << std::endl;
	}

	return opt_path;
}


bool PathSmooth::robot_update(Node newnode)
{
	Controller* controller = Controller::get_instance();
	controller->robot_update(newnode);

	if (controller->RintersectR(newnode)) {
		return false;
	}
	if (controller->RintersectW(newnode)) {
		return false;
	}
	return true;
}


// NodeList csv_to_nodelist(std::string fn)
// {
// 	NodeList nl;
// 	std::string tmp_line, tmp_str;
// 	std::vector<double> node(6);
// 	std::ifstream ifs(fn);

// 	while(getline(ifs, tmp_line)){
// 		std::istringstream i_stream(tmp_line);
// 		int index = 0;
// 		while(getline(i_stream, tmp_str, ',')){
// 			node[index] = std::stod(tmp_str);
// 			++index;
// 			if(index > 5)	break;
// 		}
// 		nl.push_back(node);
// 	}
// 	return nl;
// }

// ───────────────────────────────────────────────
// pathsmooth.cpp  内  (★ csv_to_nodelist の定義部分 ★)
// NodeList csv_to_nodelist(std::string fn)
// {
//     NodeList nl;
//     std::ifstream ifs(fn);

//     if (!ifs.is_open()) {                       // ← 追加③
//         std::cerr << "[ERROR] cannot open '" << fn << "'\n";
//         return nl;
//     }

//     while(getline(ifs, tmp_line)){
// 		std::istringstream i_stream(tmp_line);
// 		int index = 0;
// 		while(getline(i_stream, tmp_str, ',')){
// 			node[index] = std::stod(tmp_str);
// 			++index;
// 			if(index > 5)	break;
// 		}   // 既存のループ処理

//     std::cout << "[INFO] read "                 // ← 追加④
//               << nl.size() << " rows from '" << fn << "'\n";
//     return nl;
// }
// ───────────────────────────────────────────────
// pathsmooth.cpp  の中
NodeList csv_to_nodelist(std::string fn)
{
    NodeList nl;
    std::ifstream ifs(fn);

    if (!ifs.is_open()) {                           // open チェック
        std::cerr << "[ERROR] cannot open '" << fn << "'\n";
        return nl;
    }

    std::string tmp_line;               // ここから先は元の変数宣言
    while (std::getline(ifs, tmp_line)) {
        if (tmp_line.empty()) continue;  // 空行スキップ

        std::istringstream i_stream(tmp_line);
        std::string tmp_str;
        std::vector<double> node(6);
        int index = 0;

        while (std::getline(i_stream, tmp_str, ',') && index < 6) {
            // 末尾のスペースや CR を取り除く
            while (!tmp_str.empty() &&
                   (tmp_str.back() == ' ' || tmp_str.back() == '\r'))
                tmp_str.pop_back();

            node[index] = std::stod(tmp_str);
            ++index;
        }
        if (index == 6) nl.push_back(Node(node));   // 行が6 DoF揃っていれば追加
    }

    std::cout << "[INFO] read " << nl.size()
              << " rows from '" << fn << "'\n";
    return nl;
}


bool PathSmooth::debug()
{
	std::string fn = "../icmlog/debug.txt";
	std::ofstream ofs(fn, std::ios::app);

	Node ini = orig_path[0];
	CFreeICS ics(ini);

	std::cout << ini << std::endl;
	std::vector<PointCloud> init_CFree = ics.extract();
	int num = 0;
	for (const auto& cls : init_CFree) {
		std::cout << num;	std::cout << ":" << cls.size() << std::endl;
		for (int i = 0; i < (int)cls.size(); ++i) {
			std::cout << "[";	std::cout << cls.get(i).x;	std::cout << ",";
			std::cout << cls.get(i).y;	std::cout << ",";	std::cout << cls.get(i).th;	std::cout << "],";
		//	if (i > 4)	break;
		}
		std::cout << std::endl << std::endl;
		++num;
	}

	std::cout << "Select the cluster:";
	int index = -1;
	std::cin >> index;
	assert(0 <= index && index < (int)init_CFree.size());

	PointCloud pre_cfo = init_CFree[index];

	for(int i=1; i<orig_path.size(); ++i){
		std::cout << i << ": " << orig_path[i] << " -> ";
		ofs << i << ": " << orig_path[i] << std::endl;
		if(!robot_update(orig_path[i])){
			return false;
		}

		DfsCFO dfs;
		std::vector<PointCloud> cfo_now = dfs.extract(pre_cfo, orig_path[i]);
		for(int i=0; i<(int)cfo_now.size(); ++i){
			ofs << cfo_now[i] << std::endl;
		}

		if((int)cfo_now.size() != 1){
			return false;
		}
		std::cout << cfo_now[0].size() << std::endl;	
		pre_cfo = cfo_now[0];
	}
	return true;
}
