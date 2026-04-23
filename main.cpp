#include "CFreeICS.h"
#include "Node.h"
#include "TaskSet.h"
#include "Visual.h"
#include "RRT.h"
// #include "RRTStar.h"
// #include "InformedRRTStar.h"
#include "Problem.h"
#include "pathsmooth.h"
#include "pathshortcut.h"
#include "FormClosure.h"
#include "PSO.h"
#include "TShape.h"
#include "clusters.h"
#include "SpaceConfig.h"

#include <cassert>
#include <ctime>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <string>
#include <fstream>

//#pragma warning(disable : 4996)

std::string get_time_now()
{
	time_t curr_time;
	tm* curr_tm;
	char date[100];
	time(&curr_time);
	curr_tm = localtime(&curr_time);
	strftime(date, 50, "%Y-%B%d-%T", curr_tm);
	return std::string(date);
}

int main(int argc, char* argv[])
{
	std::ofstream log("../ICM_Log/icm.log", std::ios::app);
	std::string fn = get_time_now();
	log << "\n\n\n" << fn << std::endl;

    std::cout << "Welcome to Sensorless ICM planner!\n";
	std::cout << "----------------------------------------------\n";
    std::cout << "Update SpaceConfig (apply belt_width -> [range]) -> 0" << std::endl;
    std::cout << "Setting                     -> 1" << std::endl;
    std::cout << "Generate Path(RRT)          -> 2" << std::endl;
    std::cout << "Generate Path(Reverse RRT)  -> 3" << std::endl;
    std::cout << "Generate Path(RRT-Connect)  -> 4" << std::endl;
	std::cout << "Path Shortcut               -> 5" << std::endl;
	std::cout << "Debug                       -> 6" << std::endl;
	std::cout << "Calc Cluster                -> 7" << std::endl;
	std::cout << "Form Closure                -> 8" << std::endl;
	std::cout << "Optimization                -> 9" << std::endl;
	// std::cout << "----------------------------------------------\n";
	// std::cout << "Generate Path(RRT*)         -> 11" << std::endl;
	// std::cout << "Generate Path(Reverse RRT*) -> 12" << std::endl;
	// std::cout << "Generate Path(Bi-RRT*)      -> 13" << std::endl;
	// std::cout << "Generate Path(Informed RRT*)         -> 14" << std::endl;
	// std::cout << "Generate Path(Reverse Informed RRT*) -> 15" << std::endl;
	// std::cout << "Generate Path(Bi-Informed RRT*)      -> 16" << std::endl;
	std::cout << "----------------------------------------------\n";

    int i = 0;
    std::cout << ">";
    std::cin >> i;
	assert(i >= 0 && i <= 16);

    if (i == 0) {
        log << "--Update SpaceConfig--" << std::endl;
        std::string path = "config/SpaceConfig.ini";
        SpaceConfig cfg;
        if (!cfg.load(path)) {
            std::cerr << "[Error] Failed to load " << path << std::endl;
            log << "[Error] Failed to load " << path << std::endl;
            return 1;
        }

        std::cout << "[Info] belt_width read: " << cfg.belt_width << " mm\n";
        std::cout << "[Info] computed x'=" << cfg.x_nd << " y'=" << cfg.y_nd << " th'=" << cfg.th_nd << "\n";
        log << "belt_width: " << cfg.belt_width
            << " x': " << cfg.x_nd
            << " y': " << cfg.y_nd
            << " th': " << cfg.th_nd << std::endl;

        if (!cfg.save(path)) {
            std::cerr << "[Error] Failed to save " << path << std::endl;
            log << "[Error] Failed to save " << path << std::endl;
            return 2;
        }

        std::cout << "[Done] SpaceConfig updated and saved to " << path << std::endl;
        log << "[Done] SpaceConfig updated and saved to " << path << std::endl;
        return 0;
    }

    if (i == 1) {
		log << "--Task Setting--" << std::endl;
        TaskSet setting;

		int opt = 0;
		std::cout << "-----------------------------------------\n";
		std::cout << "Check current config         -> 1\n";
		std::cout << "Set the shape                -> 2\n";
		std::cout << "Set the Robot angle config   -> 3\n";
		std::cout << "Set the goal condition       -> 4\n";
		std::cout << "Set the all config           -> 5\n\n";
		std::cout << "Set the hand config          -> 6\n";
		std::cout << "Set the discretized variable -> 7\n";
		std::cout << "-----------------------------------------\n";
		std::cout << ">";
        std::cin >> opt;

		if(opt == 1)		setting.check();
		else if(opt == 2)	setting.set_shape();
		else if(opt == 3)	setting.set_robotangle();
		else if(opt == 4)	setting.set_goal();
		else if(opt == 5)	setting.set_all();
		else if(opt == 6)	setting.set_handtype();
		else if(opt == 7)	setting.set_discretization();

        return 0;
    }

	else if (i == 5) {
    	log << "--Path shortcut--" << std::endl;
    	std::string fn_in;
        std::cin >> fn_in;

    	NodeList nl = csv_to_nodelist("../ICM_Log/path/" + fn_in + ".csv");

		auto t0 = std::chrono::high_resolution_clock::now();

    	PathShortcut shortcut(nl);
    	NodeList smooth_path = shortcut.shortcut();

		auto t1 = std::chrono::high_resolution_clock::now();
    	double sec = std::chrono::duration<double>(t1 - t0).count();
    	std::cout << "[TIME] shortcut took " << std::fixed << std::setprecision(3) << sec << " s\n";

    	smooth_path.printIO();
    	std::string outname = fn_in + "_sh";
    	smooth_path.print_file(outname);
    	std::cout << "[DONE] Saved to ../ICM_Log/path/" << outname << ".csv\n";
        return 0;
	}

	else if(i == 6){
		log << "--Debug--" << std::endl;
		std::string fn_in;
		std::cin >> fn_in;
		fn_in = "../ICM_Log/path/" + fn_in + ".csv";
		NodeList nl = csv_to_nodelist(fn_in);
		PathSmooth* ps = new PathSmooth(nl);
		bool valid = ps->debug();
		if(valid){
			std::cout << "Valid Route" << std::endl;
		}
		else{
			std::cout << "Invalid Route" << std::endl;
		}
        delete ps;
        return 0;
	}

	else if(i == 7){
		log << "--Calculate C_free_ICS--" << std::endl;
		Node node(
            7.04488, -16.0875, 24.5879, 36.2258, -29.1194, -43.128
        );

		CFreeICS ics(node);
		std::vector<PointCloud> cics = ics.extract();

		for(int idx=0; idx<(int)cics.size(); ++idx){
            std::cout << idx << ":\n" << "size: " << cics[idx].size() << "\n{";
            for(int j=0; j<(int)cics[idx].size(); ++j){
                if(j > 0) std::cout << ",";
                std::cout << "[" << cics[idx].get(j).x << "," << cics[idx].get(j).y << "," << cics[idx].get(j).th << "]";
            }
            std::cout << "}\n" << std::endl;
        }

		std::cout << "Select the cluster No." << std::endl;
		std::cout << "-> ";
		int cls = 0;
        std::cin >> cls;
		PointCloud pc = cics[cls];
		std::ofstream ofs("../cluster.csv");
		for(int idx=0; idx<pc.size(); ++idx){
			ofs << pc.get(idx).x << "," << pc.get(idx).y << "," << pc.get(idx).th << std::endl;
		}
        return 0;
	}

	else if(i == 8){
		Node fin(34.338, -49.249, -51.9, 29.55, -46.73, -38.71);
		FormClosure fc(fin);
		fc.close();
		Node fcfin = fc.get_fcangle();
		std::cout << "Closed hand config : " << fcfin << std::endl;
        return 0;
	}

	else if(i == 9){
		Node fin(-1.494141,32.75,-95.537109,34.771484,-52.855469,-21.105469);

		PSO opti;
		opti.optimize(fin);
        return 0;
	}

    // else if (i == 11 || i == 12 || i == 13 || i == 14 || i == 15 || i == 16) {
    //     Problem* p = nullptr;

    //     if (i == 11) {
    //         log << "--Forward RRT* exploring--\n";
    //         p = new Problem(new RRTStar);
    //     }
    //     else if (i == 12) {
    //         log << "--Reverse RRT* exploring--\n";
    //         p = new Problem(new RevRRTStar);
    //     }
    //     else if (i == 13) {
    //         log << "--RRT*-Connect exploring--\n";
    //         p = new Problem(new RRTStarConnect);
    //     }
    //     else if (i == 14) {
    //         log << "--Forward Informed RRT* exploring--\n";
    //         p = new Problem(new InformedRRTStar);
    //     }
    //     else if (i == 15) {
    //         log << "--Reverse Informed RRT* exploring--\n";
    //         p = new Problem(new RevInformedRRTStar);
    //     }
    //     else if (i == 16) {
    //         log << "--Informed RRT*-Connect exploring--\n";
    //         p = new Problem(new InformedRRTStarConnect);
    //     }

    //     log.flush();
    //     NodeList path = p->pathplanning();

	// 	long cpu_time = clock();
	// 	double sec = (double)cpu_time / CLOCKS_PER_SEC;
	// 	printf("%f[s]\n", sec);
	// 	log << "Calculation time : " << sec << "[s]\n";

	// 	path.printIO();
	// 	path.print_file(fn);
	// 	log << "Output to " << fn << ".csv\n";

	// 	delete p;
    //     return 0;
	// }

    else if (i == 2) {
		log << "--Forward RRT exploring--\n";
        Problem* p = new Problem(new RRT);
		log.flush();
		NodeList path = p->pathplanning();

		long cpu_time = clock();
		double sec = (double)cpu_time / CLOCKS_PER_SEC;
		printf("%f[s]\n", sec);
		log << "Calculation time : " << sec << "[s]\n";

		path.printIO();
		path.print_file(fn);
		log << "Output to " << fn << ".csv\n";

		delete p;
        return 0;
	}
	else if (i == 3) {
		log << "--Reverse RRT exploring--\n";
        Problem* p = new Problem(new RevRRT);
		log.flush();
		NodeList path = p->pathplanning();

		long cpu_time = clock();
		double sec = (double)cpu_time / CLOCKS_PER_SEC;
		printf("%f[s]\n", sec);
		log << "Calculation time : " << sec << "[s]\n";

		path.printIO();
		path.print_file(fn);
		log << "Output to " << fn << ".csv\n";

		delete p;
        return 0;
	}
	else if (i == 4) {
		log << "--RRT-Connect exploring--\n";
        Problem* p = new Problem(new RRTConnect);
		log.flush();
		NodeList path = p->pathplanning();

		long cpu_time = clock();
		double sec = (double)cpu_time / CLOCKS_PER_SEC;
		printf("%f[s]\n", sec);
		log << "Calculation time : " << sec << "[s]\n";

		path.printIO();
		path.print_file(fn);
		log << "Output to " << fn << ".csv\n";

		delete p;
        return 0;
	}
    else if (i == 10) {
        std::cout << "Option 10 is not used.\n";
        return 0;
    }

    std::cout << "Invalid selection.\n";
    return 0;
}
