
#include <climits>

#include "icmMath.h"
#include "CFreeICS.h"
#include "Controller.h"
#include "Problem.h"
#include "RRT.h"
#include "TaskSet.h"

#define WIDTH 20.0
#define th_max 140.0


double caging_func(Node node);


class PSO
{
private:
	std::vector<Node> particles;
	std::vector<Node> velocity;
	std::vector<Node> personal_best;
	std::vector<double> personal_score;
	Node global_best;
	double global_score;

	int repeat_times, particle_nums;

public:
	PSO(int _repeat_times, int _particle_nums)
		:repeat_times(_repeat_times), 
	     particle_nums(_particle_nums)
	{
	}

	PSO()
		:repeat_times(200),
		 particle_nums(100)
	{}

	std::vector<double> optimize(Node ini)
	{
		init(ini);

		for(int i=0; i<repeat_times; ++i)
		{
			std::cout << i << ": " << std::endl;
			update_personal();
			update_global();
		}

		std::cout << global_best << std::endl;
		return std::vector<double>();
	}


	void init(Node ini)
	{
		std::srand(time(NULL));

		double diffusion_width = WIDTH;
		for(int i=0; i<particle_nums; ++i){
			std::vector<double> tmpnode;
			for(int n=0; n<Node::dof; ++n){
				double random = (double)rand()/RAND_MAX;
				double tmpangle = ini.get_element(n) + diffusion_width*random - (diffusion_width/2); //ini.get_element(n) + diffusion_width*random - (diffusion_width/2);を変更	
				if(tmpangle>th_max)		tmpangle=th_max;
				if(tmpangle<-th_max)	tmpangle=-th_max;
				tmpnode.push_back(tmpangle);
			}
			assert(tmpnode.size() == 6);
			particles.push_back(Node(tmpnode));
		}
	
		velocity.resize(particle_nums);

		personal_best = particles;
		for(int i=0; i<(int)personal_best.size(); ++i){
			double tmpscore = caging_func(personal_best[i]);
			personal_score.push_back(tmpscore);
			std::cout << personal_best[i] << " : " << tmpscore << std::endl;
		}
	
		double tmpmin = DBL_MAX;	int global_index = 0;
		for(int i=0; i<(int)personal_score.size(); ++i){
			if(tmpmin > personal_score[i]){
				tmpmin = personal_score[i];
				global_index = i;
			}
		}
		global_best = personal_best[global_index];
		global_score = tmpmin;

		std::cout << "\nglobal best -> " << global_best << " : " << global_score << std::endl << std::endl;
	}



	void update_personal()
	{
		static int cnt = 0;	++cnt;
		const double wh = 1.2, wl = 0.3;
		const double c1 = 0.7, c2 = 0.2;
		double w = wh - (wh - wl)*cnt/repeat_times; 

		for(int i=0; i<(int)particles.size(); ++i){
			double r1 = (double)rand()/RAND_MAX;
			double r2 = (double)rand()/RAND_MAX;
			velocity[i] = velocity[i] * w +
				r1 * c1 * (personal_best[i] - particles[i]) + 
				r2 * c2 * (global_best - particles[i]);
			particles[i] = particles[i] + velocity[i];
			for(int n=0; n<Node::dof; ++n){
				if(particles[i][n]>th_max)	particles[i][n] = th_max;
				if(particles[i][n]<-th_max)	particles[i][n] = -th_max;
			}

			double tmpscore = caging_func(particles[i]);
			if(tmpscore < personal_score[i]){
				personal_score[i] = tmpscore;
				personal_best[i] = particles[i];
			}
			std::cout << i << ":" << personal_best[i] << " -> " << tmpscore << std::endl;
		}
	}


	void update_global()
	{
		for(int i=0; i<(int)personal_best.size(); ++i){
			if(personal_score[i] < global_score){
				global_score = personal_score[i];
				global_best = personal_best[i];
			}
		}
		std::cout << "\nglobal best -> " << global_best << " : " << global_score << std::endl << std::endl;
	}

};

