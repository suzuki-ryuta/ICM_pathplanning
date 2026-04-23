#pragma once

#include "PointCloud.h"
#include "CSpace.h"

class Labeling
{
private:
	int*** label;
	CSpace cspace;
	int numx, numy, numth;

	void label_initialize();
	int simple_labeling();
	int search_neighbors(int x, int y, int th);
	void neighbor_visit(int ix, int iy, int ith);
	void modify_label(int num1, int num2);
	void shift(int ix, int iy, int ith);

public:
	Labeling(CSpace cs);	
	~Labeling();
	int get_label(int ix, int iy, int ith);

	int labeling3D();
	void integrate();
	int ordering();
};


