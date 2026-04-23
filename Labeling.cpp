#include <cassert>

#include "Labeling.h"

Labeling::Labeling(CSpace cs)
	:label(nullptr), cspace(cs), numx(), numy(), numth()
{
	CSpaceConfig* conf = CSpaceConfig::get_instance();
	numx = conf->getnumx();
	numy = conf->getnumy();
	numth = conf->getnumth(); 

	label = new int** [numx];
	for (int i = 0; i < numx; i++) {
		label[i] = new int* [numy];
		for (int j = 0; j < numy; j++) {
			label[i][j] = new int[numth];
		}
	}
}

Labeling::~Labeling()
{
	for (int i = 0; i < numx; ++i) {
		for (int j = 0; j < numy; ++j) {
			delete[] label[i][j];
		}
		delete[] label[i];
	}
	delete[] label;
}


int Labeling::labeling3D()
{
	label_initialize();
	int count = simple_labeling();

	int n_count = 0;
	if (count > 0) {
		integrate();
		n_count = ordering();
	}
	
	return n_count;
}


void Labeling::label_initialize()
{
	// Initialize
	for (int ix = 0; ix < numx; ++ix)
		for (int iy = 0; iy < numy; ++iy)
			for (int ith = 0; ith < numth; ++ith) {
				label[ix][iy][ith] = 0;
			}

}

//　ネストが深すぎ、後でメソッド分割
int Labeling::simple_labeling()
{
	CSpaceConfig* conf = CSpaceConfig::get_instance();
	// first labeling
	int count = 0;
	for (int i = 0; i < cspace.size(); ++i) {
		State3D cur_pos = cspace.get_pt(i);
		int ix = (int)((cur_pos.x - conf->getbottom().x) / conf->getrange().x);
		int iy = (int)((cur_pos.y - conf->getbottom().y) / conf->getrange().y);
		int ith = (int)((cur_pos.th - conf->getbottom().th) / conf->getrange().z);

		//Later you can delete below assert func.
		assert((cur_pos.x - conf->getbottom().x) % conf->getrange().x == 0);
		assert((cur_pos.y - conf->getbottom().y) % conf->getrange().y == 0);
		assert((cur_pos.th - conf->getbottom().th) % conf->getrange().z == 0);


		if (cspace.elm[i].mk && label[ix][iy][ith] == 0) {
			int clust = search_neighbors(ix, iy, ith);
			if (clust == 0)	// New area
				label[ix][iy][ith] = ++count;
			else
				label[ix][iy][ith] = clust;
		}
	}

	return count;
}

// 冗長すぎ、後でデバッグしながらメソッド分割
int Labeling::search_neighbors(int x, int y, int th)
{
	int max = 0;

	// search neighbor 26 area
	int topx = numx;
	int topy = numy;
	int topth = numth;

	// ======================================================================================= 
	if (x - 1 >= 0) {
		if (y - 1 >= 0) {
			if (th - 1 >= 0 && label[x - 1][y - 1][th - 1] > max)
				max = label[x - 1][y - 1][th - 1];
			if (label[x - 1][y - 1][th] > max)
				max = label[x - 1][y - 1][th];
			if (th + 1 < topth && label[x - 1][y - 1][th + 1] > max)
				max = label[x - 1][y - 1][th + 1];
		}

		if (th - 1 >= 0 && label[x - 1][y][th - 1] > max)
			max = label[x - 1][y][th - 1];
		if (label[x - 1][y][th] > max)
			max = label[x - 1][y][th];
		if (th + 1 < topth && label[x - 1][y][th + 1] > max)
			max = label[x - 1][y][th + 1];

		if (y + 1 < topy) {
			if (th - 1 >= 0 && label[x - 1][y + 1][th - 1] > max)
				max = label[x - 1][y + 1][th - 1];
			if (label[x - 1][y + 1][th] > max)
				max = label[x - 1][y + 1][th];
			if (th + 1 < topth && label[x - 1][y + 1][th + 1] > max)
				max = label[x - 1][y + 1][th + 1];
		}
	}

	//=========================================================================================
	if (y - 1 >= 0) {
		if (th - 1 >= 0 && label[x][y - 1][th - 1] > max)
			max = label[x][y - 1][th - 1];
		if (label[x][y - 1][th] > max)
			max = label[x][y - 1][th];
		if (th + 1 < topth && label[x][y - 1][th + 1] > max)
			max = label[x][y - 1][th + 1];
	}

	if (th - 1 >= 0 && label[x][y][th - 1] > max)
		max = label[x][y][th - 1];
	//	here center	
	if (th + 1 < topth && label[x][y][th + 1] > max)
		max = label[x][y][th + 1];

	if (y + 1 < topy) {
		if (th - 1 >= 0 && label[x][y + 1][th - 1] > max)
			max = label[x][y + 1][th - 1];
		if (label[x][y + 1][th] > max)
			max = label[x][y + 1][th];
		if (th + 1 < topth && label[x][y + 1][th + 1] > max)
			max = label[x][y + 1][th + 1];
	}

	//==================================================================================================
	if (x + 1 < topx) {
		if (y - 1 >= 0) {
			if (th - 1 >= 0 && label[x + 1][y - 1][th - 1] > max)
				max = label[x + 1][y - 1][th - 1];
			if (label[x + 1][y - 1][th] > max)
				max = label[x + 1][y - 1][th];
			if (th + 1 < topth && label[x + 1][y - 1][th + 1] > max)
				max = label[x + 1][y - 1][th + 1];
		}

		if (th - 1 >= 0 && label[x + 1][y][th - 1] > max)
			max = label[x + 1][y][th - 1];
		if (label[x + 1][y][th] > max)
			max = label[x + 1][y][th];
		if (th + 1 < topth && label[x + 1][y][th + 1] > max)
			max = label[x + 1][y][th + 1];

		if (y + 1 < topy) {
			if (th - 1 >= 0 && label[x + 1][y + 1][th - 1] > max)
				max = label[x + 1][y + 1][th - 1];
			if (label[x + 1][y + 1][th] > max)
				max = label[x + 1][y + 1][th];
			if (th + 1 < topth && label[x + 1][y + 1][th + 1] > max)
				max = label[x + 1][y + 1][th + 1];
		}
	}

	return max;
}


void Labeling::integrate()
{
	for (int ix = 0; ix < numx; ++ix)
		for (int iy = 0; iy < numy; ++iy)
			for (int ith = 0; ith < numth; ++ith) {
				neighbor_visit(ix, iy, ith);
			}
}

void Labeling::neighbor_visit(int x, int y, int th)
{
	int nowlabel = label[x][y][th];

	if (nowlabel != 0) {
		int clust = search_neighbors(x, y, th);
		//if (clust > nowlabel)	modify_label(clust, nowlabel);
		if (clust > nowlabel)	modify_label(nowlabel, clust);
	}
}

// num1をnum2に合わせる
void Labeling::modify_label(int num1, int num2)
{
	for (int ix = 0; ix < numx; ++ix)
		for (int iy = 0; iy < numy; ++iy)
			for (int ith = 0; ith < numth; ++ith) {
				if (label[ix][iy][ith] == num1)	label[ix][iy][ith] = num2;
			}
}

int Labeling::ordering()
{
	int new_count = 0;
	for (int ix = 0; ix < numx; ++ix)
		for (int iy = 0; iy < numy; ++iy)
			for (int ith = 0; ith < numth; ++ith) {
				if (label[ix][iy][ith] > new_count) {
					new_count++;
					modify_label(label[ix][iy][ith], new_count);
				}
			}

	return new_count;
}


void Labeling::shift(int ix, int iy, int ith)
{
	int new_count = 0;
	if (label[ix][iy][ith] > new_count) {
		new_count++;
		modify_label(label[ix][iy][ith], new_count);
	}
}


int Labeling::get_label(int ix, int iy, int ith)
{
	return label[ix][iy][ith];
}
