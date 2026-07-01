#pragma once


class TaskSet
{
private:
	void robotconfig();
	void object();

public:
	void set_all();
	void set_shape();
	void set_robotangle();
	void set_goal();

	void set_handtype();
	void set_discretization();

	void check();	// check current task setting 
	void space_config(int x, int y, int th);
};

