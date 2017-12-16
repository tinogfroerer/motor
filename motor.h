#pragma once

#include <vector>
#include <libpololu-tic-1/tic.hpp>


// Not implemented

using namespace std;

class Chuck {

private:

	float x_position;
	float y_position;
	bool error_flag;

	std::vector<tic::device> devices_list;
	tic::device motor_a;
	tic::device motor_b;

	tic::handle motor_a_handle;
	tic::handle motor_b_handle;

	void initialize(tic::handle *motor_handle);
	void from_steps_to_lin(int steps_a, int steps_b, float* x, float* y);
	void from_lin_to_steps(float x, float y, int* steps_a, int* steps_b);
	void update_position();

public:
	Chuck();
	void start();
	void motorSetPosition(const float dest_pos_x, const float dest_pos_y, const chuckState posChuck);
	void motorSetSpeed(const float v_x, const float v_y);
	void motorGetPosition(float *pos_x, float *pos_y);
	bool device_error();
};

