#pragma once

#include <vector>
#include <tic.hpp>


// Not implemented

using namespace std;

class Chuck {

private:

	float x_position;
	float y_position;
	bool error_flag;
	bool alive;

	std::vector<tic::device> devices_list;
	tic::device motor_a;
	tic::device motor_b;

	tic::handle motor_a_handle;
	tic::handle motor_b_handle;

	void initialize(tic::handle *motor_handle);
	void from_steps_to_lin(int steps_a, int steps_b, float* x, float* y);
	void from_lin_to_steps(float x, float y, int* steps_a, int* steps_b);
	void update_position();
	void check_boundaries();

public:
	/// a valid chuckState needs to be passed when creating the object
	Chuck(const chuckState posChuck);
	/// call before start using the chuck
	void start();
	/// call before exiting main programm
	void stop();
	/// will move the chuck to the specified position in meters
	void motorSetPosition(const float dest_pos_x, const float dest_pos_y, const chuckState posChuck);
	/// will set the x- and y-speed of the chuck in meters/second
	void motorSetSpeed(const float v_x, const float v_y);
	/// will give the actual x- and y-position in meters
	void motorGetPosition(float *pos_x, float *pos_y);
	/// returns true if the chucks current position lies outside the boundaries
	bool position_out_of_bounds();
	/// returns true if the devices are not connected properly
	bool device_error();
};

