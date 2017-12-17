#include "estimator.h"
#include "motor.h"
#include "transformation.h"

#include <iostream>
#include <tic.hpp>
#include <thread>
#include <unistd.h>

using namespace std;

/// Motor Settings
uint32_t Max_Possible_Acceleration = 2147483647;
uint32_t Max_Possible_Deceleration = 2147483647;
uint32_t Max_Acceleration;
uint32_t Max_Deceleration;

uint32_t Current_Limit = 1000;
uint8_t Step_Mode = 1;
uint32_t Max_Speed = 500000000/10000;
float Acceleration = 0.0000001;
float Deceleration = 0.1;

/// Motor Serial Numbers
string motor_a_serial_number = "00195103"; /// left motor
string motor_b_serial_number = "00195303"; /// right motor

/// Boundaries for Chuck movement
float x_max = -0.13;
float x_min = -0.67;
float y_max = 0.28;
float y_min = -0.28;

/// Methods of Class Chuck
Chuck::Chuck(const chuckState posChuck)
{
	/// updates the x- and y- position of the chuck
	/// if no valid chuckState is passed, an error will appear
	if (posChuck.valid)
	{
		x_position = posChuck.x;
		y_position = posChuck.y;
	}
	else
	{
		cerr << "no valid chuckState passed!";
		x_position = 0;
		y_position = 0;
	}
	
	/// Detect all connected devices
	devices_list = tic::list_connected_devices();

	/// Count number of connected devices
	/// This number must be 2, else the motors are connected wrongly -> error_flag will be set accordingly
	int device_count = 0;
	for(tic::device device : devices_list) {
		device_count++;
	}

	/// if 2 devices were found, associate them with motor_a and motor_b according to their serial number
	if (device_count == 2)
	{
		error_flag = true;
		alive = false;
		tic::device first_device = devices_list[0];
		tic::device second_device = devices_list[1];
		string first_device_serial_number = first_device.get_serial_number();
		string second_device_serial_number = second_device.get_serial_number();

		if (first_device_serial_number == motor_a_serial_number)
		{
			motor_a = first_device;
			if (second_device_serial_number == motor_b_serial_number)
			{
				motor_b = second_device;
				error_flag = false;
			}
		}
		else if (first_device_serial_number == motor_b_serial_number)
		{
			motor_b = first_device;
			if (second_device_serial_number == motor_a_serial_number)
			{
				motor_a = second_device;
				error_flag = false;
			}
		}
	}
	else
	{
		error_flag = true;
	}

	/// set up handles for motor_a and motor_b
	/// then initialize handles with motor settings
	if (!(device_error()))
	{
		motor_a_handle = tic::handle(motor_a);
		motor_b_handle = tic::handle(motor_b);
		initialize(&motor_a_handle);
		initialize(&motor_b_handle);
	}
}

/// initializes the specified motor_handle with the motor settings
void Chuck::initialize(tic::handle* motor_handle)
{
	/// apply motor settings
	motor_handle->deenergize();
	motor_handle->set_current_limit(Current_Limit);
	motor_handle->set_step_mode(Step_Mode);
	motor_handle->set_max_speed(Max_Speed);
	Max_Acceleration = static_cast<uint32_t>(Acceleration*Max_Possible_Acceleration);
	Max_Deceleration = static_cast<uint32_t>(Deceleration*Max_Possible_Deceleration);
	motor_handle->set_max_accel(Max_Acceleration);
	motor_handle->set_max_decel(Max_Deceleration);
	motor_handle->set_decay_mode(TIC_DECAY_MODE_MIXED);
	motor_handle->clear_driver_error();

	/// deactivate command timeout
	tic::settings settings;
	settings = motor_handle->get_settings();
	uint16_t timeout = 0;
	tic_settings_set_command_timeout(settings.get_pointer(),timeout);
	motor_handle->set_settings(settings);
	motor_handle->reinitialize();
}

/// will energize the stepper motors
void Chuck::start()
{
	alive = true;

	int a_position;
	int b_position;
	from_lin_to_steps(x_position, y_position, &a_position, &b_position);

	motor_a_handle.exit_safe_start();
	motor_a_handle.energize();
	motor_a_handle.halt_and_set_position(a_position);
	motor_b_handle.exit_safe_start();
	motor_b_handle.energize();
	motor_b_handle.halt_and_set_position(b_position);
	
	thread boundary_checker(&Chuck::check_boundaries, this);
	boundary_checker.detach();
}

/// will stop and deenergize stepper motors
void Chuck::stop()
{
	alive = false;

	motor_a_handle.halt_and_hold();
	motor_b_handle.halt_and_hold();

	motor_a_handle.deenergize();
	motor_b_handle.deenergize();
}

/// checks continously if the chuck still lies withing the boundaries
/// if chuck lies outside the boundaries, the movement of the motors will be stopped
void Chuck::check_boundaries()
{
	while(alive)
	{
		update_position();

		if(position_out_of_bounds())
		{
			motor_a_handle.halt_and_hold();
			motor_b_handle.halt_and_hold();

			cerr << "out of bounds error!";
			cerr << x_position << ";" << y_position << endl;
		}
	}
}

/// returns true if the position of the chuck lies outside the boundaries
bool Chuck::position_out_of_bounds()
{
	if((y_position > y_max) || (y_position < y_min))
	{
		return true;
	}
	else if((x_position > x_max) || (x_position < x_min))
	{
		return true;
	}
	else
	{
		return false;
	}
}

/// returns true if the devices could not be identified correctly
bool Chuck::device_error()
{
	return error_flag;
}

/// updates the information about motors x- and y-position according tho the stepper motor positions
void Chuck::update_position()
{
	float new_x_position;
	float new_y_position;

	int a_position;
	int b_position;
	tic::variables variables;
	variables = motor_a_handle.get_variables();
	a_position = variables.get_current_position();
	
	variables = motor_b_handle.get_variables();
	b_position = variables.get_current_position();
	/// Convert step-positons in x- and y-positions
	from_steps_to_lin(a_position, b_position, &new_x_position, &new_y_position);
	/// Write new positions
	x_position = new_x_position;
	y_position = new_y_position;
}

/// gives the motor x- and y-position
void Chuck::motorGetPosition(float *pos_x, float *pos_y)
{
	update_position();
	*pos_x = x_position;
	*pos_y = y_position;
}

/// will move the chuck to the specified position
void Chuck::motorSetPosition(const float dest_pos_x, const float dest_pos_y, const chuckState posChuck)
{
	int steps_a;
	int steps_b;

	/// First update motor position-information
	update_position();

	/// Convert delta x- and y- position into amounts of steps
	from_lin_to_steps(dest_pos_x, dest_pos_y, &steps_a, &steps_b);
	/// Set target positions of the stepper motors
	
	cout << "steps_a: " << steps_a << endl;
	cout << "steps_b: " << steps_b << endl;
	
	float current_x;
	float current_y;
	motorGetPosition(&current_x, &current_y);
	int current_a;
	int current_b;
	from_lin_to_steps(current_x, current_y, &current_a, &current_b);
	
	int delta_a = steps_a - current_a;
	int delta_b = steps_b - current_b;
	
	cout << "delta_a: " << delta_a << endl;
	cout << "delta_b: " << delta_b << endl;
	
	motor_a_handle.set_target_position(steps_a);
	motor_b_handle.set_target_position(steps_b);
}

/// will set the chucks speed to the specified values
void Chuck::motorSetSpeed(const float v_x, const float v_y)
{
	int speed_a;
	int speed_b;
	/// Convert x- and y- speed to amount of pulses per second
	from_lin_to_steps(v_x, v_y, &speed_a, &speed_b);
	/// Multiply with 10000 because the motor takes amount of steps per 10000 seconds
	speed_a = speed_a*10000;
	speed_b = speed_b*10000;
	/// Set target speed of the stepper motors
	motor_a_handle.set_target_velocity(speed_a);
	motor_b_handle.set_target_velocity(speed_b);
}

/// Converting amount of steps from each motor to x- and y-movement in meters
void Chuck::from_steps_to_lin(int steps_a, int steps_b, float* x, float* y)
{
	*x = 0.001*0.1*(-steps_a+steps_b);
	*y = 0.001*-0.1*(steps_a+steps_b);
}

/// Converting x- and y-movement in meters to amount of steps for each motor
void Chuck::from_lin_to_steps(float x, float y, int* steps_a, int* steps_b)
{
	*steps_a = 1000*-5*(x+y);
	*steps_b = 1000*5*(x-y);
}

int main()
{
	float x;
	float y;

	chuckState chuck_pos;
	chuck_pos.x=-0.45;
	chuck_pos.y=0;
	chuck_pos.valid=true;

	Chuck chuck(chuck_pos);

	do
	{
	   cout << '\n' << "Press a key to continue...";
	} while (cin.get() != '\n');
	
	chuck.start();
	cout << "started" << endl;
	/*chuck.motorSetSpeed(-0.1,0.1);
	cout << "speed set" << endl;
	usleep(1000000);
	chuck.motorSetSpeed(0,-0.1);
	usleep(1000000);
	chuck.motorSetSpeed(0,0);
	usleep(1000000);	
	chuck.motorGetPosition(&x,&y);
	chuck_pos.x = x;
	chuck_pos.y = y;
	
	cout << "x-position: " << x << endl;
	cout << "y-position: " << y << endl;
	
	chuck.motorSetSpeed(0.1,0);
	usleep(5000000);
	
	chuck.motorSetSpeed(0,0);
	usleep(1000);
	
	chuck.motorGetPosition(&x,&y);
	chuck_pos.x = x;
	chuck_pos.y = y;
	
	cout << "x-position: " << x << endl;
	cout << "y-position: " << y << endl;*/
	
	chuck.motorSetPosition(-0.45,0.2,chuck_pos);
	usleep(3000000);
	
	chuck.motorGetPosition(&x, &y);
	cout << "x-position: " << x << endl;
	cout << "y-position: " << y << endl;
	
	chuck.motorSetPosition(-0.45,-0.2,chuck_pos);
	usleep(3000000);
	

	
	
	
	chuck.motorGetPosition(&x, &y);
	cout << "x-position: " << x << endl;
	cout << "y-position: " << y << endl;
	chuck.stop();
}

