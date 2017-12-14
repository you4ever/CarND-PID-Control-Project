#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
	total_error = 0.0f;
}

PID::~PID() {}

void PID::Init(double kp, double ki, double kd) {
	Kp = kp;
	Ki = ki;
	Kd = kd;
	total_error = 0.0f;
	return;
}

void PID::UpdateError(double cte) {
	// Update differential error
	d_error = cte - p_error;
	// Update cross-track-error
	p_error = cte;
	// Update integral error
	i_error += cte;
	// Update total error
	total_error = Kp*p_error + Kd*d_error + Ki*i_error;

	return;
}

double PID::GetControl() {
	return -total_error;
}

double PID::TotalError() {

	return total_error;
}

twiddle::twiddle(vector<double> init_parameters) {
	best_error = INFINITY;
	error = INFINITY;
	parameters = init_parameters;
	return;
}

twiddle::~twiddle() {}
