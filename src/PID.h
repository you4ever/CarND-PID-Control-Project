#ifndef PID_H
#define PID_H

#include <math.h>
#include <vector>

using namespace std;

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double total_error;
  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  double GetControl();
  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

class twiddle {

public:
	double error;
	double best_error;
	vector<double> parameters;
	/*
	* Constructor
	*/
	twiddle(vector<double>);

	/*
	* Destructor.
	*/
	virtual ~twiddle();

};

#endif /* PID_H */
