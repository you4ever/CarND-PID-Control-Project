# PID Controller 

---


## Introduction
A proportional–integral–derivative controller (PID controller) is a feedback based controlling mechanism widely used in a variety of industrial applications. In the self-driving car context, a PID controller continuously calculates an error value e(t) as the difference between the car's posiiton and center of track and a measured process variable and applies a correction based on proportional, integral, and derivative terms (denoted P, I, and D respectively).

## Implementation
The implementation of the MPC consists of three main aspects:

1) On each measurement occasion, compute the instantaneous cross-track error (CTE). In the meanwhile, update the integral and derivative terms based on the previous values and current CTE;
2) The PID generate the control vector for each actuator under PID's control based on the weighted combination of three feedback error terms computed in Step 1);
3) Fine tune the weights for each of feedback terms

 
### Discusison on PID controller

#### Mathematical form 
A ditigal PID controller computes the controlling vector at each sample time k based on the equation:
```
u[k] = -kp * ep[k] - kd * ed[k] - ki * ei[k]
```
where
* ``ep[k]``: the propotional error term -- the CTE at time k
* ``kp``: the gain for propotional term
* ``ed[k]``: the derivative error term
* ``kd``: the gain for derivative term
* ``ei[k]``: the integral error term
* ``ki``: the gain for integral term

#### Propotional gain
The proportional term contribute to the actuation proportional to the current error value scaled by gain ``kp``. If the proportional gain is too high, the system can become unstable. On the other hand, if the proportional gain is too low, the control action may be too small when responding to system disturbances. Tuning experiments indicate that the proportional term should contribute the significant part of the actuation in order to follow the track correctly.

#### Derivative gain
The derivative term captures the changing rate of the error over time by multiplying it by the derivative gain Kd. Intuitively derivative term predicts the future state of the system and compensate the actuation accordingly. Thus derivative term improves settling time and stability of the system by the dampening effect. 

#### Integration gain
The integration gain is multiplied to the accumulated past values of the CTE. The integral term tends to compensate residual error. However, due to its accumulative nature, the response to changing environment is slow --  it is slow to correct residual error and slow to react when the curve changes. 

### Implementation of PID controller

#### Implemntation of PID class
The general PID class is implemented in ``PID.cpp``. The error are updated at each iteration in the ``Pid::updateError`` method. Differential is approximated by the numerical difference between the current error and previous error, i.e. ``d_error = cte - p_error``. The integration term is computed as accumulated error. The actuation is computed as the weighted sum of each of the PID terms in ``PID::GetControl()`` (see ``PID.cpp:35``).  

### Implementation of steering control
For simplicity only the controller to steering angle is controlled by a PID controller (See ``main.cpp:66``). 

### Implementation speed control 
To control the speed, I used a heuristic controller based on well-defined logics as elaborated in the following:
* When the speed is less than some target speed ``vt`` throttle is set to 0.9. This sets the lowest speed for the car.
* When speed is greater than 60mph and CTE is less than 0.85. The throttle is set to 0. So that the car will coast at the top speed of 60mph if error is small. If 
* Under any case the brake is applied when CTE is greater than 0.85. 
For detailed implementation, see ``main.app:74-88``

### Tuning of the gains
Manual twiddling with some intuition is employed in this project to find the final values. I followed the following modified twiddling steps:

 1. Find ``kp`` by repeating 1.1-1.3. Use the last good values for ``kp`` and ``vt``: 
  1.1.  Initialize ``kp = 0.05, kd = 0, ki = 0`` and initialize ``vt`` to 5 mph 
  1.2. Test drive. 
  1.3. If car is albe to drive on track, increase the target speed by 5mph, else increase kp by 0.05. Remember the last working ``vt`` and ``kp`` Go to step 1.2). 
 2. Find the ``ki``:
  2.1. Increase ``kd`` by 1
  2.2. Test drive
  2.3. Go to to 2.1 until no improvement on oscillation behavior is noticable 
 3. Find the ``kd``: 
  3.1 Increase ``ki`` by 0.00001
  3.2 Test drive
  3.3 Go to 3.1 until the local minima in accumulated absolute error is found. 

One may iterate Step 1)-3) a few times to increase the target speet ``vt``. The final values for gains are choosen as ``kp = 0.2, ki = 0.00001, kd = 25.0``. Since the simulator does not have the problem of systematic bias so teh ``ki`` gain is set to very small number.


## Simualtion result

After tuning the gain of PID controller, the PID is able to drive the car safely and smoothly by tracking the waypoints. The target speed is set to 60 mph and is sustainable most of the time. 
