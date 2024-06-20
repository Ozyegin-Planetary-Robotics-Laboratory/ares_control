#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/* MIT License

Copyright (c) 2020 Philip Salmony

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */

struct PIDController {

	/* Derivative low-pass filter time constant */
	float tau;

	/* Output limits */
	float limMin;
	float limMax;

	/* Integrator limits */
	float limMinInt;
	float limMaxInt;

	/* Sample time (in seconds) */
	float T;

	/* Controller "memory" */
	float integrator;
	float prevError;			/* Required for integrator */
	float differentiator;
	float prevMeasurement;		/* Required for differentiator */

	/* Controller output */
	float out;

  PIDController(float T) :
    integrator(0.0f),
    prevError(0.0f),
    differentiator(0.0f),
    prevMeasurement(0.0f),
    out(0.0f),
    T(T),
    tau(0.1f),
    limMin(-360.0f),
    limMax(360.0f),
    limMinInt(-1000.0f),
    limMaxInt(1000.0f)
  {}

  void reset() {
    integrator = 0.0f;
    prevError  = 0.0f;
    differentiator  = 0.0f;
    prevMeasurement = 0.0f;
    out = 0.0f;
  }


  float update(float setpoint, float measurement, float kp, float ki, float kd) {
    /*
    * Error signal
    */
    float error = setpoint - measurement;
    /*
    * Proportional
    */
    float proportional = kp * error;
    /*
    * Integral
    */
    integrator = integrator + 0.5f * ki * T * (error + prevError);

    /* Anti-wind-up via integrator clamping */
    if (integrator > limMaxInt) {
        integrator = limMaxInt;
    } else if (integrator < limMinInt) {
        integrator = limMinInt;
    }

    /*
    * Derivative (band-limited differentiator)
    */
    differentiator = -(2.0f * kd * (measurement - prevMeasurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
                        + (2.0f * tau - T) * differentiator)
                        / (2.0f * tau + T);

    /*
    * Compute output and apply limits
    */
    out = proportional + integrator + differentiator;
    if (out > limMax) {
        out = limMax;
    } else if (out < limMin) {
        out = limMin;
    }

    /* Store error and measurement for later use */
    prevError       = error;
    prevMeasurement = measurement;

    /* Return controller output */
    return out;
  }

}; // struct PIDController

#endif // PID_CONTROLLER_H