#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <mutex>

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

class PIDController
{
  public:

  PIDController() :
    T(0.0f)
  {}

  PIDController(float T_in) :
    T(T_in)
  {}
  
  void setSamplePeriod(float T_in) {
    std::lock_guard<std::mutex> lock(m_lock);
    T = T_in;
  }

  void reset() {
    std::lock_guard<std::mutex> lock(m_lock);
    integrator = 0.0f;
    prevError  = 0.0f;
    differentiator  = 0.0f;
    prevMeasurement = 0.0f;
    out = 0.0f;
  }

  float update(float setpoint, float measurement, float kp, float ki, float kd) {
    std::lock_guard<std::mutex> lock(m_lock);
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

  private:
  std::mutex m_lock;
	/* Derivative low-pass filter time constant */
	float tau = 0.1f;
	/* Output limits */
	float limMin = -360.0f;
	float limMax = 360.0f;
	/* Integrator limits */
	float limMinInt = -1000.0f;
	float limMaxInt = 1000.0f;
	/* Sample time (in seconds) */
	float T;
	/* Controller "memory" */
	float integrator = 0.0f;
	float prevError = 0.0f;			/* Required for integrator */
	float differentiator = 0.0f;
	float prevMeasurement = 0.0f;		/* Required for differentiator */
	/* Controller output */
	float out = 0.0f;

}; // struct PIDController

#endif // PID_CONTROLLER_H