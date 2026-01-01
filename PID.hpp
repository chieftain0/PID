#pragma once
#include <iostream>
#include <chrono>

class PID
{
private:
    double Kp = 1;
    double Ki = 0;
    double Kd = 0;
    double setpoint = 0;
    std::chrono::high_resolution_clock::time_point prev_time;
    double prev_error = 0;
    double integral = 0;
    double derivative = 0;
    double integral_limit = 1.7e308;

public:
    /**
     * Default constructor for PID controller.
     */
    PID()
    {
        prev_time = std::chrono::high_resolution_clock::now();
    }

    /**
     * Constructor for PID controller.
     * @param Kp Proportional gain coefficient.
     * @param Ki Integral gain coefficient.
     * @param Kd Derivative gain coefficient.
     * @param setpoint Desired target value for the PID controller.
     */
    PID(double Kp, double Ki, double Kd, double setpoint)
    {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
        this->setpoint = setpoint;
        prev_time = std::chrono::high_resolution_clock::now();
    }

    /**
     * Sets the proportional gain coefficient.
     *
     * @param Kp Proportional gain coefficient.
     */
    void set_Kp(double Kp)
    {
        this->Kp = Kp;
    }

    /**
     * Sets the integral gain coefficient.
     *
     * @param Ki Integral gain coefficient.
     */
    void set_Ki(double Ki)
    {
        this->Ki = Ki;
    }

    /**
     * Sets the derivative gain coefficient.
     *
     * @param Kd Derivative gain coefficient.
     */
    void set_Kd(double Kd)
    {
        this->Kd = Kd;
    }

    /**
     * Sets the desired target value for the PID controller.
     *
     * @param setpoint The desired target value.
     */
    void set_setpoint(double setpoint)
    {
        this->setpoint = setpoint;
    }

    /**
     * Sets the limit of the integral term.
     * If the integral term exceeds this limit, it will be capped at this value.
     * This is useful for preventing windup, where the integral term grows without bound
     * due to a large error or a large integral gain coefficient.
     * @param integral_limit The limit of the integral term.
     */
    void set_integral_limit(double integral_limit)
    {
        this->integral_limit = integral_limit;
    }

    /**
     * Resets the PID controller, setting the previous time, error, and integral to zero.
     * This is useful for reinitializing the PID controller after a reset or when starting a new control loop.
     */
    void reset_PID()
    {
        prev_time = std::chrono::high_resolution_clock::now();
        prev_error = 0;
        integral = 0;
    }

    /**
     * Updates the PID controller with a new measured value.
     * 
     * @param measured_value The current measured value.
     * 
     * @return The output of the PID controller.
     */
    double update(double measured_value)
    {
        std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds = current_time - prev_time;
        double dt = elapsed_seconds.count();
        prev_time = current_time;

        double error = setpoint - measured_value;

        integral += dt * 0.5 * (error + prev_error);
        if (integral > integral_limit)
        {
            integral = integral_limit;
        }
        else if (integral < -integral_limit)
        {
            integral = -integral_limit;
        }

        if (dt > 0)
        {
            derivative = (error - prev_error) / dt;
        }
        else
        {
            derivative = 0;
        }

        double output = Kp * error + Ki * integral + Kd * derivative;
        prev_error = error;
        return output;
    }

    /**
     * Updates the PID controller with a new measured value and time difference.
     * This function is similar to update(double measured_value), but it allows the user to specify the time difference dt directly.
     * This can be useful in situations where the time difference is not easily calculated using std::chrono, such as when using a separate timer or when the PID controller is being used in a simulation.
     * @param measured_value The current measured value.
     * @param dt The time difference between the current time and the previous time.
     * @return The output of the PID controller.
     */
    double update(double measured_value, double dt)
    {
        prev_time += std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(std::chrono::duration<double>(dt));

        double error = setpoint - measured_value;

        integral += dt * 0.5 * (error + prev_error);
        if (integral > integral_limit)
        {
            integral = integral_limit;
        }
        else if (integral < -integral_limit)
        {
            integral = -integral_limit;
        }

        if (dt > 0)
        {
            derivative = (error - prev_error) / dt;
        }
        else
        {
            derivative = 0;
        }

        double output = Kp * error + Ki * integral + Kd * derivative;
        prev_error = error;
        return output;
    }
};