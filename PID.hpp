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
    PID()
    {
        prev_time = std::chrono::high_resolution_clock::now();
    }
    PID(double Kp, double Ki, double Kd, double setpoint)
    {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
        this->setpoint = setpoint;
        prev_time = std::chrono::high_resolution_clock::now();
    }

    void set_Kp(double Kp)
    {
        this->Kp = Kp;
    }

    void set_Ki(double Ki)
    {
        this->Ki = Ki;
    }

    void set_Kd(double Kd)
    {
        this->Kd = Kd;
    }

    void set_setpoint(double setpoint)
    {
        this->setpoint = setpoint;
    }

    void set_integral_limit(double integral_limit)
    {
        this->integral_limit = integral_limit;
    }

    void reset_PID()
    {
        prev_time = std::chrono::high_resolution_clock::now();
        prev_error = 0;
        integral = 0;
    }

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
};