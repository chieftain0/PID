# PID Controller (C++ & Python)

A lightweight, dependency-free implementation of a PID (Proportional–Integral–Derivative) controller in both C++ and Python. Designed for real-time control loops, simulations, and embedded or robotics-oriented projects where simplicity and clarity matter.

## Control Law

The controller output is computed as:

```math
\text{output} = K_p \cdot \text{error} + K_i \cdot \text{integral} + K_d \cdot \text{derivative}
```

Where:

- `error = setpoint - measured_value`
- `integral` is accumulated using trapezoidal integration
- `derivative` is computed from the error difference over time

## C++ Usage

```cpp
#include "pid.hpp"

PID pid(1.0, 0.1, 0.01, 10.0);  // Kp, Ki, Kd, setpoint

double measured_value = sensor_reading();
double control_signal = pid.update(measured_value);

pid.set_Kp(1.2);
pid.set_Ki(0.05);
pid.set_Kd(0.01);
pid.set_setpoint(15.0);
pid.set_integral_limit(100.0);
pid.reset_PID();
```

## Python Usage

The Python implementation mirrors the C++ logic and API as closely as possible.

```python
from pid import PID

pid = PID(Kp=1.0, Ki=0.1, Kd=0.01, setpoint=10.0)

measured_value = read_sensor()
control_signal = pid.update(measured_value)

pid.set_Kp(1.2)
pid.set_Ki(0.05)
pid.set_Kd(0.01)
pid.set_setpoint(15.0)
pid.set_integral_limit(100.0)
pid.reset_PID()
```
