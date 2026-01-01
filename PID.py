import time


class PID:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, setpoint=0.0):
        """
        Initialize the PID controller with specified gains and setpoint.

        Parameters:
        Kp (float): Proportional gain coefficient. Default is 1.0.
        Ki (float): Integral gain coefficient. Default is 0.0.
        Kd (float): Derivative gain coefficient. Default is 0.0.
        setpoint (float): Desired target value for the PID controller. Default is 0.0.
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint

        self._prev_time = time.perf_counter()
        self._prev_error = 0.0
        self._integral = 0.0
        self._derivative = 0.0
        self._integral_limit = None

    def set_Kp(self, Kp):
        """
        Sets the proportional gain coefficient.

        Parameters
        ----------
        Kp : float
            Proportional gain coefficient.
        """
        self.Kp = Kp

    def set_Ki(self, Ki):
        """
        Sets the integral gain coefficient.

        Parameters
        ----------
        Ki : float
            Integral gain coefficient.
        """
        self.Ki = Ki

    def set_Kd(self, Kd):
        """
        Sets the derivative gain coefficient.

        Parameters
        ----------
        Kd : float
            Derivative gain coefficient.
        """
        self.Kd = Kd

    def set_setpoint(self, setpoint):
        """
        Sets the desired target value for the PID controller.

        Parameters
        ----------
        setpoint : float
            The desired target value.
        """
        self.setpoint = setpoint

    def set_integral_limit(self, limit):
        """
        Sets the limit of the integral term.

        If the integral term exceeds this limit, it will be capped at this value.
        This is useful for preventing windup, where the integral term grows
        without bound due to a large error or large integral gain coefficient.

        Parameters
        ----------
        limit : float
            The maximum absolute value of the integral term.
        """
        self._integral_limit = limit

    def reset_PID(self):
        """
        Resets the PID controller, setting the previous time, error, and integral to zero.

        This is useful for reinitializing the PID controller after a reset
        or when starting a new control loop.
        """
        self._prev_time = time.perf_counter()
        self._prev_error = 0.0
        self._integral = 0.0

    def update(self, measured_value, dt=None):
        """
        Updates the PID controller with a new measured value.

        If dt is provided, uses the given time difference. Otherwise,
        calculates dt based on the wall-clock time.

        Parameters
        ----------
        measured_value : float
            The current measured value.
        dt : float, optional
            The time difference since the last update. If None, will be
            computed automatically. Default is None.

        Returns
        -------
        float
            The output of the PID controller.
        """
        if dt is None:
            current_time = time.perf_counter()
            dt = current_time - self._prev_time
            self._prev_time = current_time
        else:
            self._prev_time += dt

        error = self.setpoint - measured_value

        self._integral += 0.5 * (error + self._prev_error) * dt
        if self._integral_limit is not None:
            if self._integral > self._integral_limit:
                self._integral = self._integral_limit
            elif self._integral < -self._integral_limit:
                self._integral = -self._integral_limit

        if dt > 0:
            self._derivative = (error - self._prev_error) / dt
        else:
            self._derivative = 0

        output = (self.Kp * error) + (self.Ki * self._integral) + \
            (self.Kd * self._derivative)
        self._prev_error = error

        return output
