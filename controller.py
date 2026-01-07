class PID:
    """
    Discrete PID controller with anti-windup and derivative filtering.
    """

    def __init__(self, kp, ki, kd, dt, output_limits=(None, None), tau=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

        self.min_out, self.max_out = output_limits
        self.tau = tau  # derivative filter time constant

        self.integral = 0.0
        # self.prev_error = 0.0
        self.prev_measurement = 0.0
        self.d_state = 0.0

    def reset(self):
        self.integral = 0.0
        # self.prev_error = 0.0
        self.prev_measurement = 0.0
        self.d_state = 0.0

    def update(self, setpoint, measurement, dt, kp, ki, kd) -> float:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
    
        error = setpoint - measurement

        # Proportional
        p = self.kp * error

        # Integral with anti-windup
        self.integral += error * self.dt
        i = self.ki * self.integral

        # Derivative on measurement (noise-robust)
        # d_raw = -(error - self.prev_error) / self.dt
        d_raw = -(measurement - self.prev_measurement) / self.dt
        if self.tau > 0.0:
            # First-order low-pass filter
            self.d_state += (self.dt / (self.tau + self.dt)) * (d_raw - self.d_state)
            d = self.kd * self.d_state
        else:
            # No filtering
            d = self.kd * d_raw

        # Control output
        u = p + i + d

        # Apply saturation
        if self.min_out is not None:
            u = max(self.min_out, u)
        if self.max_out is not None:
            u = min(self.max_out, u)

        # self.prev_error = error
        self.prev_measurement = measurement

        return u
    
class LowPassFilter:
    def __init__(self, tau, dt, initial=0.0):
        self.tau = tau
        self.alpha = dt / (tau + dt)
        self.y = initial

    def update(self, x):
        self.y += self.alpha * (x - self.y)
        return self.y
    
    def update_dynamic(self, x, dt):
        self.alpha = dt / (self.tau + dt)
        self.y += self.alpha * (x - self.y)
        return self.y