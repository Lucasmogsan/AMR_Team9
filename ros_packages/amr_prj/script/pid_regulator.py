# pid_regulator.py

class PIDRegulator:
    def __init__(self, p_gain, i_gain, d_gain, saturation):
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain
        self.saturation = saturation
        self.error_sum = 0
        self.last_error = 0

    def reset(self):
        self.error_sum = 0
        self.last_error = 0

    def update(self, setpoint, measurement, dt):
        error = setpoint - measurement
        self.error_sum += error * dt
        d_error = (error - self.last_error) / dt
        self.last_error = error

        # Calculate PID output before saturation
        output = (self.p_gain * error +
                  self.i_gain * self.error_sum +
                  self.d_gain * d_error)

        # Apply saturation limit
        output = max(min(output, self.saturation), -self.saturation)

        return output
