
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx
        self.p_effort = 0
        self.i_effort = 0
        self.d_effort = 0

        self.int_val = self.last_error = 0.

    def reset(self):
        self.int_val = 0.0
        self.last_error = 0.0
        self.p_effort = 0
        self.i_effort = 0
        self.d_effort = 0

<<<<<<< HEAD
    def step(self, error, sample_time):
        derivative = (error - self.last_error) / sample_time
        # We calculate derivative portion only if we already had at least one measurement since reset
        if not any([self.last_error, self.int_val, self.p_effort, self.i_effort, self.d_effort]):
            derivative = 0
        self.int_val += error
        self.last_error = error
        
        self.p_effort = self.kp * error
        self.i_effort = self.ki * self.int_val
        self.d_effort = self.kd * derivative

        y = self.p_effort + self.i_effort + self.d_effort
        if y > self.max:
            # we want to ensure that the integral part won't grow too much and pid remains continuous
            self.int_val = max((self.max - self.p_effort) / self.ki, 0)
            self.i_effort = self.ki * self.int_val
        val = max(self.min, min(y, self.max))
	
        return val

    def get_PID(self):

        return self.p_effort, self.i_effort, self.d_effort
=======
    def step(self, error, sample_time, log_handle):
        self.last_int_val = self.int_val

        integral = self.int_val + error * sample_time;
        derivative = (error - self.last_error) / sample_time;
        
        p_effort = self.kp * error
        i_effort = self.ki * self.int_val
        d_effort = self.kd * derivative

        y = p_effort + i_effort + d_effort
        val = max(self.min, min(y, self.max))
	
        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            self.int_val = integral
        self.last_error = error
        
        self.log_data(log_handle, p_effort, i_effort, d_effort)
   
        return val

    def log_data(self, log_handle, *args):
        log_handle.write(','.join(str(arg) for arg in args) + ',')
>>>>>>> bad2e61290983b1b85958c96b83e35c80c0c8b22
