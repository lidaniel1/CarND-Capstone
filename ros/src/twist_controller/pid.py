import rospy

MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx

        self.int_val = self.last_error = 0.

    def reset(self):
        self.int_val = 0.0

    def step(self, error, sample_time):

        integral = self.int_val + error * sample_time;
        derivative = (error - self.last_error) / sample_time;

        val = self.kp * error + self.ki * integral + self.kd * derivative;

        #rospy.loginfo("kp %s, ki %s, kd %s", self.kp, self.ki, self.kd)

        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            self.int_val = integral
        self.last_error = error

        #ospy.loginfo("throttle %s, error %s, integral error %s, derivative error %s", val, error, integral, derivative)
        return val
