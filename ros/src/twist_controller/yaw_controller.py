from math import atan
import rospy

class YawController(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = min_speed
        self.max_lat_accel = max_lat_accel

        self.min_angle = -max_steer_angle
        self.max_angle = max_steer_angle
        #rospy.loginfo("wheel base %s, steer ratio is %s, max steer angle %s", self.wheel_base,self.steer_ratio,self.max_angle)

    def get_angle(self, radius):
        angle = atan(self.wheel_base / radius) * self.steer_ratio
        #rospy.loginfo("steering angle %s", angle)
        return max(self.min_angle, min(self.max_angle, angle))

    def get_steering(self, linear_velocity, angular_velocity, current_velocity):
        angular_velocity = current_velocity * angular_velocity / linear_velocity if abs(linear_velocity) > 0. else 0.
        #rospy.loginfo("cmd linear %s, cmd angular vel %s, act linear vel %s", linear_velocity,angular_velocity,current_velocity)

        if abs(current_velocity) > 0.1:
            max_yaw_rate = abs(self.max_lat_accel / current_velocity);
            angular_velocity = max(-max_yaw_rate, min(max_yaw_rate, angular_velocity))
            #rospy.loginfo("max yaw rate %s, ang velocity %s", max_yaw_rate, angular_velocity)

        return self.get_angle(max(current_velocity, self.min_speed) / angular_velocity) if abs(angular_velocity) > 0. else 0.0;
