
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import math
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base,steer_ratio,min_speed,max_lat_accel,max_steer_angle,vehicle_mass,
                    brake_deadband,decel_limit,accel_limit,wheel_radius):
        # TODO: Implement
        self.yawctrl = YawController(wheel_base,steer_ratio,min_speed,max_lat_accel,max_steer_angle)
        self.throttlectrl = PID(0.4, 0.05, 0, mn=0, mx=1)
        self.lowpass = LowPassFilter(0.1,0.02) #sample time is 1/50 = 0.02
        self.lowpass_angular = LowPassFilter(0.1,0.02) #sample time is 1/50 = 0.02

        self.vehicle_mass = vehicle_mass
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        #rospy.loginfo("decel limit %s, accel limit %s", self.decel_limit,self.accel_limit)

        self.last_actlinear_vel = 0
        #self.last_cmd_linvel = 0

        self.last_time = rospy.get_time()

    def brakectrl(self,throttle, cmdlinear_vel,cmdlinear_accel,filt_actlinear_vel):
        if cmdlinear_vel == 0. and filt_actlinear_vel < 0.1:
            brake = 700
        elif  cmdlinear_accel < 0 and throttle < 0.1:
            brake = abs(cmdlinear_accel) * self.vehicle_mass * self.wheel_radius 
        else:
            brake = 0
        
        if brake < self.brake_deadband:
            brake = 0

        return brake    

    def control(self, actlinear_vel,actangular_vel,cmdlinear_vel,cmdangular_vel,dbw_status):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        if not dbw_status:
            self.throttlectrl.reset()
            return 0.0, 0.0, 0.0 

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        filt_actlinear_vel = self.lowpass.filt(actlinear_vel)

        #rospy.loginfo("sample time %s", sample_time)
        actlinear_accel = (filt_actlinear_vel - self.last_actlinear_vel)/sample_time  
        cmdlinear_accel = (cmdlinear_vel - filt_actlinear_vel)/sample_time  #50Hz (cmd vs actual accel)

        if cmdlinear_accel > self.accel_limit:
            cmdlinear_accel = self.accel_limit               
        elif cmdlinear_accel < self.decel_limit:
            cmdlinear_accel = self.decel_limit
        
        throttle = self.throttlectrl.step(cmdlinear_vel - filt_actlinear_vel,sample_time) 
       # throttle = self.throttlectrl.step(5.0 - filt_actlinear_vel,sample_time) 

        brake = self.brakectrl(throttle,cmdlinear_vel,cmdlinear_accel,filt_actlinear_vel)

        if brake > 0:
            throttle = 0

        steer = self.yawctrl.get_steering(cmdlinear_vel,cmdangular_vel,filt_actlinear_vel)
        #steer = self.lowpass_angular.filt(steer)

        #rospy.loginfo("throttle %s, brake %s, steer %s", throttle,brake,steer)

        self.last_actlinear_vel = filt_actlinear_vel
        self.last_time = current_time
       

        return throttle, brake, steer
