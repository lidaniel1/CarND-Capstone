
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import math
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base,steer_ratio,min_speed,max_lat_accel,max_steer_angle,vehicle_mass,brake_deadband,decel_limit,accel_limit,wheel_radius):
        # TODO: Implement
        self.yawctrl = YawController(wheel_base,steer_ratio,min_speed,max_lat_accel,max_steer_angle)
        self.throttlectrl = PID(0.05, 0.0005, 1.5, mn=0, mx=1)
        #self.brakectrl = PID(10, 0.1, 5, mn=0, mx=6000)
        self.lowpass = LowPassFilter(0.5,0.02) #sample time is 1/50 = 0.02

        self.vehicle_mass = vehicle_mass
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_actlinear_vel = 0
        #self.last_cmd_linvel = 0

        self.last_time = rospy.get_time()

    def brakectrl(self,throttle, cmdlinear_vel,cmdlinear_accel,filt_actlinear_vel):
        if cmd_linvel == 0. and filt_actlinear_vel < 0.1:
            brake = 700
        elif  cmdlinear_accel < 0 and throttle < 0.1:
            brake = abs(cmdlinear_accel) * self.vehicle_mass * self.wheel_radius 
        else
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
       
        actlinear_accel = (filt_actlinear_vel - self.last_actlinear_vel)/sample_time  
        cmdlinear_accel = (cmdlinear_vel - filt_actlinear_vel)/sample_time  #50Hz (cmd vs actual accel)
        if cmdlinear_accel > self.accel_limit:
            cmdlinear_accel = self.accel_limit
            cmdlinear_vel = cmdlinear_accel * sample_time + filt_actlinear_vel                
        elif cmdlinear_accel < self.decel_limit:
            cmdlinear_accel = self.decel_limit
            cmdlinear_vel = cmdlinear_accel * sample_time + filt_actlinear_vel  
        
        throttle = self.throttlectrl.step(cmdlinear_vel - filt_actlinear_vel,sample_time)
        if throttle > 1:
            throttle = 1               
        elif throttle < 0:
            throttle = 0     

        brake = self.brakectrl(throttle,cmdlinear_vel,cmdlinear_accel,filt_actlinear_vel)

        if brake > 0:
            throttle = 0

        steer = self.yawctrl.get_steering(cmdlinear_vel,cmdangular_vel,filt_actlinear_vel)


        self.last_actlinear_vel = filt_actlinear_vel
        self.last_time = current_time
        #self.last_cmd_linvel = filt_cmd_linvel

        return throttle, brake, steer
