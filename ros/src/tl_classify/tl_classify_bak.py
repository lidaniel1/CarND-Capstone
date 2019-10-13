import rospy
import numpy as np 
import tensorflow as tf
from tensorflow.contrib.layers import flatten
import pickle
import matplotlib.pyplot as plt
import random
from sklearn.utils import shuffle
import cv2


class tl_class(Object):
    def __init__(self):       
        rospy.Subscriber('/vehicle/traffic_lights_throttle',TrafficLightArray,self.tf_light_cb)
        rospy.Subscriber('/image_color_throttle',Image, self.image_cb)
        
        self.img = None 
        self.tf_state = None 
        

    def tf_light_cb(self,msg):
        self.tf_state = msg.state 
        self.tf_x = msg.pose.position.x 
        self.tf_y = msg.pose.position.y 
    
    def image_cb(self,msg):
        self.img = msg.data 
        self.height = msg.height
        self.width = msg.width 

    def analysis(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.img and self.tf_state:
                

if __name__ == '__main__' : 
    tl_class()

        