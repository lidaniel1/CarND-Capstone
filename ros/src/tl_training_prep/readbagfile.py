import rosbag
import numpy as np 
import tensorflow as tf
from tensorflow.contrib.layers import flatten
import pickle
import matplotlib.pyplot as plt
import random
#from sklearn.utils import shuffle
import cv2

from PIL import Image as PIL_Image

from styx_msgs.msg import TrafficLightArray, TrafficLight
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

# topics
topic1 = '/vehicle/traffic_lights_throttle'
topic2 = '/image_color_throttle'

# read messages
def readmsg ():
    label = []
    imgs = []
    t_label=[]
    t_imgs=[]
    cnt1 = 0
    cnt2 = 0
    state = -1 
    bag = rosbag.Bag('/home/student/catkin_ws/rosbag/run11.bag')
    for topic, msg, t in bag.read_messages():
        if topic == topic1:
            x = msg.lights[0].pose.pose.position.x
            y = msg.lights[0].pose.pose.position.y
            z = msg.lights[0].pose.pose.position.z
            state = msg.lights[0].state
            if state ==0:
                tl_color = 'red'
            elif state ==1:
                tl_color = 'yellow'
            elif state ==2:
                tl_color = 'green'
            elif state == 4:
                tl_color = 'unknown'
            label.append([x,y,z,state])
            t_label.append(t.secs)
            cnt1 = cnt1 + 1
            #print("light array len is {}").format(len(msg.lights))
            print ("label msg time {}").format(t.secs)
        elif topic == topic2:
            cnt2 = cnt2 + 1
            img = np.fromstring(msg.data,dtype=np.uint8)
            img = img.reshape(msg.height,msg.width,3)
            img_file = PIL_Image.fromarray(img,'RGB')
            if state >= 0:
                img_file.save('/home/student/catkin_ws/CarND-Capstone/ros/image_label/'+ tl_color + '/run11_' +str(cnt2)+  '.jpg')
            #plt.figure(figsize=(10,10))
            #plt.imshow(img)
            #plt.show()
            imgs.append(img)
            t_imgs.append(t.secs)
           
            print ("image msg time {}").format(t.secs)
    bag.close()
    return label, imgs, t_label,t_imgs,cnt1, cnt2

if __name__ == '__main__':
    label, imgs, t_label,t_imgs, Nlabel, Nimg = readmsg()
    print("number of label {}, number of image {}").format(Nlabel, Nimg)