from light_classification.tl_classifier import TLClassifier

from styx_msgs.msg import TrafficLightArray, TrafficLight
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

import rosbag
import numpy as np 
from utils import label_map_util
from utils import visualization_utils as vis_util 
import matplotlib.pyplot as plt


if __name__== "__main__":

    light_class = TLClassifier()
    
    # topics
    topic1 = '/vehicle/traffic_lights_throttle'
    topic2 = '/image_color_throttle'

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
        elif topic == topic2:

            img = np.fromstring(msg.data,dtype=np.uint8)
            img = img.reshape(msg.height,msg.width,3)
            result = light_class.get_classification(msg)
            plt.figure(figsize=(10,10))
            plt.imshow(img)
            plt.show()
            print ("classification result is {}").format(result)

    bag.close()