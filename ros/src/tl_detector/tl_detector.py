#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
from scipy.spatial import KDTree
import math

STATE_COUNT_THRESHOLD = 1
LOOKAHEAD_WPS = 200 
IMAGE_PROCESS_PERIOD = 3 # process only 1 image every 3 images

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.base_2dwps = [] 
        self.tree = None
        self.light_wp = None 

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.image_process_count = 0


        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()


        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.base_2dwps:
            for item in waypoints.waypoints:
                x = item.pose.pose.position.x
                y = item.pose.pose.position.y
                self.base_2dwps.append([x,y])

            self.tree = KDTree(self.base_2dwps)
            
    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        rospy.loginfo("image call back start")

        # image classification only called once every # (IMAGE_PROCESS_PERIOD) of images
        if self.image_process_count == 0:
            now = rospy.get_rostime()
            rospy.loginfo("traffic light processing start time sec %i ,nsec %i",now.secs,now.nsecs)
            light_wp, state = self.process_traffic_lights()
            now = rospy.get_rostime()
            rospy.loginfo("traffic light processing end time sec %i ,nsec %i",now.secs,now.nsecs)
        else:
            light_wp = self.last_wp
            state = self.last_state

        self.image_process_count += 1
        self.image_process_count = self.image_process_count % IMAGE_PROCESS_PERIOD
        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            # self.state_count = 0  # original
            self.state_count = 1
            self.state = state   
        #elif self.state_count >= STATE_COUNT_THRESHOLD:

        if self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
            rospy.loginfo("stop line way point idx %s", light_wp)
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))

        self.state_count += 1

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        if self.tree:
            dist, idx = self.tree.query([x,y],1)          
            return idx

        return -1

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #return light.state

        
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        #Get classification
        # Tensorflow need some time to initialize, if click "camera" too soon after launching the node it will error out
        while True:
            try:
                return self.light_classifier.get_classification(self.camera_image)
            except:
                continue
            break

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light = None
        light_wp_idx = None
        dist = 999

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']

        if self.pose and self.tree:
            x = self.pose.pose.position.x
            y = self.pose.pose.position.y 
            car_position = self.get_closest_waypoint(x,y)
            
            #TODO find the closest visible traffic light (if one exists)
            min_diff = len(self.waypoints.waypoints)

            for i, light in enumerate(self.lights):
                # stop line location
                item = stop_line_positions[i]
                item_idx = self.get_closest_waypoint(item[0],item[1])

                diff = item_idx - car_position
                if diff >=0 and diff < min_diff:
                    min_diff = diff
                    #dist = math.sqrt( pow(x-item[0]) + pow(y-item[1]))
                    closest_light = light
                    light_wp_idx = item_idx

        # only classify traffic ligh if the stop line to car position is less than LOOKAHEAD_WPS, 
        # LOOKAHEAD_WPS +2 counts for car_position is closest not necessarily ahead of car as the code in the waypoint_update.     
        if closest_light and min_diff <= LOOKAHEAD_WPS +2:
            state = self.get_light_state(light)
            rospy.loginfo("tl detector light state, %s", state)
            return light_wp_idx, state

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
