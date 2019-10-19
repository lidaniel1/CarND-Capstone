#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32
import numpy as np
import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 10

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint',Int32,self.traffic_cb)
        #rospy.Subscriber('/obstacle_waypoint',Lane,self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None 
        self.base_2dwps = [] 
        self.pose = None 
        self.tree = None
        self.stopline_wp_idx = None
       
        self.idx = -1

        self.waypoints_pub()
        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        rospy.loginfo('base waypoint cb is called')
        self.base_waypoints = waypoints
        if not self.base_2dwps:
            for item in waypoints.waypoints:
                x = item.pose.pose.position.x
                y = item.pose.pose.position.y
                self.base_2dwps.append([x,y])

            self.tree = KDTree(self.base_2dwps)

           # rospy.loginfo("base 2d %s, %s", x,y)
           # rospy.loginfo("base 2d length %s", len(self.base_2dwps))
                                 
    def waypoints_pub(self):                  
            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                # find cloest waypoint 
                self.get_closest_idx()
                if self.idx > 0:
                    #final_lane = Lane()
                    #final_lane.header = self.base_waypoints.header
                    #final_lane.waypoints = self.base_waypoints.waypoints[self.idx:self.idx+LOOKAHEAD_WPS]
                    final_lane = self.generate_final_waypoint()
                    self.final_waypoints_pub.publish(final_lane)
                    #rospy.loginfo("final waypoint index %s, %s",self.idx, self.idx+LOOKAHEAD_WPS )
                    rate.sleep()

    def get_closest_idx(self):
        if self.pose and self.base_waypoints and self.tree:
            x = self.pose.pose.position.x
            y = self.pose.pose.position.y
            #rospy.loginfo("current pose %s, %s", x,y)
            dist, idx = self.tree.query([x,y],1)  
            
            closest_pt = self.base_2dwps[idx]
            prev_pt = self.base_2dwps[idx-1]

            np_closest = np.array(closest_pt)
            np_prev = np.array(prev_pt)
            np_veh = ([x,y])

            vect_path = np_closest - np_prev
            vect_veh = np_closest - np_veh

            dir = np.dot(vect_path,vect_veh)

            # dir < 0, closest pt is behind vehicle
            if dir < 0:
                idx = (idx+1) % len(self.base_2dwps)
            
            self.idx = idx 
            #rospy.loginfo("current pose x %s, y %s",x,y)
            #rospy.loginfo("nearest point in front %s, %s",self.base_2dwps[idx][0], self.base_2dwps[idx][1])
            #rospy.loginfo("nearest point behind %s, %s",self.base_2dwps[idx-1][0], self.base_2dwps[idx-1][1])


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def generate_final_waypoint(self):        
        final_lane = Lane()
        final_lane.header = self.base_waypoints.header

        rospy.loginfo("stop line idx %s", self.stopline_wp_idx)
        if self.idx > 0:
            st_wp_idx = self.idx
            ed_wp_idx = self.idx+LOOKAHEAD_WPS
            base_wp = self.base_waypoints.waypoints[st_wp_idx:ed_wp_idx] 
            rospy.loginfo("closet waypoint index %s",st_wp_idx)
            
                     
            # if traffic light is red or stopline is closer than the look ahead waypoint 
            if self.stopline_wp_idx > -1 and (self.stopline_wp_idx < ed_wp_idx):
                # zero velocity waypoint index relative to st_wp_idx
                # two waypoints back from line so front of car stops at line
                rospy.loginfo("red light detected, stopline wp idx %s, car idx %s, look ahead idx %s", self.stopline_wp_idx,st_wp_idx,ed_wp_idx)
                zero_wp_idx = max(self.stopline_wp_idx - st_wp_idx -2,0)
                temp = []
                for i, wp in enumerate(base_wp):
                    ith_wp = Waypoint()
                    ith_wp.pose = wp.pose

                    dist = self.distance(base_wp,i,zero_wp_idx)
                    vel = math.sqrt(2*MAX_DECEL*dist)
                    if vel < 1.0:
                        vel = 0.0

                    ith_wp.twist.twist.linear.x = min(vel,wp.twist.twist.linear.x)
                    temp.append(ith_wp)

                    rospy.loginfo("ith %s modified way poin cmd vel %s", i,ith_wp.twist.twist.linear.x)  
                    rospy.loginfo("ith way point %s, cmd velocity %s, %s", i,vel,wp.twist.twist.linear.x)
                final_lane.waypoints = temp
            else:
                final_lane.waypoints = base_wp

            rospy.loginfo("base way poin cmd vel %s and final lane cmd vel %s",  self.base_waypoints.waypoints[st_wp_idx], final_lane.waypoints[0].twist.twist.linear.x)  

        return final_lane                

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
