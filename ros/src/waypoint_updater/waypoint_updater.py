#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
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


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        #rospy.Subscriber('/traffic_waypoint',Lane,self.traffic_cb)
        #rospy.Subscriber('/obstacle_waypoint',Lane,self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None 
        self.base_2dwps = [] 
        self.pose = None 
        self.tree = None    
       
        self.idx = -1

        self.waypoints_pub()
        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints
        if not self.base_2dwps:
            for item in waypoints.waypoints:
                x = item.pose.pose.position.x
                y = item.pose.pose.position.y
                self.base_2dwps.append([x,y])

            self.tree = KDTree(self.base_2dwps)

            rospy.loginfo("base 2d %s, %s", x,y)
            rospy.loginfo("base 2d length %s", len(self.base_2dwps))
                       
           
    def waypoints_pub(self):                  
            rate = rospy.Rate(50)
            while not rospy.is_shutdown():
                self.get_finalwaypoints()
                if self.idx > 0:
                    final_lane = Lane()
                    final_lane.header = self.base_waypoints.header
                    final_lane.waypoints = self.base_waypoints.waypoints[self.idx::self.idx+LOOKAHEAD_WPS]
                    self.final_waypoints_pub.publish(final_lane)
                    #rospy.loginfo("final waypoints is published")
                    rate.sleep()

    def get_finalwaypoints(self):
        if self.pose and self.base_waypoints and self.tree:
            x = self.pose.pose.position.x
            y = self.pose.pose.position.y
            rospy.loginfo("current pose %s, %s", x,y)
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
            #rospy.loginfo("closest idx %s", idx)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
