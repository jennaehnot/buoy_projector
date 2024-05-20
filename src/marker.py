#! /usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from mdt_msgs.msg import CartesianMeasure

class MarkerVisualizer:
    def __init__(self):
        #self.marker = Marker()
        self.projection = CartesianMeasure()
        self.projection_sub = rospy.Subscriber("/rgb_projections", CartesianMeasure, self.buildMarker)
        self.marker_pub = rospy.Publisher("/rgb_visualisation_marker",Marker,queue_size=2)

    def buildMarker(self,projection):
        for  detect in projection.detections:
            marker = Marker()
            marker.header.frame_id = projection.header.frame_id
            marker.header.stamp = projection.header.stamp
            marker.type = 2
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            #marker.lifetime = rospy.Duration(3)
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5
            marker.pose.position.x = detect.pos.x  
            marker.pose.position.y = detect.pos.y  
            marker.pose.position.z = detect.pos.z  
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.frame_locked = False #True
            self.marker_pub.publish(marker)

if __name__ == '__main__':
   rospy.init_node('rgb_visualizer')
   visualizer = MarkerVisualizer()
   rospy.spin()

