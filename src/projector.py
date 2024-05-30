#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
import image_geometry.cameramodels as cm
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped
from mdt_msgs.msg import CartesianMeasure, CartesianPlot
from cortix_msgs.msg import Ins
import std_msgs.msg
from sensor_msgs.msg import CameraInfo


class Projector:
   def __init__(self):
      self.camera_model = None
      self.to_tf = ''
      self.from_tf = ''
      self.cartPlot_confidence = ''
      self.heave=''
      self.tf_buffer = tf2_ros.Buffer()
      self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
      self.detection_sub = rospy.Subscriber("/yolo/detections", Detection2DArray, self.detection_callback)
      self.cam_info_sub = rospy.Subscriber("/rgb_camera_info/camera_info", CameraInfo, self.camera_info_callback)
      self.projection_pub = rospy.Publisher("/rgb_projections",CartesianMeasure,queue_size=5)
      self.cartesian_publish = rospy.Publisher("cortix/sense/ros_interface/detection/cartesian_plots",CartesianMeasure,queue_size=5)
      self.ins_subscriber = rospy.Subscriber("/pos/d_phins/ins",Ins, self.ins_callback)

   def detection_callback(self,detection_msg):

      # load params for tf
      if rospy.has_param('/buoy_projection/from_tf') and rospy.has_param('/buoy_projection/to_tf'):
         self.from_tf = rospy.get_param('/buoy_projection/from_tf')
         self.to_tf = rospy.get_param('/buoy_projection/to_tf')
         self.cartPlot_confidence =rospy.get_param('/buoy_projection/cartPlot_confidence')
      else:
         rospy.logerr("Somethings wrong with you tf params")

      # make tranformation function to map from rgb_cam at time of time stamp, timeout if longer than 1 sec
      try:
         transformation = self.tf_buffer.lookup_transform(self.to_tf, self.from_tf, detection_msg.header.stamp, rospy.Duration(1.0))
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
         rospy.logwarn('Detection callback transformation exception: '+ str(e) + "/n time diff: " + str((rospy.Time.now()-detection_msg.header.stamp).to_sec()))
      else:
         # create camera position in ship reference frame
         camera_origin = PoseStamped()
         camera_origin.pose.orientation.w = 1.0

         camera_in_ship_frame = tf2_geometry_msgs.do_transform_pose(camera_origin, transformation)
         p1 = camera_in_ship_frame.pose.position
         p1.z = p1.z - self.heave
         # prep msg to be published
         target = CartesianMeasure() 
         target.header = detection_msg.header
         target.header.frame_id = self.to_tf
         target.sensor_id = "rgb_cam"

         for detect in detection_msg.detections:
            # for each detection in the detection message, create a cartesianPlot msg
            P = CartesianPlot()
            # Pack dictionary with detection info
            buoy = {'Class': detect.results[0].id, 'Confidence':detect.results[0].score, 
                  'x': detect.bbox.center.x, 'y': detect.bbox.center.y, 'w': detect.bbox.size_x, 'h': detect.bbox.size_y}
            buoy['Bottom Coord']=(detect.bbox.center.x, detect.bbox.center.y - detect.bbox.size_y/2)

            #verify camera callback worked, rectify point at center,bottom of bbox with cal matrix
            if self.camera_model is not None: 
               try:
                  #rectify pixel
                  rectifiedTarget = self.camera_model.rectifyPoint(buoy['Bottom Coord'])
               except Exception as e:
                  rospy.logwarn("Issue with rectifying target"+ str(r))
                  rectifiedTarget = None
               if rectifiedTarget is not None:
                  #project pixel to ray (in camera coords)
                  try: 
                     ray = self.camera_model.projectPixelTo3dRay(rectifiedTarget)
                  except Exception as e:
                     rospy.logwarn('projectPixelTo3dRay exception:'+str(e))
                     P = None
                  pixel_in_camera_frame = PoseStamped()
                  pixel_in_camera_frame.pose.position.x = ray[0]
                  pixel_in_camera_frame.pose.position.y = ray[1]
                  pixel_in_camera_frame.pose.position.z = ray[2]
                  pixel_in_camera_frame.pose.orientation.w = 1.0
                  try:
                     transformation = self.tf_buffer.lookup_transform(self.to_tf, "rgb_cam_optical", detection_msg.header.stamp, rospy.Duration(1.0))
                     pixel_in_ship_frame = tf2_geometry_msgs.do_transform_pose(pixel_in_camera_frame, transformation)
                     p2 = pixel_in_ship_frame.pose.position

                  except Exception as e:
                     rospy.logwarn("Issue with pixel transform to ship ref frame: ", e)
                     P = None
               
                  #if there's no issues with the pixel projection
                  if P is not None:

                     u = -p1.z/(p2.z-p1.z) #constant val needed in point calc
                     P.pos.x = p1.x + u*(p2.x - p1.x)

                     #filter out impossible detections
                     if (P.pos.x > 0) and (P.pos.x < 100):
                        P.pos.y = p1.y + u*(p2.y - p1.y) 
                        P.pos.z = 0
                        P.sigma_xx = P.sigma_yy = P.sigma_xy = 5
                        P.confidence = self.cartPlot_confidence
                        P.speed_confidence = P.cog = P.sog = 0
                        target.detections.append(P)

            self.projection_pub.publish(target)

            # publish to tracker
            if rospy.get_param('/rgb2tracker') is 1:
               self.cartesian_publish.publish(target)            


   def camera_info_callback(self, cam_msg):
      self.camera_model = cm.PinholeCameraModel()
      self.camera_model.fromCameraInfo(cam_msg)
      pass

   def ins_callback(self, ins_msg):
      self.heave=ins_msg.smart_heave_m
      pass

if __name__ == '__main__':
   rospy.init_node('projector')
   Projector()
   rospy.spin()
