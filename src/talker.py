import rospy
import tf_conversions
import tf2_ros
import geometry_msgs.msg

def broadcaster():
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "ship_ref_point"
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = 0
    t.transform.rotation.w = 1
    br.sendTransform(t)

if __name__ == '__main__':
    try:
        broadcaster()
    except rospy.ROSInterruptException:
        pass
