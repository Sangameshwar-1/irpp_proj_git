#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseArray, TransformStamped
from gazebo_msgs.msg import ModelStates
import tf2_ros


def _match_humans(model_names, prefix, explicit_list):
    if explicit_list:
        return [n for n in model_names if n in explicit_list]
    return [n for n in model_names if n.startswith(prefix)]


class HumanPosePublisher(object):
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.human_prefix = rospy.get_param("~human_prefix", "human_")
        self.human_names = rospy.get_param("~human_names", [])
        self.pub = rospy.Publisher("/human_pose", PoseArray, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self._cb)

    def _cb(self, msg):
        names = _match_humans(msg.name, self.human_prefix, self.human_names)
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = self.frame_id

        for name in names:
            idx = msg.name.index(name)
            pose = msg.pose[idx]
            pose_array.poses.append(pose)

            tf_msg = TransformStamped()
            tf_msg.header.stamp = pose_array.header.stamp
            tf_msg.header.frame_id = self.frame_id
            tf_msg.child_frame_id = name
            tf_msg.transform.translation.x = pose.position.x
            tf_msg.transform.translation.y = pose.position.y
            tf_msg.transform.translation.z = pose.position.z
            tf_msg.transform.rotation = pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)

        self.pub.publish(pose_array)


if __name__ == "__main__":
    rospy.init_node("human_pose_publisher")
    HumanPosePublisher()
    rospy.loginfo("Human pose publisher started")
    rospy.spin()
