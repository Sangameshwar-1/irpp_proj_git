#!/usr/bin/env python
import math
import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState


def _quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class HumanMover(object):
    def __init__(self):
        self.rate_hz = rospy.get_param("~rate", 10.0)
        self.humans = rospy.get_param("~humans", [
            {
                "name": "human_moving_1",
                "speed": 0.4,
                "waypoints": [
                    [-2.5, 0.0, 0.0],
                    [0.0, 0.0, 0.0],
                    [2.5, 0.0, math.pi],
                    [0.0, 0.0, math.pi],
                ],
            },
            {
                "name": "human_moving_2",
                "speed": 0.3,
                "waypoints": [
                    [2.5, 0.3, math.pi],
                    [0.0, 0.8, math.pi],
                    [-2.5, 0.3, 0.0],
                    [0.0, -0.8, 0.0],
                ],
            },
        ])
        self.state = {}
        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self._init_segments()

    def _init_segments(self):
        now = rospy.Time.now().to_sec()
        for human in self.humans:
            wps = human["waypoints"]
            self.state[human["name"]] = {
                "idx": 0,
                "start": now,
                "duration": self._segment_duration(wps[0], wps[1], human["speed"]),
            }

    @staticmethod
    def _segment_duration(a, b, speed):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        dist = math.hypot(dx, dy)
        return max(dist / max(speed, 0.01), 0.1)

    def _update_human(self, human, now):
        name = human["name"]
        wps = human["waypoints"]
        st = self.state[name]
        idx = st["idx"]
        nxt = (idx + 1) % len(wps)
        t = (now - st["start"]) / st["duration"]

        if t >= 1.0:
            st["idx"] = nxt
            st["start"] = now
            st["duration"] = self._segment_duration(wps[nxt], wps[(nxt + 1) % len(wps)], human["speed"])
            idx = st["idx"]
            nxt = (idx + 1) % len(wps)
            t = 0.0

        ax, ay, ayaw = wps[idx]
        bx, by, byaw = wps[nxt]
        x = ax + (bx - ax) * t
        y = ay + (by - ay) * t
        yaw = ayaw + (byaw - ayaw) * t

        qx, qy, qz, qw = _quat_from_yaw(yaw)
        model_state = ModelState()
        model_state.model_name = name
        model_state.pose.position.x = x
        model_state.pose.position.y = y
        model_state.pose.position.z = 0.0
        model_state.pose.orientation.x = qx
        model_state.pose.orientation.y = qy
        model_state.pose.orientation.z = qz
        model_state.pose.orientation.w = qw
        model_state.reference_frame = "world"
        self.set_state(model_state)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            for human in self.humans:
                self._update_human(human, now)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("move_humans")
    mover = HumanMover()
    rospy.loginfo("Human mover started")
    mover.spin()
