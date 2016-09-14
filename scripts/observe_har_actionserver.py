#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty
from har_observe_task.msg import ObserveHarAction, ObserveHarGoal, ObserveHarResult, ObserveHarFeedback

class ObserveHARActionServer(object):
    # Create feedback and result messages
    _feedback = ObserveHarFeedback()
    _result   = ObserveHarResult()

    def __init__(self):
        rospy.init_node("observe_har_actionserver")

        rospy.loginfo("Starting %s", "observe_har_actionserver")
        self._action_name = "/observe_har"
        rospy.loginfo("Creating action server.")
        self._as = actionlib.SimpleActionServer(self._action_name, ObserveHarAction, execute_cb = self.executeCallback, auto_start = False)
        self._as.register_preempt_callback(self.preemptCallback)
        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")

        #Publishers
        self.empty_pub = rospy.Publisher("/direct_ptu_at_origin", Empty)
        self.pose_pub = rospy.Publisher("/direct_ptu_at_point", Pose)

        rospy.loginfo(" ... Init done")


    def executeCallback(self, goal):
        self.cancelled = False

        self.pose_pub.publish(goal.observe_point)

        rate = rospy.Rate(1.0)
        start = rospy.Time().now()
        while !rospy.is_shutdown:
            rate.sleep()
            now = rospy.Time().now()
            if now - start > rospy.Duration(60*1) or self.cancelled:
                break

        self.empty_pub.publish(Empty())
        if not self.cancelled :
            self._result.success = True
            self._as.set_succeeded(self._result)


    def preemptCallback(self):
        self.cancelled = True

        self.empty_pub.publish(Empty())

        self._result.success = False
        self._as.set_preempted(self._result)

if __name__ == '__main__':

    ps = ObserveHARActionServer()
    rospy.spin()
