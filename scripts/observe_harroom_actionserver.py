#!/usr/bin/env python

import rospy
import actionlib
from std_msgs.msg import Empty
from har_observe_task.msg import ObserveHarRoomAction, ObserveHarRoomGoal, ObserveHarRoomResult, ObserveHarRoomFeedback
from deep_object_detection.msg import Object
from deep_object_detection.srv import DetectObjects, DetectObjectsRequest
import rospy
import sys
from datetime import datetime
import rosbag
import copy
import os
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import json

class ObserveHARRoomActionServer(object):
    # Create feedback and result messages
    _feedback = ObserveHarRoomFeedback()
    _result   = ObserveHarRoomResult()

    def __init__(self):
        rospy.init_node("observe_harroom_actionserver")
        self.personcount = 0
        self.runcount = 0
        self.placename = ""
        self.rosbag = None

        ''' Prepare the root path for data storage '''
        self.datarootdir =  os.path.expanduser('~') # Get the home directory
        self.datarootdir +="/harscheduling/"

        if not os.path.exists(self.datarootdir):
            os.makedirs(self.datarootdir)

        self.initializeLogFiles()

        rospy.loginfo("Starting %s", "observe_harroom_actionserver")
        self._action_name = "/observe_harroom"
        rospy.loginfo("Creating action server.")
        self._as = actionlib.SimpleActionServer(self._action_name, ObserveHarRoomAction, execute_cb = self.executeCallback, auto_start = False)
        self._as.register_preempt_callback(self.preemptCallback)
        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")

        self.result_pub = rospy.Publisher("/observe_harroom/observation_result",String,queue_size=5)
        #Publishers
        #self.empty_pub = rospy.Publisher("/direct_ptu_at_origin", Empty)
        #self.pose_pub = rospy.Publisher("/direct_ptu_at_point", Pose)

        rospy.loginfo(" ... Init done")

    def logdata(self,success,place="",person_count=0):
        f = open(self.logfilename, 'a')
        if f:
            if success == 1:
                s = "Success\t"+str(self.runcount)+"\t"+datetime.now().strftime('%Y-%m-%d_%H:%M')+'\t'+place+'\t'+str(person_count)+'\n'
            else:
                s = "Fail\t"+str(self.runcount)+"\t"+datetime.now().strftime('%Y-%m-%d_%H:%M')+'\n'
            f.write(s)
            f.close()
        else:
            rospy.logerr("Log file could not be opened")

    def initializeLogFiles(self):

        self.logfilename = copy.deepcopy(self.datarootdir)
        self.logfilename += datetime.now().strftime('%Y-%m-%d')
        self.logfilename += ".txt"


        if not os.path.isfile(self.logfilename):
            f = open(self.logfilename, 'w')
            f.close()

    def executeCallback(self, goal):
        self.cancelled = False
        self.placename = goal.placename
        self.runcount = goal.runcount
        self.personcount = 0
        filename = copy.deepcopy(self.datarootdir)
        filename += str(goal.runcount)+"_"+goal.placename#datetime.now().strftime('%Y-%m-%d_%H:%M')
        filename += ".bag"
        rospy.loginfo("Rosbag path: %s",filename)
        self.rosbag = rosbag.Bag(filename, 'w')
        self.observation_sub = rospy.Subscriber('/head_xtion/rgb/image_rect_color', Image, callback=self.observationCB)

        rate = rospy.Rate(10.0)
        start = rospy.Time().now()
        while not rospy.is_shutdown():
            rate.sleep()
            self._feedback.personcount = self.personcount
            self._as.publish_feedback(self._feedback)
            #print "Sleeping..."
            now = rospy.Time().now()
            if now - start > rospy.Duration(20) or self.cancelled:
                rospy.loginfo("Time is up or got cancelled")
                self.observation_sub.unregister()
                res = json.dumps([self.placename,self.personcount])
                self.result_pub.publish(res)
                break

        self._result.success = False
        if self.rosbag:
           self.rosbag.close()
           self.rosbag  = None
        if not self.cancelled:
            self.logdata(success=1,place=self.placename,person_count=self.personcount)
            if self.personcount >=2:
                self._result.success = True
            self._as.set_succeeded(self._result)
        else:
            if self.personcount >=2:
                self.logdata(success=1,place=self.placename,person_count=self.personcount)
            else:
                self.logdata(success=0)
        #self._as.set_succeeded(self._result)



    def preemptCallback(self):
        print "Damn, now I got cancelled in the callback"
        #if self.rosbag:
        #    self.rosbag.close()
        #self.logdata(success=0)
        self.observation_sub.unregister()
        self.cancelled = True
        self._result.success = False
        if self.personcount >=2:
            self._result.success = True
        self._as.set_preempted(self._result)


    ''' This is called when a new frame from the camera is received '''
    def observationCB(self, msg):
        #self.sweep_detections.append(self.latest_detections)
        if self.rosbag:
            rospy.loginfo("Writing into rosbag")
            self.rosbag.write('/head_xtion/rgb/image_rect_color',msg,rospy.Time.now())
            if self.personcount >= 2:
                return

        images = []
        images.append(msg)
        try:
            server = rospy.ServiceProxy('/deep_object_detection/detect_objects', DetectObjects)
            req = DetectObjectsRequest()
            req.images=images
            req.confidence_threshold=0.85
            resp = server(req)
        except rospy.ServiceException, e:
            self.observation_sub.unregister()
            print "Service call failed: %s"%e
            return

        person_objs = []

        for obj in resp.objects:
            if obj.label == "person" and obj.width*obj.height > 4096:
                self.personcount +=1
                person_objs.append(obj)

        if self.personcount >= 2:

            rospy.loginfo("Human observation suceeded")

            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            for person_obj in person_objs:
                cv2.rectangle(cv_image,(person_obj.x,person_obj.y),(person_obj.x+person_obj.width, person_obj.y+person_obj.height),color=(255,255,0),thickness=2)
                text = "%.2f"%person_obj.confidence
                cv2.putText(cv_image,text,(person_obj.x+person_obj.width/4,person_obj.y+person_obj.height/2),cv2.FONT_HERSHEY_SIMPLEX,0.7,color=(255,0,255),thickness=2)

            filename = copy.deepcopy(self.datarootdir)
            filename += datetime.now().strftime('%Y-%m-%d_%H:%M:%S')
            filename += ".jpg"
            cv2.imwrite(filename,cv_image)


if __name__ == '__main__':

    ps = ObserveHARRoomActionServer()
    rospy.spin()
