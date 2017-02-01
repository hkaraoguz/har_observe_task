#!/usr/bin/python
from strands_executive_msgs import task_utils
from strands_executive_msgs.msg import Task, TaskEvent
import strands_executive_msgs
from strands_executive_msgs.srv import AddTasks, AddTask
from strands_navigation_msgs.srv import LocalisePose
from find_waypoints import *
from create_tasks import *
import rospy
import sys
from datetime import datetime
import numpy as np
from random import randint
from deep_object_detection.msg import Object
from deep_object_detection.srv import DetectObjects, DetectObjectsRequest
from semantic_map.msg import RoomObservation
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import rosbag
import copy
import os
from collections import namedtuple
Rectangle = namedtuple('Rectangle', 'xmin ymin xmax ymax')

def area(a, b):  # returns None if rectangles don't intersect
    dx = min(a.xmax, b.xmax) - max(a.xmin, b.xmin)
    dy = min(a.ymax, b.ymax) - max(a.ymin, b.ymin)
    if (dx>=0) and (dy>=0):
        return dx*dy
    return 0

class HARTaskManager():


    def __init__(self):

        self.minutes=[]
        self.images = []

        self.previous_person_locations = []
        self.current_person_locations = []

        self.current_task_id = -1

        self.previous_task_num = -1
        self.current_task_num = -1

        self.current_wait_task_id = -1

        self.previoustasktimeslot = -1
        self.should_update_model = 0
        self.person_count  = 0
        self.timer = None
        self.rosbag = None
        self.waypoints = ['WayPoint19','WayPoint20','WayPoint23','WayPoint10']
        self.placenames =['Office612','Office621','MeetingRoom','Kitchen']
        self.goto_tasks = []
        self.initialize_bandit(len(self.waypoints))

        ''' Prepare the root path for data storage '''
        self.datarootdir =  os.path.expanduser('~') # Get the home directory
        self.datarootdir +="/harscheduling/"

        if not os.path.exists(self.datarootdir):
            os.makedirs(self.datarootdir)

        self.logfilename = copy.deepcopy(self.datarootdir)
        self.logfilename += datetime.now().strftime('%Y-%m-%d')
        self.logfilename += ".txt"

        self.observationfilename = copy.deepcopy(self.datarootdir)
        self.observationfilename += datetime.now().strftime('%Y-%m-%d_observations')
        self.observationfilename += ".txt"

        if not os.path.isfile(self.logfilename):
            f = open(self.logfilename, 'w')
            f.close()

        if not os.path.isfile(self.observationfilename):
            f = open(self.observationfilename, 'w')
            f.close()

        ''''''

        ''' Services and message subscriptions '''
        self.add_task_srv_name = '/task_executor/add_task'
        self.set_exe_stat_srv_name = '/task_executor/set_execution_status'
        self.finished_sub = rospy.Subscriber('/local_metric_map/room_observations', RoomObservation, callback=self.finishedCB)

        sub = rospy.Subscriber("task_executor/events",TaskEvent,self.taskexecutorCB)
        ''''''

        for waypoint in self.waypoints:
            self.goto_tasks.append(create_go_to_waypoint_task(waypoint))

        # wait at Charging Point middle of the corridor
        #self.wait_task = create_wait_action_task()
        ''' Create the charging task '''
        self.wait_task = create_wait_action_task("ChargingPoint")
        ''''''
        ''' Check For Services '''
        try:
            rospy.wait_for_service(self.add_task_srv_name,timeout=10)
        except:
            rospy.logerr("Service not available!!")
            sys.exit(-1)
        try:
            rospy.wait_for_service(self.set_exe_stat_srv_name,timeout=10)
        except:
            rospy.logerr("Service not available!!")
            sys.exit(-1)
        ''''''

        self.create_timeslot_array()



        #self.sweep_tasks = create_har_sweep_tasks("journalExpKTH","roi","configHARRooms")

        #print self.sweep_tasks.keys()
    def logdata(self,success,place="",person_count=0):
        f = open(self.logfilename, 'a')
        if f:
            if success is 1:
                s = "Success\t"+datetime.now().strftime('%Y-%m-%d_%H:%M')+'\t'+place+'\t'+str(person_count)+'\n'
            else:
                s = "Fail\t"+datetime.now().strftime('%Y-%m-%d_%H:%M')+'\n'
            f.write(s)
            f.close()
        else:
            rospy.logerr("Log file could not be opened")



    def timerCB(self,event):
        rospy.loginfo("Timer callback")

        self.observation_sub.unregister()
        self.logdata(success=1,place=self.placenames[self.current_task_num],person_count=self.person_count)

        self.update_observations(self.observed_data,self.person_count)

        self.current_wait_task_id=self.send_task(self.wait_task)
        self.rosbag.close()
        self.rosbag = None
        self.person_count = 0

    def observationCB(self, msg):
        #self.sweep_detections.append(self.latest_detections)
        self.images.append(msg)
        try:
            rospy.wait_for_service('/deep_object_detection/detect_objects',timeout=10)
        except:
            self.images=[]
            self.observation_sub.unregister()
            rospy.logerr("No deep_object detection service")
            return
        try:
            server = rospy.ServiceProxy('/deep_object_detection/detect_objects', DetectObjects)
            req = DetectObjectsRequest()
            req.images=self.images
            req.confidence_threshold=0.85
            resp = server(req)
        except rospy.ServiceException, e:
            self.images=[]
            print "Service call failed: %s"%e
            return

        if self.rosbag:
            rospy.loginfo("Writing into rosbag")
            self.rosbag.write('/head_xtion/rgb/image_rect_color',msg,rospy.Time.now())

        person_objs = []
        self.images = []

        for obj in resp.objects:
            if obj.label == "person" and obj.width*obj.height > 4096:
                self.person_count +=1
                aRect = Rectangle(obj.x-obj.width/2,obj.x+obj.width/2,obj.y-obj.height/2,obj.y+obj.height/2)
                self.current_person_locations.append(aRect)
                person_objs.append(obj)

        if self.person_count >= 2:

            self.observation_sub.unregister()
            if self.timer:
                self.timer.shutdown()
                self.timer =None

            rospy.loginfo("Human observation suceeded")
            self.logdata(success=1,place=self.placenames[self.current_task_num],person_count=self.person_count)
            self.update_observations(self.observed_data,self.person_count)
            self.rosbag.close()
            self.rosbag = None
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
            self.images = []

            self.current_wait_task_id=self.send_task(self.wait_task)




    def finishedCB(self, msg):

        sweep_dir = path.abspath(path.join(path.abspath(msg.xml_file_name), path.pardir))

        req = DetectObjectRequest()
        req.images = self.images

        try:
            rospy.wait_for_service('/deep_object_detection/detect_objects',timeout=10)
        except:
            rospy.logerr("No deep_object detection service")
            return

        try:
            rospy.wait_for_service('/topological_localisation/localise_pose',timeout=10)
        except:
            rospy.logerr("No topological localization service")
            return

    def startObservationCB(self,event):
        rospy.loginfo("Start Observation callback")

        filename = copy.deepcopy(self.datarootdir)
        filename += datetime.now().strftime('%Y-%m-%d_%H:%M')
        filename += ".bag"
        rospy.loginfo("Rosbag path: %s",filename)
        self.rosbag = rosbag.Bag(filename, 'w')
        self.person_count = 0
        self.observation_sub = rospy.Subscriber('/head_xtion/rgb/image_rect_color', Image, callback=self.observationCB)
        self.timer = rospy.Timer(rospy.Duration(25), self.timerCB,oneshot=True)

            #sys.exit(-1)
    def taskexecutorCB(self,taskevent):
        #task is not succeeded
        #print taskevent.event
        #print taskevent.task.task_id
        #print self.current_task_id
        if taskevent.event > 9 and taskevent.event is not 16:

            if taskevent.task.task_id is self.current_task_id:
                rospy.logwarn("Goto task did not succeed")
                self.should_update_model = 0
                self.logdata(success=0)
                self.current_wait_task_id=self.send_task(self.wait_task)
           # elif taskevent.task.task_id is self.current_wait_task_id:
           #     self.current_wait_task_id = self.send_task(self.wait_task)
        if taskevent.task.task_id is self.current_task_id:
            if taskevent.event is 16:
                rospy.loginfo("Task suceeded")
                self.timer = rospy.Timer(rospy.Duration(10), self.startObservationCB,oneshot=True)




    def create_timeslot_array(self):
        self.minutes = [-1]*1440
        count = 0
        arange = np.arange(1,10.1,0.2)
        #print arange
        for i in arange:
            for j in range(-2,2):
                #print int((i+8)*60)+j
                self.minutes[int((i+8)*60)+j] = count
            count+=1

    def check_timeslot(self):
        currentTime = rospy.Time.now().secs
        date = datetime.fromtimestamp(currentTime)
        #print dir(date)
        #print date.hour,date.minute
        timeminutes = date.hour*60 + date.minute
        #print timeminutes
        if(self.minutes[timeminutes] >= 0 and self.minutes[timeminutes] is not self.previoustasktimeslot):
            rospy.loginfo("Now it is time to do a task")
            # make sure the executive is running -- this only needs to be done once for the whole system not for every task
            set_execution_status = rospy.ServiceProxy(self.set_exe_stat_srv_name, strands_executive_msgs.srv.SetExecutionStatus)
            set_execution_status(True)
            self.previoustasktimeslot = self.minutes[timeminutes]
            return True
        return False

    def check_person_overlaps(self,current_observations,previous_observations):
        overlaps = 0
        for curobs in current_observations:
            for prevobs in previous_observations:
                if area(curobs,prevobs) >= 0.6:
                    overlaps += 1
        rospy.loginfo("Overlaps %d, current observations %d",overlaps, len(current_observations))
        if float(overlaps)/len(current_observations) > 0.5:
            return True

        return False


    def update_observations(self,observed_data,person_count):

        if  person_count >= 2:
            if self.previous_task_num is self.current_task_num:
                if(self.check_person_overlaps(self.current_person_locations,self.previous_person_locations)):
                    update_ind = 1
                else:
                    update_ind = 0
            else:
                update_ind = 0
        else:
            update_ind = 1

        self.observed_data[self.current_task_num,update_ind] += 1
        np.savetxt(self.observationfilename,self.observed_data)
        #f = open(self.observationfilename, 'w')
        #f.write(self.observed_data)

    def thompson_sampling(self,observed_data):
        return np.argmax( np.random.beta(observed_data[:,0], observed_data[:,1]) )


    def UCB(self,observed_data):
        #print observed_data
        t = float(observed_data.sum()) # total number of rounds so far
        #print t
        totals = observed_data.sum(1)
        #print totals
        successes = observed_data[:,0]
        estimated_means = successes/totals # sample mean
        estimated_variances = estimated_means - estimated_means**2
        UCB = estimated_means + np.sqrt( np.minimum( estimated_variances + np.sqrt(2*np.log(t)/totals), 0.25 ) * np.log(t)/totals )
        return np.argmax(UCB)

    def initialize_bandit(self,num_classes):

        prior_a = 1. # aka successes
        prior_b = 1. # aka failures
        self.observed_data = np.zeros((num_classes,2))
        self.observed_data[:,0] += prior_a # allocating the initial conditions
        self.observed_data[:,1] += prior_b
        #regret = np.zeros(num_samples)
        #print observed_data

    def send_task(self,task):

        add_task_srv = rospy.ServiceProxy(self.add_task_srv_name, AddTask)
        task_id = -1
        try:
            # add task to the execution framework
            task_id = add_task_srv(task)

            #print 'hey', task_id
            return task_id.task_id

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return task_id
    '''
    def send_tasks(self, roi_db_name,roi_collection_name,roi_config):
        tasks = create_har_sweep_tasks()
        print len(tasks)

        add_tasks_srv = rospy.ServiceProxy(self.add_tasks_srv_name, AddTasks)
        set_execution_status = rospy.ServiceProxy(set_exe_stat_srv_name, strands_executive_msgs.srv.SetExecutionStatus)
        try:
            # add task to the execution framework
            task_id = add_tasks_srv([tasks[6]])
            # make sure the executive is running -- this only needs to be done once for the whole system not for every task
            set_execution_status(True)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    '''
if __name__ =="__main__":

    rospy.init_node('har_task_manager')
    print 'Running har_task_manager'
    rate = rospy.Rate(10)
    #send_tasks()

    hartask_manager= HARTaskManager()

    while not rospy.is_shutdown():
        if hartask_manager.check_timeslot():
            hartask_manager.previous_task_num = hartask_manager.current_task_num
            hartask_manager.previous_person_locations = hartask_manager.current_person_locations
            hartask_manager.current_task_num = hartask_manager.UCB(hartask_manager.observed_data)
            #hartask_manager.current_task_num = randint(0,len(hartask_manager.goto_tasks)-1)
            rospy.loginfo("Sending task with id %s",hartask_manager.current_task_num)
            hartask_manager.current_task_id = hartask_manager.send_task(hartask_manager.goto_tasks[hartask_manager.current_task_num])
            #hartask_manager.current_wait_task_id = hartask_manager.send_task(hartask_manager.wait_task)
            #tasks.append(hartask_manager.sweep_tasks[key])
            #tasks.append(hartask_manager.wait_task)


        rate.sleep()
