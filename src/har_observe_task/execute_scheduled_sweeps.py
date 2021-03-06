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


class HARTaskManager():


    def __init__(self):

        self.minutes=[]
        self.images = []
        self.current_task_id = -1
        self.current_wait_task_id = -1
        self.previoustasktimeslot = -1
        self.should_update_model = 0
        self.add_task_srv_name = '/task_executor/add_task'
        self.set_exe_stat_srv_name = '/task_executor/set_execution_status'
        self.finished_sub = rospy.Subscriber('/local_metric_map/room_observations', RoomObservation, callback=self.finishedCB)
        sub = rospy.Subscriber("task_executor/events",TaskEvent,self.taskexecutorCB)
        self.observation_sub = rospy.Subscriber('/local_metric_map/rgb/rgb_filtered', Image, callback=self.observationCB)


        # wait at waypoint 22 middle of the corridor
        self.wait_task = create_go_to_waypoint_task("WayPoint22")
        #self.deep_object_detection_srv_name = 'deep_net/detect_objects'
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

        # make sure the executive is running -- this only needs to be done once for the whole system not for every task
        #set_execution_status = rospy.ServiceProxy(self.set_exe_stat_srv_name, strands_executive_msgs.srv.SetExecutionStatus)
        #set_execution_status(True)
        self.sweep_tasks = create_har_sweep_tasks("journalExpKTH","roi","configHARRooms")
        self.create_timeslot_array()
        #print self.sweep_tasks.keys()
     def observationCB(self, msg):

        #self.sweep_detections.append(self.latest_detections)
        self.images.append(msg)

    def log_data(self):

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

        try:
            server = rospy.ServiceProxy('/deep_object_detection/detect_objects', DetectObject)
            resp = server(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return

        detections = [[] for im in self.images]
        for obj in resp.objects:
            if obj.label != "person":
                continue

            #sys.exit(-1)
    def taskexecutorCB(self,taskevent):
        #task is not succeeded
        print taskevent.event
        print taskevent.task.task_id
        print self.current_task_id
        if taskevent.event > 11 and taskevent.event is not 16:
            if taskevent.task.task_id is self.current_task_id:
                self.should_update_model = 0
                self.current_wait_task_id=self.send_task(self.wait_task)
           # elif taskevent.task.task_id is self.current_wait_task_id:
           #     self.current_wait_task_id = self.send_task(self.wait_task)
        if taskevent.event is 16 and taskevent.task.task_id is self.current_task_id:
            self.current_wait_task_id=self.send_task(self.wait_task)



    def create_timeslot_array(self):
        self.minutes = [-1]*1440
        count = 0
        arange = np.arange(1,10.1,0.25)
        #print arange
        for i in arange:
            for j in range(-5,5):
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
            self.previoustasktimeslot = self.minutes[timeminutes]
            return True
        return False


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
            tasks = []
            taskid = randint(0,len(hartask_manager.sweep_tasks)-1)
            keys = hartask_manager.sweep_tasks.keys()
            key = keys[taskid]
            rospy.loginfo("Sending task with id %s",key)
            hartask_manager.current_task_id = hartask_manager.send_task(hartask_manager.sweep_tasks[key])
            #hartask_manager.current_wait_task_id = hartask_manager.send_task(hartask_manager.wait_task)
            #tasks.append(hartask_manager.sweep_tasks[key])
            #tasks.append(hartask_manager.wait_task)


        rate.sleep()
