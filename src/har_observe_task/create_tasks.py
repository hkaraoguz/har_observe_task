#!/usr/bin/python
from strands_executive_msgs import task_utils
from strands_executive_msgs.msg import Task
from find_waypoints import *
import numpy as np
from scipy.spatial import ConvexHull
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from soma_msgs.msg import SOMAROIObject

def create_har_observation_tasks(duration=rospy.Duration(30*60)):

    topological_nodes = getNodes()
    tables = getTables()

    tablenodepairs = []

    for table in tables:
         table_position = [table.pose.pose.position.x, table.pose.pose.position.y]
         distances = []
         for topological_node in topological_nodes:
             node_position = [topological_node.pose.position.x, topological_node.pose.position.y]
             distances.append(distance(table_position,node_position))
         minindex =  distances.index(min(distances))
         tablenodepairs.append((table_position, topological_nodes[minindex]))

    tasks = []

    for pair in tablenodepairs:
        task = Task(start_node_id=pair[1].name, end_node_id=pair[1].name, action='observe_har', max_duration=duration)
        task_utils.add_float_argument(task, pair[0][0])
        task_utils.add_float_argument(task, pair[0][1])
        task_utils.add_float_argument(task, 1.0)
        tasks.append(task)

    return tasks

def create_harroom_observation_task(waypoint,runcount,placename,duration=rospy.Duration(2*60)):

    task = Task(start_node_id=waypoint, end_node_id=waypoint, action='observe_harroom', max_duration=duration)
    task_utils.add_int_argument(task, runcount)
    task_utils.add_string_argument(task,placename)

    return task


def create_wait_action_task(waypoint):
    task = Task(action='wait_action', max_duration=rospy.Duration(1*60), start_node_id=waypoint)
    return task

def create_go_to_waypoint_task(waypoint):
     task = Task(start_node_id=waypoint)
     task.max_duration.secs = int(2 * 60)
     return task

def create_har_sweep_tasks(roi_db_name,roi_collection_name,roi_config):
    topological_nodes = getNodes()
    rois = getRegions(roi_db_name,roi_collection_name,roi_config)
    roinodepairs = []
    for roi in rois:
        vertices = []
        for apose in roi.posearray.poses:
            vertices.append([apose.position.x,apose.position.y])
        hull = ConvexHull(vertices)
        cx = np.mean(hull.points[hull.vertices,0])
        cy = np.mean(hull.points[hull.vertices,1])
        roi_position = [cx, cy]
        distances = []
        for topological_node in topological_nodes:
            node_position = [topological_node.pose.position.x, topological_node.pose.position.y]
            distances.append(distance(roi_position,node_position))
        minindex =  distances.index(min(distances))
        roinodepairs.append((roi.id, topological_nodes[minindex]))
    #print roinodepairs
    tasks = dict()
    for pair in roinodepairs:
        task = Task(start_node_id=pair[1].name, action='do_sweep', max_duration=rospy.Duration(60 * 5))
        #task.start_after = rospy.Time.now()
        #task.end_before = task.start_after + rospy.Duration(task.start_after * 3)
        task_utils.add_string_argument(task, 'medium')
        tasks[pair[0]] = task
    print tasks.keys()
    return tasks
