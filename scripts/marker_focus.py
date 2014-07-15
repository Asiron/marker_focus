#!/usr/bin/env python  
import roslib
import rospy
import tf
import actionlib
from threading import Thread, Lock

import time

from operator import sub
from math import *
from nao_msgs.msg import (JointAnglesWithSpeed,
                          JointAnglesWithSpeedAction,
                          JointAnglesWithSpeedGoal)
from sensor_msgs.msg import JointState

transform_lock = Lock()
latest_transform = None
transform_changed = False

transformations = [("CameraTop_frame0","4x4_1"),("CameraTop_frame1", "4x4_2")]

class HeadStateSubscriber(Thread):

    head_state_lock = None

    head_state_sub = None
    current_head_yaw   = None
    current_head_pitch = None

    def __init__(self):
        Thread.__init__(self)
        self.head_state_lock = Lock()

    def update_head_state(self, data):
        #print data.name[0], data.position[0]
        #print data.name[1], data.position[1]

        self.head_state_lock.acquire(True)

        self.current_head_yaw   = data.position[0]
        self.current_head_pitch = data.position[1]

        self.head_state_lock.release()

    def get_current_head_positions(self):

        self.head_state_lock.acquire(True)

        position = (self.current_head_yaw, 
                    self.current_head_pitch)

        self.head_state_lock.release()

        return position

    def run(self):
        self.head_state_sub = rospy.Subscriber("joint_states", JointState, self.update_head_state)
        rospy.spin()

class HeadMoverPublisher(Thread):

    speed = 0.05

    movement_coeff = 0.5

    horizontal_angle_treshold = 5
    vertical_angle_treshold   = 5

    head_mover_pub = None
    head_mover_client = None

    head_state_manager = None

    def __init__(self, head_state_manager):
        Thread.__init__(self)
        self.head_state_manager = head_state_manager
        self.head_mover_pub = rospy.Publisher('joint_angles', JointAnglesWithSpeed, queue_size = 10)
        self.head_mover_client = actionlib.SimpleActionClient("joint_angles_action", JointAnglesWithSpeedAction)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
      
            transform_lock.acquire(True)
            
            angles = None

            global latest_transform, transform_changed
            if latest_transform != None and transform_changed == True:
                transform_changed = False
                angles = self.get_angles(latest_transform)

            transform_lock.release()

            if angles != None:
                self.move_head(angles)

            r.sleep()

    def move_head(self,relative_angles):
        if (abs(relative_angles["angle_left"]) < self.horizontal_angle_treshold) and \
           (abs(relative_angles["angle_up"])   < self.vertical_angle_treshold):
            return

        print relative_angles
        '''
        message = JointAnglesWithSpeed()
        message.relative     = 1
        message.joint_names  = ["HeadYaw", "HeadPitch"]
        message.joint_angles = [-radians(relative_angles["angle_left"])*self.movement_coeff, 
                                 radians(relative_angles["angle_up"])  *self.movement_coeff]
        message.speed = self.speed
        self.head_mover_pub.publish(message)
        '''

        goal = JointAnglesWithSpeedGoal()
        goal.joint_angles.relative = 1
        goal.joint_angles.joint_names = ["HeadYaw", "HeadPitch"]
        goal.joint_angles.joint_angles = [-radians(relative_angles["angle_left"])*self.movement_coeff, 
                                 radians(relative_angles["angle_up"])  *self.movement_coeff]
        goal.joint_angles.speed = self.speed
        self.head_mover_client.send_goal(goal)

    def get_angles(self, trans):
        return {"angle_up":   degrees(atan(trans[1]/trans[2])),
                "angle_left": degrees(atan(trans[0]/trans[2]))}

class MarkerTransformListener():

    listener = None

    def __init__(self):
        self.listener = tf.TransformListener()

    def did_transform_change(self, old_transform, new_transform):
        if old_transform == None and new_transform != None:
            return True
        else:
            return True
            #print map(abs, map(sub, old_transform, new_transform))
            #return all(map(float.__ge__, 
            #    map(abs, map(sub, old_transform, new_transform)),
            #    (0.01, 0.01, 0.01)))

    def run(self):
        rate = rospy.Rate(10.0)
        #self.listener.waitForTransform("/CameraTop_frame0", "4x4_1", rospy.Time(), rospy.Duration(4.0))
        while not rospy.is_shutdown():
            '''
            try:
                now = rospy.Time().now()
                self.listener.waitForTransform("/CameraTop_frame1", "4x4_2", now, rospy.Duration(4.0))
                (new_transform, _) = self.listener.lookupTransform('/CameraTop_frame1', '/4x4_2', now)
                
                transform_lock.acquire(True)
                global latest_transform, transform_changed

                if self.did_transform_change(latest_transform, new_transform):
                    transform_changed = True
                    latest_transform  = new_transform
                
                transform_lock.release()

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception):
                continue
            '''

if __name__ == '__main__':
    
    rospy.init_node('marker_focus')
    
    head_state_sub  = HeadStateSubscriber()
    head_controller = HeadMoverPublisher(head_state_sub)

    head_state_sub.daemon = True
    head_state_sub.start()

    head_controller.daemon = True
    head_controller.start()


    marker_listener = MarkerTransformListener()
    marker_listener.run()
