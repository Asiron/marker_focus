#!/usr/bin/env python  
import roslib
roslib.load_manifest('ar_pose')

from collections import defaultdict, namedtuple

import rospy, tf, actionlib, time, message_filters, math

from threading import Thread, Lock

from operator import sub
from math import *

from std_msgs.msg import String
from ar_pose.msg import ARMarkers
from nao_msgs.msg import (FadeRGB,
                          JointAnglesWithSpeed,
                          JointAnglesWithSpeedAction,
                          JointAnglesWithSpeedGoal)
from sensor_msgs.msg import JointState

transform_lock = Lock()

Transformation = namedtuple('Transformation', ['transform', 'timestamp'])

latest_transforms  = {}
transforms_changed = False

loaded_markers = ["4x4_8", "4x4_11", "4x4_28", "4x4_48", "4x4_89"]

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

    horizontal_angle_treshold = radians(5)
    vertical_angle_treshold   = radians(5)

    head_mover_client = None

    head_state_manager = None

    def __init__(self, head_state_manager):
        Thread.__init__(self)
        self.head_state_manager = head_state_manager
        self.head_mover_client = actionlib.SimpleActionClient("joint_angles_action", JointAnglesWithSpeedAction)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            
            transform_lock.acquire(True)
            
            angles = None

            global latest_transforms, transforms_changed
            if latest_transforms != None and len(latest_transforms) > 0 and transforms_changed == True:
                transforms_changed = False
                transform = self.get_best_transform(latest_transforms)
                angles    = self.get_angles_desc(transform)
            transform_lock.release()

            if angles != None:
                pass
                self.move_head(angles)

            r.sleep()

    def move_head(self,relative_angles):
        if (abs(relative_angles["angle_left"]) < self.horizontal_angle_treshold) and \
           (abs(relative_angles["angle_up"])   < self.vertical_angle_treshold):
            return

        #print relative_angles

        goal = JointAnglesWithSpeedGoal()
        goal.joint_angles.relative = 1
        goal.joint_angles.joint_names = ["HeadYaw", "HeadPitch"]
        goal.joint_angles.joint_angles = [ relative_angles["angle_left"]*self.movement_coeff, 
                                           relative_angles["angle_up"]  *self.movement_coeff]
        goal.joint_angles.speed = self.speed
        self.head_mover_client.send_goal(goal)
    
    
    def get_angles_desc(self, trans):
        return {"angle_up":   atan(trans[1]/trans[2]),
                "angle_left": -atan(trans[0]/trans[2])}

    def geometric_average(self, angles, head_positions):
        return math.sqrt((angles["angle_left"]+head_positions[0])**2+
                         (angles["angle_up"]+head_positions[1])**2)

    def get_best_transform(self, transformations):
        head_positions = self.head_state_manager.get_current_head_positions()

        best = min(transformations.items(), key=lambda x: self.geometric_average(self.get_angles_desc(x[1].transform),head_positions))
        if len(transformations) > 1:
            print "Best is ", best[0]
            print head_positions
            for k,v in transformations.items():
                print k, round(self.get_angles_desc(v.transform)["angle_left"], 2), round(self.get_angles_desc(v.transform)["angle_up"],2), self.geometric_average(self.get_angles_desc(v.transform), head_positions)
        return best[1].transform

class MarkerTransformListener():

    speech_pub = None

    def __init__(self):
        self.speech_pub = rospy.Publisher("speech", String)

    def did_transform_change(self, old_transform, new_transform):
        if old_transform == None and new_transform != None:
            return True
        elif all(map(float.__ge__, 
                map(abs, map(sub, old_transform, new_transform)),
                (0.01, 0.01, 0.01))):
            return True
        else:
            return False

    def marker_lost(self, marker):
        message = ""
        if marker == "4x4_1":
            message = "First"
        elif marker == "4x4_2":
            message = "Second"
        else:
            message = "Unknown"

        message += "marker lost"

        self.speech_pub.publish(message)

    def check_timeouts(self):
        transform_lock.acquire(True)
        
        global latest_transforms
        for k, v in latest_transforms.items():
            if (rospy.Time.now() - v.timestamp).to_sec() > 3:
                print "Lost marker !!! ", k
                #self.marker_lost(k)
                del latest_transforms[k]
                
        transform_lock.release()

    def marker_callback(self, data):

        self.check_timeouts()

        if len(data.markers) == 0:
            return

        for marker in data.markers:

            if not marker.header.frame_id in loaded_markers:
                continue

            marker_name = marker.header.frame_id

            new_transform = marker.pose.pose.position
            new_transform = (new_transform.x, new_transform.y, new_transform.z)

            transform_lock.acquire(True)
            global latest_transforms, transforms_changed

            now = rospy.Time.now()

            if latest_transforms.get(marker_name,None) == None:
                print "Marker found ", marker.header.frame_id 
                latest_transforms[marker_name] = Transformation(transform=new_transform, timestamp=now)
            elif self.did_transform_change(latest_transforms[marker_name].transform, new_transform):
                transforms_changed = True
                latest_transforms[marker_name] = Transformation(transform=new_transform, timestamp=now)
            transform_lock.release()

    def run(self):

        rospy.Subscriber("markers_filtered/map_to_footprint", ARMarkers, self.marker_callback)

        while not rospy.is_shutdown():
            pass

class MarkerSearcher(Thread):

    eye_led_pub = None
    head_mover_pub = None
    head_state_manager = None
    stiffness_publisher = None

    speed = 0.05
    direction = 1
    horizontal_limit = 1.0
    head_is_moving = False

    # if the head moves less than horizontal limit,
    # we have to move further than the limit
    error_accept_ratio = 1.04

    def __init__(self, head_state_manager):        
        Thread.__init__(self)
        self.head_state_manager = head_state_manager
        self.eye_led_pub = rospy.Publisher("/fade_rgb", FadeRGB, queue_size = 10)
        self.head_mover_pub = rospy.Publisher("/joint_angles", JointAnglesWithSpeed, queue_size = None)
        self.stiffness_publisher = rospy.Publisher("/joint_stiffness", JointState, queue_size = None)

        #sleep so that subscribers are aware of the new publisher
        rospy.sleep(1)

        self.set_stiffness()

    def run(self):

        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            marker_observed = self.marker_found()
            if not self.head_is_moving and not marker_observed:
                self.start_moving()
            elif self.head_is_moving and not marker_observed:
                self.move_head()
            elif self.head_is_moving and marker_observed:
                self.stop_head()
            r.sleep()

    def marker_found(self):
        transform_lock.acquire(True)
        
        global latest_transforms
        result = False if len(latest_transforms) == 0 else True
        
        transform_lock.release()

        if result:
            self.light_eye_led([1,0,0])
        else:
            self.light_eye_led([0,0,0])

        return result

    def light_eye_led(self, color):
        msg = FadeRGB()
        msg.led_name = "FaceLeds"
        msg.color.r = color[0]
        msg.color.g = color[1]
        msg.color.b = color[2]
        msg.color.a = 0
        msg.fade_duration = rospy.Duration(0)
        self.eye_led_pub.publish(msg)

    def set_stiffness(self):
        stiffness_message = JointState()
        stiffness_message.name = ["HeadYaw", "HeadPitch"]
        stiffness_message.position = [0,0]
        stiffness_message.velocity = [0,0]
        stiffness_message.effort   = [1,1]
        self.stiffness_publisher.publish(stiffness_message)

    def start_moving(self):
        print "Start moving"
        self.head_is_moving = True
        self.send_command(0,self.get_next_angle(),0.0)

    def move_head(self):
        head_yaw = self.head_state_manager.get_current_head_positions()[0]
        if head_yaw == None:
            return
        if (((head_yaw > self.horizontal_limit) and (self.direction == 1)) or
           ((head_yaw < -self.horizontal_limit) and (self.direction == -1))):
            self.change_direction()

    def change_direction(self):
        self.direction *= -1
        self.send_command(0, self.get_next_angle(), 0)

    def stop_head(self):
        print "Stop moving"
        self.head_is_moving = False
        self.send_command(1,0.0,0.0)

    def send_command(self, relative, head_yaw, head_pitch):
        message = JointAnglesWithSpeed()
        message.relative = relative
        message.joint_names  = ["HeadYaw", "HeadPitch"]
        message.joint_angles = [head_yaw,   head_pitch]
        message.speed = self.speed
        self.head_mover_pub.publish(message)

    def get_next_angle(self):
        return self.direction*self.horizontal_limit*self.error_accept_ratio

if __name__ == '__main__':
    
    rospy.init_node('marker_focus')

    head_state_sub  = HeadStateSubscriber()
    head_controller = HeadMoverPublisher(head_state_sub)
    marker_searcher = MarkerSearcher(head_state_sub)

    marker_searcher.daemon = True
    marker_searcher.start()

    head_state_sub.daemon = True
    head_state_sub.start()

    head_controller.daemon = True
    head_controller.start()

    marker_listener = MarkerTransformListener()
    marker_listener.run()
