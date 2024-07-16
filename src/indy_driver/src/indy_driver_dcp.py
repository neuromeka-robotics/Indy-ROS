#!/usr/bin/python3
#-*- coding: utf-8 -*-

import sys
import json
import time

import math
import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryFeedback

from indy_define import *
from indy_driver.srv import IndyService, IndyServiceResponse
from neuromeka import IndyDCP3

def rads2degs(rad_list):
    degs = [math.degrees(rad) for rad in rad_list]
    return degs

def degs2rads(deg_list):
    rads = [math.radians(deg) for deg in deg_list]
    return rads

class IndyROSConnector:
    def __init__(self, indy_ip, indy_type):
        
        # Connect to Robot
        self.indy_ip = indy_ip
        self.indy_type = indy_type

        # Initialize ROS node
        rospy.init_node('indy_driver_dcp')
        self.rate = rospy.Rate(20) # hz

        # Publish current robot state
        self.joint_state_pub = rospy.Publisher("joint_states", JointState, queue_size=10)
        self.indy_state_pub = rospy.Publisher("/indy/status", GoalStatusArray, queue_size=10)
        self.control_state_pub = rospy.Publisher("/feedback_states", FollowJointTrajectoryFeedback, queue_size=1)

        # Subscribe desired joint position
        self.joint_execute_plan_sub = rospy.Subscriber("/joint_path_command", JointTrajectory, self.execute_callback, queue_size=1)
        self.spacenav_plan_sub = rospy.Subscriber("/joint_group_position_controller/command", Float64MultiArray, self.jointCallback, queue_size=1)

        # Sub/Pub command
        self.indy_srv = rospy.Service('indy_srv', IndyService, self.indy_srv_callback)

        # Misc variable
        self.joint_state_list = []
        self.indy_msg_status = MSG_TELE_STOP
        self.previous_joint_trajectory_sub = []

    def __enter__(self):
        self.connect()
        return self
    
    def __del__(self):
        self.disconnect()
        
    # Connect to Indy
    def connect(self):
        print("ROBOT IP: ", self.indy_ip)
        print("ROBOT TYPE: ", self.indy_type)
        self.indy = IndyDCP3(self.indy_ip)

    # Disconnect to Indy
    def disconnect(self):
        print("DISCONNECT TO ROBOT")
        self.indy.stop_teleop()
        time.sleep(1)
        self.indy.stop_motion()
        time.sleep(1)
        del self.indy

    def execute_callback(self, msg):
        # check if robot is in ILDE mode
        if self.indy.get_control_data()['op_state'] != OP_IDLE:
            print("ROBOT IS NOT READY")
            return
        self.joint_state_list = []
        if msg.points:
            self.joint_state_list = [p.positions for p in msg.points]
            
        if self.joint_state_list:
            # start teleop
            self.indy.stop_teleop()
            time.sleep(0.1)
            self.indy.start_teleop(method=TELE_JOINT_ABSOLUTE) 
            time.sleep(0.2)

            # wait for telemode actually start
            cur_time = time.time()
            while self.indy.get_control_data()['op_state'] != TELE_OP:
                if (time.time() - cur_time) > 0.5:
                    self.indy.start_teleop(method=TELE_JOINT_ABSOLUTE)  
                    cur_time = time.time()
                time.sleep(0.2)
                
            # send waypoints
            for j_pos in self.joint_state_list:
                try:
                    self.indy.movetelej_abs(jpos=rads2degs(j_pos))
                except Exception as e:
                    print('THERE ARE ISSUE WHEN EXECUTE WAYPOINT, PLEASE TRY AGAIN!')
                    break
                time.sleep(0.05) #20Hz

            time.sleep(0.5) # wait for robot stable

            self.indy.stop_teleop()
            time.sleep(0.3)
            while self.indy.get_control_data()['op_state'] != OP_IDLE:
                time.sleep(0.2)

    def indy_srv_callback(self, request):    
        response = IndyServiceResponse()                    
        print('Incoming request | MODE: %d' % (request.data))
        self.indy.stop_motion()
        
        if request.data == MSG_RECOVER:
            self.indy.stop_teleop()
            time.sleep(0.3)
            while self.indy.get_control_data()['op_state'] != OP_IDLE:
                time.sleep(0.1)
            self.indy.recover()
            self.indy_msg_status = request.data

        elif request.data == MSG_MOVE_HOME:
            self.indy.stop_teleop()
            time.sleep(0.3)
            while self.indy.get_control_data()['op_state'] != OP_IDLE:
                time.sleep(0.1)
            self.indy.movej(jtarget = self.indy.get_home_pos()['jpos'])
            time.sleep(0.2)
            self.indy_msg_status = request.data

        elif request.data == MSG_MOVE_ZERO:
            self.indy.stop_teleop()
            time.sleep(0.3)
            while self.indy.get_control_data()['op_state'] != OP_IDLE:
                time.sleep(0.1)
            self.indy.movej(jtarget = [0,0,0,0,0,0])
            time.sleep(0.2)
            self.indy_msg_status = request.data

        elif request.data == MSG_TELE_STOP:
            self.indy.stop_teleop()
            time.sleep(0.3)
            while self.indy.get_control_data()['op_state'] != OP_IDLE:
                time.sleep(0.1)
            self.indy_msg_status = request.data
                
        elif request.data in [MSG_TELE_TASK_ABS, MSG_TELE_TASK_RLT, MSG_TELE_JOINT_ABS, MSG_TELE_JOINT_RLT]:
            method = TELE_TASK_RELATIVE # default is task
            if request.data == MSG_TELE_TASK_ABS: # Joint
                method = TELE_TASK_ABSOLUTE
            elif request.data == MSG_TELE_JOINT_ABS:
                method = TELE_JOINT_ABSOLUTE
            elif request.data == MSG_TELE_JOINT_RLT:
                method = TELE_JOINT_RELATIVE

            # start teleop
            self.indy.stop_teleop()
            time.sleep(0.1)
            self.indy.start_teleop(method=method) 
            time.sleep(0.2)

            # wait for telemode actually start
            cur_time = time.time()
            timeout = time.time()
            while self.indy.get_control_data()['op_state'] != TELE_OP:
                if (time.time() - cur_time) > 0.5:
                    self.indy.start_teleop(method=method) 
                    cur_time = time.time()
                if (time.time() - timeout) > 3:
                    response.success = False
                    response.message = "TIMEOUT WHEN TRYING TO START TELEOP!!!"
                    print("TIMEOUT WHEN TRYING TO START TELEOP!!!")
                    return response
                time.sleep(0.2)
            self.indy_msg_status = request.data

        response.success = True
        return response

    def jointCallback(self, msg):
        if msg.data and self.previous_joint_trajectory_sub != msg.data:
            # if TELE MODE
            if self.indy_msg_status == MSG_TELE_JOINT_ABS:
                self.indy.movetelej_abs(jpos=rads2degs(msg.data))
            self.previous_joint_trajectory_sub = msg.data
        else:
            self.indy.stop_motion()

    def joint_state_publisher(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()

        if self.indy_type == 'indyrp2' or self.indy_type == 'indyrp2_v2':
            joint_state_msg.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        else:
            joint_state_msg.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']

        control_data = self.indy.get_control_data()
        joint_state_msg.position = degs2rads(control_data['q'])
        joint_state_msg.velocity = degs2rads(control_data['qdot'])
        # joint_state_msg.effort = get_control_torque() TODO
        
        self.joint_state_pub.publish(joint_state_msg)

        control_state_msg = FollowJointTrajectoryFeedback()
        control_state_msg.header.stamp = rospy.Time.now()
        
        if self.indy_type == 'indyrp2' or self.indy_type == 'indyrp2_v2':
            control_state_msg.joint_names = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        else:
            control_state_msg.joint_names = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']

        control_state_msg.actual.positions = degs2rads(control_data['q'])
        control_state_msg.desired.positions = degs2rads(control_data['qdot'])
        control_state_msg.error.positions = [0 for i in control_state_msg.joint_names]

        self.control_state_pub.publish(control_state_msg)

    def run(self):
        self.connect()
        while not rospy.is_shutdown():
            self.joint_state_publisher()

def main():
    indy_ip = rospy.get_param("indy_ip", "192.168.1.29")
    indy_type = rospy.get_param("indy_type", "indy7")
    roscon = IndyROSConnector(indy_ip, indy_type)
    roscon.run()
                
if __name__ == '__main__':
    main()
