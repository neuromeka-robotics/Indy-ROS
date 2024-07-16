#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_msgs.msg import String

import sys
import termios
import tty
import select
import threading
import time

# from indy_interfaces.srv import IndyService
from indy_define import *
from indy_driver.srv import IndyService

# -------KEYBOARD----------
KEYCODE_RIGHT       = 0x43
KEYCODE_LEFT        = 0x44
KEYCODE_UP          = 0x41
KEYCODE_DOWN        = 0x42
KEYCODE_PERIOD      = 0x2E
KEYCODE_SEMICOLON   = 0x3B
KEYCODE_N           = 0x6E
KEYCODE_M           = 0x6D
KEYCODE_COMMA       = 0x2C
KEYCODE_0           = 0x30
KEYCODE_1           = 0x31
KEYCODE_2           = 0x32
KEYCODE_3           = 0x33
KEYCODE_4           = 0x34
KEYCODE_5           = 0x35
KEYCODE_6           = 0x36
KEYCODE_7           = 0x37
KEYCODE_9           = 0x39
KEYCODE_W           = 0x77
KEYCODE_E           = 0x65
KEYCODE_R           = 0x72
KEYCODE_N           = 0x6E
KEYCODE_M           = 0x6D
KEYCODE_EQUAL       = 0x3D
KEYCODE_PLUS        = 0x2B
KEYCODE_MINUS       = 0x2D
KEYCODE_P           = 0x70
KEYCODE_H           = 0x68
KEYCODE_Z           = 0x7A
KEYCODE_Q           = 0x71
KEYCODE_S           = 0x73

# -----TELE STATUS-----
TELE_STOP   = 0
TELE_TASK   = 1
TELE_JOINT  = 2

# JOINT_STATES_TOPIC  = "/joint_states"
ROS_QUEUE_SIZE      = 10
EEF_FRAME_ID        = "tcp"
BASE_FRAME_ID       = "link0"

# Task key
TASK_KEY = [chr(KEYCODE_RIGHT), chr(KEYCODE_LEFT), chr(KEYCODE_UP), chr(KEYCODE_DOWN), 
            chr(KEYCODE_PERIOD), chr(KEYCODE_SEMICOLON), chr(KEYCODE_N), chr(KEYCODE_M), chr(KEYCODE_COMMA)]
# Joint key
JOINT_KEY = [chr(KEYCODE_1), chr(KEYCODE_2), chr(KEYCODE_3), chr(KEYCODE_4), 
             chr(KEYCODE_5), chr(KEYCODE_6), chr(KEYCODE_7), chr(KEYCODE_R)]

# Mapping from keycodes to joint names
KEYCODE_TO_JOINTS = {
    KEYCODE_1: "joint0",
    KEYCODE_2: "joint1",
    KEYCODE_3: "joint2",
    KEYCODE_4: "joint3",
    KEYCODE_5: "joint4",
    KEYCODE_6: "joint5",
    KEYCODE_7: "joint6",
}

class KeyboardReader:
    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)

    def read_one(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

class KeyboardControl:
    def __init__(self):
        rospy.init_node('servo_keyboard_input')

        # topic pub/sub
        self.frame_to_publish = BASE_FRAME_ID
        self.twist_pub = rospy.Publisher('servo_server/delta_twist_cmds', TwistStamped, queue_size=ROS_QUEUE_SIZE)
        self.joint_pub = rospy.Publisher('servo_server/delta_joint_cmds', JointJog, queue_size=ROS_QUEUE_SIZE)

        # parameters
        self.isSim = rospy.get_param('~is_sim', True)
        # rospy.loginfo(f'isSim: {self.isSim}')
        
        # service client for indy
        if not self.isSim:
            self.indy_srv_cli = rospy.ServiceProxy('indy_srv', IndyService)
            rospy.wait_for_service('indy_srv')

        # default speed of joint, task
        self.joint_angular_vel  = 0.5
        self.task_linear_vel    = 0.5
        self.task_angular_vel   = 0.5

        self.key = 0x00
        self.teleop_status = TELE_STOP

        self.reader = KeyboardReader()

    def indy_service(self, data):
        indy_req = IndyService._request_class()
        indy_req.data = data
        try:
            response = self.indy_srv_cli(indy_req)
            return response
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed %s' % str(e))
            return None

    def isValid(self):
        if self.isSim:
            return True
        elif not self.isSim:
            if (self.key in TASK_KEY or self.key in JOINT_KEY) and self.teleop_status != TELE_JOINT:
                rospy.logwarn('TELE MODE IS ACTIVATING...')
                if self.indy_service(MSG_TELE_JOINT_ABS):
                    self.teleop_status = TELE_JOINT
                    rospy.loginfo('TELE MODE IS ACTIVATED!')
                    return True
                else:
                    return False
            else:
                return True
        else:
            return False

    def key_loop(self):
        threading.Thread(target=lambda: rospy.spin()).start()
        
        print("Reading from keyboard")
        print("---------Common Use-------------")
        print("Use arrow keys and the '.' and ';' keys to Cartesian jog")
        print("Use 'W' to Cartesian jog in the world frame, and 'E' for the End-Effector frame")
        print("Use 'N' 'M' ',' for the Task move UVW")
        print("Use 1|2|3|4|5|6|7 keys to joint jog. 'R' to reverse the direction of jogging.")
        print("Use '-' '+' to adjust joint speed")
        print("Use '9' '0' to adjust task speed")
        print("'Q' to quit.")
        print("---------Only Real Robot----------")
        print("Use 'H' to move Home, 'Z' to move Zero, 'S' to Recover, 'P' to stop Teleop")

        try:
            while not rospy.is_shutdown():
                self.key = self.reader.read_one()

                twist_msg = TwistStamped()
                joint_msg = JointJog()
                publish_twist = False
                publish_joint = False

                if not self.isSim: # if this is real robot
                    if self.key == chr(KEYCODE_P):
                        if self.indy_service(MSG_TELE_STOP):
                            self.teleop_status = TELE_STOP
                            rospy.loginfo('Stop Teleop!')

                    elif self.key == chr(KEYCODE_H):
                        if self.indy_service(MSG_MOVE_HOME):
                            self.teleop_status = TELE_STOP
                            rospy.loginfo('Call Move Home Success')

                    elif self.key == chr(KEYCODE_Z):
                        if self.indy_service(MSG_MOVE_ZERO):
                            self.teleop_status = TELE_STOP
                            rospy.loginfo('Call Move Zero Success')

                    elif self.key == chr(KEYCODE_S):
                        if self.indy_service(MSG_RECOVER):
                            self.teleop_status = TELE_STOP
                            rospy.loginfo('Call Recover Success')

                if self.key == chr(KEYCODE_Q):
                    if not self.isSim:
                        if self.indy_service(MSG_TELE_STOP):
                            rospy.loginfo('Stop Teleop!')
                        else:
                            rospy.logwarn('Cannot Stop Teleop!')
                    self.teleop_status = TELE_STOP
                    rospy.loginfo('Exit Servo Keyboard!')
                    break

                # -------------------------
                elif self.key == chr(KEYCODE_LEFT):
                    if self.isValid():
                        twist_msg.twist.linear.y = self.task_linear_vel
                        publish_twist = True
                        # print("MOVE LEFT")
                elif self.key == chr(KEYCODE_RIGHT):
                    if self.isValid():
                        twist_msg.twist.linear.y = -self.task_linear_vel
                        publish_twist = True
                        # print("MOVE RIGHT")
                elif self.key == chr(KEYCODE_UP):
                    if self.isValid():
                        twist_msg.twist.linear.x = self.task_linear_vel
                        publish_twist = True
                        # print("MOVE FORWARD")
                elif self.key == chr(KEYCODE_DOWN):
                    if self.isValid():
                        twist_msg.twist.linear.x = -self.task_linear_vel
                        publish_twist = True
                        # print("MOVE BACKWARD")
                elif self.key == chr(KEYCODE_PERIOD):
                    if self.isValid():
                        twist_msg.twist.linear.z = -self.task_linear_vel
                        publish_twist = True
                        # print("MOVE DOWN")
                elif self.key == chr(KEYCODE_SEMICOLON):
                    if self.isValid():
                        twist_msg.twist.linear.z = self.task_linear_vel
                        publish_twist = True
                        # print("MOVE UP")

                # -------------------------
                elif self.key == chr(KEYCODE_N):
                    if self.isValid():
                        twist_msg.twist.angular.x = self.task_angular_vel
                        publish_twist = True
                        # print("ANGULAR X")
                elif self.key == chr(KEYCODE_M):
                    if self.isValid():
                        twist_msg.twist.angular.y = self.task_angular_vel
                        publish_twist = True
                        # print("ANGULAR Y")
                elif self.key == chr(KEYCODE_COMMA):
                    if self.isValid():
                        twist_msg.twist.angular.z = self.task_angular_vel
                        publish_twist = True
                        # print("ANGULAR Z")

                # -------------------------
                elif self.key == chr(KEYCODE_E):
                    self.frame_to_publish = EEF_FRAME_ID
                    print("END FRAME")
                elif self.key == chr(KEYCODE_W):
                    self.frame_to_publish = BASE_FRAME_ID
                    print("BASE FRAME")
                
                # -------------------------
                elif self.key == chr(KEYCODE_R):
                    self.joint_angular_vel *= -1
                    self.task_angular_vel *= -1
                    print("Reverse ANGULAR")

                elif self.key in [chr(code) for code in KEYCODE_TO_JOINTS]:
                    if self.isValid():
                        joint_msg.joint_names.append(KEYCODE_TO_JOINTS[ord(self.key)])
                        joint_msg.velocities.append(self.joint_angular_vel)
                        joint_msg.displacements.append(0.0)
                        publish_joint = True
                        # print(f"MOVE {joint_name}")

                # -------------------------
                elif self.key == chr(KEYCODE_EQUAL):
                    self.joint_angular_vel = min(self.joint_angular_vel + 0.05, 2.0)
                    print(f"Joint speed: {self.joint_angular_vel}")
                    if self.joint_angular_vel >= 0.5:
                        print("MAX Speed")
                    
                elif self.key == chr(KEYCODE_MINUS):
                    self.joint_angular_vel = max(self.joint_angular_vel - 0.05, 0.05)
                    print(f"Joint speed: {self.joint_angular_vel}")
                    if self.joint_angular_vel <= 0.05:
                        print("MIN Speed")

                elif self.key == chr(KEYCODE_0):
                    self.task_linear_vel = min(self.task_linear_vel + 0.05, 2.0)
                    print(f"Task speed: {self.task_linear_vel}")
                    if self.task_linear_vel >= 2.0:
                        print("MAX Speed")
                    
                elif self.key == chr(KEYCODE_9):
                    self.task_linear_vel = max(self.task_linear_vel - 0.05, 0.05)
                    print(f"Task speed: {self.task_linear_vel}")
                    if self.task_linear_vel <= 0.05:
                        print("MIN Speed")

                if publish_twist:
                    twist_msg.header.stamp = rospy.Time.now()
                    twist_msg.header.frame_id = self.frame_to_publish
                    self.twist_pub.publish(twist_msg)
                elif publish_joint:
                    joint_msg.header.stamp = rospy.Time.now()
                    joint_msg.header.frame_id = BASE_FRAME_ID
                    self.joint_pub.publish(joint_msg)

            if not self.isSim:
                self.indy_service(MSG_TELE_STOP)
                        
        except Exception as e:
            if not self.isSim:
                self.indy_service(MSG_TELE_STOP)
            print(f"Exception: {e}")

if __name__ == '__main__':
    try:
        control = KeyboardControl()
        control.key_loop()
    except rospy.ROSInterruptException:
        pass
