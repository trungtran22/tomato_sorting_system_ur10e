#!/usr/bin/env python3
import sys
import copy
import rospy
from math import pi
from std_msgs.msg import String,Int32
from moveit_commander.conversions import pose_to_list
from move_group_joint_state import MoveGroupPythonJointState
from geometry_msgs.msg
 import Pose
from onrobot import RG
import pyfirmata
import time
import asyncio
import rtde_control
import rtde_receive
          
class DataHolder:
    def __init__(self):
            self.pose = Pose()

            self.label = String()
            self.radius = Int32()
            self.sub = [rospy.Subscriber("/tomato_position", Pose, self.poseCallback, queue_size=10)]
            self.sub_label = [rospy.Subscriber("/tomato_label", String, self.labelCallback, queue_size=10)]
            self.sub_radius = [rospy.Subscriber("/tomato_rad", Int32, self.radiusCallback, queue_size=10)]
            # rospy.spin()         
    def poseCallback(self, msg):
        self.pose = msg
    def labelCallback(self, msg):
        self.label = msg
    def radiusCallback(self, msg):
        self.radius = msg

def picking(x_,y_):
    x = (y_/1000)*(-1.230) - 0.31
    y = (x_/1000) + 0.68
    z = 0.27
    #  width = r_*4
    #  r = r_*4 - 40 
    # arm.go_to_initial_state()
    rtde_c.moveL([-0.176, 0.666, 0.86, 0, 0, 1.572], 2.0, 1.5)
    rg2.open_gripper()
    rtde_c.moveL([x, y, 0.69, 0, 0, 1.572], 2.0, 1.5)
    rtde_c.moveL([x, y, 0.53, 0, 0, 1.572], 2.0, 1.5)
    rg2.close_gripper(90)
    time.sleep(3)
    # time.sleep(4)
    rtde_c.moveL([x, y, 0.69, 0, 0, 1.572], 2.0, 1.5)

def put_red_small():
    # arm.go_to_initial_state()
    rtde_c.moveL([-0.083, 0.923, 0.69, 0, 0, 1.572], 2.0, 1.5)
    rtde_c.moveL([-0.083,0.923, 0.53, 0, 0, 1.572], 2.0, 1.5)
    rg2.open_gripper()
    time.sleep(1)
    rtde_c.moveL([-0.083, 0.923, 0.69, 0, 0, 1.572], 2.0, 1.5)
    rtde_c.moveL([-0.176, 0.666, 0.86, 0, 0, 1.572], 2.0, 1.5)
    
def put_green():
    # arm.go_to_initial_state()
    rtde_c.moveL([-0.677, 0.008, 0.69, 0, 0, 1.572], 2.0, 1.5)
    rtde_c.moveL([-0.677, 0.008, 0.35, 0, 0, 1.572], 2.0, 1.5)   
    rg2.open_gripper()
    time.sleep(1)
    rtde_c.moveL([-0.677, 0.008, 0.69, 0, 0, 1.572], 2.0, 1.5)
    rtde_c.moveL([-0.176, 0.666, 0.86, 0, 0, 1.572], 2.0, 1.5)
    
def put_turning():
    # arm.go_to_initial_state()
    rtde_c.moveL([-0.778, -0.214, 0.69, 0, 0, 1.572], 2.0, 1.5)
    rtde_c.moveL([-0.778, -0.214, 0.35, 0, 0, 1.572], 2.0, 1.5)
    rg2.open_gripper()
    time.sleep(1)
    rtde_c.moveL([-0.778, -0.214, 0.69, 0, 0, 1.572], 2.0, 1.5)
    rtde_c.moveL([-0.176, 0.666, 0.86, 0, 0, 1.572], 2.0, 1.5)
    
def put_pink():
    # arm.go_to_initial_state()
    rtde_c.moveL([-0.518, -0.221, 0.69, 0, 0, 1.572], 2.0, 1.5)
    rtde_c.moveL([-0.518, -0.221, 0.35, 0, 0, 1.572], 2.0, 1.5)
    rg2.open_gripper()
    time.sleep(1)
    rtde_c.moveL([-0.518, -0.221, 0.69, 0, 0, 1.572], 2.0, 1.5)
    rtde_c.moveL([-0.176, 0.666, 0.86, 0, 0, 1.572], 2.0, 1.5)
    
def put_red_big():
    # arm.go_to_initial_state()
    rtde_c.moveL([0.252,0.923, 0.69, 0, 0, 1.572], 2.0, 1.5)
    rtde_c.moveL([0.252,0.923, 0.53, 0, 0, 1.572], 2.0, 1.5)
    rg2.open_gripper()
    time.sleep(1)
    rtde_c.moveL([0.252,0.923, 0.69, 0, 0, 1.572], 2.0, 1.5)
    rtde_c.moveL([-0.176, 0.666, 0.86, 0, 0, 1.572], 2.0, 1.5)
    
if __name__=="__main__":
    rospy.init_node("ur10e",anonymous=True)
    try:
        rtde_c = rtde_control.RTDEControlInterface("192.168.1.201")
        print("RTDE - UR10E Successfully Connected!")
    except:
        rtde_c.reconnect()
        print("Reconnecting...")
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.201")
    time_stamp = rtde_r.getTimestamp()
    # arm = MoveGroupPythonJointState()
    rtde_c.moveL([-0.176, 0.666, 0.86, 0, 0, 1.572], 2.0, 1.5)
    try:
        rg2 = RG('rg2','192.168.1.1',502)
        rg2.close_gripper()
        rg2.open_gripper()
    except:
        print("Lost connection!Retrying...")
        rg2.open_connection()
        rg2.close_gripper()
        rg2.open_gripper()
    board = pyfirmata.Arduino('/dev/ttyUSB0')
    it = pyfirmata.util.Iterator(board)
    it.start()
    rad = []
    relay = board.get_pin('d:8:o')
    sensor = board.get_pin('d:3:i127.0.0.1')
    # rospy.init_node('robot_arm', anonymous=True)
    data = DataHolder()
    rate = rospy.Rate(10)
    radi = 0
    count_red = 0
    count_green = 0
    count_turning = 0
    count_pink = 0
    pub_count_red = rospy.Publisher("/count_red",Int32,queue_size=10)
    while not rospy.is_shutdown():
        relay.write(0)
        ir = sensor.read()
        if ir is not True:
            relay.write(1)
            time.sleep(1)
            count = 0
            while True:
                y = data.pose.position.y
                if data.pose.position.y < y+2 and data.pose.position.y > y-2:
                    rad.append(data.radius.data)
                    count += 1
                    if count == 50:
                        try:
                            rad.remove(0)
                            break
                        except:
                            break
            radi = sum(rad)/len(rad)
            # print("radius: ", radi)
            if data.label.data == "redripe":
                count_red +=1
                pub_count_red.publish(count_red)
                print("Number of red: ", count_red)
                print("Tomato width:", radi)
                picking(data.pose.position.x, data.pose.position.y)
                if radi > 0:
                   rad = []
                   put_red_big()
                else:
                   rad = []
                   put_red_small()
            elif data.label.data == "green":
                count_green +=1
                print("Number of green: ", count_green)
                picking(data.pose.position.x, data.pose.position.y)
                put_green()
            elif data.label.data == "turning":
                count_turning +=1
                print("Number of turning: ", count_turning)
                picking(data.pose.position.x, data.pose.position.y)
                put_turning()
            elif data.label.data == "pink":
                count_pink +=1
                print("Number of pink: ", count_pink)
                picking(data.pose.position.x, data.pose.position.y)
                put_pink()
                
