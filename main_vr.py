#!/usr/bin/env python3
import sys
import copy
import rospy
from math import pi
from std_msgs.msg import String,Int32, Float32
from moveit_commander.conversions import pose_to_list
from move_group_joint_state import MoveGroupPythonJointState
from geometry_msgs.msg import Pose,PoseStamped
from onrobot import RG
import pyfirmata
import time
import asyncio
import moveit_commander
from math import pi, tau
          
class DataHolder:
    def __init__(self):
            self.pose = Pose()
            self.label = String()
            self.radius = Int32()

            self.sub = [rospy.Subscriber("/tomato_position", Pose, self.poseCallback, queue_size=10)]
            self.sub_label = [rospy.Subscriber("/tomato_label", String, self.labelCallback, queue_size=10)]
            self.sub_radius = [rospy.Subscriber("/tomato_rad", Int32, self.radiusCallback, queue_size=10)]
            
            self.pose_vr = PoseStamped()
            self.trigger = Float32()
            self.menu = Int32()
            self.grip = Int32()

            self.sub_pose = rospy.Subscriber("/vive/controller/right/pose", PoseStamped, self.pose_vrCallback)
            self.sub_trigger = rospy.Subscriber("/vive/controller/right/trigger", Float32, self.triggerCallback)
            self.sub_menu = rospy.Subscriber("/vive/controller/right/buttons/menu", Int32, self.menuCallback)
            self.sub_grip = rospy.Subscriber("/vive/controller/right/buttons/grip", Int32, self.gripCallback)

    #camera publisher
    def poseCallback(self, msg):
            self.pose = msg
    def labelCallback(self, msg):
        self.label = msg
    def radiusCallback(self, msg):
        self.radius = msg

    #vive publisher
    def pose_vrCallback(self,msg):
        self.pose_vr = msg
    def triggerCallback(self,msg):
        self.trigger = msg
    def menuCallback(self,msg):
        self.menu = msg
    def gripCallback(self,msg):
        self.grip = msg

    def make_pose(self,x,y,z):
        
        pose = Pose()
        pose.position.x = x -1.1
        pose.position.y = - (y + 0.28)
        pose.position.z = z + 1.0

        pose.orientation.x = 0
        pose.orientation.y = 1.0
        pose.orientation.z = 0
        pose.orientation.w = 0

        return pose

def go_to_pose_goal(x,y,z, qx, qy, qz, qw):
        
        
        pose_goal = Pose()

        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        #Fixed orientation
        pose_goal.orientation.x = 0
        pose_goal.orientation.y = 1.0
        pose_goal.orientation.z = 0
        pose_goal.orientation.w = 0

        move_group.set_pose_target(pose_goal)

        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        move_group.clear_pose_targets()

def go_to_initial_state():
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = -tau / 4
        joint_goal[1] = -tau/4
        joint_goal[2] = tau / 5
        joint_goal[3] = -37*tau / 180
        joint_goal[4] = -tau / 4
        joint_goal[5] = -tau/4# 1/6 of a turn
        move_group.go(joint_goal, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

def picking(x_,y_):
    x = (y_/1000)*1.230 + 0.33
    y = (x_/1000)*(-1.3) - 0.69
    z = 0.27 # I fixed z-coor since the data from OAK-D for z is not stable
    #  width = r_*4
    #  r = r_*4 - 40 
    go_to_initial_state()
    rg2.open_gripper()
    go_to_pose_goal(x,y,0.69,0.0, 1.0, 0.0, 0.0)
    go_to_pose_goal(x,y,0.53,0.0, 1.0, 0.0, 0.0)
    # arm.go_to_goal(x,y,0.69,0,0,1.572)
    # arm.go_to_goal(x,y,0.53,0,0,1.572)
    #  print("Tomato_width:", width)
    #  rg2.move_gripper(r,100)
    #  rg2.close_gripper(20)
    rg2.close_gripper(90)
    time.sleep(3)
    # time.sleep(4)
    go_to_pose_goal(x,y,0.69,0.0, 1.0, 0.0, 0.0)
    # arm.go_to_goal(x,y,0.69,0,0,1.572)


# Fixed position for Green, Turning and Pink
def put_green():
    # arm.go_to_initial_state()
    # rtde_c.moveL([-0.677, 0.008, 0.69, 0, 0, 1.572], 1.5, 1)
    # rtde_c.moveL([-0.677, 0.008, 0.35, 0, 0, 1.572], 1.5, 1)
    go_to_pose_goal(0.677, -0.008, 0.69, 0.0, 1.0, 0.0, 0.0)
    go_to_pose_goal(0.677, -0.008, 0.35, 0.0, 1.0, 0.0, 0.0)
    
    rg2.open_gripper()
    time.sleep(1)
    # rtde_c.moveL([-0.677, 0.008, 0.69, 0, 0, 1.572], 1.5, 1)
    # rtde_c.moveL([-0.176, 0.666, 0.86, 0, 0, 1.572], 1.5, 1)
    go_to_pose_goal(0.677, -0.008, 0.69, 0.0, 1.0, 0.0, 0.0)
    # arm.go_to_pose_goal(0.677, -0.008, 0.35, 0.0, 1.0, 0.0, 0.0)
    go_to_initial_state()
    
def put_turning():
    # arm.go_to_initial_state()
    # rtde_c.moveL([-0.778, -0.214, 0.69, 0, 0, 1.572], 1.5, 1)
    # rtde_c.moveL([-0.778, -0.214, 0.35, 0, 0, 1.572], 1.5, 1)
    go_to_pose_goal(0.778, 0.214, 0.69, 0.0, 1.0, 0.0, 0.0)
    go_to_pose_goal(0.778, 0.214, 0.53, 0.0, 1.0, 0.0, 0.0)
    
    rg2.open_gripper()
    time.sleep(1)
    go_to_pose_goal(0.778, 0.214, 0.69, 0.0, 1.0, 0.0, 0.0)
    # arm.go_to_pose_goal(0.778, 0.214, 0.53, 0.0, 1.0, 0.0, 0.0)
    go_to_initial_state()
    
def put_pink():
    # arm.go_to_initial_state()
    # rtde_c.moveL([-0.518, -0.221, 0.69, 0, 0, 1.572], 1.5, 1)
    # rtde_c.moveL([-0.518, -0.221, 0.35, 0, 0, 1.572], 1.5, 1)
    go_to_pose_goal(0.518, 0.221, 0.69, 0.0, 1.0, 0.0, 0.0)
    go_to_pose_goal(0.518, 0.221, 0.53, 0.0, 1.0, 0.0, 0.0)
    
    rg2.open_gripper()
    time.sleep(1)
    # rtde_c.moveL([-0.518, -0.221, 0.69, 0, 0, 1.572], 1.5, 1)
    # rtde_c.moveL([-0.176, 0.666, 0.86, 0, 0, 1.572], 1.5, 1)
    go_to_pose_goal(0.518, 0.221, 0.69, 0.0, 1.0, 0.0, 0.0)
    go_to_initial_state()


# Saving the path for Big Red and Small Red by HTC VIVE
def vr_red_big():
    i = []
    data = DataHolder()
    while True:
        vr = data.pose_vr
        x_ = vr.pose.position.x
        y_ = vr.pose.position.y
        z_ = vr.pose.position.z
        print(x_)
        pose_goal = data.make_pose(x_,y_,z_)
        print(pose_goal)
        print("Please choose path for Big Red")
    
        if data.trigger.data > 0.99:
            move_group.set_pose_target(pose_goal)
            move_group.go(wait=True)
            move_group.stop()
            move_group.clear_pose_targets()
            i.append(pose_goal)
        elif data.menu.data == 1:
                print("Finished")
                print(len(i))
                go_to_initial_state()
                break
    return i

def vr_red_small():
    i = []
    data = DataHolder()
    while True:
        vr = data.pose_vr
        x_ = vr.pose.position.x
        y_ = vr.pose.position.y
        z_ = vr.pose.position.z
        pose_goal = data.make_pose(x_,y_,z_)
        print(pose_goal)
        print("Please choose path for Small Red")
    
        if data.trigger.data > 0.99:
            move_group.set_pose_target(pose_goal)
            move_group.go(wait=True)
            move_group.stop()
            move_group.clear_pose_targets()
            i.append(pose_goal)
        elif data.menu.data == 1:
                print("Finished")
                print(len(i))
                go_to_initial_state()
                break
    return i

if __name__=="__main__":
    #Init MOVEVIT
    rospy.init_node('robot_arm', anonymous=True)
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    go_to_initial_state()
    #Init RG2 Gripper
    try:
        rg2 = RG('rg2','192.168.1.1',502)
        rg2.close_gripper()
        rg2.open_gripper()
    except:
         print("Lost connection to Gripper/Please check")
    # Init Conveyor Belt     
    board = pyfirmata.Arduino('/dev/ttyUSB0')
    it = pyfirmata.util.Iterator(board)
    it.start()
    relay = board.get_pin('d:8:o')
    sensor = board.get_pin('d:3:i')
    #Init Subscribed Data from Camera and VIVE
    data = DataHolder()
    rate = rospy.Rate(10)
    rad = [] # This array is for calculating mean of tomato radius
    radi = 0
    count_red = 0
    pub_count_red = rospy.Publisher("/count_red",Int32,queue_size=10) #publish total number of red
    # Only choose path for Red Big and Red Small
    redbig_path =[]
    redsmall_path=[]
    redbig_path = vr_red_big()
    redsmall_path = vr_red_small()
    print("Please put tomato on the Conveyor Belt")
    while not rospy.is_shutdown():
        relay.write(0)
        ir = sensor.read()
        if ir is not True: # Check Sensor and Stop Conveyor for Picking
            relay.write(1)
            time.sleep(1)

            # Approximating the Radius of Tomato by calculating mean since data from OpenCV Hough Circle Transform is fluctuated
            count = 0
            while True: 
                y = data.pose.position.y
                if data.pose.position.y < y+2 and data.pose.position.y > y-2: 
                    rad.append(data.radius.data)
                    count += 1
                    if count == 50: 
                        rad.remove(0) # Eliminate 0 values
                        break

            radi = sum(rad)/len(rad) # Mean Radius
            if data.label.data == "redripe":
                        count_red +=1
                        pub_count_red.publish(count_red)
                        print("Number of red: ", count_red)
                        print("Tomato width:", radi)
                        picking(data.pose.position.x, data.pose.position.y)
                        if radi < 60:
                            rad = []
                            for i in redsmall_path:
                                move_group.set_pose_target(i)
                                move_group.go(wait=True)
                                move_group.stop()
                                move_group.clear_pose_targets()
                            rg2.open_gripper()
                            go_to_initial_state()
                        else:
                            rad = []
                            for i in redbig_path:
                                move_group.set_pose_target(i)
                                move_group.go(wait=True)
                                move_group.stop()
                                move_group.clear_pose_targets()
                            rg2.open_gripper()
                            go_to_initial_state()
            elif data.label.data == "green":
                # count_green +=1
                # print("Number of green: ", count_green)
                picking(data.pose.position.x, data.pose.position.y)
                put_green()
            elif data.label.data == "turning":
                picking(data.pose.position.x, data.pose.position.y)
                put_turning()
            elif data.label.data == "pink":
                picking(data.pose.position.x, data.pose.position.y)
                put_pink()