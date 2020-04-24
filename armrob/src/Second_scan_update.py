#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 20 15:12:49 2020

@author: shane
"""
import numpy as np
import rospy
import std_msgs.msg
from armrob_util.msg import ME439WaypointXYZ 
from sensor_msgs.msg import JointState

# =============================================================================
#   # define node and publishers
# =============================================================================
rospy.init_node('Second_second', anonymous = False)
Scan_finish = rospy.Publisher('/second_degree_scan_done ', Bool, queue_size = 0)
point_scan = rospy.Publisher('/xyz_goal', ME439WaypointXYZ, queue_size=0)
final_point = rospy.Publisher('/final_point', ME439WaypointXYZ, queue_size=0)
pub_joint_angles_desired = rospy.Publisher('/joint_angles_desired', JointState, queue_size=0)

Forward = rospy.Publisher('/joint_states', JointState, queue_size=0)

# =============================================================================
#   # Initialize global variables
# =============================================================================
global first_degree_scan_done 
global alpha
global R
global ultraSonic
R = 0.0
ultraSonic = 0.0
alpha = 0.0
first_degree_scan_done = False


# =============================================================================
#   # listener nodes
# =============================================================================
def listener(): 

    sub = rospy.Subscriber('/alpha', Float32, Angle1 ) 
    
    sub2 =  rospy.Subscriber('/first_degree_scan_done', Bool, First_scan)
    
    sub3 = rospy.Subscriber('/sensors_data_processed', Float32, Second_scan)
    
    sub4 = rospy.Subscriber('/joint_angles_desired', JointState, Angle_desired)
    
    sub5 = rospy.Subscriber('/endpoint_xyz', ME439WaypointXYZ, Current_position)
    
    sub6 = rospy.Subscriber('/r', Float32, R)
    
    
    rospy.spin()    # keep the node from exiting

# =============================================================================
#   # listener node callback functions
# =============================================================================
def First_scan(msg_in):
    global first_degree_scan_done
    
    if msg_in.data == True:
        first_degree_scan_done = True

def R(msg_in):
    global R
    R = msg_in.data

def Second_scan(msg_in):
    global ultraSonic
    ultraSonic = msg_in.data

def Angle1(msg_in):
    global alpha
    alpha = msg_in.data

def Angle_desired(msg_in):
    global angle1
    global angle2
    global angle3
    global angle4
    global angle5
    global angle6
    
    angle1 = msg_in.position[0]
    angle2 = msg_in.position[1]
    angle3 = msg_in.position[2]
    angle4 = msg_in.position[3]
    angle5 = msg_in.position[4]
    angle6 = msg_in.position[5]
    
    
    
    forward_calculation = JointState()
    forward_calculation.name = ['base_joint', 'shoulder_joint', 'elbow_joint', 'forearm_joint', 'wrist_joint', 'fingers_joint'];
    forward_calculation.position = [angle1, angle2, angle3 ,angle4, angle5, angle6]   
    Forward.publish(forward_calculation)
    
    
def Current_position(msg_in):
    global current_x
    global current_y
    global current_z
    
    current_x = msg_in.xyz[0]
    current_y = msg_in.xyz[1]
    current_z = msg_in.xyz[2]
# =============================================================================
#   # Main function
# =============================================================================    
def main():      
    listener()
    global first_degree_scan_done
    global alpha
    global R
    global ultraSonic
    
    ############## define constants and initialize array ##############   
    
    scan_point_R = 0.2
    scan_point_height_delta = 0.3   # 
    scan_point_height_start = 0.05
    interpolation = 20.0 # 20 interpolations
    tolerance = 0.05

    if first_degree_scan_done == True:
        
        x = scan_point_R*np.arccos(alpha)
        y = scan_point_R*np.arcsin(alpha)
        ultra_sonic = np.array(ultraSonic)
        scan_height = np.array(scan_point_height_start)
        
        for i in range (0, interpolation-1):
            scan_height_current = (scan_point_height_delta/interpolation)*i+scan_point_height_start

            location = np.array(x,y,scan_height_current)
            point_scan.publish(location)    #move the arm by using manual end point
            rospy.sleep(1)
            
            x = current_x
            y = current_y
            scan_height_new = np.array(current_z)
            scan_height = np.append(scan_height,scan_height_new)
            ultra_sonic_new = np.array(ultraSonic)
            ultra_sonic =np.append(ultra_sonic,ultra_sonic_new)
      
        
############## scan is not complete yet but I think we should test the code to make sure the arm is moving as expected ##############   
# =============================================================================
#   # determine the height
# =============================================================================    
        for i in range (0, interpolation-1):
            if ultra_sonic[i]-R <=tolerance:
                item_height = scan_height[i]



        
        Final_location = np.array(x,y,item_height)
        final_point.publish(Final_location)
        Scan_finish.publish(True)
        
        
if __name__ == "__main__": 
    try: 
        main()
    except rospy.ROSInterruptException:
        pass