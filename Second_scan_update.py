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

rospy.init_node('Second_second', anonymous = False)
Scan_finish = rospy.Publisher('/second_degree_scan_done ', Bool, queue_size = 0)
Height = rospy.Publisher('/Height ',Float32, queue_size = 0)

point_scan = rospy.Publisher('/point_scan', ME439WaypointXYZ, queue_size=0)
final_point = rospy.Publisher('/final_point', ME439WaypointXYZ, queue_size=0)

global first_degree_scan_done 
global alpha
global R
global ultraSonic


R = 0.0
ultraSonic = 0.0
alpha = 0.0
first_degree_scan_done = False



def listener(): 

    sub = rospy.Subscriber('/alpha', Float32, Angle1 ) 
    
    sub2 =  rospy.Subscriber('/first_degree_scan_done', Bool, First_scan)
    
    sub3 = rospy.Subscriber('/sensors_data_processed', Float32, Second_scan)
    
    sub4 =  rospy.Subscriber('/R', Float32, R)
    
    rospy.spin()    # keep the node from exiting

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
    
    
def main():      
    listener()
    global first_degree_scan_done
    global alpha
    global R
    global ultraSonic
    
    scan_height = 0.2   # this will be the Z direction
    interpolation = 20 # 20 interpolations
    
    
    if first_degree_scan_done == True:
        
        ultra_sonic = np.array(ultraSonic)
        for i in range (0, interpolation-1):
            R_new = R - 0.2105
            delta = R_new/interpolation/1.0
            
            x = (0.2105+i*delta)*np.arccos(alpha)
            y = (0.2105+i*delta)*np.arcsin(alpha)
            z = scan_height
            location = np.array(x,y,z)
            point_scan.publish(location)
            ultra_sonic_new = np.array(ultraSonic)
            
            ultra_sonic =np.append(ultra_sonic,ultra_sonic_new)
            rospy.sleep(0.5)
            
        array_sort = np.sort(ultra_sonic)
        Minimal = array_sort[0]
        i = np.searchsorted(ultra_sonic,Minimal)
        
        Position_x = (0.2105+i*delta)*np.arccos(alpha)
        Position_y = (0.2105+i*delta)*np.arcsin(alpha)
        Position_z = scan_height-Minimal
        Final_location = np.array(Position_x,Position_y,Position_z)
        final_point.publish(Final_location)
        Scan_finish.publish(True)
        
        
if __name__ == "__main__": 
    try: 
        main()
    except rospy.ROSInterruptException:
        pass