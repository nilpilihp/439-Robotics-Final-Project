#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from armrob_util.msg import ME439WaypointXYZ 

rospy.init_node('final_movement_node', anonymous = False)

pub_final_move = rospy.Publisher('/xyz_goal', ME439WaypointXYZ, queue_size=1)

second_degree_scan_done = False
final_x = 0.
final_y = 0.
final_z = 0.

def listener(): 
    sub = rospy.Subscriber('/second_degree_scan_done', Bool, Final_movement)
    sub2 = rospy.Subscriber('/final_point', ME439WaypointXYZ, final_point)
    
def final_point(msg_in):
    global final_x
    global final_y
    global final_z
    final_x = msg_in.xyz[0]
    final_y = msg_in.xyz[1]
    final_z = msg_in.xyz[2]
    
def Final_movement(msg_in):
    global second_degree_scan_done
    second_degree_scan_done = msg_in.data

def main():  
    listener()
    
    final_location = ME439WaypointXYZ()
    
    
    while not rospy.is_shutdown():
        if second_degree_scan_done:
            rospy.logerr("Hi im in final")
            final_location.xyz = (final_x,final_y,final_z)
            pub_final_move.publish(final_location)
            break
        


if __name__ == "__main__": 
    try: 
        main()
    except rospy.ROSInterruptException:
        pass