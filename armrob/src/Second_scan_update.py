#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Float32,Bool
from armrob_util.msg import ME439WaypointXYZ 
from sensor_msgs.msg import JointState

# =============================================================================
#   # define node and publishers
# =============================================================================
rospy.init_node('Second_scan', anonymous = False)

pub_scan_finish = rospy.Publisher('/second_degree_scan_done', Bool, queue_size = 1)
pub_point_scan = rospy.Publisher('/xyz_goal', ME439WaypointXYZ, queue_size=1)
pub_final_point = rospy.Publisher('/final_point', ME439WaypointXYZ, queue_size=1)
# pub_joint_angles_desired = rospy.Publisher('/joint_angles_desired', JointState, queue_size=0) # TODO?
# pub_forward = rospy.Publisher('/joint_states', JointState, queue_size=0)# for foward_kinematics.py ?

# =============================================================================
#   # Initialize global variables
# =============================================================================
first_degree_scan_done = False
R = 0.
ultraSonic = 0.
alpha = 0.
current_x = 0.
current_y = 0.
current_z = 0.

# =============================================================================
#   # listener nodes
# =============================================================================
def listener(): 
    sub = rospy.Subscriber('/alpha', Float32, Angle1) 
    sub1 = rospy.Subscriber('/first_degree_scan_done', Bool, First_scan)
    sub2 = rospy.Subscriber('/sensors_data_processed', Float32, Second_scan) 
    sub3 = rospy.Subscriber('/radius', Float32, R_func)
    sub4 = rospy.Subscriber('/dr_position', ME439WaypointXYZ, Current_position)
    # sub4 = rospy.Subscriber('/joint_angles_desired', JointState, Angle_desired)# TODO?
    # sub5 = rospy.Subscriber('/endpoint_xyz', ME439WaypointXYZ, Current_position)
 

# =============================================================================
#   # listener node callback functions
# =============================================================================
def First_scan(msg_in):
    global first_degree_scan_done
    if msg_in.data == True:
        first_degree_scan_done = True
def R_func(msg_in):
    global R
    R = msg_in.data
def Second_scan(msg_in):
    global ultraSonic
    ultraSonic = msg_in.data
def Angle1(msg_in):
    global alpha
    alpha = msg_in.data
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
    # scan_point_R = 0.2
    steps = 10 # 10 interpolations TODO
    scan_point_height_step_size = 0.3/steps 
    scan_point_height_start = 0.05
    tolerance = 0.05

    # initialize publish message data type
    location = ME439WaypointXYZ() 
    final_location = ME439WaypointXYZ()

    # row 0 x ; row 1 y; row 2 z; row 3 ultra_sonic at sampling instance
    data = np.ndarray((4,steps+1))
    while not rospy.is_shutdown(): 
        if first_degree_scan_done:
            first_degree_scan_done = False
            x = R*np.cos(alpha) #MIGHT BE NEGATIVE TODO
            y = R*np.sin(alpha)
            for i in range (steps+1):
                scan_height_current = scan_point_height_step_size * i + scan_point_height_start
                location.xyz = (x,y,scan_height_current)
                pub_point_scan.publish(location)    #move the arm by using manual end point
                rospy.sleep(2)  # let robot move, 
            
                # sampling instance
                data[3][i] = ultraSonic
                data[2][i] = current_z
                data[1][i] = current_y
                data[0][i] = current_x
                
                # update x, y for next iteration
                x = current_x
                y = current_y

            data_euclidean = np.sqrt(np.sum([np.square(data[1]),np.square(data[0])],axis=0))
            # determine the best height
            for j in range (steps+1):  
                # Look for when current R > true R from first sweep, no at that instance pass the top of obj 
                if np.abs((data[3][j] + data_euclidean[j]) - (R + 0.123)) >=tolerance: 
                    final_location.xyz = (data[0][j],data[1][j],data[2][j])
                    break

            pub_scan_finish.publish(True)
        
if __name__ == "__main__": 
    try: 
        main()
    except rospy.ROSInterruptException:
        pass