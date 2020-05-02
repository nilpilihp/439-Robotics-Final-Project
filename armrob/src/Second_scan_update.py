#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Float32,Bool
from armrob_util.msg import ME439WaypointXYZ 

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
maxHeightReached = False
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
    sub5 = rospy.Subscriber('/dr_max_top', Bool, Max_check)
    
    # sub4 = rospy.Subscriber('/joint_angles_desired', JointState, Angle_desired)# TODO?
    # sub5 = rospy.Subscriber('/endpoint_xyz', ME439WaypointXYZ, Current_position)

 

# =============================================================================
#   # listener node callback functions
# =============================================================================
def First_scan(msg_in):
    global first_degree_scan_done
    first_degree_scan_done = msg_in.data
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
def Max_check(msg_in):
    global maxHeightReached
    maxHeightReached = True


# =============================================================================
#   # Main function
# =============================================================================    
def main():      
    listener()
    ############## define constants and initialize array ##############   
    scan_point_R = 0.15
    steps = 10 # 10 interpolations TODO
    scan_point_height_step_size = 0.25/steps 
    scan_point_height_start = 0.05
    tolerance = 0.05
    local_second_degree_scan_done = False
    # initialize publish message data type
    location = ME439WaypointXYZ() 
    final_location = ME439WaypointXYZ()

    # row 0 x ; row 1 y; row 2 z; row 3 ultra_sonic at sampling instance
    data = np.ndarray((4,steps+1))
    # and  (not local_second_degree_scan_done)
    while not rospy.is_shutdown():
        if (first_degree_scan_done and not local_second_degree_scan_done):
            x = scan_point_R*np.cos(alpha) #MIGHT BE NEGATIVE TODO
            y = scan_point_R*np.sin(alpha) 
            z = 0.
            newBaseOffset = 0.
            for i in range (steps+1):
                z = scan_point_height_start + newBaseOffset + scan_point_height_step_size * i
                location.xyz = (x,y,z)
                pub_point_scan.publish(location)    #move the arm by using manual end point

                rospy.sleep(2)  # let robot move, 
                # store data, current_... adjusted for unreachable point clipping
                data[3][i] = ultraSonic
                data[2][i] = current_z
                data[1][i] = current_y
                data[0][i] = current_x

                lastIndexReached = i
                # when maxHeightReached true, last xyz was the max, xyz caused nan in invKin
                if(not maxHeightReached):
                    # next iteration can still go, so update x,y
                    x = current_x
                    y = current_y
                    # adjust z "base" for equidistance increments after first iteration 
                    if i == 0: 
                        newBaseOffset = current_z - scan_point_height_start
                else:
                    break
            # this z the height that wasn't acheivable
            local_second_degree_scan_done = True
            
            # so now valid data only in column 0 ~ lastIndexReached-1,this step removes bad entry and rest 
            data = data[:,0:lastIndexReached]
            data_euclidean = np.sqrt(np.sum([np.square(data[1]),np.square(data[0])],axis=0))
            # determine the best height
            # for j in range (steps+1):  
            #     # Look for when current R > true R from first sweep, no at that instance pass the top of obj 
            #     if np.abs((data[3][j] + data_euclidean[j]) - (R + 0.123)) >=tolerance: 
            #         final_location.xyz = (data[0][j],data[1][j],data[2][j])
            #         break
            pub_final_point.publish(final_location)  
            pub_scan_finish.publish(local_second_degree_scan_done)
        
if __name__ == "__main__": 
    try: 
        main()
    except rospy.ROSInterruptException:
        pass
x`