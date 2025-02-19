#!/usr/bin/env python
import traceback 
import rospy
import numpy as np
# IMPORT the messages: 
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Bool
import time


# =============================================================================
#   # Publisher for the servo commands. 
# =============================================================================
pub_joint_angles_desired = rospy.Publisher('/joint_angles_desired', JointState, queue_size=1)
joint_angles_desired_msg = JointState()
joint_angles_desired_msg.name = ['base_joint', 'shoulder_joint', 'elbow_joint', 'forearm_joint', 'wrist_joint', 'fingers_joint']

# =============================================================================
#   # Publisher for first_degree_scan_done
# =============================================================================
pub_first_degree_scan_done = rospy.Publisher('/first_degree_scan_done', Bool, queue_size=1)

# =============================================================================
#   # Publisher for best alpha
# =============================================================================
pub_alpha = rospy.Publisher('/alpha', Float32, queue_size=1)

# =============================================================================
#   # Publisher for best r
# =============================================================================
pub_r = rospy.Publisher('/radius', Float32, queue_size=1)

# Define starting alpha
starting_0 = -np.pi/2
# Define ending alpha

# TODO 
#ending_0 = np.pi/2
ending_0 = 0.

# Define number of steps for alpha to sweep through
n_steps = 10

# Define tolerance for sweep
tolerance = 0.1
cmd_all = [0]*6  # List of 6 values of 1500 each
distance = 0


# call back of /sensors_data_processed subscriber
def sensor_data(msg_in):
    global distance
    distance = msg_in.data

    
  #%% Set all angles except alpha to desired microsecond values and then sweep through alpha
def main():
    rospy.init_node('alpha_sweep', anonymous=False)
    sub_sensors_data_processed = rospy.Subscriber('/sensors_data_processed', Float32, sensor_data)

   

    # initial state TODO
    #      0       1          2            3           4          5
    ang = [0, -np.pi/2., np.radians(160), 0 , np.radians(27.37), 0.]
    joint_angles_desired_msg.position= ang
    pub_joint_angles_desired.publish(joint_angles_desired_msg)
    pub_first_degree_scan_done.publish(False)

    # Make array for storing alpha and r value
    # First row is a list of angles
    # Second row stores corresponding r
    alpha_r = np.array([np.linspace(starting_0, ending_0, num=n_steps) , [0] * n_steps])
    
    # Write for loop to sweep through all alpha
    for i in range(n_steps):
       
        # Sweep through alpha
        ang[0] = alpha_r[0, i]
        joint_angles_desired_msg.position= ang
        pub_joint_angles_desired.publish(joint_angles_desired_msg)
      
        # if first movement, give extra time for data spike to die down before recording
        if(i == 0):
            time.sleep(4)
        else :
            # Pause for 2 second to let ultrasonic data catch up
            time.sleep(2)

        # Assign the value from clean_ultrasonic_sensor
        alpha_r[1, i] = distance
        # TEST TODO
        rospy.logerr("distance: {}".format(distance))
        
    
    # nearest distance, look around there
    best_r = np.min(alpha_r[1])

    # Decide best_alpha with smallest r
    tolerance = 0.02
    # Get all alphas within range of smallest r + tolerance, [0] since tuple([],[])
    best_i_range = np.where(np.logical_and(alpha_r [1]>= best_r, alpha_r[1] <= best_r + tolerance))[0]
    size = np.size(best_i_range)
    rospy.logerr(best_i_range)
    # If there are an odd number of scan hits, then just pick the middle one
    if(size % 2 == 1):
        best_i = int(np.median(best_i_range))
        rospy.logerr("using median %d"%best_i)
        best_alpha = alpha_r[0, best_i]
        best_r = alpha_r[1, best_i]
    # Else if even number of scan hits, pick the mean of the middle 2
    elif(size % 2 == 0): 
        #best_i_1 = int(np.median(best_i_range)-0.5)
        #best_i_2 = int(np.median(best_i_range)-0.5)
        best_i_1 = best_i_range[0]
        best_i_2 = best_i_range[size-1]
        rospy.logerr("using average {}".format(best_i_1,best_i_2))
        best_alpha_1 = alpha_r[0, best_i_1]
        best_alpha_2 = alpha_r[0, best_i_2]
        
        best_r_1 = alpha_r[1, best_i_1]
        best_r_2 = alpha_r[1, best_i_2]
        
        best_r = (best_r_1 + best_r_2)/2
        best_alpha = (best_alpha_1 + best_alpha_2)/2
    rospy.logerr("br {}".format(best_r))
    rospy.logerr("balpha {}".format(best_alpha))             
    #TEST TODO
    # best_alpha = -np.pi/2.
    # best_r = 1.5
    # All done, publish best results
    pub_alpha.publish(best_alpha)
    pub_r.publish(best_r)
    pub_first_degree_scan_done.publish(True)
    
    rospy.spin()
    
if __name__=="__main__":
    try:
        main()

    except:
        traceback.print_exc()
        pass 
    