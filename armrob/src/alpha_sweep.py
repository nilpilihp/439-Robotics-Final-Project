#!/usr/bin/env python
"""
Created on Mon Apr 20 21:29:54 2020

@author: brubi
"""
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
pub_r = rospy.Publisher('/r', Float32, queue_size=1)

# Define starting locations in us
starting_0 = -np.pi/2

# Define ending alpha
ending_0 = np.pi/2

# Define number of steps for alpha to sweep through
n_steps = 10

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
    ang = [0., -np.pi/2., np.pi/2., 0., 0., 0.]
    joint_angles_desired_msg.position = ang
    pub_joint_angles_desired.pub(joint_angles_desired_msg)
    pub_first_degree_scan_done.pub(False)

    # Make array for storing alpha and r value
    # First row is a list of angles
    # Second row stores corresponding r
    alpha_r = np.array([np.linspace(starting_0, ending_0, n_steps)] , [0] * n_steps))
    
    # Write for loop to sweep through all alpha
    for i in range(n_steps):
        # Sweep through alpha
        ang[0] = alpha_r[0, i]]
        joint_angles_desired_msg.position= ang
        pub_joint_angles_desired.pub(joint_angles_desired_msg)
        
        # Pause for 1 second to let ultrasonic data catch up
        time.sleep(1)
        
        # Assign the value from clean_ultrasonic_sensor
        alpha_r[1, i] = distance
    
    
    # Decide best_alpha with smallest r
    best_i = argmin(alpha[1])
    best_alpha = alpha_r[0, best_i]
    best_r = alpha_r[1, best_i]

    # All done, publish best results
    pub_alpha.pub(best_alpha)
    pub_r.pub(best_r)
    pub_first_degree_scan_done.pub(scan_done)
    
    rospy.spin()

if __name__=="__main__":
    try:
        main()

    except:
        traceback.print_exc()
        pass  
