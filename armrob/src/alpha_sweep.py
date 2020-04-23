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
from std_msgs.msg import Int32, Float32, Bool
import time
 
distance = 0
def sensor_data(msg_in):
    global distance
    distance = msg_in.data

# =============================================================================
#   # Publisher for the servo commands. 
# =============================================================================

    
# Create the publisher. Name the topic "joint_angles_desired", with message type "JointState"
pub_joint_angles_desired = rospy.Publisher('/joint_angles_desired', JointState, queue_size=1)

joint_angles_desired_msg = JointState()
joint_angles_desired_msg.name = ['base_joint', 'shoulder_joint', 'elbow_joint', 'forearm_joint', 'wrist_joint', 'fingers_joint']
joint_angles_desired_msg.position = [0., -np.pi/2., np.pi/2., 0., 0., 0.]

# Publisher for first_degree_scan_done
first_degree_scan_done = rospy.Publisher('/first_degree_scan_done', Bool, queue_size=1)

# Publisher for alpha
pub_alpha = rospy.Publisher('/alpha', Float32, queue_size=1)

# Publisher for r
pub_r = rospy.Publisher('/r', Float32, queue_size=1)

# Listener for ultrasonic sensor
rospy.init_node('ultrasonic_listener', anonymous = True)


# Define starting locations in us
starting_0 = 0

# Define ending alpha
ending_0 = np.pi/2

# Define number of steps for alpha to sweep through
n_steps = 10

cmd_all = [0]*6  # List of 6 values of 1500 each

def listener():
    sensors_data_processed = rospy.Subscriber('/sensors_data_processed', Float32, sensor_data)
    rospy.spin()

    

  #%% Set all angles except alpha to desired microsecond values and then sweep through alpha
def main():
    rospy.init_node('alpha_sweep', anonymous=False)
    # Make scan done false
    listener()
    global scan_done, best_alpha, distance 
    scan_done = False
    
    pub_joint_angles_desired.pub(joint_angles_desired_msg)

    # Make array for storing alpha and r value
    alpha_r = np.transpose(np.array([np.linspace(starting_0, ending_0, n_steps), [0] * n_steps, [0] * n_steps, [0] * n_steps]))
    
    # Write for loop to sweep through all alpha
    for alpha_index in np.linspace(0, n_steps, n_steps+1):
        # Sweep through alpha
        joint_angles_desired_msg.position[0] = alpha_r[alpha_index, 0]
        pub_joint_angles_desired.pub(joint_angles_desired_msg)
        
        # Pause for 1 second to let ultrasonic data catch up
        time.sleep(1)
        
        # Assign the value from clean_ultrasonic_sensor
        alpha_r[alpha_index, 1] = distance
        
        # Make value for what the neighboring values are
        if alpha_index != 0 & alpha_index != n_steps:
            # Make values for upper diff and lower diff
            alpha_r[alpha_index, 2] = alpha_r[alpha_index, 2] - alpha_r[alpha_index-1, 2]
            alpha_r[alpha_index, 3] = alpha_r[alpha_index, 2] - alpha_r[alpha_index+1, 2]
        elif alpha_index == 0:
            alpha_r[alpha_index, 2] = 0
            alpha_r[alpha_index, 3] = alpha_r[alpha_index, 2] - alpha_r[alpha_index+1, 2]
        elif alpha_index == n_steps:
            alpha_r[alpha_index, 2] = alpha_r[alpha_index, 2] - alpha_r[alpha_index-1, 2]
            alpha_r[alpha_index, 3] = 0
    
    
    # Decide best_alpha based on what we agree tomorrow SO CHANGE THIS
    best_alpha_index = np.argmin(alpha_r[3])
    best_alpha = alpha_r[best_alpha_index, 0]
    best_r = alpha_r[best_alpha_index, 1]
    
    
    # Make scan done true
    scan_done = True
    
    pub_alpha.pub(best_alpha)
    first_degree_scan_done.pub(scan_done)
    pub_r.pub(best_r)
    

if __name__=="__main__":
    try:
        main()

    except:
        traceback.print_exc()
        pass  
