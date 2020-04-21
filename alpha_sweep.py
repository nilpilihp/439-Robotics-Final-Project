# -*- coding: utf-8 -*-
"""
Created on Mon Apr 20 21:29:54 2020

@author: brubi
"""

import traceback 
import rospy
import numpy as np
# IMPORT the messages: 
from sensor_msgs.msg import JointState

# =============================================================================
#   # Publisher for the servo commands. 
# =============================================================================
pub_servo_commands = rospy.Publisher('/servo_commands', JointState, queue_size=1)
servo_commands_msg = JointState()
servo_commands_msg.name = ['cmd00','cmd01','cmd02','cmd03','cmd04','cmd05']

# Publisher for first_degree_scan_done
first_degree_scan_done = rospy.Publisher('/first_degree_scan_done', scan_done, queue_size=1)

# Publisher for alpha
alpha = rospy.Publisher('/alpha', best_alpha, queue_size=1)

# Listener for ultrasonic sensor
rospy.init_node('ultrasonic_listener', anonymous = True)
sensors_data_processed = rospy.Subscriber('/sensors_data_processed', float)

# Define starting locations in us
starting_0 = 0
starting_1 = 0
starting_2 = 0
starting_3 = 0
starting_4 = 0
starting_5 = 0

# Define ending alpha
ending_0 = 600

# Define number of steps for alpha to sweep through
n_steps = 10

cmd_all = [0]*6  # List of 6 values of 1500 each
    
# Specific functions for the specific Servos/Sliders       
def move_servo_0(pulse_width_us):
    global cmd_all, servo_commands_msg
    cmd_all[0] = int(pulse_width_us)
    servo_commands_msg.position = cmd_all
    servo_commands_msg.header.stamp = rospy.Time.now()
    pub_servo_commands.publish(servo_commands_msg)

def move_servo_1(pulse_width_us):
    global cmd_all, servo_commands_msg
    cmd_all[1] = int(pulse_width_us)
    servo_commands_msg.position = cmd_all
    servo_commands_msg.header.stamp = rospy.Time.now()
    pub_servo_commands.publish(servo_commands_msg)
    
def move_servo_2(pulse_width_us):
    global cmd_all, servo_commands_msg
    cmd_all[2] = int(pulse_width_us)
    servo_commands_msg.position = cmd_all
    servo_commands_msg.header.stamp = rospy.Time.now()
    pub_servo_commands.publish(servo_commands_msg)
    
def move_servo_3(pulse_width_us):
    global cmd_all, servo_commands_msg
    cmd_all[3] = int(pulse_width_us)
    servo_commands_msg.position = cmd_all
    servo_commands_msg.header.stamp = rospy.Time.now()
    pub_servo_commands.publish(servo_commands_msg)
    
def move_servo_4(pulse_width_us):
    global cmd_all, servo_commands_msg
    cmd_all[4] = int(pulse_width_us)
    servo_commands_msg.position = cmd_all
    servo_commands_msg.header.stamp = rospy.Time.now()
    pub_servo_commands.publish(servo_commands_msg)
    
def move_servo_5(pulse_width_us):
    global cmd_all, servo_commands_msg
    cmd_all[5] = int(pulse_width_us)
    servo_commands_msg.position = cmd_all
    servo_commands_msg.header.stamp = rospy.Time.now()
    pub_servo_commands.publish(servo_commands_msg)
    

def shutdown_servos():
    cmd_all = [0,0,0,0,0,0]
    servo_commands_msg.position = cmd_all
    servo_commands_msg.header.stamp = rospy.Time.now()
    pub_servo_commands.publish(servo_commands_msg)    
    
  #%% Set all angles except alpha to desired microsecond values and then sweep through alpha
def main(): 
    # Make scan done false
    global scan_done, best_alpha
    scan_done = False
    
    
    
    
    # Move servos to starting locations
    move_servo_0(starting_0)
    move_servo_1(starting_1)
    move_servo_2(starting_2)
    move_servo_3(starting_3)
    move_servo_4(starting_4)
    move_servo_5(starting_5)
    
    # Make array for storing alpha and r value
    alpha_r = np.transpose(np.array([np.linspace(starting_0, ending_0, n_steps), [0] * n_steps, [0] * n_steps, [0] * n_steps]))
    
    # Write for loop to sweep through all alpha
    for alpha_index in np.linspace(0, n_steps, n_steps+1):
        # Sweep through alpha
        move_servo_0(alpha_r[alpha_index, 0])
        
        # Assign the value from clean_ultrasonic_sensor
        alpha_r[alpha_index, 1] = clean_ultrasonic_sensor.data
        
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
    
    
    
    # Make scan done true
    scan_done = True
    
    

if __name__=="__main__":
    try:
        main()

    except:
        traceback.print_exc()
        shutdown_servos()
        pass  
