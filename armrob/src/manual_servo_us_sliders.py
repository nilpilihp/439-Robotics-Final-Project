#!/usr/bin/python

# ROS node to command a set of 6 servos using Sliders. 
# Peter Adamczyk, University of Wisconsin - Madison
# Updated 2020-03-18
 

from Tkinter import *
import traceback 
import rospy
# IMPORT the messages: 
from sensor_msgs.msg import JointState


# =============================================================================
#   # Publisher for the servo commands. 
# =============================================================================
pub_servo_commands = rospy.Publisher('/servo_commands',JointState,queue_size=1)
servo_commands_msg = JointState()
servo_commands_msg.name = ['cmd00','cmd01','cmd02','cmd03','cmd04','cmd05']


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


#%% Section to set up a nice Tkinter GUI with sliders. 
def main(): 
    # Initialize ROS node
    rospy.init_node('manual_servo_us_sliders', anonymous=False)    
    
    # set up GUI
    root = Tk()
    root.title("Manual Robot Arm Servo Control (Pulses in Microseconds)")
    
    # draw a big slider for servo 0 position
    scale0 = Scale(root,
        from_ = 500,
        to = 2500,
        command = move_servo_0,
        orient = HORIZONTAL,
        length = 1000,
        label = 'Servo_0 us')
    scale0.set(1500)
    scale0.pack(anchor = CENTER)
    
    # draw a big slider for servo 1 position
    scale1 = Scale(root,
        from_ = 500,
        to = 2500,
        command = move_servo_1,
        orient = HORIZONTAL,
        length = 1000,
        label = 'Servo_1 us')
    scale1.set(1500)
    scale1.pack(anchor = CENTER)
    
    # draw a big slider for servo 2 position
    scale2 = Scale(root,
        from_ = 500,
        to = 2500,
        command = move_servo_2,
        orient = HORIZONTAL,
        length = 1000,
        label = 'Servo_2 us')
    scale2.set(1500)
    scale2.pack(anchor = CENTER)
    
    # draw a big slider for servo 3 position
    scale3 = Scale(root,
        from_ = 500,
        to = 2500,
        command = move_servo_3,
        orient = HORIZONTAL,
        length = 1000,
        label = 'Servo_3 us')
    scale3.set(1500)
    scale3.pack(anchor = CENTER)
    
    # draw a big slider for servo 4 position
    scale4 = Scale(root,
        from_ = 500,
        to = 2500,
        command = move_servo_4,
        orient = HORIZONTAL,
        length = 1000,
        label = 'Servo_4 us')
    scale4.set(1500)
    scale4.pack(anchor = CENTER)
    
    # draw a big slider for servo 5 position
    scale5 = Scale(root,
        from_ = 500,
        to = 2500,
        command = move_servo_5,
        orient = HORIZONTAL,
        length = 1000,
        label = 'Servo_5 us')
    scale5.set(1500)
    scale5.pack(anchor = CENTER)
    
    # run Tk event loop
    root.mainloop()
    
    # Shut the servos down if the window closes 
    shutdown_servos()


if __name__=="__main__":
    try:
        main()

    except:
        traceback.print_exc()
        shutdown_servos()
        pass