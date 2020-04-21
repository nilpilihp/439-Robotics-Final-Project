#!/usr/bin/env python

import rospy
import traceback 
import numpy as np
# IMPORT the custom message: 
# we import it "from" the ROS package we created it in (here "me439robot") with an extension of .msg ...
# and actually import the message type by name (here "ME439SensorsRaw")
from std_msgs.msg import Int32, Float32

speed_of_sound_meterspersec = rospy.get_param('/speed_of_sound_meterspersec')
alpha = rospy.get_param('/EWMA_alpha')

pub = rospy.Publisher('/sensors_data_processed', Float32, queue_size=10)
initial = 1
s_prev = 0.0
def listener(): 
    rospy.init_node('sensors_processing_node', anonymous=False)
    sub = rospy.Subscriber('/sensors_data_raw', Int32, sensors_process) # Subscribe to the "sensors_data_raw" topic
    rospy.spin()    # keep the node from exiting
    

def sensors_process(msg_in):
    # bring the Globals into this function's scope, the processed publisher
    global pub,initial,alpha,s_prev

    try:  
        if initial:
            s_prev = msg_in/10.0e6 * speed_of_sound_meterspersec
            initial = 0  
        else: 
            msg_out = Float32()
            msg_out = alpha*(msg_in/10.0e6 * speed_of_sound_meterspersec) + (1-alpha)*s_prev
            s_prev = msg_out
        
            # Publish the Processed message
            pub.publish(msg_out)
            # Log the info (optional)
            rospy.loginfo(pub)       
            
    except Exception:
        traceback.print_exc()
        pass        



if __name__ == '__main__':
    listener()
#    try: 
#        listener()
#    except rospy.ROSInterruptException: 
#        pass