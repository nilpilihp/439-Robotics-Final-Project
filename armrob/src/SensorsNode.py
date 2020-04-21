#!/usr/bin/env python

import rospy
# Import "serial" to get data from the AlaMode
import serial   
import traceback 
# Import the message types we will need
from std_msgs.msg import Int32

# Slow down the rate of publication so that it doesn't overwhelm the system
# "every n'th line" received will trigger publication of everything. 
n_decimate = 100 

# Publish sensors data at the rate it comes in
# Here we publish: 
#  Analogs (levels), 
#  Ultrasound (microseconds), 
#  and Encoders (counts) 
# all in separate topics. 
def sensors_reader(): 
    # Launch a node called "sensors_node"
    rospy.init_node('sensors_node', anonymous=False)

    # Create the publishers. Name each topic "sensors_##", with message type "Int32" 
    # (because that's what is received over the serial port)
    # Note the queue_size=1 statement: don't let it develop a backlog! 
    pub_sen = rospy.Publisher('/sensors_data_raw', Int32, queue_size=1)
    # pub_A0 = rospy.Publisher('/sensors_A0', Int32, queue_size=1)
    # pub_A1 = rospy.Publisher('/sensors_A1', Int32, queue_size=1)
    # pub_A2 = rospy.Publisher('/sensors_A2', Int32, queue_size=1)
    # pub_A3 = rospy.Publisher('/sensors_A3', Int32, queue_size=1)
    # pub_A4 = rospy.Publisher('/sensors_A4', Int32, queue_size=1)
    # pub_A5 = rospy.Publisher('/sensors_A5', Int32, queue_size=1)
    # pub_U1 = rospy.Publisher('/sensors_U0', Int32, queue_size=1)
    # pub_U1 = rospy.Publisher('/sensors_U1', Int32, queue_size=1)
    # pub_U2 = rospy.Publisher('/sensors_U2', Int32, queue_size=1)
    # pub_E0 = rospy.Publisher('/sensors_E0', Int32, queue_size=1)
    # pub_E1 = rospy.Publisher('/sensors_E1', Int32, queue_size=1)
    
    # Declare the message that will go on the topic. 
    # Here we use one of the message name types we Imported, and add parentheses to call it as a function. 
    # We put data in it using the .data field of the message.
    # msg_A0 = Int32()
    # msg_A1 = Int32()
    # msg_A2 = Int32()
    # msg_A3 = Int32()
    # msg_A4 = Int32()
    # msg_A5 = Int32()
    msg_U0 = Int32()
    # msg_U1 = Int32()
    # msg_U2 = Int32()
    # msg_E0 = Int32()
    # msg_E1 = Int32()
    # Data comes in on the Serial port. Set that up and start it. 
    #----------setup serial--------------
    ser = serial.Serial('/dev/ttyS0')  #serial port to alamode
    ser.baudrate = 115200 
    ser.bytesize = 8
    ser.parity = 'N'
    ser.stopbits = 1
    ser.timeout = 1 # one second time out. 

    ser.flushInput()
    ser.readline()
    # Initialize variables
    tstart = rospy.get_time()


    # MAIN LOOP to keep loading the message with new data. 
    # NOTE that at the moment the data are coming from a separate thread, but this will be replaced with the serial port line reader in the future. 
    while not rospy.is_shutdown():
        # newe0 = 0
        # newe1 = 0
        # newa0 = 0
        # newa1 = 0
        # newa2 = 0
        # newa3 = 0
        # newa4 = 0
        # newa5 = 0
        # newu0 = 0
        # newu1 = 0
        # newu2 = 0
        # new_data_packet = 0
        try: 
            # Here we read the serial port for a string that looks like "e0:123456", which is an Encoder0 reading. 
            # When we get a reading, update the associated motor command
            line = ser.readline().decode().strip() #blocking function, will wait until read entire line
            line = line.split(":")
            # Element 0 of "line" will be a string that says what the data are: 
            data_type = line[0]
            # Element 1 of "line" will be the value, an Integer
            data_value = int(line[1])
            if data_type == 'U0':
                msg_U0= data_value	# Ultrasonic Sensor reading 

            # if data_type == 'A0':
            #     msg_A0 = data_value	# Analog reading 
            #     pub_A0.publish(msg_A0)
            # elif data_type == 'A1':
            #     msg_A1 = data_value	# Analog reading 
            #     pub_A1.publish(msg_A1)
            # elif data_type == 'A2':
            #     msg_A2 = data_value	# Analog reading 
            #     pub_A2.publish(msg_A2)
            # elif data_type == 'A3':
            #     msg_A3 = data_value	# Analog reading 
            #     pub_A3.publish(msg_A3)
            # elif data_type == 'A4':
            #     msg_A4 = data_value	# Analog reading 
            #     pub_A4.publish(msg_A4)
            # elif data_type == 'A5':
            #     msg_A5 = data_value	# Analog reading 
            #     pub_A5.publish(msg_A5)
            # elif data_type == 'U0':
            #     msg_U0= data_value	# Ultrasonic Sensor reading 
            #     pub_sen.publish(msg_U0)
            # elif data_type == 'U1':
            #     msg_U1 = data_value	# Analog reading 
            #     pub_U1.publish(msg_U1)
            # elif data_type == 'U2':
            #     msg_U2 = data_value	# Analog reading 
            #     pub_U2.publish(msg_U2)
            # elif data_type == 'E0':    #only use it as an encoder0 reading if it is one. 
            #     msg_E0 = data_value	# Here is the actual encoder reading. 
            #     pub_E0.publish(msg_E0)
            # elif data_type == 'E1':    #only use it as an encoder1 reading if it is one. 
            #     msg_E1 = data_value	# Here is the actual encoder reading. 
            #     pub_E1.publish(msg_E1)
             
            # Increment the line counter
            cnt_line += 1
            
            if cnt_line >= n_decimate:
                cnt_line = 0
                pub_sen.publish(msg_U0)
        
        except Exception:
            traceback.print_exc()
            pass
            



if __name__ == '__main__':
    try: 
        sensors_reader()
    except rospy.ROSInterruptException: 
        pass
