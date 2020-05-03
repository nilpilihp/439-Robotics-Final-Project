#!/usr/bin/env python

import numpy as np
import rospy
import traceback
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from armrob_util.msg import ME439WaypointXYZ 

import FwdKinArmRob_serial as FK
import InvKinArmRob_serial as IK

# Load parameters from rosparam to keep handy for the functions below: 
# Matched lists of angles and microsecond commands

xyz_goal = ME439WaypointXYZ()
new_goal = False
first_degree_scan_done = False
second_degree_scan_done = False

map_ang_rad_01 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_01')))
map_ang_rad_12 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_12')))
map_ang_rad_23 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_23')))
map_ang_rad_34 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_34')))
map_ang_rad_45 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_45')))
map_ang_rad_56 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_56')))

# limits for each of the joints
rotlim_01 = np.radians(np.array(rospy.get_param('/rotational_limits_joint_01')))
rotlim_12 = np.radians(np.array(rospy.get_param('/rotational_limits_joint_12')))
rotlim_23 = np.radians(np.array(rospy.get_param('/rotational_limits_joint_23')))
rotlim_34 = np.radians(np.array(rospy.get_param('/rotational_limits_joint_34')))
rotlim_45 = np.radians(np.array(rospy.get_param('/rotational_limits_joint_45')))
rotlim_56 = np.radians(np.array(rospy.get_param('/rotational_limits_joint_56')))

# Create the publisher. Name the topic "joint_angles_desired", with message type "JointState"
pub_joint_angles_desired = rospy.Publisher('/joint_angles_desired', JointState, queue_size=1)
pub_DR_position = rospy.Publisher('/dr_position', ME439WaypointXYZ, queue_size=1)
pub_DR_reached_top = rospy.Publisher('/dr_max_top', Bool, queue_size=1)

# Create the message
joint_angles_desired_msg = JointState()
joint_angles_desired_msg.name = ['base_joint', 'shoulder_joint', 'elbow_joint', 'forearm_joint', 'wrist_joint', 'fingers_joint']
DR_position_msg =  ME439WaypointXYZ()
DR_max_reached_msg = Bool()

def xyz_goal_func(msg_in):
    global xyz_goal, new_goal
    if not new_goal:
        ###rospy.logerr("new goal received in endpoint_update")
        xyz_goal = msg_in
        new_goal = True

def First_scan(msg_in):
    global first_degree_scan_done
    if msg_in.data == True:
        first_degree_scan_done = True
        
def Second_scan(msg_in):
    global second_degree_scan_done
    if msg_in.data == True:
        second_degree_scan_done = True
        
def manual_endpoint_location(): 
    global new_goal
    global joint_angles_desired_msg,DR_position_msg,DR_max_reached_msg
    rospy.init_node('manual_joint_angles_node',anonymous=False)
    sub =  rospy.Subscriber('/xyz_goal', ME439WaypointXYZ, xyz_goal_func)
    sub1 = rospy.Subscriber('/first_degree_scan_done', Bool, First_scan)
    sub2 = rospy.Subscriber('/second_degree_scan_done', Bool, Second_scan)
    ## MODIFY HERE
    ## For continuous motion, set up a series of points and publish at a constant rate. 
    ## Use a r=rospy.Rate() object and r.sleep()
    ## inside the While loop, to move to the new angles gradually          
    while not rospy.is_shutdown(): 
        if  second_degree_scan_done and new_goal:
            new_goal = False
            # Compute Inverse Kinematics
            ###rospy.logerr("goal xyz {}".format(xyz_goal.xyz))
            ang = IK.armrobinvkin(np.array(xyz_goal.xyz))
            ###rospy.logerr("From invKin: {}".format(ang))
            # Compute limited joint angles. 
            ang_lim = ang
            ang_lim[0] = np.clip(ang[0], np.min(rotlim_01), np.max(rotlim_01))
            ang_lim[1] = np.clip(ang[1], np.min(rotlim_12), np.max(rotlim_12))
            ang_lim[2] = np.clip(ang[2], np.min(rotlim_23), np.max(rotlim_23))
            ang_lim[3] = np.clip(ang[3], np.min(rotlim_34), np.max(rotlim_34))
            ang_lim[4] = np.clip(ang[4], np.min(rotlim_45), np.max(rotlim_45))
            ang_lim[5] = np.clip(ang[5], np.min(rotlim_56), np.max(rotlim_56))
            
            joint_angles_desired_msg.position = ang_lim 
            joint_angles_desired_msg.header.stamp = rospy.Time.now()
            pub_joint_angles_desired.publish(joint_angles_desired_msg)
            
        elif first_degree_scan_done and new_goal:
            new_goal = False
            # Compute Inverse Kinematics
            ###rospy.logerr("goal xyz {}".format(xyz_goal.xyz))
            ang = IK.armrobinvkin(np.array(xyz_goal.xyz))
            ###rospy.logerr("From invKin: {}".format(ang))
            # Compute limited joint angles. 
            ang_lim = ang
            ang_lim[0] = np.clip(ang[0], np.min(rotlim_01), np.max(rotlim_01))
            ang_lim[1] = np.clip(ang[1], np.min(rotlim_12), np.max(rotlim_12))
            ang_lim[2] = np.clip(ang[2], np.min(rotlim_23), np.max(rotlim_23))
            ang_lim[3] = np.clip(ang[3], np.min(rotlim_34), np.max(rotlim_34))
            ang_lim[4] = np.clip(ang[4], np.min(rotlim_45), np.max(rotlim_45))
            ang_lim[5] = np.clip(ang[5], np.min(rotlim_56), np.max(rotlim_56))
            
            ###rospy.logerr("From clipping: {}".format(ang_lim))
            # Predict where the "limited" angles will get you. 
            xyz_pred = FK.armrobfwdkin(np.array(ang_lim))
            ###rospy.logerr("Predicted location: {}".format(xyz_pred))
            
            #finger roation at max, staapppp
            if (ang_lim[4] == np.min(rotlim_45) or ang_lim[4] == np.max(rotlim_45)):
                DR_max_reached_msg.data = True
                pub_DR_reached_top.publish(DR_max_reached_msg)
            else:
                # when xyz_pred all nan, prev was the highest it can go
                if (any(np.isnan(xyz_pred))):
                    DR_max_reached_msg.data = True
                    pub_DR_reached_top.publish(DR_max_reached_msg)
                else:
                    #xyz_err_pred = np.array(xyz_goal.xyz) - xyz_pred
                    #xyz_err_norm = np.linalg.norm(xyz_err_pred)
                    #if xyz_err_norm > 0.001:
                        ###rospy.logerr('Unreachable Endpoint!')
                    
                    # Only here when now clipping or out of z range, then publish 
                    DR_position_msg.xyz = (xyz_pred[0],xyz_pred[1],xyz_pred[2])
                    # Publish on joint_angles_desire to move to endpoint. 
                    joint_angles_desired_msg.position = ang_lim 
                    joint_angles_desired_msg.header.stamp = rospy.Time.now()
                    pub_joint_angles_desired.publish(joint_angles_desired_msg)
                    # Publish on dr_position for second scan node
                    pub_DR_position.publish(DR_position_msg)

if __name__ == "__main__":
    try:
        manual_endpoint_location()
    except:
        traceback.print_exc()
        pass
    
    
    
