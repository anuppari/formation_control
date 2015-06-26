#!/usr/bin/env python

# Source Author: Anup Parikh
#
# Low level interface between Turtlebots and formationControl

import rospy
import numpy as np
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from formation_control.msg import Gamma

qMult = tf.transformations.quaternion_multiply # quaternion multiplication function handle
qInv = tf.transformations.quaternion_inverse # quaternion inverse function handle
q2m = tf.transformations.quaternion_matrix # quaternion to 4x4 transformation matrix

Kp_ang = 1
Kp_lin = 0.5
kmin = 0.1
kmax = 3
leaderID = "/ugv0"
alpha = 0.3

def lowLevelControl(): # Main node definition
    global velPub, tfListener, agentID
    global gamma, lastLinCmd
    gamma = 100
    lastLinCmd = 0
    
    rospy.init_node('lowLevel') #initialize node
    
    agentID = rospy.get_namespace() # ID of this agent for use in sim and slave/master experimental config
    tfListener = tf.TransformListener() # get transform listener
    
    gammaSub = rospy.Subscriber('gamma',Gamma,gammaCallback) # measure of how close to achieving formation
    velPub = rospy.Publisher('cmd_vel_mux/input/navi',Twist,queue_size=1) # command velocity
    desVelSub = rospy.Subscriber('des_vel',Twist,desVelCallback) # high level controller output on this topic
    
    rospy.spin()

def gammaCallback(gammaMsg):
    global gamma
    
    gamma = gammaMsg.gamma


def desVelCallback(velocity):
    global velPub, agentID, tfListener, gamma, lastLinCmd
    
    now = rospy.Time.now()
    desVelWorld = np.array([velocity.linear.x,velocity.linear.y,velocity.linear.z])
    
    try:
        # Transform desired velocity into turtlebot frame
        tfListener.waitForTransform(agentID,"/world",now,rospy.Duration(0.5))
        (trans,quat) = tfListener.lookupTransform(agentID,"/world",now)
        desVel = rotateVec(desVelWorld,quat)
        
        if agentID == leaderID:
            gammaLimit = 0.4
        else:
            gammaLimit = 0.05
        
        twistMsg = Twist()
        if gamma > gammaLimit: # move to achieve formation
            forward = np.array([1,0,0])
            relativeAngle = np.arctan2(desVel[1],desVel[0])
            
            if np.abs(relativeAngle) < np.pi*80.0/180.0:
                gain = np.maximum(kmin,kmax*np.power(gamma,2.0/3.0)/(1+np.power(gamma,2.0/3.0)))
                angCmd = Kp_ang*relativeAngle
                linCmd = gain*desVel[0]
            else:
                angCmd = np.sign(relativeAngle)
                linCmd = 0
        
        else: # formation acheived, align with leader
            if not (agentID == leaderID):
                # Relative transform between leader and this agent
                tfListener.waitForTransform(agentID,leaderID,now,rospy.Duration(0.5))
                (trans,quat) = tfListener.lookupTransform(agentID,leaderID,now)
                desOrientVec = rotateVec(np.array([1,0,0]),quat)
                desOrientAng = np.arctan2(desOrientVec[1],desOrientVec[0])
                
                gain = np.maximum(kmin,kmax*np.power(gamma,2.0/3.0)/(1+np.power(gamma,2.0/3.0)))
                angCmd = Kp_ang*desOrientAng
                linCmd = gain*desVel[0]
            else:
                angCmd = 0
                linCmd = 0
        
        twistMsg.linear.x = alpha*linCmd + (1-alpha)*lastLinCmd
        twistMsg.angular.z = angCmd
        velPub.publish(twistMsg)
        
        lastLinCmd = linCmd
    
    except tf.Exception:
        pass


def rotateVec(p,q):
    return qMult(q,qMult(np.append(p,0),qInv(q)))[0:3]

if __name__ == '__main__':
    try:
        lowLevelControl()
    except rospy.ROSInterruptException:
        pass
