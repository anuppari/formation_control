#!/usr/bin/env python

# Source Author: Anup Parikh
#
# Ground station for formationControl. Collects positions of all agents, monitors for collisions, and
# distributes positions of all agents to all agents

import rospy
import numpy as np
import roslib
import string
import tf
from geometry_msgs.msg import PoseStamped
from formation_control.msg import PoseArray, Sensing, AbortReset
from formation_control.srv import Graphs, GraphsResponse

formationScale = 1.3

def groundStation(): # Main node
    global c, obstacles, numAgents, sensingPub, sensingDropout
    global posePub, abortPub, tfListener
    global abortFlag, mode, collisionBubble, ns_prefix
    
    rospy.init_node('groundStation')
    abortFlag = False
    
    # Get some parameters
    mode = rospy.get_param('~mode','demo')
    collisionBubble = rospy.get_param('~collisionBubble',0.1)
    ns_prefix = rospy.get_param('~ns_prefix','ugv')
    sensingDropout = rospy.get_param('~sensingDropout',False)
    
    # get formation configuration
    (numAgents,c,obstacles) = readConfig(mode)
    rospy.logwarn("GroundStation numAgents: "+str(numAgents))
    
    # start service handler
    s = rospy.Service('initFormation',Graphs,initFormationHandler)
    rospy.logwarn("initFormation service started.")
    
    # Rate for sending tf as pose msg
    if mode=='sim':
        rate = 0.2
    else:
        rate = 0.04
    
    # Send tf as pose msg
    tfListener = tf.TransformListener() # Use TF instead of UTM/GPS for sim/demo
    posePub = rospy.Publisher('/allAgentPose',PoseArray,queue_size=1)
    rospy.logwarn("Publishing agent positions on topic /allAgentPose")
    rospy.Timer(rospy.Duration.from_sec(rate),agentPoseCallback,oneshot=False)
    
    # Generate new sensing graph at specified interval and publish
    sensingPub = rospy.Publisher('/sensingGraph',Sensing,latch=True,queue_size=10)
    rospy.logwarn("Publishing sensing graph on topic /sensingGraph")
    rospy.Timer(rospy.Duration(1), sensingPublisher) # publish sensing graph info at specified rate
    
    # abort publisher
    abortPub = rospy.Publisher('/abortReset',AbortReset,latch=True,queue_size=10)
    rospy.logwarn("Monitoring agents for potential collisions...")
    
    rospy.spin() # block until rospy shutdown
    posePub.unregister() # End pose publishing (for avoiding display of errors)


def agentPoseCallback(event): # Called at specified rate. gets tf of all agents, concatenating, and publishing to all agents
    global tfListener, numAgents
    
    # Initialize message
    poseMsg = PoseArray()
    poseMsg.poseArray = [PoseStamped() for i in range(numAgents)]
    
    try:
        # Use TF during sim (weird issues between gazebo frame and simulated GPS frame)
        if mode=='sim':
            ext = "/base_footprint"
        else:
            ext = ""
        
        for agentID in range(numAgents):
            now = rospy.Time.now()
            
            # Prepare PoseStamped message for this agent
            agentPose = PoseStamped()
            agentPose.header.stamp = now
            agentPose.header.frame_id = ns_prefix+str(agentID)
            
            tfListener.waitForTransform("/world","/"+ns_prefix+str(agentID)+ext,now,rospy.Duration(0.5))
            (trans,_) = tfListener.lookupTransform("/world","/"+ns_prefix+str(agentID)+ext,now)
            (agentPose.pose.position.x,agentPose.pose.position.y) = np.array(trans[0:2])
            
            # Add this agent's PoseStamped message to array. Also, update header to reflect latest time.
            poseMsg.header.stamp = agentPose.header.stamp
            poseMsg.poseArray[agentID] = agentPose
        
        # Check for collisions, and publish
        # agentPosePublish(poseMsg,agentID)
        pos = [np.array([poseMsg.poseArray[ID].pose.position.x,poseMsg.poseArray[ID].pose.position.y]) for ID in range(numAgents)]
        error = [pos[i]-pos[j] for i in range(numAgents) for j in range(i+1,numAgents)] # relative displacement between each agent. (numAgents+1 choose 2) number of distinct pairs
        d = [np.sqrt(np.dot(errori,errori.T)) for errori in error] # distance between agents
        if any(dJ < collisionBubble for dJ in d):
            msg = AbortReset(abort=True)
            abortFlag = True
            abortPub.publish(msg)
            rospy.logerr("Ground Station: At least 2 agents broke their collision bubble!")
        else: # publish if collision bubble not breached
            posePub.publish(poseMsg)
        
    except tf.Exception:
        pass


 # generate sensing set and publish
def sensingPublisher(event):
    global sensingDropout, numAgents
    
    if sensingDropout:
        a = np.random.random_integers(0,1,(numAgents,numAgents)) # generate random array
    else:
        a = np.ones((numAgents,numAgents)) # full sensing, no communication loss
    
    Ns = ((a+a.T)>0) # symmetrize and convert to bool
    Ns.shape = Ns.size # reshape to 1D array
    
    sensingMsg = Sensing(Ns=Ns.tolist())
    sensingPub.publish(sensingMsg)


def initFormationHandler(data): # service handler, in case service is applicable over multimaster
    global c, obstacles, numAgents
    return GraphsResponse(numAgents,c.tolist(),obstacles.tolist())


def readConfig(mode):
    # Read formation configuration files
    dir = roslib.packages.get_pkg_dir('formation_control')+"/config" # package directory
    cFile = "c"+string.capitalize(mode)
    obsFile = "obs"+string.capitalize(mode)
    
    rospy.logwarn("Reading graph configuration files from directory: %s", dir)
    cMat = np.loadtxt(open(dir+"/"+cFile+".csv","rb"),dtype='Float64',delimiter=',') # Read file
    numAgents = int(cMat[:,0:2].max())+1
    c = np.zeros((2,numAgents,numAgents),dtype='Float64') # init 3D array
    for row in cMat: # convert to float array
        c[0,int(row[0]),int(row[1])] = float(row[2])*formationScale
        c[1,int(row[0]),int(row[1])] = float(row[3])*formationScale
    
    obstacles = np.loadtxt(open(dir+"/"+obsFile+".csv","rb"),dtype='Float64',delimiter=',') # Read file and Convert to float array
    
    # Reshape to 1D array for transmission over ROS topic
    #Nf.shape = Nf.size
    c.shape = c.size
    obstacles.shape = obstacles.size
    
    return (numAgents,c,obstacles)


if __name__ == "__main__":
    try:
        groundStation()
    except rospy.ROSInterruptException:
        pass


