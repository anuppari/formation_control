#!/usr/bin/env python

# Source Author: Anup Parikh
#
# Ground station for formationControl. Collects positions of all agents, monitors for collisions, and
# distributes positions of all agents to all agents

import rospy
import threading
import numpy as np
import roslib
import string
import tf
from geometry_msgs.msg import PoseStamped
from formation_control.msg import PoseArray, Sensing, AbortReset
from formation_control.srv import Graphs, GraphsResponse

lock = threading.Lock()

def groundStation(): # Main node
    global c, obstacles, numAgents, sensingPub, sensingDropout, poseMsg
    global posePub, abortPub, tfListener
    global mode, collisionBubble, ns_prefix
    
    rospy.init_node('groundStation')
    
    # Get some parameters
    mode = rospy.get_param('~mode','demo')
    collisionBubble = rospy.get_param('~collisionBubble',0.1)
    ns_prefix = rospy.get_param('~ns_prefix','ugv')
    sensingDropout = rospy.get_param('~sensingDropout',False)
    formation_file = rospy.get_param('~formation',None)
    obstacles_file = rospy.get_param('~obstacles',None)
    formationScale = rospy.get_param('~formationScale',1.0)
    use_gps = rospy.get_param('~use_gps',False)
    
    # get formation configuration
    (numAgents,c,obstacles) = readConfig(formation_file,obstacles_file,formationScale)
    rospy.logwarn("GroundStation numAgents: "+str(numAgents))
    rospy.logwarn("Collision bubble: "+str(collisionBubble))
    
    # start service handler
    s = rospy.Service('initFormation',Graphs,initFormationHandler)
    rospy.logwarn("initFormation service started.")
    
    # pose publisher
    posePub = rospy.Publisher('/allAgentPose',PoseArray,queue_size=1)
    rospy.logwarn("Publishing agent positions on topic /allAgentPose")
    
    # Use simulated sensor fusion or tf
    if use_gps:
        # Initialize message
        poseMsg = PoseArray()
        poseMsg.poseArray = [PoseStamped() for i in range(numAgents)]
        
        init_pose = [{'x':rospy.get_param('/'+ns_prefix+str(agentID)+'/initX'),'y':rospy.get_param('/'+ns_prefix+str(agentID)+'/initY')} for agentID in range(numAgents)] # initial pose of agents
        pose_subs = [rospy.Subscriber('/'+ns_prefix+str(agentID)+'/pose',PoseStamped,agentPoseCallback,callback_args=(init_pose,agentID),queue_size=1) for agentID in range(numAgents)]
    else:
        # Send tf as pose msg
        if mode=='sim':
            rate = 0.2
        else:
            rate = 0.04
        rospy.logwarn("here here here here here here here here")
        tfListener = tf.TransformListener()
        rospy.Timer(rospy.Duration.from_sec(rate),tfCallback,oneshot=False)
    
    # Generate new sensing graph at specified interval and publish
    sensingPub = rospy.Publisher('/sensingGraph',Sensing,latch=True,queue_size=10)
    rospy.logwarn("Publishing sensing graph on topic /sensingGraph")
    rospy.Timer(rospy.Duration(1), sensingPublisher) # publish sensing graph info at specified rate
    
    # abort publisher
    abortPub = rospy.Publisher('/abortReset',AbortReset,latch=True,queue_size=10)
    rospy.logwarn("Monitoring agents for potential collisions...")
    
    rospy.spin() # block until rospy shutdown
    posePub.unregister() # End pose publishing (for avoiding display of errors)


def agentPoseCallback(data,args):
    global poseMsg
    
    # Unpack args
    init_pose = args[0]
    agentID = args[1]
    numAgents = len(init_pose)
    
    # Update pose array message, at reduced rate
    if not (data.header.seq % 10):
        lock.acquire()
        poseMsg.header.stamp = data.header.stamp
        poseMsg.poseArray[agentID] = data
        poseMsg.poseArray[agentID].pose.position.x += init_pose[agentID]['x']
        poseMsg.poseArray[agentID].pose.position.y += init_pose[agentID]['y']
        lock.release()
    
    # Check for collisions, and publish if all agents pose updated at least once
    seqs = [poseMsg.poseArray[agentID].header.seq for agentID in range(numAgents)]
    if all(seqs):
        publishPose(poseMsg)


def tfCallback(event): # Called at specified rate. gets tf of all agents, concatenating, and publishing to all agents
    global tfListener, numAgents
    
    # Initialize message
    poseMsg = PoseArray()
    poseMsg.poseArray = [PoseStamped() for i in range(numAgents)]
    
    try:
        for agentID in range(numAgents):
            now = rospy.Time.now()
            
            # Prepare PoseStamped message for this agent
            agentPose = PoseStamped()
            agentPose.header.stamp = now
            agentPose.header.frame_id = ns_prefix+str(agentID)
            
            if mode=='sim':
                ext = '/base_footprint'
            else:
                ext = ''
            
            tfListener.waitForTransform("/world","/"+ns_prefix+str(agentID)+ext,now,rospy.Duration(0.5))
            (trans,_) = tfListener.lookupTransform("/world","/"+ns_prefix+str(agentID)+ext,now)
            (agentPose.pose.position.x,agentPose.pose.position.y) = np.array(trans[0:2])
            
            # Add this agent's PoseStamped message to array. Also, update header to reflect latest time.
            poseMsg.header.stamp = agentPose.header.stamp
            poseMsg.poseArray[agentID] = agentPose
        
        # Check for collisions, and publish
        publishPose(poseMsg)
        
    except tf.Exception as e:
        print e


# Check for collisions, and publish pose array message
def publishPose(poseMsg):
    pos = [np.array([poseMsg.poseArray[ID].pose.position.x,poseMsg.poseArray[ID].pose.position.y]) for ID in range(numAgents)]
    
    d = [((i,j),np.sqrt(np.dot(pos[i]-pos[j],pos[i]-pos[j]))) for i in range(numAgents) for j in range(i+1,numAgents)] # distance between agents, tupled with agent pair
    if any(dJ[1] < collisionBubble for dJ in d):
        msg = AbortReset(abort=True)
        abortPub.publish(msg)
        rospy.logerr("Ground Station: These agent pairs broke their collision bubble!: "+str([dJ[0] for dJ in d if dJ[1] < collisionBubble]))
        print pos
        print [((i,j),pos[i]-pos[j]) for i in range(numAgents) for j in range(i+1,numAgents)]
        print d
        rospy.signal_shutdown("Ground Station: At least 2 agents broke their collision bubble!") # shutdown ground station
    else: # publish if collision bubble not breached
        posePub.publish(poseMsg)


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


def readConfig(formation_file,obstacles_file,formationScale):
    # Read formation configuration files
    # dir = roslib.packages.get_pkg_dir('formation_control')+"/config" # package directory
    #cFile = "c"+string.capitalize(mode)
    #obsFile = "obs"+string.capitalize(mode)
    
    #rospy.logwarn("Reading graph configuration files from directory: %s", dir)
    #cMat = np.loadtxt(open(dir+"/"+cFile+".csv","rb"),dtype='Float64',delimiter=',') # Read file
    if formation_file is not None:
        rospy.logwarn("Reading formation configuration from file: %s", formation_file)
        cMat = np.loadtxt(open(formation_file,"rb"),dtype='Float64',delimiter=',') # Read file
        numAgents = int(cMat[:,0:2].max())+1
        c = np.zeros((2,numAgents,numAgents),dtype='Float64') # init 3D array
        for row in cMat: # convert to float array
            c[0,int(row[0]),int(row[1])] = float(row[2])*formationScale
            c[1,int(row[0]),int(row[1])] = float(row[3])*formationScale
    else:
        rospy.logerr("No formation defined!")
        rospy.signal_shutdown("No formation defined!")
    
    #obstacles = np.loadtxt(open(dir+"/"+obsFile+".csv","rb"),dtype='Float64',delimiter=',') # Read file and Convert to float array
    if obstacles_file is not None:
        rospy.logwarn("Reading obstacle positions from file: %s", obstacles_file)
        obstacles = np.loadtxt(open(obstacles_file,"rb"),dtype='Float64',delimiter=',') # Read file and Convert to float array
    else:
        rospy.logwarn("No obstacles!")
        obstacles = np.array([],dtype='Float64')
    
    # Reshape to 1D array for transmission over ROS topic
    c.shape = c.size
    obstacles.shape = obstacles.size
    
    return (numAgents,c,obstacles)


if __name__ == "__main__":
    try:
        groundStation()
    except rospy.ROSInterruptException:
        pass


