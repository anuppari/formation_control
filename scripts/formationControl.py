#!/usr/bin/env python
#import roslib; roslib.load_manifest('roscopter') # not needed for catkin. DO NOT REMOVE ABOVE LINE

# Source Author: Anup Parikh
#
# ROS wrapper for controller.py

import rospy
import numpy as np
import re
from controller import calculateControl
from formation_control.msg import Sensing, PoseArray, AbortReset, Gamma
from formation_control.srv import Graphs
from geometry_msgs.msg import Twist

defaultParam = {'k': 0.1, 'GAMMA': 300, 'delta1': 2, 'delta2': 10, 'Rs': 50} # pack gains/parameters into dictionary

def formationControl(): # Main node definition
    global agentID, parameters, graph
    global velPub, gammaPub, allAgentPoseSub
    
    rospy.init_node('formationControl') # initialize node
    agentID = int(re.findall('\d+',rospy.get_namespace())[0]) # ID of this agent
    mode = rospy.get_param('~mode','demo') # Extra initialization if sim
    
    # Initialize Publishers/Subscribers
    sensingSub = rospy.Subscriber('/sensingGraph',Sensing,sensingCallback) # topic has sensing graph
    velPub = rospy.Publisher('cmd_vel',Twist,queue_size=1) # control commands sent on this topic for sim/demo
    gammaPub = rospy.Publisher('gamma',Gamma,queue_size=1) # Measure of how close we are to achieving formation
    abortResetSub = rospy.Subscriber('/abortReset',AbortReset,abortResetCallback) # check for abort or reset over ROS topic
    
    # Get parameters, gains, graph info
    (parameters,graph) = reset_controller()
    
    # Wait for everything to finish initializing
    if mode=='sim':
        rospy.sleep(1)
        rospy.logwarn("agent %d formationControl: Waiting for sim", agentID)
        rospy.sleep(2) # wait for publisher to finish registering
        rospy.logwarn("agent %d formationControl: Sim ready. Beginning control", agentID)
        
        # stupid hack to engage motors in sim
        twistMsg = Twist()
        twistMsg.linear.z = 1
        velPub.publish(twistMsg)
        rospy.sleep(0.5)
        velPub.publish(Twist())
    
    # calculation and publication of control handled in allAgentPoseCallback. Control
    # only needs to be recalculated when new info comes in, i.e. new agent position info.
    # Position of this agent and all agents are recieved on /allAgentPose topic.
    allAgentPoseSub = rospy.Subscriber('/allAgentPose',PoseArray,allAgentPoseCallback) # topic has positions of all agents
    
    rospy.spin()


def abortResetCallback(data): # get abort from groundstation
    global parameters, graph, allAgentPoseSub
    
    if data.abort == True:
        allAgentPoseSub.unregister() # => dont calculate new control
        [velPub.publish(Twist()) for i in range(100)] # set zero velocity many times, for good measure
        velPub.unregister()
        rospy.signal_shutdown("formationControl: Abort signal recieved from ground station!") # shutdown controller
        
    if reset:
        allAgentPoseSub.unregister()
        (parameters,graph) = reset_controller()
        allAgentPoseSub = rospy.Subscriber('/allAgentPose',PoseArray,allAgentPoseCallback) # topic has positions of all agents


def reset_controller():
    # Re-Initialize Controller gains/parameters
    parameters = rospy.get_param('~gains',defaultParam)
    parameters['GAMMA'] = parameters['GAMMA']*np.eye(2)
    
    # Get formation graph
    rospy.wait_for_service('/initFormation', timeout=None)
    getGraph = rospy.ServiceProxy('/initFormation',Graphs)
    graphData = getGraph(True)
    graph = parseGraph(graphData,agentID)
    
    return (parameters,graph)

def allAgentPoseCallback(agentPoseArray):
    global graph, parameters, velPub, gammaPub
    
    # pose of all agents including this agent
    agentPoses = [(agentPoseArray.poseArray[i].pose.position.x,agentPoseArray.poseArray[i].pose.position.y) for i in range(len(agentPoseArray.poseArray))]
    
    # calculate and publish new control
    (u,gamma) = calculateControl(parameters,graph,agentPoses)
    
    twistMsg = Twist()
    (twistMsg.linear.x,twistMsg.linear.y) = u
    velPub.publish(twistMsg)
    
    gammaMsg = Gamma()
    gammaMsg.gamma = gamma
    gammaPub.publish(gammaMsg)


def sensingCallback(data): # Callback for getting sensing graph
    global graph
    
    numAgents = graph['numAgents']
    Ns = np.array(data.Ns)
    Ns.shape = (numAgents,numAgents) # Overall sensing set
    Nsi = set(np.nonzero(Ns[agentID,:])[0]) # Sensing set for agent i
    Nsi.discard(agentID) # remove this agent from the set of its sensing neighbors
    graph['Nsi'] = Nsi


def parseGraph(data,agentID):
    
    numAgents = data.numAgents
    
    # Format data appropriately
    c = np.array(data.c)
    c.shape = (2,numAgents,numAgents) # convert from 1D to 3D. numAgents rows, numAgents columns, 2 depth
    ci = c[:,agentID,:] # get desired relative positions for this matrix. 2 rows (x,y positions), numAgents columns
    Nfi = set(np.nonzero(ci)[1]) # Get formation neighbors set based on desired relative positions
    
    obstacles = np.array(data.obstacles)
    obstacles.shape = (obstacles.size/2,2)
    
    Nsi = Nfi # Initialize sensing set as full formation set
    
    # pack formation info into dictionary
    graph = dict(zip(['agentID','numAgents','Nfi','ci','obstacles','Nsi'],[agentID,numAgents,Nfi,ci,obstacles,Nsi]))
    
    return graph


if __name__ == '__main__':
    try:
        formationControl()
    except rospy.ROSInterruptException:
        pass
