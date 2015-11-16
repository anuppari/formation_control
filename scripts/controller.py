#!/usr/bin/env python

# Source Author: Anup Parikh
#
# Implementation of controller in paper:
# Teng-Hu Cheng, Zhen Kan, Joel A Rosenfeld, Warren Dixon,
# "Decentralized formation control with connectivity maintenance and
# collision avoidance under limited and intermittent sensing", 
# Proc. Am. Control Conf.,  June 2014

import numpy as np
import rospy

def calculateControl(parameters,graph,agentPos):
    # unpack dictionaries
    k = parameters['k']
    GAMMA = parameters['GAMMA']
    delta1 = parameters['delta1']
    delta2 = parameters['delta2']
    Rs = parameters['Rs']
    
    agentID = graph['agentID']
    numAgents = graph['numAgents']
    Nfi = graph['Nfi']
    ci = graph['ci']
    obstacles = graph['obstacles']
    Nsi = graph['Nsi']
    
    # Calculate some errors
    pos = np.array(agentPos[agentID]) # Position of this agent
    posError = [pos-np.array(agentJpos) for agentJpos in agentPos] # position difference between this and all agents
    d = np.array([np.sqrt(np.dot(posErrorJ,posErrorJ.T)) for posErrorJ in posError]) # distance between this and all agents
    obsError = [pos-np.array(obstacles[ii,:]) for ii in range(len(obstacles))] # position difference between this agent and obstacles
    dObs = [np.sqrt(np.dot(obsMerror,obsMerror.T)) for obsMerror in obsError] # distance between this agent and obstacles
    
    Ni = set(np.nonzero(d<=delta1)[0].tolist()) # set of all potential collisions from all agents
    Ni.discard(agentID) # remove this agent from its set of potential collisions
    
    if Nfi == Nfi.intersection(Nsi): # check to make sure agent can sense all formation neighbors (part of contol algorithm)
        
        # must use dictionaries to properly associate function values with agent IDs. For example,
        # Nfi = {2,3}. With arrays, length(b) = 2, b[3] is out of bound. With dictionaries, b[3] is the
        # correct value of b for agent 3.
        
        b = dict(zip(Nfi,[bFunction(d[ID],Rs,delta2) for ID in Nfi])) # dictionary of bij for each agent
        Bagents = dict(zip(Ni,[Bfunction(d[ID],delta1) for ID in Ni])) # dictionary of Bik for each agent
        Bobstacles = [Bfunction(dObs[ii],delta1) for ii in range(len(dObs))] # Array of Bik for each obstacle. Dont need dictionaries
        beta = np.prod(b.values())*np.prod(Bagents.values())*np.prod(Bobstacles) # beta for this agent (agent i)
        
        errorArray = [pos-np.array(agentPos[ID])-np.array(ci[:,ID]) for ID in Nfi]
        gamma = sum([np.dot(error,error.T) for error in errorArray])
        #phi = gamma/(np.power((np.power(gamma,k) + beta),(1.0/k))) # Don't need to calculate this
        
        gradb = dict(zip(Nfi,[gradbFunction(d[ID],posError[ID],Rs,delta2) for ID in Nfi])) # gradient of bih for all agents in Nf, stored in dictionary
        gradBagents = dict(zip(Ni,[gradBfunction(d[ID],posError[ID],delta1) for ID in Ni])) # gradient of Bih for all agents in Ni, stored in dictionary
        gradBobstacles = [gradBfunction(dObs[ii],obsError[ii],delta1) for ii in range(len(dObs))] # gradient of Bih for all obstacles in M. Don't need dictionary because all agents know of all obstacles
        gradBeta1 = np.prod(Bagents.values())*np.prod(Bobstacles)*sum([gradb[h]*np.prod([b[l] for l in Nfi.difference(set([h]))]) for h in Nfi]) # first term of gradient of beta
        gradBeta2 = np.prod(b.values())*sum([gradBagents[h]*np.prod([Bagents[l] for l in Ni if l != h])*np.prod(Bobstacles) for h in Ni]) # first half of second term. Distribute product to separate summations over Ni and Mi. product(b)*sum(Ni and Mi) = product(b)*sum(Ni) + product(b)*sum(Mi)
        gradBeta3 = np.prod(b.values())*sum([gradBobstacles[h]*np.prod(Bagents.values())*np.prod([Bobstacles[l] for l in range(len(dObs)) if l != h]) for h in range(len(dObs))]) # second half of second term
        gradBeta = gradBeta1 + gradBeta2 + gradBeta3
        
        gradGamma = 2.0*sum(errorArray)
        
        gradPhi = (k*beta*gradGamma - gamma*gradBeta)/(k*np.power((np.power(gamma,k) + beta),(1.0+(1.0/k))))
        
        u = -np.dot(GAMMA,gradPhi)
    
    else:
        u = np.array([0,0])
        gamma = 10*np.sum(np.power(np.sum(np.abs(ci),1),2)) # arbitrarily large
    
    return (u,gamma)


def bFunction(d,Rs,delta2):
    if d < (Rs-delta2):
        b = 1.0
    elif (((Rs-delta2)<=d) and (d <= Rs)):
        b = -(1.0/delta2**2)*(d+2.0*delta2-Rs)**2 + (2.0/delta2)*(d+2.0*delta2-Rs)
    elif d > Rs:
        b = 0.0
    
    return b


def Bfunction(d,delta1):
    if d <= delta1:
        B = -d**2/delta1**2 + 2.0*d/delta1
    else:
        B = 1.0
    
    return B


def gradbFunction(d,error,Rs,delta2):
    if ((Rs-delta2)<=d) and (d<Rs):
        gradb = -2.0*(d+delta2-Rs)*error/(d*delta2**2)
    else:
        gradb = np.array([0.0,0.0])
    
    return gradb


def gradBfunction(d,error,delta1):
    if d<=delta1:
        gradB = (-2.0*d/delta1**2 + 2.0/delta1)*(error/d)
    else:
        gradB = np.array([0.0,0.0])
    
    return gradB


