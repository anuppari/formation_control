import rosbag
import rospkg
import re
import numpy as np
from scipy import io
import yaml

# rosbag command:
# rosbag record clock tf uav0/cmd_vel uav1/cmd_vel uav2/cmd_vel uav3/cmd_vel uav4/cmd_vel uav5/cmd_vel formationInfo

# Save simulation parameters
rospack = rospkg.RosPack()
folder = rospack.get_path('formation_control')+"/config"
fileName = folder+'/sim_param.yaml'
parameters = yaml.load(open(fileName,'r'))
gains = parameters['gains']
param = yaml.load(open('formationControlSimParam.yaml','r'))
ns_prefix = param['groundStation']['ns_prefix']
gains = param[ns_prefix+'0']['formationControl']['gains']
if 'formationScale' in param['groundStation']:
    formationScale = param['groundStation']['formationScale']
else:
    formationScale = 1.0

obstacles = np.loadtxt(open(folder+"/sim_obstacles.csv","rb"),dtype='Float64',delimiter=',')

cMat = np.loadtxt(open(folder+"/sim_formation.csv","rb"),dtype='Float64',delimiter=',') # Read file
numAgents = int(cMat[:,0:2].max())+1
c = np.zeros((2,numAgents,numAgents),dtype='Float64') # init 3D array
for row in cMat: # convert to float array
    c[0,int(row[0]),int(row[1])] = float(row[2])*formationScale
    c[1,int(row[0]),int(row[1])] = float(row[3])*formationScale
cx = c[0,:,:]
cy = c[1,:,:]

# Rosbag
bag = rosbag.Bag('formationControlSim.bag')

clock = [[], []]
sensingClock = []
Ns = []


# list of numpy arrays for data. Use list instead
# of 2D array because of inconsistent number of data
poseTime = [[] for ii in range(numAgents)] 
q = [[] for ii in range(numAgents)]
cmdTime = [[] for ii in range(numAgents)]
cmd = [[] for ii in range(numAgents)]

for topic, msg, wallTime in bag.read_messages():
    if topic == 'tf':
        for transform in msg.transforms:
            if transform.header.frame_id == 'world':
                uavID = int(re.findall('\d+',transform.child_frame_id)[0]) # ID of this agent
                t = transform.header.stamp.secs + float(transform.header.stamp.nsecs)/1e9
                poseTime[uavID].append(t)
                q[uavID].append(np.array([transform.transform.translation.x,transform.transform.translation.y]))
    elif topic == 'clock': # save clock info for relation between walltime and clock time
        t = msg.clock.secs + float(msg.clock.nsecs)/1e9
        clock[0].append(wallTime.secs + float(wallTime.nsecs)/1e9)
        clock[1].append(t)
        if not (t % 5):
            print t
    elif topic == 'sensingGraph':
        sensingClock.append(msg.header.stamp.secs + float(msg.header.stamp.nsecs)/1e9)
        tempNs = np.array(msg.Ns,dtype='float64')
        tempNs.shape = (numAgents,numAgents)
        Ns.append(tempNs)
    elif 'cmd_vel' in topic:
        uavID = int(re.findall('\d+',transform.child_frame_id)[0]) # ID of this agent
        cmdTime[uavID].append(wallTime.secs + float(wallTime.nsecs)/1e9)
        cmd[uavID].append(np.array([msg.linear.x,msg.linear.y]))

bag.close()

# Converts lists to numpy arrays
q = [np.array(q[uavID]) for uavID in range(numAgents)]
cmd = [np.array(cmd[uavID]) for uavID in range(numAgents)]

# Interpolate and arrange into more useable format
t = np.arange(0,t,step=0.1) # t will have time of last data
q = [np.array([np.interp(t,poseTime[ii],q[ii][:,0]),np.interp(t,poseTime[ii],q[ii][:,1])]).T for ii in range(numAgents)]
cmdSimTime = [np.interp(cmdTime[ii],clock[0],clock[1]) for ii in range(numAgents)]
u = [np.array([np.interp(t,cmdSimTime[ii],cmd[ii][:,0]),np.interp(t,cmdSimTime[ii],cmd[ii][:,1])]).T for ii in range(numAgents)]
Ns = np.dstack(Ns)

io.savemat('simData.mat',{'t':t,'q':q,'u':u,'sensingClock':sensingClock,'cx':cx,'cy':cy,'obstacles':obstacles,'Ns':Ns,'gains':gains},oned_as='column')


