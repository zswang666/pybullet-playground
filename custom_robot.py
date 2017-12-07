import os
import pdb
import pybullet as p
import pybullet_data

serverMode = p.GUI # GUI/DIRECT
robotUrdfPath = "./urdf/sisbot.urdf"
#robotUrdfPath = "./urdf/robotiq_c2.urdf"
#robotUrdfPath = "./urdf/ur5.urdf"

# connect to engine servers
physicsClient = p.connect(serverMode)
# add search path for loadURDF
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# define world
p.setGravity(0,0,-10)
planeID = p.loadURDF("plane.urdf")

# define robot
robotStartPos = [0,0,0]
robotStartOrn = p.getQuaternionFromEuler([0,0,0])
print("----------------------------------------")
print("Loading robot from {}".format(robotUrdfPath))
robotID = p.loadURDF(robotUrdfPath, robotStartPos, robotStartOrn)

# get joint information
numJoints = p.getNumJoints(robotID) 
print("Number of joints: {}".format(numJoints))
jointTypeList = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]
for i in range(numJoints):
    jointInfo = p.getJointInfo(robotID, i)
    jointID = jointInfo[0]
    jointName = jointInfo[1].decode("utf-8")
    jointType = jointTypeList[jointInfo[2]]
    jointLowerLimit = jointInfo[8]
    jointUpperLimit = jointInfo[9]
    print("\tID: {}".format(jointID))
    print("\tname: {}".format(jointName))
    print("\ttype: {}".format(jointType))
    print("\tlower limit: {}".format(jointLowerLimit))
    print("\tupper limit: {}".format(jointUpperLimit))
print("------------------------------------------")

# start simulation
try:
    flag = True
    while(flag):
        p.stepSimulation()
        robotPos, robotOrn = p.getBasePositionAndOrientation(robotID)
    p.disconnect()
except:
    p.disconnect()
