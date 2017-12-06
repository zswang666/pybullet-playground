import os
import pdb
import pybullet as p
import pybullet_data

serverMode = p.GUI # GUI/DIRECT
robotDataPath = "/home/johnson/Desktop/workspace/urdf"
robotUrdfPath = "./urdf/sisbot.urdf"
#robotUrdfPath = "./urdf/robotiq_c2.urdf"
#robotUrdfPath = "./urdf/ur5.urdf"
#robotUrdfPath = "r2d2.urdf"

# connect to engine servers
physicsClient = p.connect(serverMode)
# add search path for loadURDF
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0,0,-10)
planeID = p.loadURDF("plane.urdf")
robotStartPos = [0,0,1]
robotStartOrn = p.getQuaternionFromEuler([0,0,0])
robotID = p.loadURDF(robotUrdfPath, robotStartPos, robotStartOrn)
flag = True
while(flag):
    p.stepSimulation()
    robotPos, robotOrn = p.getBasePositionAndOrientation(robotID)
    for i in range(p.getNumJoints(robotID)):
        jointStates = p.getJointState(robotID, i)

p.disconnect()
