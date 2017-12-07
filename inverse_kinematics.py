import os
import time
import pdb
import pybullet as p
import pybullet_data
import utils

serverMode = p.GUI # GUI/DIRECT
sisbotUrdfPath = "./urdf/sisbot.urdf"

# connect to engine servers
physicsClient = p.connect(serverMode)
# add search path for loadURDF
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# define world
#p.setGravity(0,0,-10) # NOTE
planeID = p.loadURDF("plane.urdf")

# setup sisbot
robotStartPos = [0,0,0]
robotStartOrn = p.getQuaternionFromEuler([0,0,0])
print("----------------------------------------")
print("Loading robot from {}".format(sisbotUrdfPath))
robotID = p.loadURDF(sisbotUrdfPath, robotStartPos, robotStartOrn, 
                     flags=p.URDF_USE_INERTIA_FROM_FILE)
joints, controlRobotiqC2, controlJoints, mimicParentName = utils.setup_sisbot(p, robotID)
pdb.set_trace()

# start simulation
try:
    flag = True
    userParams = dict()
    for name in controlJoints:
        joint = joints[name]
        userParam = p.addUserDebugParameter(name, joint.lowerLimit, joint.upperLimit, 0)
        userParams[name] = userParam
    while(flag):
        for name in controlJoints:
            joint = joints[name]
            pose = p.readUserDebugParameter(userParams[name])
            if name==mimicParentName:
                controlRobotiqC2(controlMode=p.POSITION_CONTROL, targetPosition=pose)
            else:
                p.setJointMotorControl2(robotID, joint.id, p.POSITION_CONTROL,
                                        targetPosition=pose, force=joint.maxForce, 
                                        maxVelocity=joint.maxVelocity)
        p.stepSimulation()
    p.disconnect()
except ValueError:
    p.disconnect()
