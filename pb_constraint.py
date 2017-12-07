import pybullet as p
import pybullet_data
import time

def testGearJointConstraint():
    cubeA = p.loadURDF("cube_small.urdf", [-0.3,-0.5,0.3])#,p.getQuaternionFromEuler([0,0,-0.7]))
    cubeB = p.loadURDF("cube_small.urdf", [-0.3,0.5,0.3])#,p.getQuaternionFromEuler([0,0,-0.7]))

    c = p.createConstraint(cubeA,-1,cubeB,-1,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,1,0])
    p.changeConstraint(c,gearRatio=1, maxForce=10000)

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf", 0, 0, -2)
testGearJointConstraint()
p.setRealTimeSimulation(1)
while(1):
    #p.setGravity(0,0,-10)
    time.sleep(0.01)
