import pybullet as p
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("r2d2.urdf")
print(p.getNumJoints(robotId))
print(p.getJointInfo(robotId,2))
