import pybullet as p
import pybullet_data
import math
from statistics import mean
import time
import decimal
from time import sleep
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane100.urdf")
# planeId = p.loadSDF("stadium.sdf")
cubeStartPos = [0, 0, 1.5]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
boxId = p.loadURDF(
        "1dHopper_withoutBeam.urdf",
        cubeStartPos,
        cubeStartOrientation,
        useFixedBase=0,
        globalScaling=1.0
        )
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
currentSpringLength = float(0.5)
useRealTimeSimulation = 0


def servoHipAngle(physicsClass, hipAngle, hipAngleDerivative,
                  desiredHipAngle, kp, kv):
    torque = -kp*(hipAngle-desiredHipAngle)-kv*hipAngleDerivative
    physicsClass.setJointMotorControl2(
                bodyUniqueId=boxId,
                jointIndex=0,
                controlMode=p.TORQUE_CONTROL,
                force=torque)


def calculateDesiredLegAndBodyAngle(
        phi, forwardSpeed, desiredForwardSpeed,
        stancePhaseDuration, r, feedBackGain):
    secondPart = (forwardSpeed * stancePhaseDuration)/(2*r) + \
                 (feedBackGain * (forwardSpeed - desiredForwardSpeed))/r
    desiredAngle = phi - math.asin(secondPart)
    # print('secondPart', secondPart)
    # print('desiredAngle', desiredAngle)
    return desiredAngle


# Hold body upright in stance phase
def controlBodyAttitude(physicsClass, phi, phiDesired, phiDerivative, kp, kv):
    physicsClass.setJointMotorControl2(
            bodyUniqueId=boxId,
            jointIndex=0,
            controlMode=p.TORQUE_CONTROL,
            force=0.4632919346498285
            )
    # print('targetHipPosition', currentPosition - torque)
    # print("Torque:", torque)


def calculateDesiredFowardSpeed(
        currentPosition, currentSpeed, targetPosition,
        maxForwardSpeed, kp, kv):
    xd = min(-kp*(currentPosition - targetPosition) -
             kv*currentSpeed, maxForwardSpeed)
    return xd


# Funtion for placing the foot position in FLIGHT state
def placeLegForward(physicsClass):
    physicsClass.setJointMotorControl2(
            bodyUniqueId=boxId,
            jointIndex=0,
            controlMode=p.TORQUE_CONTROL,
            force=0.2
            )


p.setJointMotorControl2(
            bodyUniqueId=boxId,
            jointIndex=0,
            controlMode=p.POSITION_CONTROL,
            targetPosition=0.0,
            force=0.0,
            maxVelocity=2)

if (useRealTimeSimulation):
    p.setRealTimeSimulation(1)

while 1:
    if (useRealTimeSimulation):
        p.setGravity(0, 0, -10)
        sleep(0.01)  # Time in seconds.
    else:
        robotPos = p.getLinkState(boxId, 0)[0]
        p.resetDebugVisualizerCamera(2, 5, -20, robotPos)
        servoHipAngle(physicsClass=p, hipAngle=0, hipAngleDerivative=0,
                      desiredHipAngle=0.3, kp=47, kv=1.26)
        p.stepSimulation()
