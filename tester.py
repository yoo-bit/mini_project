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
stateArray = ["LOADING", "COMPRESSION", "THRUST", "UNLOADING", "FLIGHT"]
currentState = "NULL"
hasThrusted = False
toeOffTheGround = False
isRecording = False   # Marker for stance phase time recording
startTime = time.time()
touchDownTime = time.time()
liftOffTime = time.time()
stancePhaceTimeArray = []
p.enableJointForceTorqueSensor(boxId, 0)
p.addUserDebugText('Target', [7, 0, 0])
bodyPosition = [0, 0, 0]
p.setJointMotorControl2(
            bodyUniqueId=boxId,
            jointIndex=0,
            controlMode=p.POSITION_CONTROL,
            targetPosition=0.0,
            force=0.0,
            maxVelocity=2)
# p.changeDynamics(planeId,-1,lateralFriction=1,spinningFriction=0.1,rollingFriction=0.1)
# p.createConstraint(boxId, -1, -1, -1,
                   # p.JOINT_POINT2POINT,
                   # [0, 0, 0], [0, 0, 0], [0, 0, 2])
# p.changeDynamics(
#         boxId,
#         2,
#         lateralFriction=0.8,
#         spinningFriction=0.8,
#         rollingFriction=0.8
#         )
pitchAnglephi = 0
# print(p.getDynamicsInfo(boxId,5))
# p.changeDynamics(boxId,1,lateralFriction=0)
# print(p.getDynamicsInfo(boxId,5))
# print(p.getJointInfo(boxId,4))


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
    torque = -kp*(phi-phiDesired) - kv*(phiDerivative)
    physicsClass.setJointMotorControl2(
            bodyUniqueId=boxId,
            jointIndex=0,
            controlMode=p.TORQUE_CONTROL,
            force=-torque
            )
    # print('targetHipPosition', currentPosition - torque)
    # print("Torque:", torque)


def calculateDesiredFowardSpeed(
        currentPosition, targetPosition,
        maxForwardSpeed, kp):
    xd = min(kp*(currentPosition - targetPosition), maxForwardSpeed)
    return xd


def servoHipAngle(physicsClass, hipAngle, hipAngleDerivative,
                  desiredHipAngle, kp, kv):
    torque = -kp*(hipAngle-desiredHipAngle)-kv*hipAngleDerivative
    physicsClass.setJointMotorControl2(
                bodyUniqueId=boxId,
                jointIndex=0,
                controlMode=p.TORQUE_CONTROL,
                force=torque)


if (useRealTimeSimulation):
    p.setRealTimeSimulation(1)

while 1:
    if (useRealTimeSimulation):
        p.setGravity(0, 0, -10)
        sleep(0.01)  # Time in seconds.
    else:
        robotPos = p.getLinkState(boxId, 0)[0]
        p.resetDebugVisualizerCamera(2, 5, -20, robotPos)
        baseLength = 0.5
        springLength = 0.52 - p.getJointState(boxId, 1)[0]
        legLength = springLength + baseLength
        legLengthDifference = springLength - currentSpringLength
        currentSpringLength = springLength
        stiffness = float(1)/(springLength)
        springHardness = 10.0
        zVelocity = p.getLinkState(boxId, 0, computeLinkVelocity=1)[6][2]
        legAndBodyAngle = 1.57079632679 - p.getJointState(boxId, 0)[0]
        toeLinkHeight = p.getLinkState(boxId, 2)[0][2]
        forwardVelocity = p.getLinkState(boxId, 0, computeLinkVelocity=1)[6][0]
        if abs(forwardVelocity) < 0.000001:
            forwardVelocity = 0
        pitchAnglephiCurrent = math.atan((
            p.getLinkState(boxId, 4)[4][2] - p.getLinkState(boxId, 3)[4][2]) /
            abs(p.getLinkState(boxId, 4)[4][0] -
                p.getLinkState(boxId, 3)[4][0]))
        pitchAnglePhiDerivative = pitchAnglephiCurrent - pitchAnglephi
        pitchAnglephi = pitchAnglephiCurrent
        currentHipAngleDerivative = p.getJointState(boxId, 0)[1]
        currentHipAngle = p.getJointState(boxId, 0)[0]
        if time.time() - startTime > 15:
            targetPositionX = 7.0
        else:
            targetPositionX = 7.0
        currentBodyPositionX = p.getLinkState(boxId, 0)[4][0]
        currentBodyPosition = p.getLinkState(boxId, 0)[4]
        # p.addUserDebugLine(bodyPosition, currentBodyPosition)
        bodyPosition = currentBodyPosition
        desiredForwardSpeedX = calculateDesiredFowardSpeed(
                currentPosition=currentBodyPositionX,
                targetPosition=targetPositionX,
                maxForwardSpeed=1.2,
                kp=-1.0
                )
        contactPoints = p.getContactPoints(
                bodyA=boxId, bodyB=planeId,
                linkIndexA=2, linkIndexB=-1)
        # Set up States
        if len(contactPoints) > 0:
            if currentState == "FLIGHT":
                    touchDownTime = time.time()
                    currentState = stateArray[0]
                    toeOffTheGround = False
            if legLengthDifference < 0:
                currentState = stateArray[1]  # Compression state
            if legLengthDifference > 0:
                currentState = stateArray[2]
            if (abs(legLength - 1.02) < 1 and (hasThrusted is True)):
                currentState = stateArray[3]
            legTorque = (stiffness *
                         float(abs(0.52 - springLength)) *
                         springHardness)
            if abs(0.52 - legLength) < 0.3:
                stiffness = float(3)/(springLength)
                p.setJointMotorControl2(
                            bodyUniqueId=boxId,
                            jointIndex=1,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=0.0,
                            maxVelocity=5
                            )
                hasThrusted = True
            else:
                if hasThrusted is False:
                    p.setJointMotorControl2(
                            bodyUniqueId=boxId,
                            jointIndex=1,
                            controlMode=p.TORQUE_CONTROL,
                            force=-legTorque
                            )
        else:
            if currentState == "UNLOADING":
                liftOffTime = time.time()
                stanceDuration = liftOffTime - touchDownTime
                stancePhaceTimeArray.append(liftOffTime - touchDownTime)
            hasThrusted = False
            toeOffTheGround = True
            currentState = stateArray[4]
        if currentState == "LOADING":
            pass
        elif currentState == "COMPRESSION" or currentState == "THRUST":
            controlBodyAttitude(
                        physicsClass=p,
                        phi=pitchAnglephi,
                        phiDesired=0,
                        phiDerivative=pitchAnglePhiDerivative,
                        kp=43,
                        kv=14
                        )
            pass
        elif currentState == "UNLOADING":
            pass
        elif currentState == "FLIGHT":
            # Exhaust leg to low pressure
            p.setJointMotorControl2(
                            bodyUniqueId=boxId,
                            jointIndex=1,
                            controlMode=p.POSITION_CONTROL,
                            force=10
                            )
            desiredAngle = 0
            if len(stancePhaceTimeArray) > 0:
                averageStanceDuration = mean(stancePhaceTimeArray)
                # print(averageStanceDuration)
                desiredAngle = calculateDesiredLegAndBodyAngle(
                    phi=-pitchAnglephi,
                    forwardSpeed=forwardVelocity,
                    desiredForwardSpeed=desiredForwardSpeedX,
                    stancePhaseDuration=averageStanceDuration,
                    r=legLength-0.25,
                    feedBackGain=0.1)
            servoHipAngle(physicsClass=p, hipAngle=currentHipAngle,
                          hipAngleDerivative=currentHipAngleDerivative,
                          desiredHipAngle=desiredAngle, kp=47, kv=1.26)
            # placeLegForward(p, desiredAngle)
            pass
        state = p.getJointState(boxId, 0)[3]
        # print('CurrentState', currentState, 'XPosition', currentBodyPositionX,
              # 'HipTorque', state,
              # 'Velocity', forwardVelocity, 'Vd', desiredForwardSpeedX)
        # event = p.getKeyboardEvents()
        p.stepSimulation()
        sleep(0.0005)
        # print(pitchAnglePhiDerivative)
        # print(event)
        # print('CurrentState', currentState)
        # print('PitchAngle', pitchAnglephiCurrent)
        # print('current position', currentBodyPositionX)
        # print('CurrentHipAngle', currentPosition)
        # print(legLength - 0.25)
        # print(currentState)
        # print('torque', legTorque)
        # print("targetPosition", targetPositionX)
