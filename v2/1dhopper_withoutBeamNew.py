import pybullet as p
import pybullet_data
import math
from statistics import mean
import time
import decimal
from time import sleep
physicsClient = p.connect(p.GUI)
#p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane2.urdf")
#planeId = p.loadSDF("stadium.sdf")
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
# p.changeDynamics(planeId,-1,lateralFriction=1,spinningFriction=1,rollingFriction=1)
# p.createConstraint(boxId,-1,-1,-1,p.JOINT_POINT2POINT,[0,0,0],[0,0,0],[0,0,2])
# p.changeDynamics(
        boxId,
        2,
        lateralFriction=1,
        spinningFriction=1,
        rollingFriction=1
        )
pitchAnglephi = 0
# print(p.getDynamicsInfo(boxId,5))
# p.changeDynamics(boxId,1,lateralFriction=0)
# print(p.getDynamicsInfo(boxId,5))
# print(p.getJointInfo(boxId,4))


def calculateDesiredLegAndBodyAngle(phi, forwardSpeed, desiredForwardSpeed, stancePhaseDuration, r, feedBackGain):
    secondPart = (forwardSpeed * stancePhaseDuration)/(2*r) + (feedBackGain * (forwardSpeed - desiredForwardSpeed))/r
    desiredAngle = phi - math.asin(secondPart)
    # print('secondPart', secondPart)
    # print('desiredAngle', desiredAngle)
    return desiredAngle


def controlBodyAttitude(physicsClass, phi, phiDesired, phiDerivative, kp, kv):
    currentPosition = physicsClass.getJointState(boxId, 0)[0]
    torque = -kp*(phi-phiDesired) - kv*(phiDerivative)
    physicsClass.setJointMotorControl2(
            bodyUniqueId=boxId,
            jointIndex=0,
            controlMode=p.POSITION_CONTROL,
            targetPosition=currentPosition - torque,
            maxVelocity=2
            )
    # print('targetHipPosition', currentPosition - torque)
    # print("Torque:", torque)


def calculateDesiredFowardSpeed(currentPosition, targetPosition, maxForwardSpeed, gain):
    if gain*(targetPosition - currentPosition) < 0:
        xd = max(gain*(targetPosition - currentPosition), -maxForwardSpeed)
    else:
        xd = min(gain*(targetPosition - currentPosition), maxForwardSpeed)
    #print('t-c', gain*(targetPosition - currentPosition))
    #print('desiredSpeedX', xd)
    return xd


#def printRobotStates(physicsClass, robotId):
    # Print worldLinkLinearVelocity of base link on x(forward) position
    # print(
            # 'X velocity:',
            # physicsClass.getLinkState(bodyUniqueId=robotId, linkIndex=0, computeLinkVelocity=1)[6][0])
    # print(
            # 'PitchAngle:',
            # math.atan((p.getLinkState(boxId, 4)[4][2] - p.getLinkState(boxId, 3)[4][2])/abs(p.getLinkState(boxId, 4)[4][0] - p.getLinkState(boxId, 3)[4][0]))
        # )


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
        springLength = 0.52 - p.getJointState(boxId, 1)[0];  # print('springLength', springLength);
        legLength = springLength + baseLength;  # print('legLength', legLength);
        legLengthDifference = springLength - currentSpringLength
        currentSpringLength = springLength
        if abs(legLengthDifference) < 0.000001:
            legLengthDifference = 0
        stiffness = float(1)/(springLength)
        springHardness = 80.0
        zVelocity = p.getLinkState(boxId, 0, computeLinkVelocity=1)[6][2]
        # print('zVelocity', zVelocity)
        # print('spring length', springLength)
        # print('SpringLength', springLength)
        # print('LegTorque', legTorque)
        # print('stiffness', stiffness)
        legAndBodyAngle = 1.57079632679 - p.getJointState(boxId, 0)[0]
        toeLinkHeight = p.getLinkState(boxId, 2)[0][2]
        forwardVelocity = p.getLinkState(boxId, 0, computeLinkVelocity=1)[6][0]
        # print('forwardVelocity', forwardVelocity)
        pitchAnglephiCurrent = math.atan((p.getLinkState(boxId, 4)[4][2] - p.getLinkState(boxId, 3)[4][2])/abs(p.getLinkState(boxId, 4)[4][0] - p.getLinkState(boxId, 3)[4][0]))
        pitchAnglePhiDerivative = pitchAnglephiCurrent - pitchAnglephi
        pitchAnglephi = pitchAnglephiCurrent
        # print('phi', pitchAnglephiCurrent)
        if time.time() - startTime > 15:
            targetPositionX = 7.0
        else:
            targetPositionX = 7.0
        currentBodyPositionX = p.getLinkState(boxId, 0)[4][0];  # print('x position', currentBodyPositionX)
        desiredForwardSpeedX = calculateDesiredFowardSpeed(
                currentPosition=currentBodyPositionX,
                targetPosition=targetPositionX,
                maxForwardSpeed=0.5,
                gain=0.5
                )
        # print('desiredSpeed', desiredForwardSpeedX)
        if abs(forwardVelocity) < 0.0000001:
            forwardVelocity = 0
        # Stance Phase
        if (toeLinkHeight > 0) and (toeLinkHeight < 0.2):
            if currentState == "FLIGHT":
                    touchDownTime = time.time()
                    currentState = stateArray[0]
                    toeOffTheGround = False
            if legLengthDifference < 0:
                currentState = stateArray[1]  # Compression state
            if legLengthDifference > 0:
                currentState = stateArray[2]
            if (abs(legLength - 1.02) < 1 and (hasThrusted is True)):
                currentState = stateArray[3]  # UNLOADING
            legTorque = stiffness * float(abs(0.52 - springLength)) * springHardness
            if abs(zVelocity) < 0.001:
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
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=0.0,
                            force=20
                            )
        # LIFT-OFF
        else:
            if currentState == "UNLOADING":
                liftOffTime = time.time()
                stanceDuration = liftOffTime - touchDownTime
                stancePhaceTimeArray.append(liftOffTime - touchDownTime)
                print('UNLOADING')
            currentState = stateArray[4]
        if currentState == "LOADING":
            pass
        elif currentState == "COMPRESSION" or currentState == "THRUST":
            if time.time() - startTime > 3:
                p.setJointMotorControl2(
                        bodyUniqueId=boxId,
                        jointIndex=0,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=-0.1,
                        maxVelocity=1
                        )
            print('Hip servo')
            pass
        elif currentState == "UNLOADING":
            pass
        elif currentState == "FLIGHT":
            hasThrusted = False
            toeOffTheGround = True
            desiredAngle = 0.0
            if time.time() - startTime > 3:
                averageStanceDuration = mean(stancePhaceTimeArray)
                # print('T', averageStanceDuration)
                desiredAngle = 0.0
                # desiredAngle = calculateDesiredLegAndBodyAngle(
                    # phi=-pitchAnglephi,
                    # forwardSpeed=forwardVelocity,
                    # desiredForwardSpeed=forwardVelocity,
                    # stancePhaseDuration=averageStanceDuration,
                    # r=5.5,
                    # feedBackGain=0.1)
            p.setJointMotorControl2(
                    bodyUniqueId=boxId,
                    jointIndex=0,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=desiredAngle
                    )
            p.setJointMotorControl2(
                            bodyUniqueId=boxId,
                            jointIndex=1,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=0
                            )
        p.stepSimulation()
        currentPosition = p.getJointState(boxId, 0)[0]
        # print('CurrentHipAngle', currentPosition)
        # print(legLength)
        # print(currentState)
        # print('torque', legTorque)
        # print("targetPosition", targetPositionX)
