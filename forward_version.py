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
cubeStartPos = [0, 0, 2.0]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
boxId = p.loadURDF(
        "1dHopper_withoutBeam2.urdf",
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
stancePhaceTimeArray = [0.02]
# p.changeDynamics(planeId,-1,lateralFriction=0.9,spinningFriction=0.8,rollingFriction=0.8)
p.changeDynamics(
        boxId,
        2,
        lateralFriction=0.8,
        spinningFriction=0.8,
        rollingFriction=0.8
        )
pitchAnglephi = 0
# print(p.getDynamicsInfo(boxId,5))
# p.changeDynamics(boxId,1,lateralFriction=0)
# print(p.getDynamicsInfo(boxId,5))
# print(p.getJointInfo(boxId,4))
#p.createConstraint(boxId,-1,-1,-1,p.JOINT_POINT2POINT,[0,0,0],[0,0,0],[0,0,2])

def calculateDesiredLegAndBodyAngle(phi, forwardSpeed, desiredForwardSpeed, stancePhaseDuration, r, feedBackGain):
    secondPart = (forwardSpeed * stancePhaseDuration)/(2*r) + (feedBackGain * (forwardSpeed - desiredForwardSpeed))/r
    desiredAngle = phi - math.asin(secondPart)
    #print('desiredAngle', desiredAngle)
    return desiredAngle


def controlBodyAttitude(physicsClass, phi, phiDesired, phiDerivative, kp, kv):
    torque = -kp*(phi-phiDesired) - kv*(phiDerivative)
    if torque > 0:
        torque = torque + 240
    elif torque < 0:
        torque = torque - 240
    physicsClass.setJointMotorControl2(
            bodyUniqueId=boxId,
            jointIndex=0,
            controlMode=p.TORQUE_CONTROL,
            force=-torque)
    print("Torque:", torque)


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
        # controlBodyAttitude(
                    # physicsClass=p,
                    # phi=0.3,
                    # phiDesired=0,
                    # phiDerivative=0,
                    # kp=153,
                    # kv=14
                    # )
        # p.setJointMotorControl2(
            # bodyUniqueId=boxId,
            # jointIndex=0,
            # controlMode=p.TORQUE_CONTROL,
            # force=260)

        # print(p.getJointInfo(boxId, 0))
        # print(p.getJointInfo(boxId, 1))
        # print(p.getJointInfo(boxId, 2))
        # print(p.getJointInfo(boxId, 3))
        # print(p.getJointInfo(boxId, 4))
        hasThrusted = False
        robotPos = p.getLinkState(boxId, 0)[0]
        p.resetDebugVisualizerCamera(3, 5, -20, robotPos)
        baseLength = 0.5
        springLength = 0.5 - p.getJointState(boxId, 3)[0];  # print('springLength', springLength);
        legLength = springLength + baseLength;  # print('legLength', legLength);
        legLengthDifference = springLength - currentSpringLength
        currentSpringLength = springLength
        if abs(legLengthDifference) < 0.000001:
            legLengthDifference = 0
        stiffness = float(1)/(springLength)
        springHardness = 4000.0
        legTorque = stiffness * float(abs(0.5 - springLength)) * springHardness
        legAndBodyAngle = 1.57079632679 - p.getJointState(boxId, 0)[0]
        toeLinkHeight = p.getLinkState(boxId, 4)[0][2]
        forwardVelocity = p.getLinkState(boxId, 0, computeLinkVelocity=1)[7][2]
        pitchAnglephiCurrent = math.atan((p.getLinkState(boxId, 2)[4][2] - p.getLinkState(boxId, 1)[4][2])/abs(p.getLinkState(boxId, 1)[4][0] - p.getLinkState(boxId, 2)[4][0]))
        pitchAnglePhiDerivative = pitchAnglephiCurrent - pitchAnglephi
        pitchAnglephi = pitchAnglephiCurrent
        if time.time() - startTime > 15:
            targetPositionX = 0.0
        else:
            targetPositionX = 0.0
        currentBodyPositionX = p.getLinkState(boxId, 0)[4][0]; #print('x position', currentBodyPositionX)
        desiredForwardSpeedX = calculateDesiredFowardSpeed(
                currentPosition=currentBodyPositionX,
                targetPosition=targetPositionX,
                maxForwardSpeed=0.5,
                gain=0.3
                )
        if abs(forwardVelocity) < 0.0000001:
            forwardVelocity = 0
        if (toeLinkHeight > 0) and (toeLinkHeight < 0.2):
            if currentState == "FLIGHT":
                    touchDownTime = time.time()
                    currentState = stateArray[0]
                    toeOffTheGround = False
            if legLengthDifference < 0:
                currentState = stateArray[1]  # Compression state
            if legLengthDifference > 0:
                currentState = stateArray[2]
                hasThrusted = True  # Thrust state
            if (legLengthDifference == 0 and hasThrusted is True):
                currentState = stateArray[3]
                hasThrusted = False  # ULOADING
            p.setJointMotorControl2(
                        bodyUniqueId=boxId,
                        jointIndex=3,
                        controlMode=p.TORQUE_CONTROL,
                        force=-legTorque
                        )
            # if hasThrusted is False:
                # controlBodyAttitude(
                        # physicsClass=p,
                        # phi=pitchAnglephi,
                        # phiDesired=0,
                        # phiDerivative=pitchAnglePhiDerivative,
                        # kp=153,
                        # kv=10
                        # )
                # hasThrusted = True

        else:
            if currentState == "THRUST":
                liftOffTime = time.time()
                stanceDuration = liftOffTime - touchDownTime
                stancePhaceTimeArray.append(liftOffTime - touchDownTime)
            toeOffTheGround = True
            currentState = stateArray[4]
            p.setJointMotorControl2(
                bodyUniqueId=boxId,
                jointIndex=0,
                controlMode=p.TORQUE_CONTROL,
                force=0)
            p.setJointMotorControl2(
                bodyUniqueId=boxId,
                jointIndex=3,
                controlMode=p.POSITION_CONTROL,
                targetPosition=0.3,
                force=200
                        )
            averageStanceDuration = mean(stancePhaceTimeArray)
            # desiredAngle = calculateDesiredLegAndBodyAngle(
                    # phi=-pitchAnglephi,
                    # forwardSpeed=forwardVelocity,
                    # desiredForwardSpeed=desiredForwardSpeedX,
                    # stancePhaseDuration=averageStanceDuration,
                    # r=legLength,
                    # feedBackGain=0.1)
            # if abs(desiredAngle) < 0.000001:
                # desiredAngle = 0
            # if time.time() - startTime > 0:
                # p.setJointMotorControl2(
                        # bodyUniqueId=boxId,
                        # jointIndex=0,
                        # controlMode=p.POSITION_CONTROL,
                        # targetPosition=-desiredAngle,
                        # force=200
                        # )
        if currentState == "LOADING":
            pass
        elif currentState == "COMPRESSION":
            pass
        elif currentState == "THRUST":
            pass
        elif currentState == "UNLOADING":
            pass
        elif currentState == "FLIGHT":
            pass
        p.stepSimulation()
        #print("targetPosition", targetPositionX)
