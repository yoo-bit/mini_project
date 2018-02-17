import pybullet as p
import pybullet_data
import math
from statistics import mean
import time
import decimal
from time import sleep
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadSDF("stadium.sdf")
cubeStartPos = [0,0,1.8]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("1dHopper_withoutBeam.urdf",cubeStartPos, cubeStartOrientation,useFixedBase=0,globalScaling = 2.0)
currentSpringLength = float(1)
useRealTimeSimulation = 0
# p.createConstraint(boxId,1,-1,-1,p.JOINT_FIXED,[1,0,0],[0,0,0],[0,0,2])
#print("numerOfJoints:", p.getNumJoints(boxId))
# print(p.getJointInfo(boxId,3))
stateArray = ["LOADING","COMPRESSION","THRUST","UNLOADING","FLIGHT"]
currentState = "NULL"
hasThrusted = False
toeOffTheGround = False
isRecording = False #Marker for stance phase time recording
touchDownTime = time.time()
liftOffTime = time.time()
stancePhaceTimeArray = [0.02]
robotPos, robotOrn = p.getBasePositionAndOrientation(boxId)
# p.changeDynamics(boxId,5,lateralFriction=0)
# print(p.getDynamicsInfo(boxId,5))
# p.changeDynamics(boxId,1,lateralFriction=0)
# print(p.getJointInfo(boxId,5))
# print(p.getDynamicsInfo(boxId,5))

def calculateDesiredLegAndBodyAngle(phi,forwardSpeed,desiredForwardSpeed,stancePhaseDuration,r,feedBackGain):
	secondPart = (forwardSpeed * stancePhaseDuration)/(2*r) + (feedBackGain * (forwardSpeed - desiredForwardSpeed))/r
	desiredAngle = phi - math.asin(secondPart)
	# print("desiredAngle",desiredAngle)
	return desiredAngle

if (useRealTimeSimulation):
	p.setRealTimeSimulation(1)

while 1:
	if (useRealTimeSimulation):
		p.setGravity(0,0,-10)
		sleep(0.01) # Time in seconds.
	else:
		#Update the camera
		robotPos, robotOrn = p.getBasePositionAndOrientation(boxId)
		p.resetDebugVisualizerCamera(5,0,-20,robotPos)
	#	p.setJointMotorControl2(bodyUniqueId=boxId,jointIndex=0, controlMode=p.VELOCITY_CONTROL, targetVelocity = 1, force = 500)
		footLength = p.getJointState(boxId,1)[0]
		#print("JointInfo:",footLength) #Get foot joint info
		springLength = abs(float(1) - abs(footLength))
		legLength = springLength + 0.5
		legLengthDifference = springLength - currentSpringLength
		currentSpringLength = springLength
		if abs(legLengthDifference) < 0.000001: legLengthDifference = 0;
		# print("legLengthDifference:",legLengthDifference)
		stiffness = float(1)/springLength
		springHardness = 5000.0
		#print("stiffness:", stiffness)
		legTorque = stiffness * float(abs(1 - springLength)) * springHardness
		legAndBodyAngle = 1.57079632679 - p.getJointState(boxId,0)[0]
		#print("legTorque:", legTorque)
		toeLinkHeight = p.getLinkState(boxId,2)[0][2]
		forwardVelocity = p.getLinkState(boxId,0,computeLinkVelocity=1)[6][0]
		# pitchAngle = p.getLinkState(boxId,0,computeLinkVelocity=0)[5]
		# print("pitchAngle",pitchAngle)
		if abs(forwardVelocity) < 0.0000001:
			forwardVelocity = 0
		if (toeLinkHeight > 0) and (toeLinkHeight < 0.04):
			if currentState == "FLIGHT":
				touchDownTime = time.time()
				# print("Touchdown")
			currentState = stateArray[0]
			toeOffTheGround = False
			# print("LOADING")
			if legLengthDifference < 0: currentState = stateArray[1];#Compression state
			if legLengthDifference > 0: currentState = stateArray[2]; hasThrusted = True;#Thrust state
			if (legLengthDifference == 0 and hasThrusted == True): currentState = stateArray[3]; hasThrusted = False;#ULOADING
			p.setJointMotorControl2(bodyUniqueId=boxId,jointIndex=1, controlMode=p.TORQUE_CONTROL, force = -legTorque)
			# p.setJointMotorControl2(bodyUniqueId=boxId,jointIndex=4, controlMode=p.POSITION_CONTROL, targetPosition = 0.1,force = 300)

		else:
			if currentState == "THRUST":
				liftOffTime = time.time()
				stanceDuration = liftOffTime - touchDownTime
				# print("StanceDuration:",stanceDuration)
				stancePhaceTimeArray.append(liftOffTime - touchDownTime)
				# print("Average Stance Duration:",mean(stancePhaceTimeArray))
				# print("Lift off")
		# 	# print("FLIGHT")
			toeOffTheGround = True
			currentState = stateArray[4]
			p.setJointMotorControl2(bodyUniqueId=boxId,jointIndex=1, controlMode=p.POSITION_CONTROL, targetPosition = 0.5,force = 600)
			# p.setJointMotorControl2(bodyUniqueId=boxId,jointIndex=4, controlMode=p.POSITION_CONTROL, targetPosition = 0.5,force = 200)

			#Position Leg For landing
			# try:
			averageStanceDuration = mean(stancePhaceTimeArray)
			desiredAngle = calculateDesiredLegAndBodyAngle(phi=1.57,forwardSpeed=forwardVelocity,desiredForwardSpeed=forwardVelocity-0.5,stancePhaseDuration=averageStanceDuration,r=legLength,feedBackGain=0.5)
			if abs(desiredAngle) < 0.000001:
				desiredAngle = 0
			# print("DesiredAngle:",desiredAngle)
			# except:
				# print("error")
			p.setJointMotorControl2(bodyUniqueId=boxId,jointIndex=0, controlMode=p.POSITION_CONTROL, targetPosition = -desiredAngle,force = 200)

		if currentState == "LOADING":pass;
			# print("Stop Exhausting leg, zero hip torque")
		elif currentState == "COMPRESSION":pass;
			# p.setJointMotorControl2(bodyUniqueId=boxId,jointIndex=4, controlMode=p.TORQUE_CONTROL, force = -legTorque)
			# print("Seal Upper Leg Chamber")
		elif currentState == "THRUST":pass;
			# print("THRUST")
			# p.setJointMotorControl2(bodyUniqueId=boxId,jointIndex=4, controlMode=p.TORQUE_CONTROL, force = -legTorque)
		elif currentState == "UNLOADING":pass;
			# print("Stop thrust, zero hip angle")
		elif currentState == "FLIGHT":pass;
			# print("Exhaust leg to low pressure")
			# p.setJointMotorControl2(bodyUniqueId=boxId,jointIndex=4, controlMode=p.POSITION_CONTROL, targetPosition = 0.5,force = 500)
		# print(currentState)
		# if abs(forwardVelocity)>0.00001:
			# print("forwardSpeed:",forwardVelocity)
		# print("CurrentBodyAndLegAngle:", math.pi/2 + p.getJointState(boxId,0)[0])
		p.stepSimulation()
		# constraint1 = p.createConstraint(boxId,-1,-1,-1,p.JOINT_FIXED,jointAxis=[0,0,0],parentFramePosition=[0,0,0],childFramePosition=robotPos)




		# print(legAndBodyAngle)
