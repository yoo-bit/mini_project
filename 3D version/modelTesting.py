import pybullet as p
import pybullet_data
import console
from time import sleep
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
robotStartPos = [0, 0, 1.5]
robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF("3dHopper.urdf", robotStartPos, robotStartOrientation)
robotPos, robotOrn = p.getBasePositionAndOrientation(robotId)
useRealTimeSimulation = False
#Current state
state = 'NULL'
#Initial leg joint position


def getCurrentState():
    # Get contact info
    contactPoints = p.getContactPoints(
                bodyA=robotId, bodyB=planeId,
                linkIndexA=3, linkIndexB=-1)
    global state
    if contactPoints:
        # Robot entering into loading when first contact with ground from air
        if state == 'FLIGHT':
            state = 'LOADING'
            return 'LOADING'
        elif state == 'LOADING':
            # If leg shortens, enter compression state
            if p.getJointState(robotId, 2)[1] > 0:
                state = 'COMPRESSION'
    else:
        state = 'FLIGHT'
        return 'FLIGHT'


def controlRobot():
    # Foot touch ground, stop exhausting leg and zero hip torque
    if state == 'LOADING':
        pass
    # Upper leg chamber sealed, servo body attitude with hip
    if state == 'COMPRESSION':
        legLength = p.getLinkState(robotId, 0)[0][2] - \
                    p.getLinkState(robotId, 3)[0][2]
        stiffness = 1 / legLength
        torque = stiffness * p.getJointState(robotId, 2)[0]
        p.setJointMotorControl2(
                bodyUniqueId=robotId,
                jointIndex=2,
                controlMode=p.TORQUE_CONTROL,
                force=torque)


#Contain update GUI, robot control and step simulation
def stepRobotSimulation():
    getCurrentState()
    controlRobot()
    p.setGravity(0, 0, -10)
    p.stepSimulation()
    sleep(0.01)


def debug():
    while 1:
        global useRealTimeSimulation
        print(p.getKeyboardEvents())
        # Single step simulation when pressing down arrow keyboard
        if p.getKeyboardEvents() == {65298: 1}:
            stepRobotSimulation()
        # Enter console mode when press 'c'
        elif p.getKeyboardEvents() == {99: 1}:
            console.copen(globals(), locals())
        elif p.getKeyboardEvents() == {114: 1}:
            # Enter r to trigger realtime simulation
            useRealTimeSimulation = True
            p.setRealTimeSimulation(1)
        elif p.getKeyboardEvents() == {116: 1}:
            useRealTimeSimulation = False
            p.setRealTimeSimulation(0)
        if useRealTimeSimulation:
            stepRobotSimulation()

if __name__ == '__main__':
    debug()
    


# if (useRealTimeSimulation):
	# p.setRealTimeSimulation(1)

# while 1:
	# if (useRealTimeSimulation):
		# p.setGravity(0, 0, -10)
		# sleep(0.01)  # Time in seconds.
	# else:
		# p.stepSimulation()
