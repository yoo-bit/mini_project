import pybullet as p
import pybullet_data
import console
from time import sleep
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
robotStartPos = [0, 0, 1.2]
robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF("3dHopper.urdf", robotStartPos, robotStartOrientation)
robotPos, robotOrn = p.getBasePositionAndOrientation(robotId)
useRealTimeSimulation = False
#Current state
state = 'NULL'
#Initial leg joint position
p.resetDebugVisualizerCamera(2.5, 0, -10, robotPos)

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
        # If leg shortens, enter compression state
        if p.getJointState(robotId, 2)[1] > 0:
            state = 'COMPRESSION'
        # If leg lengthens, enter thrust state
        elif p.getJointState(robotId, 2)[1] < 0:
            state = 'THRUST'

    else:
        state = 'FLIGHT'


def sendTorqueControl(joint, torque):
    p.setJointMotorControl2(
                bodyUniqueId=robotId,
                jointIndex=joint,
                controlMode=p.TORQUE_CONTROL,
                targetPosition=0,
                force=torque)
    # print('Torque', torque)
def controlRobot():
    # Foot touch ground, stop exhausting leg and zero hip torque
    if state == 'LOADING':
        pass
    # Upper leg chamber sealed, servo body attitude with hip
    if state == 'COMPRESSION':
        legLength = p.getLinkState(robotId, 0)[0][2] - \
                    p.getLinkState(robotId, 3)[0][2]
        if abs(p.getJointState(robotId, 2)[1]) < 0.6:
            stiffness = 700 / legLength
        else:
            stiffness = 1 / legLength
        torque = -stiffness * p.getJointState(robotId, 2)[0]
        sendTorqueControl(2, torque)
    if state == 'THRUST':
        legLength = p.getLinkState(robotId, 0)[0][2] - \
                    p.getLinkState(robotId, 3)[0][2]
        stiffness = 700 / legLength
        torque = -stiffness * p.getJointState(robotId, 2)[0]
        sendTorqueControl(2, torque)
    if state == 'FLIGHT':
        sendTorqueControl(2, -300)


#Contain update GUI, robot control and step simulation
#Auto camera tracking
def stepRobotSimulation():
    cameraPos = p.getLinkState(robotId, 0)[0]
    getCurrentState()
    controlRobot()
    p.setGravity(0, 0, -10)
    p.resetDebugVisualizerCamera(1, 0, -10, cameraPos)
    p.stepSimulation()


def debug():
    while 1:
        global useRealTimeSimulation
        # Single step simulation when pressing down arrow keyboard
        if p.getKeyboardEvents() == {65298: 1}:
            stepRobotSimulation()
            sleep(0.01)
        # Enter console mode when press 'c'
        elif p.getKeyboardEvents() == {99: 1}:
            console.copen(globals(), locals())
        elif p.getKeyboardEvents() == {114: 1}:
            # Enter r to trigger realtime simulation
            useRealTimeSimulation = True
        elif p.getKeyboardEvents() == {116: 1}:
            useRealTimeSimulation = False
        if useRealTimeSimulation:
            stepRobotSimulation()
            sleep(0.002)
        # print(state, p.getLinkState(robotId, 0)[0][2] - p.getLinkState(robotId, 3)[0][2])
        print(p.getLinkState(robotId,0)[0][2])
if __name__ == '__main__':
    debug()


