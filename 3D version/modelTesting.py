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
p.setJointMotorControlArray(
    robotId,
    [0,1,2],
    p.POSITION_CONTROL,
    targetPositions=[0,0,0],
    forces=[0,0,0]
    )

    
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


def sendP(joint, position):
    p.setJointMotorControl2(
        bodyUniqueId=robotId,
        jointIndex=joint,
        controlMode=p.POSITION_CONTROL,
        targetPosition=position,
        maxVelocity=5
        )


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
        # Seal leg chamber, work like a spring
        legLength = 0.25 + 0.52 - p.getJointState(robotId, 2)[0]
        if abs(p.getJointState(robotId, 2)[1]) < 0.5:
            torque = -220
        else:
            stiffness = 1 / legLength
            torque = -stiffness * p.getJointState(robotId, 2)[0]
        # Simulate spring force
        sendTorqueControl(2, torque)
        # Servo body attitude
        controlBodyAttitude()
    if state == 'THRUST':
        # Pressurize Leg
        sendTorqueControl(2, -220)
        # Servo body attitude
        controlBodyAttitude()
    if state == 'FLIGHT':
        # Exhaust leg to low pressure
        sendTorqueControl(2, -10)
        # Position leg for landing
        # X direction
        # sendTorqueControl(0, 2.5)


def controlBodyAttitude():
    kp = 153
    kv = 14
    # Balance along X axis
    pitchAngle = p.getBasePositionAndOrientation(robotId)[1][1]
    pitchAngleDerivative = p.getBaseVelocity(robotId)[1][1]
    torque = -kp * (pitchAngle - 0) - kv * pitchAngleDerivative
    sendTorqueControl(0, -torque)
    # Balance along Y axis
    rollAngle = p.getBasePositionAndOrientation(robotId)[1][0]
    rollAngleDerivative = p.getBaseVelocity(robotId)[1][0]
    torque2 = -kp * (rollAngle - 0) - kv * rollAngleDerivative
    sendTorqueControl(1, -torque2)
    print(torque)


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
        print(p.getLinkState(robotId, 0)[0][2])
        # print(p.getJointState(robotId,2)[0])
if __name__ == '__main__':
    debug()



# Debug command lines
# p.createConstraint(robotId, -1, -1, -1, p.JOINT_POINT2POINT, [0, 0, 0], [0, 0, 0], [0, 0, 2])
# p.getJointState(robotId,0)
# legLength = p.getLinkState(robotId, 0)[0][2] - p.getLinkState(robotId, 3)[0][2]
# p.getBasePositionAndOrientation(robotId)
# p.getEulerFromQuaternion([0,0,0,1])
