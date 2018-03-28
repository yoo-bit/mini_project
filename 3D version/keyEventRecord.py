import pybullet as p
physicsClient = p.connect(p.GUI_SERVER)
while 1:
    print(p.getKeyboardEvents())
