#!/usr/bin/python
# -*- coding: utf-8 -*-

####################### Imports

from roverbot_lib import *
from PIL import Image, ImageOps
import time
import keyboard

####################### Initials
sceneParameters = SceneParameters()
robotParameters = RobotParameters()
robotParameters.driveType = 'differential'  # specify if using differential (currently differential is only supported)
roverBotSim = VREP_RoverRobot('127.0.0.1', robotParameters, sceneParameters)

rotation = 0.5
rotation_counter = 0

##green is around 28, 92, 28


####################### Methods
def take_pic():
    resolution, im = roverBotSim.GetCameraImage()
    image = [im[x:x+3] for x in range(0, len(im), 3)]
    data = np.zeros((480, 640, 3), dtype=np.uint8)

    for x in range(480):
        data[x,0:640] = image[0+(x*640):640+(x*640)]

    img = Image.fromarray(data, 'RGB')
    print(image)
    img = ImageOps.flip(img)
    img.save('my.png')
    img.show()

##################### MAIN SCRIPT

roverBotSim.StartSimulator()
##START
# take_pic()
while True:

    if rotation_counter >= 360:
        rotation_counter = 0


    roverBotSim.SetTargetVelocities(10,rotation*0)
    roverBotSim.UpdateObjectPositions()
    rotation_counter += rotation




    ###QUIT
    if keyboard.is_pressed('q'):
        print('You Pressed A Key!')
        break

##END
roverBotSim.StopSimulator()
