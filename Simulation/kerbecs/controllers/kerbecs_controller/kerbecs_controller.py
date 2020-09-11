"""kerbecs_controller controller."""
"""
# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor
import socket
import math

#set up communication socket
# comm = socket.socket()
# print('socket created')

# port = 6665
# comm.bind(('', port))
# comm.listen(5)

# client, address = comm.accept()
# client.send(b'connected to kerbecs\n')

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
TIME_STEP = 64

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
hips = []
thighs = []
shins = []
hipNames = ['hip_front_left', 'hip_front_right', 'hip_back_left', 'hip_back_right']
thighNames = ['thigh_front_left', 'thigh_front_right', 'thigh_back_left', 'thigh_back_right']
shinNames = ['shin_front_left', 'shin_front_right', 'shin_back_left', 'shin_back_right']

for i in range(4):
    hips.append(robot.getMotor(hipNames[i]))
    hips[i].setPosition(float('inf'))
    hips[i].setVelocity(0.0)
    hips[i].getPositionSensor().enable(15)
    
    thighs.append(robot.getMotor(thighNames[i]))
    thighs[i].setPosition(float('inf'))
    thighs[i].setVelocity(0.0)
    
    shins.append(robot.getMotor(shinNames[i]))
    shins[i].setPosition(float('inf'))
    shins[i].setVelocity(0.0)

leftSpeed = -1.0
rightSpeed = -1.0

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    comm_out = ''
    
    #rotate shoulder joint like flaps
    # if hips[0].getPositionSensor().getValue() >= math.pi * 2.0 / 3.0:
        # leftSpeed = -1.0
        # rightSpeed = 1.0
        
    # if hips[0].getPositionSensor().getValue() <= math.pi/3.0:
        # leftSpeed = 1.0
        # rightSpeed = -1.0

    # hips[0].setVelocity(leftSpeed)
    # hips[1].setVelocity(rightSpeed)
    # hips[2].setVelocity(leftSpeed)
    # hips[3].setVelocity(rightSpeed)
    
    # thighs[0].setVelocity(leftSpeed)
    # thighs[1].setVelocity(rightSpeed)
    # thighs[2].setVelocity(leftSpeed)
    # thighs[3].setVelocity(rightSpeed)
    
    shins[0].setVelocity(leftSpeed)
    shins[1].setVelocity(rightSpeed)
    shins[2].setVelocity(leftSpeed)
    shins[3].setVelocity(rightSpeed)


    #create output string and send over socket
    for i in range(4):
        comm_out += (str(hips[i].getPositionSensor().getValue()) + '\t')
    
    comm_out += '\n'
    
    print(comm_out)
        
    # client.send(bytes(comm_out))
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
"""

from kerbecs import *

robot = Kerbecs()
robot.stand()
robot.crouch()
# robot.walk()