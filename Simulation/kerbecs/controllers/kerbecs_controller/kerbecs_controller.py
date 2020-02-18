"""kerbecs_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
TIME_STEP = 64

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
hinges = []
thighs = []
shins = []
hingeNames = ['hinge_front_left', 'hinge_front_right', 'hinge_back_left', 'hinge_back_right']
thighNames = ['thigh_front_left', 'thigh_front_right', 'thigh_back_left', 'thigh_back_right']
shinNames = ['shin_front_left', 'shin_front_right', 'shin_back_left', 'shin_back_right']

for i in range(4):
    hinges.append(robot.getMotor(hingeNames[i]))
    hinges[i].setPosition(float('inf'))
    hinges[i].setVelocity(0.0)
    
    thighs.append(robot.getMotor(thighNames[i]))
    thighs[i].setPosition(float('inf'))
    thighs[i].setVelocity(0.0)
    
    shins.append(robot.getMotor(shinNames[i]))
    shins[i].setPosition(float('inf'))
    shins[i].setVelocity(0.0)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    leftSpeed = -1.0
    rightSpeed = -1.0

    # hinges[0].setVelocity(leftSpeed)
    # hinges[1].setVelocity(rightSpeed)
    # hinges[2].setVelocity(leftSpeed)
    # hinges[3].setVelocity(rightSpeed)
    
    # thighs[0].setVelocity(leftSpeed)
    # thighs[1].setVelocity(rightSpeed)
    # thighs[2].setVelocity(leftSpeed)
    # thighs[3].setVelocity(rightSpeed)
    
    shins[0].setVelocity(leftSpeed)
    shins[1].setVelocity(rightSpeed)
    shins[2].setVelocity(leftSpeed)
    shins[3].setVelocity(rightSpeed)
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
