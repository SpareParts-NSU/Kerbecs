import math
from controller import Robot, Motor
from constants import *


class Kerbecs:

    MOTOR_NAMES = [ 'hip_front_left', 
                    'hip_front_right', 
                    'thigh_front_left', 
                    'thigh_front_right', 
                    'shin_front_left', 
                    'shin_front_right', 
                    'hip_back_left', 
                    'hip_back_right', 
                    'thigh_back_left', 
                    'thigh_back_right', 
                    'shin_back_left', 
                    'shin_back_right'    ]


    gait_name = ["trot", "walk", "gallop(transverse)", "canter", "pace", "bound", "pronk"]
    

    gait_phase_shift = [
                        [0, 0.5, 0, 0.5],   # trot
                        [0, 0.5, 0.25, 0.75],   # walk
                        [0, 0.1, 0.6, 0.5], # gallop  (transverse)
                        [0, 0.3, 0, 0.7],   # canter
                        [0, 0.5, 0.5, 0],   # pace
                        [0, 0, 0.5, 0.5],   # bound
                        [0, 0, 0, 0]    # pronk
                        ] 

    gait_setup = [[FRONT_LEFT_2, FRONT_LEFT_3],
                    [FRONT_RIGHT_2, FRONT_RIGHT_3],
                    [BACK_RIGHT_2, BACK_RIGHT_3],
                    [BACK_LEFT_2, BACK_LEFT_3]]

    motors = []
    position_sensors = []

    step_count = 0

    robot = None

    def __init__(self):
        
        self.robot = Robot()

        for i in range(len(self.MOTOR_NAMES)):
            self.motors.append(self.robot.getMotor(self.MOTOR_NAMES[i]))
            assert(self.motors[i])
            self.position_sensors.append(self.motors[i].getPositionSensor())
            assert(self.position_sensors[i])
            self.position_sensors[i].enable(SIMULATION_STEP_DURATION)
    

    def set_motor_position(self, motor_id, value):
        """
        Set motor position    
        """
        self.motors[motor_id].setPosition(value)
    
    def getMotorPosition(self, motor_id):
        """
        Get motor position
        """
        return self.position_sensors[motor_id].getValue()


    def wait(self, n):
        """
        Run simulation for n seconds
        """
        num = n / (SIMULATION_STEP_DURATION / 1000.0)

        for i in range(int(num)):
            self.robot.step(SIMULATION_STEP_DURATION)


    def stand(self):
        """
        Stand upright
        """
        self.set_motor_position(FRONT_LEFT_1, 0)
        self.set_motor_position(FRONT_RIGHT_1, 0)
        self.set_motor_position(BACK_LEFT_1, 0)
        self.set_motor_position(BACK_RIGHT_1, 0)

        self.wait(1)

    
    def crouch(self):
        """
        Bend lengs to a walking position
        """
        self.set_motor_position(FRONT_LEFT_3, math.pi/2)
        self.set_motor_position(FRONT_LEFT_2, -math.pi/4)
        self.set_motor_position(FRONT_RIGHT_3, math.pi/2)
        self.set_motor_position(FRONT_RIGHT_2, -math.pi/4)
        self.set_motor_position(BACK_LEFT_3, math.pi/2)
        self.set_motor_position(BACK_LEFT_2, -math.pi/4)
        self.set_motor_position(BACK_RIGHT_3, math.pi/2)
        self.set_motor_position(BACK_RIGHT_2, -math.pi/4)

        self.wait(1)


    def compute_walking_position(self, motor_positions, current_time, gait_frequency, gait_type, leg_id,
                                stride_length_factor, backwards):
        """
        Compute positions of the three motors according to the current time
        """

        ### start calculating inverse kinematics to compute leg position        
        frequency = gait_frequency

        # modulo to find time between time periods
        current_time = current_time%(1.0/frequency)

        # reverse time sequence for backwards walk
        if backwards:
            current_time = (1.0/frequency) - current_time
        
        # ellipsoid parameters
        a = 0.95 * L1 * stride_length_factor
        h = 0
        k = -(L1 + L2 / 2.0)
        b = -k - math.sqrt(L1 * L2 + L2 * L2)

        # compute ellipsoid points
        x = h + a * math.cos(2 * math.pi * frequency * current_time + self.gait_phase_shift[gait_type][leg_id] * 2 * math.pi)
        y = k + b * math.sin(2 * math.pi * frequency * current_time + self.gait_phase_shift[gait_type][leg_id] * 2 * math.pi)

        # compute angle A2
        A2 = math.acos((x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2))

        # compute angle A1
        A1 = math.acos(((L1 + L2 * math.cos(A2)) * x - (-L2 * math.sin(A2)) * y) / (pow(L1 + L2 * math.cos(A2), 2) + pow(-L2 * math.sin(A2), 2)))

        # subtract 2PI
        A1 = math.pi / 2 - A1

        motor_positions[0] = A1
        motor_positions[1] = A2

    
    def walk(self):
        gait_type = 1

        backwards = False   # robot direction

        # stride length parameters
        slf = 1; slf_min = 0; slf_max = 1
        stride_length_factor = [slf, slf, slf, slf]

        # frequency parameters
        freq_min = 0.4; freq_max = 2
        frequency = 1.5; freq_offset = 0.2

        # turn amount parameters
        turn_amount_factor = [0, 0, 0, 0]
        turn_amount_min = -0.6; turn_amount_max = 0.6
        turn_amount_offset = 0.6
        # turn_amount_min = -1, turn_amount_max = 1
        # turn_amount_offset = 0.1
        # According to Robotis, a turn offset of 0.6 gives a good result

        display_info = False

        while(True):

            motor_positions = [0, 0]
            # compute motor positions for each leg
            for leg_id in range(4):
                self.compute_walking_position(motor_positions, (self.step_count * (SIMULATION_STEP_DURATION / 1000)), frequency, gait_type, leg_id, stride_length_factor[leg_id], backwards)
                self.set_motor_position(self.gait_setup[leg_id][0], motor_positions[0])
                self.set_motor_position(self.gait_setup[leg_id][1], motor_positions[1])
                
            print(list(sensor.getValue() for sensor in self.position_sensors))
            self.robot.step(SIMULATION_STEP_DURATION)
            self.step_count += 1