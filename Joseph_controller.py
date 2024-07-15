import os
import time
import logging
import serial
import pygame
from pygame.locals import JOYBUTTONDOWN, JOYHATMOTION

logging.basicConfig(filename="logs.txt", filemode="w", level=logging.INFO)

# Initialize Pygame
pygame.init()
pygame.joystick.init()

# Check for joystick
if pygame.joystick.get_count() == 0:
    print("No joystick connected.")
    exit()

# Initialize the joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Joystick Name: {joystick.get_name()}")
print(f"Number of Axes: {joystick.get_numaxes()}")
print(f"Number of Buttons: {joystick.get_numbuttons()}")


class ObstacleDetectionRoutine:
    def __init__(self, target_angle: int, remaining_turns: int):
        self.current_stage = self._stage_1
        self.turning = False
        self.ticks_before_avoiding_obstacle = 0
        self.ticks_after_clearing_obstacle = 0
        self.ticks_obstacle_length = 0
        self.ultrasound_sequence = []
        self.done = False
        self.direction = -1 if target_angle == 0 else 1 if remaining_turns > 1 else -1

    def __call__(self, controller: 'RobotController'):
        if self.done:
            controller.change_state(cruise_state)
            return
        if self.turning:
            self._axis_turn(controller)
        else:
            self.current_stage(controller)

    def _axis_turn(self, controller: 'RobotController'):
        if controller.axis_turn() <= controller.angle_error_margin:
            controller.reset_encoders()
            self.turning = False

    def _obstacle_passed(self, ultra_sound_value: int):
        if self.ultrasound_sequence and self.ultrasound_sequence[-1] != ultra_sound_value:
            self.ultrasound_sequence.append(ultra_sound_value)
            if self.ultrasound_sequence in [[0, 1, 0], [1, 0]]:
                self.ultrasound_sequence = []
                return True
        else:
            self.ultrasound_sequence.append(ultra_sound_value)
        return False

    def _stage_1(self, controller: 'RobotController'):
        self.ticks_before_avoiding_obstacle = controller.sensor_data["left_encoder_raw"]
        controller.target_angle += self.direction * 90
        self.current_stage = self._stage_2
        self.turning = True

    def _stage_2(self, controller: 'RobotController'):
        ultrasound = controller.sensor_data["left_ultrasound"] if self.direction == -1 else controller.sensor_data["right_ultrasound"]
        if self._obstacle_passed(ultrasound):
            self.ticks_after_clearing_obstacle = controller.sensor_data["left_encoder"]
            controller.target_angle += -self.direction * 90
            self.current_stage = self._stage_3
            self.turning = True
        else:
            controller.forward()

    def _stage_3(self, controller: 'RobotController'):
        ultrasound = controller.sensor_data["left_ultrasound"] if self.direction == -1 else controller.sensor_data["right_ultrasound"]
        if self._obstacle_passed(ultrasound):
            self.ticks_obstacle_length = controller.sensor_data["left_encoder"]
            controller.target_angle += -self.direction * 90
            self.current_stage = self._stage_4
            self.turning = True
        else:
            controller.forward()

    def _stage_4(self, controller: 'RobotController'):
        if controller.sensor_data["left_encoder"] >= self.ticks_after_clearing_obstacle:
            controller.total_ticks = self.ticks_before_avoiding_obstacle - self.ticks_obstacle_length
            controller.target_angle += self.direction * 90
            self.turning = True
            self.done = True
        else:
            controller.forward()


class RobotController:
    def __init__(self):
        self.TURNING_SPEED = 150
        self.LEFT_CRUISE_SPEED = 120
        self.RIGHT_CRUISE_SPEED = 120
        self.WHEEL_RADIUS = 35
        self.distance_per_tick = 0.021
        self.angle_error_margin = 1

        self.sonic_ser = self.init_serial('/dev/arduinoUltrasound')
        self.angle_ser = self.init_serial('/dev/arduinoSensors')
        self.motor_ser = self.init_serial('/dev/arduinoMotors')

        self.current_state = init_state
        self.state_history = []
        self.sensor_data = {}
        self.target_angle = 0
        self.angle_delta = 0
        self.turn_right_next = True
        self.total_ticks = 0
        self.number_of_turns = 0
        self.workspace_height = 0
        self.workspace_width = 0
        self.required_turns = 0
        self.homing_turns = 0
        self.homing = False
        self.mapping = False
        self.cached_speeds = (0, 0)

    @staticmethod
    def init_serial(port, baudrate=115200):
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(3)
        ser.flushInput()
        return ser

    def update(self):
        self.update_sensor_readings()
        self.current_state(self)
        self._controller_input()

    def _controller_input(self):
        for event in pygame.event.get():
            if event.type == JOYBUTTONDOWN:
                button = event.button
                if button in [5, 7]:
                    self.adjust_speed(button == 5, 'RIGHT_CRUISE_SPEED')
                elif button in [4, 6]:
                    self.adjust_speed(button == 4, 'LEFT_CRUISE_SPEED')

    def adjust_speed(self, increase, motor_side):
        change = 2 if increase else -2
        new_speed = getattr(self, motor_side) + change
        setattr(self, motor_side, new_speed)
        print(f"{motor_side} Speed: {new_speed}")

    def change_state(self, new_state):
        state_name = new_state.__name__ if hasattr(new_state, "__name__") else type(new_state).__name__
        logging.info(f"NEW STATE: {state_name}")
        self.state_history.append(state_name)
        self.current_state = new_state

    def get_angle_deviation(self):
        return abs(abs(self.sensor_data["angle"]) - abs(self.target_angle))

    def axis_turn(self):
        deviation = self.get_angle_deviation()
        if deviation > self.angle_error_margin:
            if self.sensor_data["angle"] > self.target_angle:
                self.send_speed(self.LEFT_CRUISE_SPEED, -self.RIGHT_CRUISE_SPEED)
            else:
                self.send_speed(-self.LEFT_CRUISE_SPEED, self.RIGHT_CRUISE_SPEED)
        return deviation

    def forward(self):
        deviation = self.get_angle_deviation()
        if deviation > self.angle_error_margin:
            if self.sensor_data["angle"] > self.target_angle:
                self.send_speed(self.LEFT_CRUISE_SPEED, 0)
            else:
                self.send_speed(0, self.RIGHT_CRUISE_SPEED)
        else:
            self.send_speed(self.LEFT_CRUISE_SPEED, self.RIGHT_CRUISE_SPEED)
        return deviation

    def send_speed(self, left_speed, right_speed):
        command = f"{right_speed},{left_speed}\n"
        self.motor_ser.write(command.encode())

    def update_sensor_readings(self):
        self.angle_ser.flushInput()
        angle_data = self.angle_ser.readline().decode('utf-8').strip()
        ultrasound_data = self.sonic_ser.readline().decode("utf-8").strip()
        angle_data_list = angle_data.split(",")
        ultrasound_data_list = ultrasound_data.split(",")
        if len(angle_data_list) == 3 and len(ultrasound_data_list) == 4:
            angle, right_encoder, left_encoder = map(float, angle_data_list)
            right_ultrasound, left_ultrasound, front_ultra_1, front_ultra_2 = map(int, ultrasound_data_list)
            self.sensor_data = {
                "angle": round(angle - self.angle_delta),
                "left_encoder": left_encoder - self.total_ticks,
                "left_encoder_raw": left_encoder,
                "left_ultrasound": left_ultrasound,
                "right_ultrasound": right_ultrasound,
                "front_ultrasound_1": front_ultra_1,
                "front_ultrasound_2": front_ultra_2,
                "right_encoder": right_encoder
            }

    def reset_encoders(self):
        self.total_ticks = self.sensor_data["left_encoder_raw"]
        self.send_speed(0, 0)
        time.sleep(0.5)

    def reset_angle(self):
        self.angle_delta = self.sensor_data["angle"]

    def get_tracked_distance(self):
        return self.distance_per_tick * self.sensor_data["left_encoder"]

    def halt(self):
        self.send_speed(0, 0)
        self.motor_ser.close()
        self.angle_ser.close()
        print("Exiting Program")

def init_state(controller: RobotController):
    if controller.sensor_data:
        controller.send_speed(0, 0)
        time.sleep(1)
        controller.change_state(map_state)

def map_state(controller: RobotController):
    if controller.mapping:
        controller.forward()
    for event in pygame.event.get():
        if event.type in [JOYBUTTONDOWN, JOYHATMOTION]:
            button = event.button if event.type == JOYBUTTONDOWN else None
            hat_value = event.value if event.type == JOYHATMOTION else None
            if hat_value:
                controller.mapping = True
                direction = 'forward' if hat_value[1] == 1 else 'backward'
                controller.send_speed(150, 150) if direction == 'forward' else controller.send_speed(-150, -150)
            elif button in [1, 2, 3, 0]:
                controller.process_mapping_button(button)

def process_mapping_button(controller: RobotController, button):
    if button == 1:
        controller.workspace_height = abs(controller.sensor_data["left_encoder"]) * controller.WHEEL_RADIUS
    elif button == 2:
        controller.workspace_width = abs(controller.sensor_data["left_encoder"]) * controller.WHEEL_RADIUS
    elif button == 3:
        controller.homing_turns = controller.number_of_turns
        controller.required_turns = controller.number_of_turns + 1
        controller.homing = True
        controller.change_state(homing_state)
    elif button == 0:
        controller.change_state(end_state)

def homing_state(controller: RobotController):
    if controller.homing:
        controller.number_of_turns = controller.homing_turns
    if controller.sensor_data["front_ultrasound_1"] == 0:
        controller.send_speed(-150, -150)
    else:
        controller.change_state(cruise_state)

def cruise_state(controller: RobotController):
    controller.forward()
    if controller.sensor_data["front_ultrasound_1"] != 0:
        controller.change_state(ObstacleDetectionRoutine(controller.target_angle, controller.required_turns))

def turn_state(controller: RobotController):
    if controller.axis_turn() <= controller.angle_error_margin:
        controller.reset_encoders()
        if controller.number_of_turns == controller.required_turns:
            controller.number_of_turns = 0
        else:
            controller.number_of_turns += 1
        controller.change_state(cruise_state)

def boost_state(controller: RobotController):
    if controller.forward() <= controller.angle_error_margin:
        time.sleep(1)
        controller.change_state(cruise_state)

def end_state(controller: RobotController):
    controller.halt()
    exit()

if __name__ == "__main__":
    controller = RobotController()
    try:
        while True:
            controller.update()
    except KeyboardInterrupt:
        controller.halt()
