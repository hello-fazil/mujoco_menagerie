from stretch_mujoco import StretchMujocoSimulator
from stretch_body.gamepad_controller import GamePadController
import time


robot_sim = StretchMujocoSimulator()
robot_sim.start()

gamepad = GamePadController()
gamepad.start()

button_mapping = {'top_pad_pressed':['wrist_pitch', 1, 0.1],
                  'bottom_pad_pressed':['wrist_pitch', -1, 0.1],
                  'left_pad_pressed':['wrist_roll', -1, 0.07],
                  'right_pad_pressed':['wrist_roll', 1, 0.07],
                  'right_shoulder_button_pressed':['wrist_yaw', -1, 0.1],
                  'left_shoulder_button_pressed':['wrist_yaw', 1, 0.1],
                  'top_button_pressed': ['stow', 0],
                  'left_button_pressed': ['dex_switch', 0],
                  'right_button_pressed': ['gripper',1, 0.01],
                  'bottom_button_pressed': ['gripper',-1, 0.01]
                  }

stick_mapping = {'right_stick_x': ('arm', 'inc'),
                 'right_stick_y': ('lift', 'inc'),
                 'left_stick_x': ('turn', 'scale'),
                 'left_stick_y': ('forward', 'scale')}

dex_switch = False
while True:
    time.sleep(1/15)
    gamepad_state = gamepad.get_state()
    for button in button_mapping.keys():
        if gamepad_state[button]:
            try:
                actuator_name, dir, k = button_mapping[button]
                pos = robot_sim.status[actuator_name]['pos'] + dir*k
                print(f"Moving {actuator_name} to {pos}")
                robot_sim.move_to(actuator_name, pos)
            except:
                pass

    if gamepad_state['left_button_pressed']:
        dex_switch = not dex_switch
        print(f"Setting dex_switch to {dex_switch}")
        if dex_switch:
           button_mapping['bottom_pad_pressed'][0] = 'head_tilt'
           button_mapping['top_pad_pressed'][0] = 'head_tilt'
           button_mapping['left_pad_pressed'][0] = 'head_pan'
           button_mapping['right_pad_pressed'][0] = 'head_pan'
        else:
           button_mapping['top_pad_pressed'][0] = 'wrist_pitch'
           button_mapping['bottom_pad_pressed'][0] = 'wrist_pitch'
           button_mapping['left_pad_pressed'][0] = 'wrist_roll'
           button_mapping['right_pad_pressed'][0] = 'wrist_roll'

    for stick in stick_mapping.keys():
        if abs(gamepad_state[stick])>0.001:
            actuator_name, prop = stick_mapping[stick]
            if prop == 'inc':
                pos = robot_sim.status[actuator_name]['pos'] + gamepad_state[stick]*0.3
                robot_sim.move_to(actuator_name, pos)
                print(f"Moving {actuator_name} to {pos}")
            if prop == 'scale':
                robot_sim.move_to(actuator_name, gamepad_state[stick])
    if abs(gamepad_state['left_stick_x']) < 0.001:
        robot_sim.move_to('turn', 0)
    if abs(gamepad_state['left_stick_y']) < 0.001:
        robot_sim.move_to('forward', 0)