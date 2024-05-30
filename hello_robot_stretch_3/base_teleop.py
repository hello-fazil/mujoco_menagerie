import cv2
import mujoco
from pynput import keyboard

model = mujoco.MjModel.from_xml_path("./scene.xml")
renderer = mujoco.Renderer(model, height=480, width=640)
data = mujoco.MjData(model)
data.actuator('lift').ctrl = 1.0 # Valid names: ['arm_extend', 'forward', 'grip', 'head_pan', 'head_tilt', 'lift', 'turn', 'wrist_yaw']

keys = {'up': False, 'down': False, 'left': False, 'right': False, 'esc': False}

def on_press(key):
    if key == keyboard.Key.up:
        keys['up'] = True
    elif key == keyboard.Key.down:
        keys['down'] = True
    elif key == keyboard.Key.left:
        keys['left'] = True
    elif key == keyboard.Key.right:
        keys['right'] = True

def on_release(key):
    if key == keyboard.Key.up:
        keys['up'] = False
    elif key == keyboard.Key.down:
        keys['down'] = False
    elif key == keyboard.Key.left:
        keys['left'] = False
    elif key == keyboard.Key.right:
        keys['right'] = False
    elif key == keyboard.Key.esc:
        keys['esc'] = True

listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
    # suppress=True)
listener.start()

print("Press 'escape' to exit")
print("Use arrow keys to teleop base")
print("Note: simulation time seems to be much slower. All motions happen slowly.")
apply_translation_vel = -100.0
apply_rotational_vel = -100.0
while not keys['esc'] and cv2.waitKey(1) & 0xFF != ord('q'):
    translation_vel = apply_translation_vel * int(keys['up']) + -1.0 * apply_translation_vel * int(keys['down'])
    rotational_vel = apply_rotational_vel * int(keys['left']) + -1.0 * apply_rotational_vel * int(keys['right'])
    data.actuator('forward').ctrl = translation_vel
    data.actuator('turn').ctrl = rotational_vel
    mujoco.mj_step(model, data) # nstep=100
    renderer.update_scene(data)
    cv2.imshow('Simulation', renderer.render())
