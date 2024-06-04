import mujoco
import mujoco.viewer
from mujoco import MjModel, MjData
import time
import threading
import cv2

class StretchMujocoSimulator:
    def __init__(self, 
                 scene_xml_path: str = "./scene.xml"):
        self.mjmodel = mujoco.MjModel.from_xml_path(scene_xml_path)
        self.mjdata = mujoco.MjData(self.mjmodel)

        self.rgb_renderer = mujoco.Renderer(self.mjmodel, height=480, width=640)
        self.depth_renderer = mujoco.Renderer(self.mjmodel, height=480, width=640)
        self.depth_renderer.enable_depth_rendering()

        self.status = {
                       'base': {'x_vel': None,'theta_vel': None},
                       'lift': {'pos': None,'vel': None},
                       'arm': {'pos': None,'vel': None},
                       'head_pan': {'pos': None,'vel': None},
                       'head_tilt': {'pos': None,'vel': None},
                       'wrist_yaw': {'pos': None,'vel': None},
                       'wrist_pitch': {'pos': None,'vel': None},
                       'wrist_roll': {'pos': None,'vel': None},
                       'gripper': {'pos': None,'vel': None},
                      }
    
    def home(self)->None:
        self.mjdata.ctrl = self.mjmodel.keyframe('home').ctrl

    def move_to(self, 
                actuator_name: str, 
                pos: float)->None:
        self.mjdata.actuator(actuator_name).ctrl = pos

    def move_by(self, 
                actuator_name: str, 
                pos: float)->None:
        self.mjdata.actuator(actuator_name).ctrl = self.status[actuator_name]['pos'] + pos
    
    def set_velocity(self, 
                     actuator_name: str, 
                     vel: float)->None:
        raise NotImplementedError
    
    def pull_status(self)->None:
        self.status['lift']['pos'] = self.mjdata.actuator('lift').length[0]
        self.status['lift']['vel'] = self.mjdata.actuator('lift').velocity[0]

        self.status['arm']['pos'] = self.mjdata.actuator('arm').length[0]
        self.status['arm']['vel'] = self.mjdata.actuator('arm').velocity[0]

        self.status['head_pan']['pos'] = self.mjdata.actuator('head_pan').length[0]
        self.status['head_pan']['vel'] = self.mjdata.actuator('head_pan').velocity[0]

        self.status['head_tilt']['pos'] = self.mjdata.actuator('head_tilt').length[0]
        self.status['head_tilt']['vel'] = self.mjdata.actuator('head_tilt').velocity[0]

        self.status['wrist_yaw']['pos'] = self.mjdata.actuator('wrist_yaw').length[0]
        self.status['wrist_yaw']['vel'] = self.mjdata.actuator('wrist_yaw').velocity[0]

        self.status['wrist_pitch']['pos'] = self.mjdata.actuator('wrist_pitch').length[0]
        self.status['wrist_pitch']['vel'] = self.mjdata.actuator('wrist_pitch').velocity[0]

        self.status['wrist_roll']['pos'] = self.mjdata.actuator('wrist_roll').length[0]
        self.status['wrist_roll']['vel'] = self.mjdata.actuator('wrist_roll').velocity[0]

        self.status['gripper']['pos'] = self.mjdata.actuator('gripper').length[0]
        self.status['gripper']['vel'] = self.mjdata.actuator('gripper').velocity[0]
    
    def pull_camera_data(self)->dict:
        data = {}
        self.rgb_renderer.update_scene(self.mjdata,'d405_rgb')
        self.depth_renderer.update_scene(self.mjdata,'d405_rgb')

        data['cam_d405_rgb'] = self.rgb_renderer.render()
        data['cam_d405_depth'] = self.depth_renderer.render()

        self.rgb_renderer.update_scene(self.mjdata,'d435i_camera_rgb')
        self.depth_renderer.update_scene(self.mjdata,'d435i_camera_rgb')
        data['cam_d435i_rgb'] = self.rgb_renderer.render()
        data['cam_d435i_depth'] = self.depth_renderer.render()

        self.rgb_renderer.update_scene(self.mjdata,'nav_camera_rgb')
        data['cam_nav_rgb'] = self.rgb_renderer.render()
        return data

    def robot_control_callback(self, model: MjModel, data: MjData)->None:
        self.mjdata = data
        self.mjmodel = model
        self.pull_status()

    def __run(self)->None:
        mujoco.set_mjcb_control(self.robot_control_callback)
        mujoco.viewer.launch(self.mjmodel)
    
    def start(self)->None:
        threading.Thread(target=self.__run).start()
        time.sleep(0.5)
        self.home()

if __name__ == "__main__":
    robot_sim = StretchMujocoSimulator()
    robot_sim.start()
    # display camera feeds
    while True:
        camera_data = robot_sim.pull_camera_data()
        cv2.imshow('cam_d405_rgb', camera_data['cam_d405_rgb'])
        cv2.imshow('cam_d405_depth', camera_data['cam_d405_depth'])
        cv2.imshow('cam_d435i_rgb', camera_data['cam_d435i_rgb'])
        cv2.imshow('cam_d435i_depth', camera_data['cam_d435i_depth'])
        cv2.imshow('cam_nav_rgb', camera_data['cam_nav_rgb'])
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break