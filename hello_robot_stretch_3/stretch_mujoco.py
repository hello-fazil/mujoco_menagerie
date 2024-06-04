import mujoco
import mujoco.viewer
from mujoco import MjModel, MjData
import time
import threading

class StretchMujocoSimulator:
    def __init__(self, 
                 scene_xml_path: str = "./scene.xml"):
        self.model = mujoco.MjModel.from_xml_path(scene_xml_path)
        self.data = mujoco.MjData(self.model)

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

                       'cam_d435i': {'rgb': None, 'depth': None},
                       'cam_d405': {'rgb': None, 'depth': None}, 
                      }
    
    def move_to(self, 
                actuator_name: str, 
                pos: float):
        self.data.actuator(actuator_name).ctrl = pos

    def move_by(self, 
                actuator_name: str, 
                pos: float):
        self.data.actuator(actuator_name).ctrl = self.status[actuator_name]['pos'] + pos
    
    def set_velocity(self, 
                     actuator_name: str, 
                     vel: float):
        raise NotImplementedError
    
    def pull_status(self):
        self.status['lift']['pos'] = self.data.actuator('lift').length[0]
        self.status['lift']['vel'] = self.data.actuator('lift').velocity[0]

        self.status['arm']['pos'] = self.data.actuator('arm').length[0]
        self.status['arm']['vel'] = self.data.actuator('arm').velocity[0]

        self.status['head_pan']['pos'] = self.data.actuator('head_pan').length[0]
        self.status['head_pan']['vel'] = self.data.actuator('head_pan').velocity[0]

        self.status['head_tilt']['pos'] = self.data.actuator('head_tilt').length[0]
        self.status['head_tilt']['vel'] = self.data.actuator('head_tilt').velocity[0]

        self.status['wrist_yaw']['pos'] = self.data.actuator('wrist_yaw').length[0]
        self.status['wrist_yaw']['vel'] = self.data.actuator('wrist_yaw').velocity[0]

        self.status['wrist_pitch']['pos'] = self.data.actuator('wrist_pitch').length[0]
        self.status['wrist_pitch']['vel'] = self.data.actuator('wrist_pitch').velocity[0]

        self.status['wrist_roll']['pos'] = self.data.actuator('wrist_roll').length[0]
        self.status['wrist_roll']['vel'] = self.data.actuator('wrist_roll').velocity[0]

        self.status['gripper']['pos'] = self.data.actuator('gripper').length[0]
        self.status['gripper']['vel'] = self.data.actuator('gripper').velocity[0]

    def robot_control_callback(self, model: MjModel, data: MjData):
        self.data = data
        self.model = model
        self.pull_status()

    def __run(self):
        mujoco.set_mjcb_control(self.robot_control_callback)
        mujoco.viewer.launch(self.model)
    
    def start(self):
        threading.Thread(target=self.__run).start()

if __name__ == "__main__":
    stretch_mujoco_sim = StretchMujocoSimulator()
    stretch_mujoco_sim.start()