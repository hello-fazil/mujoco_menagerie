import mujoco
import mujoco.viewer
from mujoco import MjModel, MjData
import time

class StretchMujocoSimulator:
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path("./scene.xml")
        self.data = mujoco.MjData(self.model)
    
    def get_state_vector(self):
        return {'time':self.data.time, 
                'qpos':self.data.qpos, 
                'qvel':self.data.qvel, 
                'act': self.data.act}

    def robot_control_callback(self, model: MjModel, data: MjData):
        self.data = data
        print(self.get_state_vector())

    def run(self):
        mujoco.set_mjcb_control(self.robot_control_callback)
        mujoco.viewer.launch(self.model)


if __name__ == "__main__":
    stretch_mujoco_sim = StretchMujocoSimulator()
    stretch_mujoco_sim.run()