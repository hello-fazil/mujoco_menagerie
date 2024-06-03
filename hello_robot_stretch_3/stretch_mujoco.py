import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("./scene.xml")
mujoco.viewer.launch(model)