import mujoco
import mujoco.viewer
import numpy as np

# Wczytanie modelu UR3, xml UR3 jest w pliku scene.xml
model = mujoco.MjModel.from_xml_path("/home/bartek/PycharmProjects/UR3_Ready/scene.xml")
data = mujoco.MjData(model)

# Podgląd UR3
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)

        # UR3 zero irl
        data.ctrl[1] = np.pi/2
        data.ctrl[3] = np.pi/2

        viewer.sync()


