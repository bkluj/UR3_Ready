import mujoco
import mujoco.viewer
import numpy as np

# Wczytanie modelu UR3, xml UR3 jest w pliku scene.xml
model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model) 

# Podgląd UR3
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        data.ctrl[0] = np.sin(data.time * 2) #ustawia konkretną os od 0-5
        viewer.sync()
