import mujoco
import mujoco.viewer

# Wczytanie modelu UR3, xml UR3 jest w pliku scene.xml
model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)

# Podgląd UR3
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
