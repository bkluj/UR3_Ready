import mujoco
import glfw
import numpy as np
import cv2
import time

# === Inicjalizacja GLFW i kontekstu OpenGL ===
if not glfw.init():
    raise Exception("GLFW init failed")

# Tworzymy niewidoczne okno OpenGL (do renderowania MuJoCo)
glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
window = glfw.create_window(640, 480, "MuJoCo Offscreen", None, None)
glfw.make_context_current(window)

# === Ładowanie modelu MuJoCo ===
model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)  # wstępne przeliczenie pozycji

# === Kamera zdefiniowana w pliku XML ===
camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "top")
cam = mujoco.MjvCamera()
cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
cam.fixedcamid = camera_id

# === Przygotowanie renderera ===
width, height = 640, 480
viewport = mujoco.MjrRect(0, 0, width, height)
rgb = np.zeros((height, width, 3), dtype=np.uint8)
depth = np.zeros((height, width), dtype=np.float32)

option = mujoco.MjvOption()
scene = mujoco.MjvScene(model, maxgeom=1000)
context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)

# === Pętla renderująca ===
while not glfw.window_should_close(window):
    mujoco.mj_step(model, data)  # wykonaj krok symulacji

    # Aktualizuj scenę i renderuj kamerę
    mujoco.mjv_updateScene(model, data, option, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scene)
    mujoco.mjr_render(viewport, scene, context)
    mujoco.mjr_readPixels(rgb, depth, viewport, context)

    # RGB → BGR (OpenCV) i wyświetl
    rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    cv2.imshow("MuJoCo RGB", rgb_bgr)

    # Depth – normalizacja i wyświetlenie
    depth_norm = (depth - np.min(depth)) / (np.max(depth) - np.min(depth) + 1e-6)
    cv2.imshow("MuJoCo Depth", depth_norm)

    # ESC = wyjście
    if cv2.waitKey(1) == 27:
        break

    time.sleep(1 / 30)  # 30 FPS (opcjonalne)

# === Zakończ ===
cv2.destroyAllWindows()
glfw.terminate()
