import mujoco
import glfw
import numpy as np
import cv2
import time

# === Inicjalizacja OpenGL (GLFW) ===
if not glfw.init():
    raise Exception("GLFW init failed")

glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
window = glfw.create_window(640, 480, "MuJoCo Offscreen", None, None)
glfw.make_context_current(window)

# === Wczytaj model i dane MuJoCo ===
model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)

# === Ustawienia kamery ===
camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "top")
if camera_id == -1:
    raise RuntimeError("❌ Kamera 'top' nie istnieje w scene.xml")

cam = mujoco.MjvCamera()
cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
cam.fixedcamid = camera_id

# === Bufory renderowania ===
width, height = 640, 480
viewport = mujoco.MjrRect(0, 0, width, height)
rgb = np.zeros((height, width, 3), dtype=np.uint8)
depth = np.zeros((height, width), dtype=np.float32)
option = mujoco.MjvOption()
scene = mujoco.MjvScene(model, maxgeom=1000)
context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)

# === Pętla renderująca na żywo ===
fps_timer = time.time()
frame_count = 0

while not glfw.window_should_close(window):
    # --- krok symulacji ---
    mujoco.mj_step(model, data)

    # --- aktualizacja kamery i render ---
    mujoco.mjv_updateScene(model, data, option, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scene)
    mujoco.mjr_render(viewport, scene, context)
    mujoco.mjr_readPixels(rgb, depth, viewport, context)

    # --- RGB image ---
    rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    cv2.imshow("MuJoCo RGB", rgb_bgr)

    # --- Depth image ---
    depth_norm = (depth - np.min(depth)) / (np.max(depth) - np.min(depth) + 1e-6)
    cv2.imshow("MuJoCo Depth", depth_norm)

    # --- ESC = wyjście ---
    if cv2.waitKey(1) == 27:
        break

    # --- FPS licznik (opcjonalnie) ---
    frame_count += 1
    if time.time() - fps_timer >= 1.0:
        print(f"FPS: {frame_count}")
        frame_count = 0
        fps_timer = time.time()

    # --- Można dodać: time.sleep(1/30) dla stabilnego 30 FPS ---
    # time.sleep(1 / 30)

cv2.destroyAllWindows()
glfw.terminate()
