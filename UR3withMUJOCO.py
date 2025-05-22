import glfw
if not glfw.init():
    print("GLFW init failed")
else:
    print("GLFW initialized successfully")
    glfw.terminate()
