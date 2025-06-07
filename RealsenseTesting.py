import pyrealsense2 as rs
import numpy as np
import cv2

# pipeline
pipeline = rs.pipeline()
config = rs.config()

# streaming depth
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# start
pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            continue

        # conversion depth to numpy
        depth_image = np.asanyarray(depth_frame.get_data())

        # depth coloring
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03),
            cv2.COLORMAP_JET
        )

        cv2.imshow('Depth Stream', depth_colormap)

        # close ESC
        if cv2.waitKey(1) == 27:
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
