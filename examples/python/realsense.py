import pyrealsense2 as rs
import numpy as np
from multiprocessing.shared_memory import SharedMemory


def main():
    try:
        # initialize pipeline
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 15)
        config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 15)
        profile = pipeline.start(config)

        # set depth preset and get scale factor
        device = profile.get_device()
        depth_sensor = device.first_depth_sensor()
        depth_sensor.set_option(rs.option.visual_preset, 5) # Default - 0, High Accuracy - 3, High Density - 4, Medium Density - 5
        depth_scale = depth_sensor.get_depth_scale()

        # initialize frame aligner
        align_to = rs.stream.color
        align = rs.align(align_to)

        # init shared memory
        mock_depth_img = np.zeros((480, 848), dtype=np.float32)
        mock_color_img = np.zeros((480, 848, 3), dtype=np.uint8)
        depth_shm = SharedMemory(
            name="depth_img",
            create=True, 
            size=mock_depth_img.nbytes
        )
        color_shm = SharedMemory(
            name="color_img",
            create=True, 
            size=mock_color_img.nbytes
        )
        depth_image = np.ndarray(
            mock_depth_img.shape, 
            dtype=mock_depth_img.dtype, 
            buffer=depth_shm.buf
        )
        color_image = np.ndarray(
            mock_color_img.shape, 
            dtype=mock_color_img.dtype, 
            buffer=color_shm.buf
        )

        print("starting frame acq loop...")
        while True:
            # get rgbd image
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
    
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
    
            depth_image[:] = (np.asanyarray(depth_frame.get_data(), dtype=np.float32) * depth_scale)[:]
            color_image[:] = np.asanyarray(color_frame.get_data())[:]
    except KeyboardInterrupt:
        print("User keyboard interrupt")
    finally:
        pipeline.stop()
        for shm in (depth_shm, color_shm):
            shm.close()
            try:
                shm.unlink()
            except FileNotFoundError:
                pass


if __name__ == "__main__":
    main()
