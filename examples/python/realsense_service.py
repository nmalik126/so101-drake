import grpc
import logging
from concurrent.futures import ThreadPoolExecutor
import service_pb2
import service_pb2_grpc
import pyrealsense2 as rs
import numpy as np
from multiprocessing.shared_memory import SharedMemory
import time


class RealsenseProcessor:

    hres, vres = (848, 480)
    fps = 15
    depth_preset = 5 # Default - 0, High Accuracy - 3, High Density - 4, Medium Density - 5
    depth_shm_name = "depth_img"
    color_shm_name = "color_img"

    @classmethod
    def start(cls):
        try:
            logging.info("starting realsense processor...")
            # initialize pipeline
            cls.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, cls.hres, cls.vres, rs.format.z16, cls.fps)
            config.enable_stream(rs.stream.color, cls.hres, cls.vres, rs.format.bgr8, cls.fps)
            profile = cls.pipeline.start(config)
    
            # set depth preset and get scale factor
            device = profile.get_device()
            depth_sensor = device.first_depth_sensor()
            depth_sensor.set_option(rs.option.visual_preset, cls.depth_preset)
            cls.depth_scale = depth_sensor.get_depth_scale()
    
            # initialize frame aligner
            align_to = rs.stream.color
            cls.align = rs.align(align_to)
    
            # init shared memory
            mock_depth_img = np.zeros((cls.vres, cls.hres), dtype=np.float32)
            mock_color_img = np.zeros((cls.vres, cls.hres, 3), dtype=np.uint8)
            cls.depth_shm = SharedMemory(
                name=cls.depth_shm_name,
                create=True, 
                size=mock_depth_img.nbytes
            )
            cls.color_shm = SharedMemory(
                name=cls.color_shm_name,
                create=True, 
                size=mock_color_img.nbytes
            )
            cls.depth_image = np.ndarray(
                mock_depth_img.shape, 
                dtype=mock_depth_img.dtype, 
                buffer=cls.depth_shm.buf
            )
            cls.color_image = np.ndarray(
                mock_color_img.shape, 
                dtype=mock_color_img.dtype, 
                buffer=cls.color_shm.buf
            )
            logging.info(f"realsense processor started")
        except Exception:
            logging.error(f"realsense processor start unsuccesssful, stopping...")
            cls.stop()
            raise

    @classmethod
    def stop(cls):
        logging.info("stopping realsense processor...")
        cls.pipeline.stop()
        for shm in (cls.depth_shm, cls.color_shm):
            shm.close()
            try:
                shm.unlink()
            except FileNotFoundError:
                pass
        logging.info("stopped realsense processor")

    @classmethod
    def get_image(cls) -> bool:
        frames = cls.pipeline.wait_for_frames()
        aligned_frames = cls.align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            logging.warning(f"failed to get frame")
            return False
        logging.debug(f"got frame")

        cls.depth_image[:] = (np.asanyarray(
            depth_frame.get_data(), dtype=np.float32) * cls.depth_scale)[:]
        cls.color_image[:] = np.asanyarray(color_frame.get_data())[:]

        return True


class UpdateServicer(service_pb2_grpc.UpdateServicer):

    def GetImage(self, request, context):
        success = RealsenseProcessor.get_image()
        return service_pb2.Response(
            id=request.id,
            success=success
        )


def serve():
    server = grpc.server(ThreadPoolExecutor(max_workers=10))
    service_pb2_grpc.add_UpdateServicer_to_server(
        UpdateServicer(), server
    )
    server.add_insecure_port("127.0.0.1:50051")
    server.start()
    server.wait_for_termination()


def main():
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )

    try:
        RealsenseProcessor.start()
        serve()
        # while True:
        #     RealsenseProcessor.get_image()
        #     time.sleep(1.0)
    except KeyboardInterrupt:
        logging.info("User keyboard interrupt")
    except Exception as e:
        logging.exception(f"An unexpected error occured: {e}")
    finally:
        RealsenseProcessor.stop()


if __name__ == "__main__":
    main()
