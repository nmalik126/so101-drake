import grpc
import pipeline_pb2
import pipeline_pb2_grpc
import logging
from concurrent.futures import ThreadPoolExecutor
import pyrealsense2 as rs
import numpy as np


class RealsenseProcessor:

    hres, vres = (848, 480)
    fps = 5
    depth_preset = 5  # Default - 0, High Accuracy - 3, High Density - 4, Medium Density - 5

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
            cls.depth_scale = float(depth_sensor.get_depth_scale())
    
            # initialize frame aligner
            align_to = rs.stream.color
            cls.align = rs.align(align_to)
    
            logging.info(f"realsense processor started")
            
        except Exception:
            logging.error(f"realsense processor start unsuccesssful, stopping...")
            cls.stop()
            raise

    @classmethod
    def stop(cls):
        logging.info("stopping realsense processor...")
        cls.pipeline.stop()
        logging.info("stopped realsense processor")

    @classmethod
    def get_image(cls):
        frames = cls.pipeline.wait_for_frames()
        aligned_frames = cls.align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            logging.warning(f"failed to get frame")
            return None
        logging.debug(f"got frame")

        depth_image = np.asanyarray(depth_frame.get_data(), dtype=np.float32) * cls.depth_scale
        color_image = np.asanyarray(color_frame.get_data(), dtype=np.uint8)

        return depth_image, color_image


class PipelineServicer(pipeline_pb2_grpc.PipelineServicer):

    def GetImage(self, request: pipeline_pb2.Request, context):
        res = RealsenseProcessor.get_image()
        if res is None:
            return pipeline_pb2.ImagePairResponse(
                response=pipeline_pb2.Response(
                    id=request.id,
                    success=False
                )
            )
        else:
            depth_image, color_image = res
            return pipeline_pb2.ImagePairResponse(
                response=pipeline_pb2.Response(
                    id=request.id,
                    success=True
                ),
                imagePair=pipeline_pb2.ImagePair(
                    depth_data=depth_image.tobytes(),
                    color_data=color_image.tobytes()
                )
            )


def serve():
    server = grpc.server(ThreadPoolExecutor(max_workers=10))
    pipeline_pb2_grpc.add_PipelineServicer_to_server(
        PipelineServicer(), server
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
    except KeyboardInterrupt:
        logging.info("User keyboard interrupt")
    except Exception as e:
        logging.exception(f"An unexpected error occured: {e}")
    finally:
        RealsenseProcessor.stop()


if __name__ == "__main__":
    main()
