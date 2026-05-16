import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
from scipy.spatial.transform import Rotation


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

# get calibration
intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
camera_matrix = np.array([
    [intrinsics.fx, 0, intrinsics.ppx],
    [0, intrinsics.fy, intrinsics.ppy],
    [0, 0, 1],
])
distCoeffs = np.asarray(intrinsics.coeffs)
extrinsics = np.load('../calibrations/cam_to_world.npy')

# define object reference model
box_mesh = o3d.geometry.TriangleMesh.create_box(0.0318, 0.0478, 0.0280)
box_pcd = box_mesh.sample_points_uniformly(number_of_points=25000)
box_pcd = box_mesh.sample_points_poisson_disk(number_of_points=5000, pcl=box_pcd)
box_pcd.translate([-0.0318 / 2, -0.0478 / 2, 0.005])
box_obb = box_pcd.get_oriented_bounding_box()
R2, C2 = np.asarray(box_obb.R), np.asarray(box_obb.center)

try:
    while True:
        # get rgbd image
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(color_image), 
            o3d.geometry.Image(depth_image),
            depth_scale=1/depth_scale,
            depth_trunc=1.0,
            convert_rgb_to_intensity=False,
        )

        # create pointcloud
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            image=rgbd_image,
            intrinsic=o3d.camera.PinholeCameraIntrinsic(
                width=848,
                height=480,
                fx=camera_matrix[0, 0],
                fy=camera_matrix[1, 1],
                cx=camera_matrix[0, 2],
                cy=camera_matrix[1, 2],
            ),
            extrinsic=extrinsics,
            project_valid_depth_only=True
        )
        pcd = pcd.crop(
            o3d.geometry.AxisAlignedBoundingBox(
                min_bound=(-0.2, 0.0, 0.01), 
                max_bound=(-0.05, 0.2, 0.05),
            )
        )

        # compute initial alignment
        pcd_obb = pcd.get_oriented_bounding_box()

        R1, C1 = np.asarray(pcd_obb.R), np.asarray(pcd_obb.center)
        R_rel = R2 @ R1.T
        t_rel = C2 - R_rel @ C1
        T_obb = np.eye(4)
        T_obb[:3, :3] = R_rel
        T_obb[:3, 3] = t_rel

        # perform local pose refinement
        reg_p2p = o3d.pipelines.registration.registration_icp(
            pcd, box_pcd, 0.02, T_obb,
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )
        T = np.linalg.inv(reg_p2p.transformation)
        rot = Rotation.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)
        trans = T[:3, 3]
        print(T)
        print()

        # display depth image
        colorizer = rs.colorizer()
        colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())
        cv2.namedWindow('RealSense Depth', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense Depth', colorized_depth)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()