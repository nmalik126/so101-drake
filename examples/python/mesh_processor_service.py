import numpy as np
from pathlib import Path
from multiprocessing.shared_memory import SharedMemory
from multiprocessing import resource_tracker
import logging
import open3d as o3d
import pinocchio as pin
from scipy.spatial.transform import Rotation
import grpc
import service_pb2_grpc
import service_pb2
import uuid
import time


class MeshProcessor:

    hres, vres = (848, 480)
    depth_shm_name = "depth_img"
    color_shm_name = "color_img"
    project_dir = Path("/home/noor/so101-drake")

    T_world_base = np.eye(4)
    rotvec = np.array([0, 0, 1]) * np.pi/2
    T_world_base[:3, :3] = Rotation.from_rotvec(rotvec).as_matrix()
    T_world_base[:3, 3] = [0, -0.1775, 0.0074]

    @classmethod
    def start(cls):
        try:
            logging.info("starting mesh processor...")
            cls.camera_matrix = np.load(cls.project_dir / "assets" / "intrinsics.npy")
            cls.extrinsics = np.load(cls.project_dir / "assets" / "cam_to_world.npy")
            cls._init_shm()
            cls._init_meshes()
            logging.info(f"mesh processor started")
        except Exception:
            logging.error(f"mesh processor start unsuccesssful, stopping...")
            cls.stop()
            raise

    @classmethod
    def _init_shm(cls):
        mock_depth_img = np.zeros((cls.vres, cls.hres), dtype=np.float32)
        mock_color_img = np.zeros((cls.vres, cls.hres, 3), dtype=np.uint8)
        cls.depth_shm = SharedMemory(name=cls.depth_shm_name)
        cls.color_shm = SharedMemory(name=cls.color_shm_name)
        resource_tracker.unregister(cls.depth_shm._name, "shared_memory")
        resource_tracker.unregister(cls.color_shm._name, "shared_memory")
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

    @classmethod
    def _init_meshes(cls):
        urdf_path = cls.project_dir / "models" / "SO101" / "so101_new_calib.urdf"
        
        cls.model = pin.buildModelFromUrdf(urdf_path)
        cls.collision_model = pin.buildGeomFromUrdf(
            cls.model,
            urdf_path,
            pin.GeometryType.COLLISION,
            package_dirs=[cls.project_dir / "models" / "SO101"]
        )

        cls.data = cls.model.createData()
        cls.collision_data = cls.collision_model.createData()

        q_init = np.zeros(6)

        pin.forwardKinematics(cls.model, cls.data, q_init)
        pin.updateGeometryPlacements(
            cls.model, cls.data, cls.collision_model, cls.collision_data)

        cls.meshes = []
        cls.transforms = []
        for i, geom in enumerate(cls.collision_model.geometryObjects):
            T_base_mesh = cls.collision_data.oMg[i]
            T_world_mesh = cls.T_world_base @ T_base_mesh
            cls.transforms.append(T_world_mesh)
            mesh = o3d.io.read_triangle_mesh(geom.meshPath)
            mesh.transform(T_world_mesh)
            cls.meshes.append(mesh)

    @classmethod
    def stop(cls):
        logging.info("stopping mesh processor...")
        for shm in (cls.depth_shm, cls.color_shm):
            try:
                shm.close()
            except Exception as e:
                logging.exception(f"error closing shared memory: {e}")
        logging.info("stopped mesh processor")

    @classmethod
    def process_mesh(cls, q: np.ndarray) -> bool:
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(cls.color_image), 
            o3d.geometry.Image(cls.depth_image),
            depth_scale=1,
            depth_trunc=1.0,
            convert_rgb_to_intensity=False,
        )

        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            image=rgbd_image,
            intrinsic=o3d.camera.PinholeCameraIntrinsic(
                width=cls.hres,
                height=cls.vres,
                fx=cls.camera_matrix[0, 0],
                fy=cls.camera_matrix[1, 1],
                cx=cls.camera_matrix[0, 2],
                cy=cls.camera_matrix[1, 2],
            ),
            extrinsic=cls.extrinsics,
            project_valid_depth_only=True
        )

        logging.debug(f"pcd: {pcd}")
        # logging.debug(f"pcd: {np.asarray(pcd.colors)}")

        # points = np.asarray(pcd.points)
        # logging.debug(np.min(points, axis=0))
        # logging.debug(np.max(points, axis=0))
        # logging.debug(np.mean(points, axis=0))
        # logging.debug(np.std(points, axis=0))

        pin.forwardKinematics(cls.model, cls.data, q)
        pin.updateGeometryPlacements(
            cls.model, cls.data, cls.collision_model, cls.collision_data)

        for i in range(len(cls.collision_model.geometryObjects)):
            T_base_mesh = cls.collision_data.oMg[i]
            T_world_mesh = cls.T_world_base @ T_base_mesh
            cls.meshes[i].transform(np.linalg.inv(cls.transforms[i]))
            cls.meshes[i].transform(T_world_mesh)
            cls.transforms[i] = T_world_mesh

        scene = o3d.t.geometry.RaycastingScene()

        for mesh in cls.meshes:
            mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
            scene.add_triangles(mesh_t)

        pcd_t = o3d.t.geometry.PointCloud.from_legacy(pcd)
        signed_distance = scene.compute_signed_distance(pcd_t.point["positions"])
        filtered_t = pcd_t.select_by_mask(signed_distance > 0.02)

        filtered = filtered_t.to_legacy()
        filtered = filtered.crop(
            o3d.geometry.AxisAlignedBoundingBox(
                min_bound=(-0.19, -0.19, -0.1),
                max_bound=(0.19, 0.19, 0.1),
            )
        )

        logging.debug(f"filtered: {filtered}")

        filtered.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=0.1, max_nn=30
            )
        )
        filtered.orient_normals_consistent_tangent_plane(k=15)
        filtered.normals = o3d.utility.Vector3dVector(
            -np.asarray(filtered.normals)
        )

        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            filtered, depth=9)

        densities = np.asarray(densities)
        vertices_to_remove = densities < np.quantile(densities, 0.05)
        mesh.remove_vertices_by_mask(vertices_to_remove)

        logging.debug("writing...")
        mesh_write_res = o3d.io.write_triangle_mesh(
            cls.project_dir / "assets" / "mesh.obj", mesh)
        pcd_write_res = o3d.io.write_point_cloud(
            cls.project_dir / "assets" / "filtered.ply", filtered)
        logging.debug("wrote")
        return mesh_write_res and pcd_write_res


def read_loop():
    with grpc.insecure_channel("127.0.0.1:50051") as channel:
        stub = service_pb2_grpc.UpdateStub(channel)
        for _ in range(3):
            request = service_pb2.Request(id=str(uuid.uuid4()))
            response = stub.GetImage(request)
            logging.debug(f"response status: {response.success}")
            q_rest = np.array([0, -1.822, 1.55, 0.906, 0, 0])
            mesh_success = MeshProcessor.process_mesh(q_rest)
            logging.debug(f"mesh success: {mesh_success}")
            time.sleep(1.0)


def main():
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )

    try:
        MeshProcessor.start()
        read_loop()
    except KeyboardInterrupt:
        logging.info("User keyboard interrupt")
    except Exception as e:
        logging.exception(f"An unexpected error occured: {e}")
    finally:
        MeshProcessor.stop()


if __name__ == "__main__":
    main()
