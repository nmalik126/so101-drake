import numpy as np
import coacd
import trimesh
from pathlib import Path
from scipy.spatial.transform import Rotation
import open3d as o3d
import time

from pydrake.all import (
    RobotDiagramBuilder,
    RobotDiagram,
    Convex,
    InMemoryMesh,
    MemoryFile,
    CoulombFriction,
    MeshcatVisualizer,
    MeshcatVisualizerParams,
    Role,
    Parser,
    RigidTransform,
    RotationMatrix,
    StartMeshcat,
    PointCloud,
    Fields,
    BaseField,
)
from manipulation.meshcat_utils import PublishPositionTrajectory

from inverse_kinematics import SO101InverseKinematics
from ompl_planning import SO101SamplingPlanner


def main():
    project_dir = Path("/home/noor/so101-drake")

    meshcat = StartMeshcat()

    mesh = trimesh.load(project_dir / "assets" / "mesh.obj", force="mesh")
    mesh = coacd.Mesh(mesh.vertices, mesh.faces)
    parts = coacd.run_coacd(mesh, threshold=0.01, real_metric=True)

    robot_builder = RobotDiagramBuilder()
    builder = robot_builder.builder()
    plant = robot_builder.plant()
    scene_graph = robot_builder.scene_graph()

    parser = Parser(plant)

    print("starting mesh registration...")
    for i, (v, f) in enumerate(parts):
        obj_str = trimesh.Trimesh(v, f).export(file_type="obj")
        shape = Convex(InMemoryMesh(mesh_file=MemoryFile(obj_str, ".obj", f"part_{i}.obj")))
        plant.RegisterCollisionGeometry(
            plant.world_body(), RigidTransform(), shape,
            f"scene_part_{i}", CoulombFriction(1.0, 1.0))
    print("finished mesh registration")    

    T_world_base = np.eye(4)
    rotvec = np.array([0, 0, 1]) * np.pi/2
    T_world_base[:3, :3] = Rotation.from_rotvec(rotvec).as_matrix()
    T_world_base[:3, 3] = [0, -0.1775, 0.0074]

    so101 = parser.AddModels(
        project_dir / "models" / "SO101" / "so101_new_calib_drake_hydro.urdf"
    )[0]
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("base_link"),
        RigidTransform(
            RotationMatrix(T_world_base[:3, :3]), 
            T_world_base[:3, 3]
        )
    )

    plant.Finalize()

    # AddDefaultVisualization(builder, meshcat)
    visualizer = MeshcatVisualizer.AddToBuilder(
        builder,
        scene_graph,
        meshcat,
        MeshcatVisualizerParams(role=Role.kIllustration),
    )
    collision_visualizer = MeshcatVisualizer.AddToBuilder(
        builder,
        scene_graph,
        meshcat,
        MeshcatVisualizerParams(
            prefix="collision", role=Role.kProximity, visible_by_default=False
        ),
    )

    diagram: RobotDiagram = robot_builder.Build()
    context = diagram.CreateDefaultContext()

    q_init = np.zeros(6)
    q_rest = np.array([0, -1.822, 1.55, 0.906, 0, 0])
    plant.SetPositions(plant.GetMyMutableContextFromRoot(context), q_rest)

    diagram.ForcedPublish(context)

    filtered = o3d.io.read_point_cloud(project_dir / "assets" / "filtered.ply")

    points = np.asarray(filtered.points)
    colors = np.asarray(filtered.colors)  # sRGB in [0, 1] from Open3D

    def srgb_to_linear(c):
        c = np.clip(c, 0.0, 1.0)
        return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)

    colors_linear = srgb_to_linear(colors)

    # Round and clip before casting to avoid uint8 wraparound.
    rgbs_uint8 = np.clip(np.round(255.0 * colors_linear), 0, 255).astype(np.uint8)
    rgbs_uint8 = np.flip(rgbs_uint8, axis=1)

    pc = PointCloud(
        new_size=len(points),
        fields=Fields(BaseField.kXYZs | BaseField.kRGBs),
    )
    pc.mutable_xyzs()[:] = points.T
    pc.mutable_rgbs()[:] = rgbs_uint8.T

    meshcat.SetObject("/my_surface/rgb_points", pc, point_size=0.003)

    grasp = np.array([
        [ 0.20856473, -0.96235112, -0.17430128, -0.04784185],
        [-0.97346751, -0.22142707,  0.05771393,  0.04177497],
        [-0.09413604,  0.15763952, -0.98299962,  0.15351727],
        [ 0.        ,  0.        ,  0.        ,  1.        ],
    ])
    q_place = SO101InverseKinematics.solve_ik_place(plant)
    q_pick = SO101InverseKinematics.solve_ik_pick(plant, grasp)
    if (q_place is None) or (q_pick is None):
        print("IK failed")
        return
    print(q_pick)
    print(q_place)

    waypoints = SO101SamplingPlanner.generate_path(diagram, q_pick, q_place)
    if waypoints is None:
        print("OMPL planning failed")
        return

    waypoints_trunc = np.hstack((waypoints[:, :5], waypoints[:, -3:]))
    trajectory = SO101SamplingPlanner.generate_trajectory(plant, diagram, waypoints_trunc)
    if trajectory is None:
        print("TrajOpt failure")
        return

    meshcat.Flush()
    context = diagram.CreateDefaultContext()
    PublishPositionTrajectory(trajectory, context, plant, visualizer)
    collision_visualizer.ForcedPublish(collision_visualizer.GetMyContextFromRoot(context))
    time.sleep(5.0)


if __name__ == "__main__":
    main()
