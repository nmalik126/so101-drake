import numpy as np
from scipy.spatial.transform import Rotation
from pydrake.all import (
    InverseKinematics,
    RotationMatrix,
    MultibodyPlant,
    Solve,
)


class SO101InverseKinematics:

    @classmethod
    def solve_ik_place(cls, plant: MultibodyPlant):
        grasp = np.eye(4)
        grasp[:3, :3] = Rotation.from_rotvec([0, 0, np.pi/2]).as_matrix()
        grasp[:3, 3] = [0.13, 0.0, 0.2]

        # initial guess
        ik = InverseKinematics(plant)
        q_init = np.zeros(6)
        ik.get_mutable_prog().AddQuadraticErrorCost(1.0, q_init, ik.q())
        ik.get_mutable_prog().SetInitialGuess(ik.q(), q_init)

        # constraints
        so101 = plant.GetModelInstanceByName("so101_new_calib")
        ik.AddPositionConstraint(
            plant.GetFrameByName("gripper_link", so101),
            [0.0, 0.0, 0.0],
            plant.world_frame(),
            grasp[:3, 3],
            grasp[:3, 3]
        )
        ik.AddOrientationConstraint(
            plant.GetFrameByName("gripper_link", so101),
            RotationMatrix(),
            plant.world_frame(),
            RotationMatrix(grasp[:3, :3]),
            np.pi/16
        )
        ik.get_mutable_prog().AddBoundingBoxConstraint(
            np.pi/4, np.pi/4, ik.q()[5]
        )

        # solve
        result = Solve(ik.prog())
        if result.is_success():
            return result.GetSolution(ik.q())
        else:
            return None

    @classmethod
    def solve_ik_pick(cls, plant: MultibodyPlant, grasp: np.ndarray):
        # initial guess
        ik = InverseKinematics(plant)
        q_init = np.zeros(6)
        ik.get_mutable_prog().AddQuadraticErrorCost(1.0, q_init, ik.q())
        ik.get_mutable_prog().SetInitialGuess(ik.q(), q_init)

        # constraints
        so101 = plant.GetModelInstanceByName("so101_new_calib")
        ik.AddPositionConstraint(
            plant.GetFrameByName("gripper_link", so101),
            [0.0, 0.0, 0.0],
            plant.world_frame(),
            grasp[:3, 3] + np.array([-0.015, 0.01, 0]),
            grasp[:3, 3] + np.array([-0.015, 0.01, 0])
        )
        ik.AddOrientationConstraint(
            plant.GetFrameByName("gripper_link", so101),
            RotationMatrix(),
            plant.world_frame(),
            RotationMatrix.MakeYRotation(np.pi) @ RotationMatrix(grasp[:3, :3]),
            np.pi/16
        )
        ik.get_mutable_prog().AddBoundingBoxConstraint(
            np.pi/4, np.pi/4, ik.q()[5]
        )

        # solve
        result = Solve(ik.prog())
        if result.is_success():
            return result.GetSolution(ik.q())
        else:
            return None


def main():
    from pathlib import Path
    import open3d as o3d
    import time
    from pydrake.all import (
        StartMeshcat,
        DiagramBuilder,
        AddMultibodyPlantSceneGraph,
        Parser,
        RigidTransform,
        AddDefaultVisualization,
        PointCloud,
        Fields,
        BaseField,
    )

    meshcat = StartMeshcat()

    project_dir = Path("/home/noor/so101-drake")

    builder = DiagramBuilder()

    plant, scene_graph = AddMultibodyPlantSceneGraph(
        builder,
        time_step=0.001,
    )

    parser = Parser(plant)
    parser.AddModels(project_dir / "models" / "objects" / "surface.sdf")
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("surface_link")
    )

    T_world_base = np.eye(4)
    rotvec = np.array([0, 0, 1]) * np.pi/2
    T_world_base[:3, :3] = Rotation.from_rotvec(rotvec).as_matrix()
    T_world_base[:3, 3] = [0, -0.1775, 0.0074]

    parser.AddModels(
        project_dir / "models" / "SO101" / "so101_new_calib_drake_hydro.urdf"
    )
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("base_link"),
        RigidTransform(
            RotationMatrix(T_world_base[:3, :3]), 
            T_world_base[:3, 3]
        )
    )

    plant.Finalize()

    AddDefaultVisualization(builder, meshcat)

    diagram = builder.Build()
    context = diagram.CreateDefaultContext()

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

    print("solving ik...")

    # q_place = SO101InverseKinematics.solve_ik_place(plant)
    # print(q_place)
    # if q_place is not None:
    #     plant.SetPositions(
    #         plant.GetMyMutableContextFromRoot(context), 
    #         q_place
    #     )
    #     diagram.ForcedPublish(context)

    grasp = np.array([
        [ 0.20856473, -0.96235112, -0.17430128, -0.04784185],
        [-0.97346751, -0.22142707,  0.05771393,  0.04177497],
        [-0.09413604,  0.15763952, -0.98299962,  0.15351727],
        [ 0.        ,  0.        ,  0.        ,  1.        ],
    ])
    q_pick = SO101InverseKinematics.solve_ik_pick(plant, grasp)
    print(q_pick)
    if q_pick is not None:
        plant.SetPositions(
            plant.GetMyMutableContextFromRoot(context), 
            q_pick
        )
        diagram.ForcedPublish(context)

    time.sleep(5.0)

    
if __name__ == "__main__":
    main()
