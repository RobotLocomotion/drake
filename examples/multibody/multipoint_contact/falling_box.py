"""Drops two boxes onto a slab to show what the "mujoco_multipoint" point
contact algorithm changes.

Drake's default point contact narrowphase reports one contact point per
colliding geometry pair, the point of deepest penetration, even when two flat
faces overlap. Setting the proximity property
("material", "point_contact_algorithm") to "mujoco_multipoint" on both
geometries of a Convex (or Mesh) pair makes the pair report a contact manifold
of up to four points instead, so every shape in this scene is a box modeled as
a Convex shape.

The slab and the box on the right select "mujoco_multipoint"; the box on the
left selects "single_point" (the default), so its contact with the slab
reports one point. Both boxes fall flat onto the slab. A box supported at a
single point has no static equilibrium: the support point hops between
corners of the bottom face, so the left box keeps rocking, creeps sideways,
and sinks into the slab. The right box lands on a four-point manifold and
comes to rest at once.

The console prints, for each box, how many contact points it has with the
slab, the height of its center above the slab, and its angular speed;
Meshcat draws one contact force arrow per contact point.
"""

import argparse

import numpy as np

from pydrake.common import MemoryFile
from pydrake.geometry import (
    AddContactMaterial,
    Convex,
    InMemoryMesh,
    Meshcat,
    ProximityProperties,
)
from pydrake.math import RigidTransform
from pydrake.multibody.plant import (
    AddMultibodyPlant,
    CoulombFriction,
    MultibodyPlantConfig,
)
from pydrake.multibody.tree import SpatialInertia
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization

# Dimensions are in meters; masses in kilograms.
SLAB_SIZE = (1.0, 0.5, 0.05)
BOX_SIZE = (0.2, 0.2, 0.04)
BOX_MASS = 0.5


def make_convex_box(size, name):
    """Returns the axis-aligned box of the given (x, y, z) size, centered at
    the origin, as a Convex shape: the convex hull of an in-memory OBJ file
    that lists the eight corners. Drake's Box shape always reports a single
    contact point; only Convex and Mesh shapes are eligible for the
    multi-point contact manifold.
    """
    hx, hy, hz = np.asarray(size) / 2
    lines = [
        f"v {x} {y} {z}"
        for z in (-hz, hz)
        for y in (-hy, hy)
        for x in (-hx, hx)
    ]
    # Faces reference the vertices above by 1-based index, wound
    # counter-clockwise when seen from outside the box.
    lines += [
        "f 1 3 4 2",  # -z
        "f 5 6 8 7",  # +z
        "f 1 2 6 5",  # -y
        "f 3 7 8 4",  # +y
        "f 1 5 7 3",  # -x
        "f 2 4 8 6",  # +x
    ]
    obj = "\n".join(lines) + "\n"
    return Convex(InMemoryMesh(mesh_file=MemoryFile(obj, ".obj", name)))


def make_contact_properties(point_contact_algorithm):
    """Returns proximity properties that select the given point contact
    algorithm, either "single_point" or "mujoco_multipoint". Stiffness and
    dissipation are left to the SceneGraph defaults. To select an algorithm
    for a whole scene at once, set
    SceneGraphConfig.default_proximity_properties.point_contact_algorithm
    instead of a per-geometry property.
    """
    properties = ProximityProperties()
    AddContactMaterial(
        properties=properties, friction=CoulombFriction(0.5, 0.5)
    )
    properties.AddProperty(
        "material", "point_contact_algorithm", point_contact_algorithm
    )
    return properties


def add_box(plant, name, point_contact_algorithm, color):
    body = plant.AddRigidBody(
        name, SpatialInertia.SolidBoxWithMass(BOX_MASS, *BOX_SIZE)
    )
    shape = make_convex_box(BOX_SIZE, f"{name}.obj")
    plant.RegisterCollisionGeometry(
        body,
        RigidTransform(),
        shape,
        f"{name}_collision",
        make_contact_properties(point_contact_algorithm),
    )
    plant.RegisterVisualGeometry(
        body, RigidTransform(), shape, f"{name}_visual", color
    )
    return body


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--simulation_time",
        type=float,
        default=5.0,
        help="Duration of the simulation in seconds.",
    )
    parser.add_argument(
        "--target_realtime_rate",
        type=float,
        default=1.0,
        help="Target realtime rate.",
    )
    parser.add_argument(
        "--time_step",
        type=float,
        default=1e-3,
        help="Discrete time step of the MultibodyPlant in seconds.",
    )
    parser.add_argument(
        "--drop_height",
        type=float,
        default=0.05,
        help="Initial gap between the boxes and the slab in meters.",
    )
    args = parser.parse_args()

    builder = DiagramBuilder()
    plant, _ = AddMultibodyPlant(
        MultibodyPlantConfig(time_step=args.time_step, contact_model="point"),
        builder,
    )

    # The slab is a box modeled as a Convex shape, fixed to the world with its
    # top face at z = 0.
    slab_shape = make_convex_box(SLAB_SIZE, "slab.obj")
    X_WSlab = RigidTransform([0.0, 0.0, -SLAB_SIZE[2] / 2])
    plant.RegisterCollisionGeometry(
        plant.world_body(),
        X_WSlab,
        slab_shape,
        "slab_collision",
        make_contact_properties("mujoco_multipoint"),
    )
    plant.RegisterVisualGeometry(
        plant.world_body(),
        X_WSlab,
        slab_shape,
        "slab_visual",
        [0.8, 0.8, 0.75, 1.0],
    )

    # A pair reports a manifold only when *both* of its geometries select
    # "mujoco_multipoint". The left box selects "single_point", so its contact
    # with the slab behaves like Drake's default narrowphase.
    boxes = [
        add_box(
            plant, "single_point_box", "single_point", [0.9, 0.5, 0.2, 1.0]
        ),
        add_box(
            plant, "multipoint_box", "mujoco_multipoint", [0.2, 0.5, 0.9, 1.0]
        ),
    ]
    plant.Finalize()

    meshcat = Meshcat()
    AddDefaultVisualization(builder=builder, meshcat=meshcat)
    meshcat.SetCameraPose(
        camera_in_world=[0.6, -1.2, 0.6], target_in_world=[0.0, 0.0, 0.0]
    )
    diagram = builder.Build()

    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(args.target_realtime_rate)
    plant_context = plant.GetMyMutableContextFromRoot(
        simulator.get_mutable_context()
    )

    # Both boxes start level, a small distance above the slab.
    z0 = BOX_SIZE[2] / 2 + args.drop_height
    for body, x in zip(boxes, (-0.25, 0.25)):
        plant.SetFreeBodyPose(plant_context, body, RigidTransform([x, 0.0, z0]))

    def report(t):
        """Prints, for each box, the number of point contacts it has with the
        slab, the height of its center above the slab in millimeters (a box
        at rest on the slab reads 20.00), and its angular speed in rad/s.
        """
        results = plant.get_contact_results_output_port().Eval(plant_context)
        counts = {body.index(): 0 for body in boxes}
        for i in range(results.num_point_pair_contacts()):
            info = results.point_pair_contact_info(i)
            for index in (info.bodyA_index(), info.bodyB_index()):
                if index in counts:
                    counts[index] += 1
        columns = [f"{t:7.2f}"]
        for body in boxes:
            height_mm = (
                1000
                * plant.EvalBodyPoseInWorld(plant_context, body).translation()[
                    2
                ]
            )
            w_WB = plant.EvalBodySpatialVelocityInWorld(
                plant_context, body
            ).rotational()
            columns.append(
                f"{counts[body.index()]:8d}  {height_mm:7.2f}  "
                f"{np.linalg.norm(w_WB):6.3f}"
            )
        print("   ".join(columns))

    print("               single_point box               multipoint box")
    print("   time   contacts   height    |w|     contacts   height    |w|")
    simulator.Initialize()
    t = 0.0
    while t < args.simulation_time:
        t = min(t + 0.5, args.simulation_time)
        simulator.AdvanceTo(t)
        report(t)


if __name__ == "__main__":
    main()
