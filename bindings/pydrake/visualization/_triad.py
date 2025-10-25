import numpy as np

from pydrake.common.eigen_geometry import AngleAxis
from pydrake.geometry import (
    Cylinder,
    FrameId,
    GeometryInstance,
    MakePhongIllustrationProperties,
    SceneGraph,
)
from pydrake.math import RigidTransform
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import (
    Frame,
    FrameIndex,
    RigidBody,
)


def AddFrameTriadIllustration(
    *,
    scene_graph: SceneGraph,
    body: RigidBody = None,
    frame: Frame = None,
    frame_index: FrameIndex = None,
    frame_id: FrameId = None,
    plant: MultibodyPlant = None,
    name: str = None,
    length: float = 0.3,
    radius: float = 0.005,
    opacity: float = 0.9,
    X_FT: RigidTransform = None,
):
    """
    Adds illustration geometry representing the given frame using an RGB triad,
    with the x-axis drawn in red, the y-axis in green and the z-axis in blue.
    The given frame can be either a geometry FrameId, a multibody RigidBody,
    a multibody FrameIndex or a multibody Frame.

    Note: exactly one of body=, frame=, frame_index= or frame_id= must be
    provided.

    Note: body frames and frames fixed to bodies are colored differently (body
    frames are brighter).

    Args:
      scene_graph: the SceneGraph where geometry will be added.
      body: when provided, illustrates the frame of the given body.
      frame: when provided, illustrates the frame from a plant.
      frame_index: when provided, illustrates the indexed frame from a plant.
        `plant` must not be None.
      frame_id: when provided, illustrates the given geometry.FrameId
        registered with the given plant and scene_graph.
      plant: MultibodyPlant associated with the given frame_id;
        required if frame_index= or frame_id= is being used. If body= or frame=
        is supplied, they must belong to this plant.
      name: the added geometries will have names "_frames::{name}::x-axis",
        etc. If None, the name is inferred from the indicated frame.
      length: the length of each axis in meters.
      radius: the radius of each axis in meters.
      opacity: the opacity each axis, between 0.0 and 1.0.
      X_FT: optional rigid transform relating frame T (the triad geometry) and
        frame F (the given body=, frame=, or frame_id=  frame); when None, X_FT
        is the identity transform and therefore the triad will depict F.
    Returns:
      The newly-added geometry ids for (x, y, z) respectively.
    """
    if (
        sum(
            [
                body is not None,
                frame is not None,
                frame_index is not None,
                frame_id is not None,
            ]
        )
        != 1
    ):
        raise ValueError(
            "Must provide exactly one of body=, frame=, "
            "frame_index=, or frame_id="
        )
    if X_FT is None:
        X_FT = RigidTransform()
    resolved_plant_arg = "body="
    if frame_index is not None:
        if plant is None:
            raise ValueError(
                "When using frame_index=, the plant cannot be None."
            )
        frame = plant.get_frame(frame_index)
    if frame is not None:
        body = frame.body()
        X_FT = frame.GetFixedPoseInBodyFrame() @ X_FT
        resolved_plant_arg = "frame=" if frame_index is None else "frame_index="
    if body is not None:
        if plant is not None:
            if plant is not body.GetParentPlant():
                raise ValueError(
                    f"Mismatched {resolved_plant_arg} and plant=; remove the "
                    f"plant= arg"
                )
        else:
            plant = body.GetParentPlant()
        frame_id = plant.GetBodyFrameIdOrThrow(body.index())
    if frame is None:
        assert frame_id is not None
        frame = plant.GetBodyFromFrameId(frame_id).body_frame()
    if name is None:
        name = f"{frame.name()}({int(frame.model_instance())})"
    brightness = 1 if frame.is_body_frame() else 0.5

    source_id = plant.get_source_id()
    eye = np.eye(3)
    result = []
    for i, char in enumerate(("x", "y", "z")):
        geom_name = f"_frames::{name}::{char}-axis"
        # p_TG centers the cylinder halfway along the i'th axis.
        p_TG = 0.5 * length * eye[i]
        # R_TG rotates the canonical cylinder (aligned with +z) to align with
        # the i'th axis instead. When i == 2, it spins the cylinder around the
        # z axis, but this is effectively a no-op.
        R_TG = AngleAxis(angle=np.pi / 2, axis=eye[1 - i])
        X_FG = X_FT @ RigidTransform(R_TG, p_TG)
        geom = GeometryInstance(X_FG, Cylinder(radius, length), geom_name)
        phong = MakePhongIllustrationProperties(
            np.append(eye[i] * brightness, [opacity])
        )
        geom.set_illustration_properties(phong)
        geometry_id = scene_graph.RegisterGeometry(source_id, frame_id, geom)
        result.append(geometry_id)
    return tuple(result)
