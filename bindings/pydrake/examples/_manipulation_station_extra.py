# See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
# rationale.
from pydrake.math import RigidTransform, RollPitchYaw


def _xyz_rpy(xyz, rpy):
    return RigidTransform(RollPitchYaw(rpy), xyz)


def CreateDefaultYcbObjectList():
    """Creates a list of (model_file, pose) pairs to add six YCB objects to a
    ManipulationStation.
    """
    ycb_object_pairs = []

    X_WCracker = _xyz_rpy([-0.3, -0.55, 0.36], [-1.57, 0, 3])
    ycb_object_pairs.append(
        ("drake/manipulation/models/ycb/sdf/003_cracker_box.sdf", X_WCracker))

    # The sugar box pose.
    X_WSugar = _xyz_rpy([-0.3, -0.7, 0.33], [1.57, 1.57, 0])
    ycb_object_pairs.append(
        ("drake/manipulation/models/ycb/sdf/004_sugar_box.sdf", X_WSugar))

    # The tomato soup can pose.
    X_WSoup = _xyz_rpy([-0.03, -0.57, 0.31], [-1.57, 0, 3.14])
    ycb_object_pairs.append(
        ("drake/manipulation/models/ycb/sdf/005_tomato_soup_can.sdf", X_WSoup))

    # The mustard bottle pose.
    X_WMustard = _xyz_rpy([0.05, -0.66, 0.35], [-1.57, 0, 3.3])
    ycb_object_pairs.append(
        ("drake/manipulation/models/ycb/sdf/006_mustard_bottle.sdf",
         X_WMustard))

    # The gelatin box pose.
    X_WGelatin = _xyz_rpy([-0.15, -0.62, 0.38], [-1.57, 0, 3.7])
    ycb_object_pairs.append(
        ("drake/manipulation/models/ycb/sdf/009_gelatin_box.sdf", X_WGelatin))

    # The potted meat can pose.
    X_WMeat = _xyz_rpy([-0.15, -0.62, 0.3], [-1.57, 0, 2.5])
    ycb_object_pairs.append(
        ("drake/manipulation/models/ycb/sdf/010_potted_meat_can.sdf", X_WMeat))

    return ycb_object_pairs
