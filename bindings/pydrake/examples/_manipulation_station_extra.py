# See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
# rationale.

import numpy as np

from pydrake.math import RigidTransform, RollPitchYaw


def _xyz_rpy_deg(xyz, rpy_deg):
    return RigidTransform(RollPitchYaw(np.asarray(rpy_deg) * np.pi / 180), xyz)


def CreateManipulationClassYcbObjectList():
    """Creates a list of (model_file, pose) pairs to add five YCB objects to a
    ManipulationStation with the ManipulationClass setup.
    """
    ycb_object_pairs = []

    # The cracker box pose
    X_WCracker = _xyz_rpy_deg([0.35, 0.14, 0.09], [0, -90, 230])
    ycb_object_pairs.append(
        ("drake/manipulation/models/ycb/sdf/003_cracker_box.sdf", X_WCracker))

    # The sugar box pose.
    X_WSugar = _xyz_rpy_deg([0.28, -0.17, 0.03], [0, 90, 180])
    ycb_object_pairs.append(
        ("drake/manipulation/models/ycb/sdf/004_sugar_box.sdf", X_WSugar))

    # The tomato soup can pose.
    X_WSoup = _xyz_rpy_deg([0.40, -0.07, 0.03], [-90, 0, 180])
    ycb_object_pairs.append(
        ("drake/manipulation/models/ycb/sdf/005_tomato_soup_can.sdf", X_WSoup))

    # The mustard bottle pose.
    X_WMustard = _xyz_rpy_deg([0.44, -0.16, 0.09], [-90, 0, 190])
    ycb_object_pairs.append(
        ("drake/manipulation/models/ycb/sdf/006_mustard_bottle.sdf",
         X_WMustard))

    # The potted meat can pose.
    X_WMeat = _xyz_rpy_deg([0.35, -0.32, 0.03], [-90, 0, 145])
    ycb_object_pairs.append(
        ("drake/manipulation/models/ycb/sdf/010_potted_meat_can.sdf", X_WMeat))

    return ycb_object_pairs


def CreateClutterClearingYcbObjectList():
    """Creates a list of (model_file, pose) pairs to add six YCB objects to a
    ManipulationStation with the ClutterClearing setup.
    """
    ycb_object_pairs = []

    # The cracker box pose.
    X_WCracker = _xyz_rpy_deg([-0.3, -0.55, 0.2], [-90, 0, 170])
    ycb_object_pairs.append(
        ("drake/manipulation/models/ycb/sdf/003_cracker_box.sdf", X_WCracker))

    # The sugar box pose.
    X_WSugar = _xyz_rpy_deg([-0.3, -0.7, 0.17], [90, 90, 0])
    ycb_object_pairs.append(
        ("drake/manipulation/models/ycb/sdf/004_sugar_box.sdf", X_WSugar))

    # The tomato soup can pose.
    X_WSoup = _xyz_rpy_deg([-0.03, -0.57, 0.15], [-90, 0, 180])
    ycb_object_pairs.append(
        ("drake/manipulation/models/ycb/sdf/005_tomato_soup_can.sdf", X_WSoup))

    # The mustard bottle pose.
    X_WMustard = _xyz_rpy_deg([0.05, -0.66, 0.19], [-90, 0, 190])
    ycb_object_pairs.append(
        ("drake/manipulation/models/ycb/sdf/006_mustard_bottle.sdf",
         X_WMustard))

    # The gelatin box pose.
    X_WGelatin = _xyz_rpy_deg([-0.15, -0.62, 0.22], [-90, 0, 210])
    ycb_object_pairs.append(
        ("drake/manipulation/models/ycb/sdf/009_gelatin_box.sdf", X_WGelatin))

    # The potted meat can pose.
    X_WMeat = _xyz_rpy_deg([-0.15, -0.62, 0.14], [-90, 0, 145])
    ycb_object_pairs.append(
        ("drake/manipulation/models/ycb/sdf/010_potted_meat_can.sdf", X_WMeat))

    return ycb_object_pairs
