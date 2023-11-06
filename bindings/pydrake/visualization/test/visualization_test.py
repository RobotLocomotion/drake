from pydrake.visualization import (
    MeshcatPoseSliders,
)

import unittest

import numpy as np

from pydrake.geometry import (
    Meshcat,
)
from pydrake.math import (
    RigidTransform,
)


class TestSliders(unittest.TestCase):

    def test_meshcat_pose_sliders(self):
        # Construct a sliders system, using every available option.
        meshcat = Meshcat()
        dut = MeshcatPoseSliders(
            meshcat=meshcat,
            initial_pose=RigidTransform(),
            lower_limit=[-0.5] * 6,
            upper_limit=[0.5] * 6,
            step=[0.1] * 6,
            decrement_keycodes=[
                "KeyA", "KeyB", "KeyC", "KeyD", "KeyE", "KeyF"
            ],
            increment_keycodes=[
                "KeyG", "KeyH", "KeyI", "KeyJ", "KeyK", "KeyL"
            ],
            prefix="pre",
            visible=[False, False, True, True, True, False])

        # Various methods should not crash.
        dut.get_output_port()
        dut.Delete()

        # The constructor has default values.
        dut = MeshcatPoseSliders(meshcat)
        context = dut.CreateDefaultContext()

        # The Run function doesn't crash.
        X = dut.Run(system=dut,
                    context=context,
                    timeout=1.0,
                    stop_button_keycode="ArrowLeft")
        self.assertTrue(X.IsExactlyIdentity())

        X2 = RigidTransform([0.1, 0.2, 0.3])
        dut.SetPose(pose=X2)
        X_out = dut.get_output_port().Eval(context)
        self.assertTrue(X_out.IsExactlyEqualTo(X2))
