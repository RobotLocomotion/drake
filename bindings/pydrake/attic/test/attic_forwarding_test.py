"""
Tests existence of all relevant attic forwarding symbols.
"""

import unittest


class TestAtticForwarding(unittest.TestCase):
    def test_multibody_modules(self):
        from pydrake.multibody import (
            collision,
            joints,
            parsers,
            rigid_body_plant,
            rigid_body,
            rigid_body_tree,
            shapes,
        )

    def test_solvers_modules(self):
        from pydrake.solvers import ik

    def test_systems_symbols(self):
        from pydrake.systems.controllers import (
            RbtInverseDynamics, RbtInverseDynamicsController,
        )
        from pydrake.systems.sensors import RgbdCamera, RgbdCameraDiscrete
