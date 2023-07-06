import unittest
from pydrake.common.test_utilities import numpy_compare

from pydrake.autodiffutils import AutoDiffXd
from pydrake.multibody.fem import (
    MaterialModel,
    DeformableBodyConfig_,
)


class TestMultibodyFem(unittest.TestCase):
    @numpy_compare.check_nonsymbolic_types
    def test_deformable_body_config(self, T):
        """
        Provides basic acceptance tests for DeformableBodyConfig.
        """
        dut = DeformableBodyConfig_[T]()
        dut.set_youngs_modulus(youngs_modulus=1000.0)
        dut.set_poissons_ratio(poissons_ratio=0.4)
        dut.set_mass_damping_coefficient(mass_damping_coefficient=0.1)
        dut.set_stiffness_damping_coefficient(
            stiffness_damping_coefficient=0.2)
        dut.set_mass_density(mass_density=100)
        self.assertEqual(dut.youngs_modulus(), 1000)
        self.assertEqual(dut.poissons_ratio(), 0.4)
        self.assertEqual(dut.mass_damping_coefficient(), 0.1)
        self.assertEqual(dut.stiffness_damping_coefficient(), 0.2)
        self.assertEqual(dut.mass_density(), 100)

        models = [
            MaterialModel.kLinearCorotated,
            MaterialModel.kCorotated,
            MaterialModel.kLinear,
        ]
        for model in models:
            dut.set_material_model(model)
            self.assertEqual(dut.material_model(), model)
