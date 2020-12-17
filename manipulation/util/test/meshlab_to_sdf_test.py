import unittest

import drake.manipulation.util.meshlab_to_sdf as mut


class TestShowModel(unittest.TestCase):

    def setUp(self):
        self.maxDiff = None  # Unlimited.

    def test_case1(self):
        """Test one particular input-output pair."""

        log_text = """
LOG: 0 Opened mesh myfile.obj in 54 msec
LOG: 0 All files opened in 55 msec
LOG: 2 Mesh Bounding Box Size 11.434200 8.568000 9.952500
LOG: 2 Mesh Bounding Box Diag 17.412748
LOG: 2 Mesh Volume  is 147.826782
LOG: 2 Mesh Surface is 515.939026
LOG: 2 Thin shell barycenter   0.408610   0.000000   5.138147
LOG: 2 Center of Mass  is 0.421246 -0.000000 4.383873
LOG: 2 Inertia Tensor is :
LOG: 2     | 2053.508545  -0.000000  -72.085564 |
LOG: 2     | -0.000000  2370.329590  -0.000000 |
LOG: 2     | -72.085564  -0.000000  1909.948364 |
LOG: 2 Principal axes are :
LOG: 2     |  0.923473   0.000000   0.383664 |
LOG: 2     | -0.000000   1.000000   0.000000 |
LOG: 2     | -0.383664  -0.000000   0.923473 |
LOG: 2 axis momenta are :
LOG: 2     | 2083.457031  2370.329590  1879.999878 |
"""[1:]  # Remove leading newline.
        scale = 0.01
        mass_kg = 0.321
        expected = """
<?xml version='1.0'?>
<sdf version='1.7'>
  <model name='myfile'>
    <link name='myfile'>
      <inertial>
        <pose>0.00421 0 0.0438 0 0 0</pose>
        <mass>0.321</mass>
        <inertia>
          <ixx> 0.000446</ixx>
          <ixy> 0.000000</ixy>
          <ixz>-0.000016</ixz>
          <iyy> 0.000515</iyy>
          <iyz> 0.000000</iyz>
          <izz> 0.000415</izz>
        </inertia>
      </inertial>
      <visual name='visual'>
        <geometry>
          <mesh>
            <uri>myfile.obj</uri>
            <scale>0.01 0.01 0.01</scale>
          </mesh>
        </geometry>
    </link>
  </model>
</sdf>
"""[1:]  # Remove leading newline.
        self.assertMultiLineEqual(
            expected,
            mut.convert(
                log_text=log_text,
                scale=scale,
                mass_kg=mass_kg
            ))

    def test_zeros(self):
        """Confirm that truncated values are noticed."""

        log_text = """
LOG: 0 Opened mesh myfile.obj in 54 msec
LOG: 2 Mesh Volume  is 147.826782
LOG: 2 Center of Mass  is 0.421246 -0.000000 4.383873
LOG: 2 Inertia Tensor is :
LOG: 2     | 0.000000  0.000000  0.000000 |
LOG: 2     | 0.000000  0.000000  0.000000 |
LOG: 2     | 0.000000  0.000000  0.000000 |
"""[1:]  # Remove leading newline.
        scale = 1.0
        mass_kg = 1.0
        with self.assertRaisesRegex(
                RuntimeError,
                "inertia matrix elements are too small"):
            mut.convert(
                log_text=log_text,
                scale=scale,
                mass_kg=mass_kg
            )
