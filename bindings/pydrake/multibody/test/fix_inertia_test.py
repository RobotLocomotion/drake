"""Unit tests for fix_inertia.

Still missing:
in-place file processing
invalid-only selective processing
non-rigid body
"""

from pathlib import Path
import subprocess
import sys
import unittest

from pydrake.common import (
    FindResourceOrThrow,
    temp_directory,
)
from pydrake.multibody._inertia_fixer import (
    fix_inertia_from_string,
    InertiaFixer,
)


class TestFixInertiaFromString(unittest.TestCase):
    def setUp(self):
        self.maxDiff = None  # no limit to diff output.

    def test_min_urdf(self):
        input_text = '<robot name="a"/>'
        output = fix_inertia_from_string(input_text, "urdf")
        self.assertEqual(output, input_text)
        output = fix_inertia_from_string(input_text, "urdf", invalid_only=True)
        self.assertEqual(output, input_text)

    def test_min_sdf(self):
        input_text = """\
<sdf version="1.6"><model name="a"><link name="b"/></model></sdf>"""
        output = fix_inertia_from_string(input_text, "sdf")
        self.assertEqual(output, input_text)
        output = fix_inertia_from_string(input_text, "sdf", invalid_only=True)
        self.assertEqual(output, input_text)

    def test_sphere_sdf(self):
        input_text = """\
<sdf version="1.6">
  <model name="a">
    <link name="b">
      <inertial>
        <mass>1</mass>
      </inertial>
      <collision name="sphere">
        <geometry>
          <sphere>
            <radius>0.1</radius>
          </sphere>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>"""

        expected_text = """\
<sdf version="1.6">
  <model name="a">
    <link name="b">
      <inertial>
        <inertia>
          <ixx>0.004000000000000001</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.004000000000000001</iyy>
          <iyz>0.0</iyz>
          <izz>0.004000000000000001</izz>
        </inertia>
        <pose>0 0 0 0 0 0</pose>
        <mass>1.0</mass>
      </inertial>
      <collision name="sphere">
        <geometry>
          <sphere>
            <radius>0.1</radius>
          </sphere>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>"""
        output = fix_inertia_from_string(input_text, "sdf")
        self.assertEqual(expected_text, output)
        output = fix_inertia_from_string(input_text, "sdf", invalid_only=True)
        self.assertEqual(input_text, output)  # XXX?

    def test_massless_sphere_sdf(self):
        input_text = """\
<sdf version="1.6">
  <model name="a">
    <link name="b">
      <collision name="sphere">
        <geometry>
          <sphere>
            <radius>0.1</radius>
          </sphere>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>"""

        expected_text = """\
<sdf version="1.6">
  <model name="a">
    <link name="b">
      <inertial>
        <inertia>
          <ixx>0.004000000000000001</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.004000000000000001</iyy>
          <iyz>0.0</iyz>
          <izz>0.004000000000000001</izz>
        </inertia>
        <mass>1.0</mass>
        <pose>0 0 0 0 0 0</pose>
      </inertial>
      <collision name="sphere">
        <geometry>
          <sphere>
            <radius>0.1</radius>
          </sphere>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>"""
        output = fix_inertia_from_string(input_text, "sdf")
        self.assertEqual(expected_text, output)
        output = fix_inertia_from_string(input_text, "sdf", invalid_only=True)
        self.assertEqual(input_text, output)  # XXX?

    def test_sphere_urdf(self):
        input_text = """\
<robot name="a">
  <link name="world"/>
  <link name="b">
    <inertial>
      <mass value="1"/>
    </inertial>
    <collision name="sphere">
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>
</robot>"""

        expected_text = """\
<robot name="a">
  <link name="world"/>
  <link name="b">
    <inertial>
      <inertia ixx="0.004000000000000001" ixy="0.0" ixz="0.0"\
 iyy="0.004000000000000001" iyz="0.0" izz="0.004000000000000001"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="1.0"/>
    </inertial>
    <collision name="sphere">
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>
</robot>"""
        output = fix_inertia_from_string(input_text, "urdf")
        self.assertEqual(expected_text, output)
        output = fix_inertia_from_string(input_text, "urdf", invalid_only=True)
        self.assertEqual(input_text, output)  # XXX?

    def test_massless_sphere_urdf(self):
        # XXX Understand why this emits 0 mass, and the similar sdf test emits
        # a mass of 1.
        input_text = """\
<robot name="a">
  <link name="world"/>
  <link name="b">
    <collision name="sphere">
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>
</robot>"""

        expected_text = """\
<robot name="a">
  <link name="world"/>
  <link name="b">
    <inertial>
      <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
      <mass value="0.0"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </inertial>
    <collision name="sphere">
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>
</robot>"""
        output = fix_inertia_from_string(input_text, "urdf")
        self.assertEqual(expected_text, output)
        output = fix_inertia_from_string(input_text, "urdf", invalid_only=True)
        self.assertEqual(input_text, output)  # XXX?


class TestInertiaFixer(unittest.TestCase):
    def setUp(self):
        self._temp_dir = Path(temp_directory())
        self._box_urdf = FindResourceOrThrow(
            "drake/multibody/models/box.urdf")

    def test_simple_valid_invocation(self):
        """Smoke tests that a straightforward invocation works."""
        dut = InertiaFixer(input_file=self._box_urdf)
        dut.fix_inertia()
        # XXX verify!

    def test_package_none(self):
        pass
        # XXX do something!

    def test_package_xml(self):
        pass
        # XXX do something!

    def test_package_errors(self):
        pass
        # XXX do something!


class TestFixInertiaProcess(unittest.TestCase):
    """
    Tests the command-line tool fix_inertia by invoking the process
    directly.

    The above tests on the underlying class confirm correctness of the SDFormat
    file and logic for processing parameters. For these tests, we need to
    confirm that command-line parameters are passed as expected.

    We'll simply create two output files, one via subprocess and one via
    direct calls to InertiaFixer and compare the files.
    """
    def setUp(self):
        self._temp_dir = Path(temp_directory())
        self._dut = FindResourceOrThrow(
            "drake/bindings/pydrake/multibody/fix_inertia")

    def assert_files_equal(self, dut_path, ref_path):
        with open(dut_path) as dut, open(ref_path) as ref:
            self.assertEqual(dut.read(), ref.read())

    def subprocess_fix_inertia(self, input_path, output_path):
        subprocess.run(
            [self._dut, input_path, output_path],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT)

    def do_test_model(self, resource):
        model_file = FindResourceOrThrow(resource)
        file_type = Path(model_file).suffix
        direct_result = self._temp_dir / "direct"
        InertiaFixer(input_file=model_file,
                     output_file=direct_result).fix_inertia()
        self.assertIn('inertial', direct_result.read_text())

        subprocess_result = self._temp_dir / f"subprocess{file_type}"
        self.subprocess_fix_inertia(model_file, subprocess_result)
        self.assertIn('inertial', subprocess_result.read_text())

        # Direct invocation and subprocess invocation give the same result.
        self.assert_files_equal(direct_result, subprocess_result)

        repeat_result = self._temp_dir / "repeat"
        self.subprocess_fix_inertia(subprocess_result, repeat_result)
        self.assertIn('inertial', repeat_result.read_text())

        # Repeated processing of the output gives the same result. This step
        # also ensures XML well-formed-ness and Drake parsing compatibility.
        self.assert_files_equal(direct_result, repeat_result)

    def test_urdf(self):
        self.do_test_model("drake/multibody/models/box.urdf")

    def test_urdf_no_inertia_link(self):
        # Contains a non-ignored link with no inertial properties.
        self.do_test_model("drake/examples/acrobot/Acrobot.urdf")

    def test_sdf(self):
        self.do_test_model("drake/multibody/benchmarks/acrobot/acrobot.sdf")

    def test_sdf_no_geom(self):
        # Contains links without geometry.
        self.do_test_model(
            "drake/bindings/pydrake/multibody/test/pendulum_on_rail.sdf")

    def test_sdf_no_inertia_link(self):
        # Contains a non-ignored link with no inertial properties.
        self.do_test_model("drake/examples/acrobot/Acrobot.sdf")

    def test_sdf_nested_models(self):
        # Contains nested models.
        self.do_test_model(
            "drake/manipulation/util/test/simple_nested_model.sdf")
