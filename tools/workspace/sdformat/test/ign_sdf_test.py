"""Naively checks that `ign_sdf` works as intended."""

import os
from subprocess import run, PIPE
import unittest

EXAMPLE_INPUT = """\
<?xml version='1.0'?>
<sdf version='1.6'>
  <!-- Sample comment -->
  <model name='simple_test'>
    <link name='L1'/>

    <joint name='J1' type='revolute'>
      <child>L1</child>
      <parent>L2</parent>
      <axis>
        <xyz>1 0 0</xyz>
        <use_parent_model_frame>1</use_parent_model_frame>
      </axis>
    </joint>

    <link name='L2'>
      <pose frame='L1'>0 0 0  0 0 0</pose>
    </link>
  </model>
</sdf>
"""

# The following expected output may need to be changed as we upgrade
# libsdformat. Changes necessary for the test to pass should be blindly
# accepted.

EXPECTED_OUTPUT = """\
<sdf version='1.9'>
  <model name='simple_test'>
    <link name='L1'/>
    <joint name='J1' type='revolute'>
      <child>L1</child>
      <parent>L2</parent>
      <axis>
        <xyz expressed_in='__model__'>1 0 0</xyz>
        <limit>
          <lower>-1e16</lower>
          <upper>1e16</upper>
        </limit>
      </axis>
    </joint>
    <link name='L2'>
      <pose relative_to='L1'>0 0 0 0 0 0</pose>
    </link>
  </model>
</sdf>
"""


class TestIgnSdf(unittest.TestCase):
    def setUp(self):
        self.maxDiff = None
        self.bin = "tools/workspace/sdformat/ign_sdf"
        self.input_file = os.path.join(
            os.environ["TEST_TMPDIR"], "example.sdf"
        )
        with open(self.input_file, "w", encoding="utf8") as f:
            f.write(EXAMPLE_INPUT)

    def ign_sdf(self, argv):
        return run(
            [self.bin] + argv,
            stdout=PIPE,
            encoding="utf8",
            check=True,
        ).stdout

    def test_check(self):
        stdout = self.ign_sdf(["--check", self.input_file])
        self.assertEqual(stdout.strip(), "Valid.")

    def test_describe(self):
        stdout = self.ign_sdf(["--describe", "1.7"])
        self.assertIn("<element name ='model'", stdout)

    def test_print(self):
        stdout = self.ign_sdf(["--print", self.input_file])
        self.assertEqual(EXPECTED_OUTPUT, stdout)
