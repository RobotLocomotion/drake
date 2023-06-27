import unittest

import pydrake.visualization as mut


class TestModelVisualizerCamera(unittest.TestCase):
    """
    Tests the camera feature of the ModelVisualizer class.

    The camera testing is carved into a separate file vs model_visualizer_test
    because our camera code tends to be flaky when run using the emulated video
    driver on our continuous integration builds.
    """

    def test_camera(self):
        """
        Checks that the rgbd sensor code does not crash.
        """
        model = """<?xml version="1.0"?>
          <sdf version="1.9">
            <model name="sample">
              <link name="base"/>
            </model>
          </sdf>
        """
        dut = mut.ModelVisualizer(show_rgbd_sensor=True)
        dut.parser().AddModelsFromString(model, "sdf")
        dut.Run(loop_once=True)
