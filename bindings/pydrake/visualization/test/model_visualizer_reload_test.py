import subprocess
import time
import unittest

from pydrake.common import FindResourceOrThrow
from pydrake.geometry import Meshcat
import pydrake.visualization as mut


class TestModelVisualizerReload(unittest.TestCase):
    """
    Tests the reload feature of the ModelVisualizer class.

    The reload testing is carved into a separate file vs model_visualizer_test
    because networking can be flaky on our continuous integration builds.
    """

    def test_reload(self):
        """
        Checks that the _reload() function does not crash.
        """
        # Prepare a model that should allow reloading.
        meshcat = Meshcat()
        dut = mut.ModelVisualizer(meshcat=meshcat)
        filename = "drake/multibody/benchmarks/acrobot/acrobot.sdf"
        dut.AddModels(FindResourceOrThrow(filename))
        dut.Finalize()

        # Check that it allowed reloading.
        self.assertIsNotNone(dut._reload_button_name)
        button = dut._reload_button_name
        self.assertEqual(meshcat.GetButtonClicks(button), 0)

        # Remember the originally-created diagram.
        orig_diagram = dut._diagram

        # Click the reload button.
        cli = FindResourceOrThrow("drake/geometry/meshcat_websocket_client")
        message = f"""{{
            "type": "button",
            "name": "{button}"
        }}"""
        subprocess.check_call([
            cli,
            f"--ws_url={meshcat.ws_url()}",
            f"--send_message={message}"])

        # Wait up to 5 seconds for the button click to be processed.
        for _ in range(500):
            if meshcat.GetButtonClicks(button) > 0:
                break
            time.sleep(1 / 100)
        self.assertEqual(meshcat.GetButtonClicks(button), 1)

        # Run once. If a reload() happened, the diagram will have changed out.
        # Use a non-default position so we can check that it is maintained.
        original_q = [1.0, 2.0]
        dut.Run(position=original_q, loop_once=True)
        self.assertNotEqual(id(orig_diagram), id(dut._diagram))

        # Ensure the reloaded slider and joint values are the same.
        slider_q = dut._sliders.get_output_port().Eval(
            dut._sliders.GetMyContextFromRoot(dut._context))
        self.assertListEqual(list(original_q), list(slider_q))
        joint_q = dut._diagram.plant().GetPositions(
            dut._diagram.plant().GetMyContextFromRoot(dut._context))
        self.assertListEqual(list(original_q), list(joint_q))
