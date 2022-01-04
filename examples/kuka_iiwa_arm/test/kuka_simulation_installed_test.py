import os
import subprocess
import unittest

import install_test_helper


class TestKukaSimulation(unittest.TestCase):
    def test_install(self):
        """Makes sure the simulation can run without error."""
        install_dir = install_test_helper.get_install_dir()
        simulation = os.path.join(
            install_dir,
            "share/drake/examples/kuka_iiwa_arm/kuka_simulation")
        self.assertTrue(os.path.exists(simulation), "Can't find " + simulation)
        console_output = install_test_helper.check_output(
            [simulation, "--simulation_sec=0.01"], stderr=subprocess.STDOUT)
        self.assertIn("kuka_simulation outside of Drake", console_output)


if __name__ == '__main__':
    unittest.main()
