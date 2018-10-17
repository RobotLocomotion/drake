import os
import unittest

import install_test_helper


class TestKukaSimulation(unittest.TestCase):
    def test_install(self):
        # Get install directory
        install_dir = install_test_helper.get_install_dir()
        # Make sure the simulation can run without error.
        simulation = os.path.join(
            install_dir,
            "share/drake/examples/kuka_iiwa_arm/kuka_simulation")
        self.assertTrue(os.path.exists(simulation), "Can't find " + simulation)
        install_test_helper.check_call([simulation, "--simulation_sec=0.01"])


if __name__ == '__main__':
    unittest.main()
