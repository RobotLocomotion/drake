import unittest
import install_test_helper


class TestKukaSimulation(unittest.TestCase):
    def test_install(self):
        install_test_helper.run_and_kill(
            ['share/drake/examples/kuka_iiwa_arm/kuka_plan_runner'])


if __name__ == '__main__':
    unittest.main()
