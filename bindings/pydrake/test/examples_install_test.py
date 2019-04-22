import os
from subprocess import check_call
import sys
import unittest

import install_test_helper


class TestExamplesInstall(unittest.TestCase):
    def test_examples(self):
        install_dir = install_test_helper.get_install_dir()
        py_dir = install_test_helper.get_python_site_packages_dir(install_dir)
        tool_env = install_test_helper.get_python_env(install_dir)

        def check_script(args):
            args = [sys.executable, os.path.join(py_dir, args[0])] + args[1:]
            check_call(args, env=tool_env)

        check_script([
            "pydrake/examples/multibody/cart_pole_passive_simulation.py",
             "--target_realtime_rate=0", "--simulation_time=0.1"])
        check_script([
            "pydrake/examples/multibody/pendulum_lqr_monte_carlo_analysis.py",
            "--num_samples=10", "--torque_limit=2.0"])


if __name__ == '__main__':
    unittest.main()
