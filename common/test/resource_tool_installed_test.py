"""Performs tests for resource_tool as used _after_ installation.
"""

import os
import unittest
import subprocess
import sys

import install_test_helper


class TestResourceTool(unittest.TestCase):
    def test_install_and_run(self):
        install_dir = install_test_helper.get_install_dir()
        resource_subfolder = "share/drake/"
        installed_sentinel = os.path.join(install_dir,
                                          resource_subfolder,
                                          ".drake-find_resource-sentinel")
        # Verifies that the sentinel file exists in the installed directory.
        # If it has been removed, we need to update this test.
        self.assertTrue(os.path.isfile(installed_sentinel))
        # Create a resource in the temporary directory.
        tmp_dir = install_test_helper.create_temporary_dir()

        resource_folder = os.path.join(tmp_dir, resource_subfolder)
        test_folder = os.path.join(resource_folder, "common/test")
        os.makedirs(test_folder)
        # Create sentinel file.
        sentinel = os.path.join(resource_folder,
                                os.path.basename(installed_sentinel))
        os.symlink(installed_sentinel, sentinel)
        # Create resource file.
        resource = os.path.join(test_folder, "tmp_resource")
        resource_data = "tmp_resource"
        with open(resource, "w") as f:
            f.write(resource_data)

        # Cross-check the resource root environment variable name.
        env_name = "DRAKE_RESOURCE_ROOT"
        resource_tool = os.path.join(
            install_dir, "share/drake/common/resource_tool")
        output_name = install_test_helper.check_output(
            [resource_tool,
             "--print_resource_root_environment_variable_name",
             ],
            ).strip()
        self.assertEqual(output_name, env_name)

        # Use the installed resource_tool to find a resource.
        tool_env = dict(os.environ)
        tool_env[env_name] = os.path.join(tmp_dir, "share")
        absolute_path = install_test_helper.check_output(
            [resource_tool,
             "--print_resource_path",
             "drake/common/test/tmp_resource",
             ],
            env=tool_env,
            ).strip()
        with open(absolute_path, 'r') as data:
            self.assertEqual(data.read(), resource_data)

        # Use the installed resource_tool to find a resource, but with a bogus
        # DRAKE_RESOURCE_ROOT that should be ignored.
        tool_env[env_name] = os.path.join(tmp_dir, "share", "drake")
        full_text = install_test_helper.check_output(
            [resource_tool,
             "--print_resource_path",
             "drake/examples/pendulum/Pendulum.urdf",
             ],
            env=tool_env,
            stderr=subprocess.STDOUT,
            ).strip()
        warning = full_text.splitlines()[0]
        absolute_path = full_text.splitlines()[-1]
        self.assertIn("FindResource ignoring DRAKE_RESOURCE_ROOT", warning)
        self.assertTrue(os.path.exists(absolute_path),
                        absolute_path + " does not exist")


if __name__ == '__main__':
    unittest.main()
