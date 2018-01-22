#!/usr/bin/env python

"""Performs tests for resource_tool as used _after_ installation.
"""

import os
import subprocess
import unittest
import install_test_helper


class TestResourceTool(unittest.TestCase):
    def test_install_and_run(self):
        # Create a resource in the temporary directory.
        install_dir = install_test_helper.get_install_dir()

        test_folder = os.path.join(install_dir, "share/drake/common/test")
        os.makedirs(test_folder)
        resource = os.path.join(test_folder, "tmp_resource")
        resource_data = "tmp_resource"
        with open(resource, "w") as f:
            f.write(resource_data)

        # Cross-check the resource root environment variable name.
        env_name = "DRAKE_RESOURCE_ROOT"
        resource_tool = os.path.join(
            install_dir, "share/drake/common/resource_tool")
        output_name = subprocess.check_output(
            [resource_tool,
             "--print_resource_root_environment_variable_name",
             ],
            cwd='/',
            ).strip()
        self.assertEqual(output_name, env_name)

        # Use the installed resource_tool to find a resource.
        tool_env = dict(os.environ)
        tool_env[env_name] = os.path.join(install_dir, "share")
        absolute_path = subprocess.check_output(
            [resource_tool,
             "--print_resource_path",
             "drake/common/test/tmp_resource",
             ],
            env=tool_env,
            ).strip()
        with open(absolute_path, 'r') as data:
            self.assertEqual(data.read(), resource_data)

        # Remove environment variable.
        absolute_path = subprocess.check_output(
            [resource_tool,
             "--print_resource_path",
             "drake/common/test/tmp_resource",
             "--add_resource_search_path",
             os.path.join(install_dir, "share"),
             ],
            ).strip()
        with open(absolute_path, 'r') as data:
            self.assertEqual(data.read(), resource_data)


if __name__ == '__main__':
    unittest.main()
