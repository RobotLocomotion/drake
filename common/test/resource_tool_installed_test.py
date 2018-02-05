"""Performs tests for resource_tool as used _after_ installation.
"""

import os
import shutil
import subprocess
import sys
import unittest


# Set on command-line to the location of common/install.
_install_exe = None


class TestResourceTool(unittest.TestCase):
    def test_install_and_run(self):
        # Install into a temporary directory.
        os.mkdir("tmp")
        subprocess.check_call(
            [_install_exe,
             os.path.abspath("tmp"),
             ])

        # Create a resource in the temporary directory.
        os.makedirs("tmp/share/drake/common/test")
        with open("tmp/share/drake/common/test/tmp_resource", "w") as f:
            f.write("tmp_resource")

        # Remove the un-installed copy, so we _know_ it won't be used.
        content_test_folder = os.listdir(os.getcwd())
        content_test_folder.remove("tmp")
        for element in content_test_folder:
            if os.path.isdir(element):
                shutil.rmtree(element)
            else:
                os.remove(element)
        self.assertEqual(os.listdir(os.getcwd()), ["tmp"])

        # Cross-check the resource root environment variable name.
        env_name = "DRAKE_RESOURCE_ROOT"
        resource_tool = "tmp/share/drake/common/resource_tool"
        output_name = subprocess.check_output(
            [resource_tool,
             "--print_resource_root_environment_variable_name",
             ],
            ).strip()
        self.assertEqual(output_name, env_name)

        # Use the installed resource_tool to find a resource.
        tool_env = dict(os.environ)
        tool_env[env_name] = "tmp/share"
        absolute_path = subprocess.check_output(
            [resource_tool,
             "--print_resource_path",
             "drake/common/test/tmp_resource",
             ],
            env=tool_env,
            ).strip()
        with open(absolute_path, 'r') as data:
            self.assertEqual(data.read(), "tmp_resource")

        # Remove environment variable.
        absolute_path = subprocess.check_output(
            [resource_tool,
             "--print_resource_path",
             "drake/common/test/tmp_resource",
             "--add_resource_search_path",
             "tmp/share",
             ],
            ).strip()
        with open(absolute_path, 'r') as data:
            self.assertEqual(data.read(), "tmp_resource")


if __name__ == '__main__':
    _install_exe = sys.argv.pop(1)
    unittest.main()
