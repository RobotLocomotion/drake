"""Performs tests for resource_tool as used _after_ installation.
"""

import subprocess
import os
import unittest


class TestResourceTool(unittest.TestCase):
    def test_install_and_run(self):
        # Install into a temporary directory.
        os.mkdir("tmp")
        subprocess.check_call(
            ["drake/common/install",
             os.path.abspath("tmp"),
             ])

        # Create a resource in the temporary directory.
        os.makedirs("tmp/share/drake/drake/common/test")
        with open("tmp/share/drake/drake/common/test/tmp_resource", "w") as f:
            f.write("tmp_resource")

        # Remove the un-installed copy, so we _know_ it won't be used.
        os.remove(".drake-resource-sentinel")
        os.remove("drake/__init__.py")
        os.remove("drake/common/__init__.py")
        os.remove("drake/common/install")
        os.remove("drake/common/resource_tool")
        os.remove("drake/common/resource_tool_installed_test")
        os.remove("drake/common/test/__init__.py")
        os.remove("drake/common/test/resource_tool_installed_test.py")
        os.rmdir("drake/common/test")
        os.rmdir("drake/common")
        os.rmdir("drake")
        self.assertEqual(os.listdir("."), ["tmp"])

        # Cross-check the resource root environment variable name.
        env_name = "DRAKE_RESOURCE_ROOT"
        resource_tool = "tmp/libexec/drake/drake/common/resource_tool"
        output_name = subprocess.check_output(
            [resource_tool,
             "--print_resource_root_environment_variable_name",
             ],
            ).strip()
        self.assertEqual(output_name, env_name)

        # Use the installed resource_tool to find a resource.
        tool_env = dict(os.environ)
        tool_env[env_name] = "tmp/share/drake"
        absolute_path = subprocess.check_output(
            [resource_tool,
             "--print_resource_path",
             "drake/common/test/tmp_resource",
             ],
            env=tool_env,
            ).strip()
        with open(absolute_path, 'r') as data:
            self.assertEqual(data.read(), "tmp_resource")


if __name__ == '__main__':
    unittest.main()
