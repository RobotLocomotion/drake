"""Performs tests for resource_tool as used _after_ installation."""

import os
from pathlib import Path
import subprocess
import sys
import unittest

import install_test_helper


class TestResourceTool(unittest.TestCase):
    def setUp(self):
        # Establish the path to resource_tool.
        self._install_dir = Path(install_test_helper.get_install_dir())
        self._resource_tool = (
            self._install_dir / "share/drake/common/resource_tool"
        )
        self.assertTrue(self._resource_tool.is_file())

        # Establish the path to the sentinel dotfile.
        self._resource_subfolder = Path("share/drake")
        self._installed_sentinel = (
            self._install_dir
            / self._resource_subfolder
            / ".drake-find_resource-sentinel"
        )
        self.assertTrue(self._installed_sentinel.is_file())

        # A valid, well-known resource path.
        self._resource = "drake/examples/pendulum/Pendulum.urdf"

    def test_basic_finding(self):
        """The installed resource_tool uses the installed files by default."""
        absolute_path = install_test_helper.check_output(
            [self._resource_tool, "--print_resource_path", self._resource],
            stderr=subprocess.STDOUT,
        ).strip()
        self.assertTrue(
            Path(absolute_path).is_file(),
            f"Path does not exist: {absolute_path}",
        )

    @unittest.skipIf(
        sys.platform == "darwin",
        "Our scaffolding uses LD_LIBRARY_PATH, which is not a thing on macOS",
    )
    def test_symlinked_finding(self):
        # Prepare a copy of resource_tool that loads its library via a symlink.
        special_bin = self._install_dir / "foo/bar/baz/quux/bin"
        special_bin.mkdir(parents=True)
        (special_bin / "libdrake_marker.so").symlink_to(
            self._install_dir / "lib/libdrake_marker.so"
        )
        env = {"LD_LIBRARY_PATH": str(special_bin)}
        special_tool = special_bin / "resource_tool"
        special_tool.hardlink_to(self._resource_tool)
        absolute_path = install_test_helper.check_output(
            [special_tool, "--print_resource_path", self._resource],
            env=env,
            stderr=subprocess.STDOUT,
        ).strip()
        self.assertTrue(
            Path(absolute_path).is_file(),
            f"Path does not exist: {absolute_path}",
        )

    def test_ignores_runfiles(self):
        """The installed resource_tool ignores non-Drake runfiles."""
        tool_env = dict(os.environ)
        tool_env["TEST_SRCDIR"] = "/tmp"
        absolute_path = install_test_helper.check_output(
            [self._resource_tool, "--print_resource_path", self._resource],
            env=tool_env,
            stderr=subprocess.STDOUT,
        ).strip()
        self.assertTrue(
            Path(absolute_path).is_file(),
            f"Path does not exist: {absolute_path}",
        )

    def test_resource_root_environ(self):
        """The installed resource_tool obeys DRAKE_RESOURCE_ROOT."""
        # Create a resource in the temporary directory.
        tmp_dir = Path(install_test_helper.create_temporary_dir())

        resource_folder = tmp_dir / self._resource_subfolder
        test_folder = resource_folder / "common/test"
        test_folder.mkdir(parents=True)
        # Create sentinel file.
        sentinel = resource_folder / self._installed_sentinel.name
        sentinel.symlink_to(self._installed_sentinel)
        # Create resource file.
        resource = test_folder / "tmp_resource"
        resource_data = "tmp_resource"
        with open(resource, "w") as f:
            f.write(resource_data)

        # Cross-check the resource root environment variable name.
        env_name = "DRAKE_RESOURCE_ROOT"
        output_name = install_test_helper.check_output(
            [
                self._resource_tool,
                "--print_resource_root_environment_variable_name",
            ]
        ).strip()
        self.assertEqual(output_name, env_name)

        # Use the installed resource_tool to find a resource.
        tool_env = dict(os.environ)
        tool_env[env_name] = tmp_dir / "share"
        absolute_path = install_test_helper.check_output(
            [
                self._resource_tool,
                "--print_resource_path",
                "drake/common/test/tmp_resource",
            ],
            env=tool_env,
        ).strip()
        with open(absolute_path, "r") as data:
            self.assertEqual(data.read(), resource_data)

        # Use the installed resource_tool to find a resource, but with a bogus
        # DRAKE_RESOURCE_ROOT that should be ignored.
        tool_env[env_name] = tmp_dir / "share/drake"
        full_text = install_test_helper.check_output(
            [self._resource_tool, "--print_resource_path", self._resource],
            env=tool_env,
            stderr=subprocess.STDOUT,
        ).strip()
        warning = full_text.splitlines()[0]
        absolute_path = full_text.splitlines()[-1]
        self.assertIn("FindResource ignoring DRAKE_RESOURCE_ROOT", warning)
        self.assertTrue(
            Path(absolute_path).is_file(),
            f"Path does not exist: {absolute_path}",
        )


if __name__ == "__main__":
    unittest.main()
