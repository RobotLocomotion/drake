import unittest
from pathlib import Path

from python import runfiles


class TestWorkspaceBzlmodSync(unittest.TestCase):

    def _read(self, respath):
        """Returns the contents of the given resource path."""
        manifest = runfiles.Create()
        path = Path(manifest.Rlocation(respath))
        return path.read_text(encoding="utf-8")

    def _parse_modules(self, content):
        """Given the contents of MODULE.bazel, returns a dictionary mapping
        from module_name to module_version.
        """
        result = {}
        for line in content.splitlines():
            # Only match bazel_dep lines.
            if not line.startswith("bazel_dep"):
                continue
            # Grab what's inside the parens.
            _, line = line.split("(")
            line, _ = line.split(")")
            # Parse out the kwargs.
            kwargs = {}
            for item in line.split(","):
                name, value = item.split(" = ")
                kwargs[name.strip()] = value.strip().replace('"', '')
            result[kwargs["name"]] = kwargs["version"]
        return result

    def _parse_workspace_bzlmod(self, content):
        """Given the contents of WORKSPACE.bzlmod, returns the list of
        REPOS_ALREADY_PROVIDED_BY_BAZEL_MODULES.
        """
        result = None
        for line in content.splitlines():
            line = line.strip()
            if line == "REPOS_ALREADY_PROVIDED_BY_BAZEL_MODULES = [":
                result = list()
                continue
            if result is None:
                # We haven't seen the REPOS_ALREADY_... line yet.
                continue
            if line == "]":
                break
            assert line.startswith('"'), line
            assert line.endswith('",'), line
            result.append(line[1:-2])
        assert result, content
        return sorted(result)

    def _parse_repo_rule(self, content):
        """Given the contents of a repository.bzl that calls 'github_archive',
        returns the version number it pins to.
        """
        assert "github_archive" in content, content
        for line in content.splitlines():
            line = line.strip()
            if not line.startswith("commit = "):
                continue
            _, version, _ = line.split('"')
            return version
        self.fail(f"No 'commit = ...' found in:\n{content}")

    def test_version_sync(self):
        """Some external version are independently listed in both MODULE.bazel
        and WORKSPACE. This test ensures that the versions pinned in each file
        are correctly synchronized.
        """
        modules = self._parse_modules(self._read(f"drake/MODULE.bazel"))

        # Don't check modules that are known to be module-only.
        del modules["rules_java"]

        # Don't check module that are documented to purposefully skew versions.
        del modules["rules_python"]

        # Check that the module version matches the workspace version.
        self.assertTrue(modules)
        for name, module_version in modules.items():
            workspace_version = self._parse_repo_rule(
                self._read(f"drake/tools/workspace/{name}/repository.bzl"))
            self.assertEqual(workspace_version, module_version)

    def test_cmake_module_sync(self):
        """WORKSPACE.bzlmod needs to know the list of modules that are already
        provided by MODULE.bazel. This test ensures that the list is correctly
        synchronized.
        """
        modules = self._parse_modules(self._read(f"drake/MODULE.bazel"))

        # Don't check modules that are known to be module-only.
        del modules["rules_java"]

        # Check that the workspace lists match the inventory of modules.
        module_names = sorted(modules.keys())
        self.assertEqual(module_names, self._parse_workspace_bzlmod(
            self._read("drake/WORKSPACE.bzlmod")))
        self.assertEqual(module_names, self._parse_workspace_bzlmod(
            self._read("drake/cmake/WORKSPACE.bzlmod.in")))


assert __name__ == '__main__'
unittest.main()
