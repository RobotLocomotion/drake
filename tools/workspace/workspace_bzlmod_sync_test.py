import unittest
from pathlib import Path

from python import runfiles


class TestWorkspaceBzlmodSync(unittest.TestCase):

    def _read(self, respath):
        """Returns the contents of the given resource path."""
        manifest = runfiles.Create()
        path = Path(manifest.Rlocation(respath))
        return path.read_text(encoding="utf-8")

    def _parse_modules(self):
        """Parses MODULE.bazel to return a dictionary mapping from module_name
        to module_version.
        """
        content = self._read(f"drake/MODULE.bazel")
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

    def _parse_repo_rule_version(self, repo_name):
        """Parses tools/workspace/{repo_name}/repository.bzl to find the call
        to 'github_archive' and returns the version number it pins to.
        """
        content = self._read(
            f"drake/tools/workspace/{repo_name}/repository.bzl")
        assert "github_archive" in content, content
        for line in content.splitlines():
            line = line.strip()
            if not line.startswith("commit = "):
                continue
            _, version, _ = line.split('"')
            if version.startswith("v"):
                version = version[1:]
            return version
        self.fail(f"No 'commit = ...' found in:\n{content}")

    def _module_name_to_repo_name(self, module_name):
        """Some of our repository rules have a different repository name
        compared to their module name for bzlmod. Given a module name, returns
        the expected repository name.
        """
        if module_name == "apple_support":
            return "build_bazel_apple_support"
        return module_name

    def test_version_sync(self):
        """Some external version are independently listed in both MODULE.bazel
        and WORKSPACE. This test ensures that the versions pinned in each file
        are correctly synchronized.
        """
        modules = self._parse_modules()

        # Don't check modules that are known to be module-only.
        del modules["bazel_features"]

        # Don't check modules whose repository rule twin is pkgconfig (and thus
        # doesn't have a github pinned version that we need to keep in sync).
        del modules["eigen"]
        del modules["fmt"]
        del modules["spdlog"]

        # Don't check module that are documented to purposefully skew versions.
        del modules["rules_cc"]
        del modules["rules_python"]

        # Check that the module version matches the workspace version.
        self.assertTrue(modules)
        for module_name, module_version in modules.items():
            repo_name = self._module_name_to_repo_name(module_name)
            workspace_version = self._parse_repo_rule_version(repo_name)
            self.assertEqual(workspace_version, module_version)

    def _parse_workspace_list_constant(self, name):
        """Returns the contents of the list constant named `name` in our
        tools/workspace/default.bzl.
        """
        content = self._read("drake/tools/workspace/default.bzl")
        result = None
        for line in content.splitlines():
            line = line.strip()
            if line == f"{name} = [":
                result = list()
                continue
            if result is None:
                # We haven't seen the opening line yet.
                continue
            if line == "]":
                break
            assert line.startswith('"'), line
            assert line.endswith('",'), line
            result.append(line[1:-2])
        assert result, content
        return sorted(result)

    def test_default_exclude_sync(self):
        """Our default.bzl needs to know the list of modules that are already
        provided by MODULE.bazel. This test ensures that the list is correctly
        synchronized.
        """
        modules = self._parse_modules()

        # These workspace-only repositories are irrelevant for bzlmod.
        modules["rust_toolchain"] = None

        repo_names_in_module = sorted([
            self._module_name_to_repo_name(module_name)
            for module_name in modules.keys()
        ])
        repo_names_in_default = self._parse_workspace_list_constant(
            name="REPOS_ALREADY_PROVIDED_BY_BAZEL_MODULES")
        self.assertEqual(repo_names_in_module, repo_names_in_default)

    def _parse_module_drake_dep_repositories(self):
        """Parses MODULE.bazel to return the list of drake_dep_repositories.
        """
        content = self._read(f"drake/MODULE.bazel")
        result = None
        for line in content.splitlines():
            line = line.strip()
            if line == "drake_dep_repositories,":
                result = list()
                continue
            if result is None:
                # We haven't seen the opening line yet.
                continue
            if line == ")":
                break
            assert line.startswith('"'), line
            assert line.endswith('",'), line
            result.append(line[1:-2])
        assert result, content
        return sorted(result)

    def test_default_exported_sync(self):
        """Our default.bzl has a list of REPOS_EXPORTED that must match the
        drake_dep_repositories listed in MODULE.bazel. This test ensures that
        the lists are correctly synchronized.
        """
        repo_names_in_module = self._parse_module_drake_dep_repositories()
        repo_names_in_default = self._parse_workspace_list_constant(
            name="REPOS_EXPORTED")
        self.assertEqual(repo_names_in_module, repo_names_in_default)


assert __name__ == '__main__'
unittest.main()
