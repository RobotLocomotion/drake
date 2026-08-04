from pathlib import Path
import unittest

from python import runfiles


class TestBzlmodSync(unittest.TestCase):
    def _read(self, respath):
        """Returns the contents of the given resource path."""
        manifest = runfiles.Create()
        path = Path(manifest.Rlocation(respath))
        return path.read_text(encoding="utf-8")

    def _parse_modules(self, respath):
        """Parses a MODULE.bazel file to return a dictionary mapping from
        module_name to module_version.
        """
        content = self._read(respath)
        result = {}
        for line in content.splitlines():
            # Only match bazel_dep lines.
            if not line.startswith("bazel_dep"):
                continue
            # Grab what's inside the parens.
            if "(" not in line or ")" not in line:
                continue
            _, line = line.split("(", maxsplit=1)
            line, _ = line.split(")", maxsplit=1)
            # Parse out the kwargs.
            kwargs = {}
            for item in line.split(","):
                name, value = item.split(" = ")
                kwargs[name.strip()] = value.strip().replace('"', "")
            if "version" in kwargs:
                result[kwargs["name"]] = kwargs["version"]
        return result

    def test_version_sync(self):
        """Checks that our cmake/MODULE.bazel.in file remains up-to-date with
        respect to our top-level MODULE.bazel file.
        """
        top = self._parse_modules("drake/MODULE.bazel")
        cmake = self._parse_modules("drake/cmake/MODULE.bazel.in")
        modules_to_sync = [
            "apple_support",
            "rules_python",
        ]
        for item in modules_to_sync:
            with self.subTest(module_to_sync=item):
                self.assertEqual(cmake[item], top[item])


assert __name__ == "__main__"
unittest.main()
