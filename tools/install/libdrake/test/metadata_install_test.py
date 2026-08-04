import os
import unittest

import install_test_helper


class MetadataTest(unittest.TestCase):
    def test_importlib_metadata(self):
        """Checks that installed pydrake can report its version number."""
        # Override PYTHONPATH to only use the installed `pydrake` module.
        install_dir = install_test_helper.get_install_dir()
        tool_env = dict(os.environ)
        tool_env["PYTHONPATH"] = (
            install_test_helper.get_python_site_packages_dir(install_dir)
        )

        # Run importlib against the installed `pydrake` module.
        version = install_test_helper.check_output(
            [
                install_test_helper.get_python_executable(),
                "-c",
                "import importlib.metadata;\
               print(importlib.metadata.version('drake'))",
            ],
            env=tool_env,
        ).strip()

        # On "Packaging" builds have a version number; here, we expect the
        # sentinel value (as opposed to crashing with PackageNotFoundError).
        self.assertEqual(version, "unknown")


if __name__ == "__main__":
    unittest.main()
