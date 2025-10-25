from os.path import join
import unittest

import install_test_helper


class TestVisualizationInstalled(unittest.TestCase):
    def test_meldis_help(self):
        """Ensures we can call `./bin/meldis --help` from install."""
        # Get install directory.
        install_dir = install_test_helper.get_install_dir()
        # N.B. Do not update PYTHONPATH, as the script should handle that
        # itself.
        bin_path = join(install_dir, "bin", "meldis")
        text = install_test_helper.check_output(
            [install_test_helper.get_python_executable(), bin_path, "--help"]
        )
        self.assertIn("usage: meldis ", text)

    def test_model_visualizer_help(self):
        """Ensures we can call `./bin/model_visualizer --help` from install."""
        # Get install directory.
        install_dir = install_test_helper.get_install_dir()
        # N.B. Do not update PYTHONPATH, as the script should handle that
        # itself.
        bin_path = join(install_dir, "bin", "model_visualizer")
        text = install_test_helper.check_output(
            [install_test_helper.get_python_executable(), bin_path, "--help"]
        )
        self.assertIn("usage: model_visualizer ", text)

    def test_drake_models_meshes(self):
        """Ensures that the package://drake_models/... can be found by testing
        a model that uses a meshfile from that location.
        """
        install_dir = install_test_helper.get_install_dir()
        install_test_helper.check_call(
            [
                install_test_helper.get_python_executable(),
                join(install_dir, "bin", "model_visualizer"),
                "--loop_once",
                "package://drake_models/ycb/meshes/004_sugar_box_textured.obj",
            ]
        )


if __name__ == "__main__":
    unittest.main()
