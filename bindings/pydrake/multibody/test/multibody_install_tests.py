from os.path import join
import sys
import unittest

import install_test_helper


class TestMultibodyInstalled(unittest.TestCase):

    def test_model_directives_to_sdformat_help(self):
        """Ensures we can call `./bin/model_directives_to_sdformat
        --help` from install.
        """
        # Get install directory.
        install_dir = install_test_helper.get_install_dir()
        # N.B. Do not update PYTHONPATH, as the script should handle that
        # itself.
        bin_path = join(install_dir, "bin", "model_directives_to_sdformat")
        text = install_test_helper.check_output([bin_path, "--help"])
        self.assertIn("usage: model_directives_to_sdformat ", text)


if __name__ == '__main__':
    unittest.main()
