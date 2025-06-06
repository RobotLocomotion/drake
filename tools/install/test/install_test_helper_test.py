import os
import unittest
import install_test_helper
from install_test_helper import IS_JAMMY_DEBUG


class TestInstallTestHelperTest(unittest.TestCase):
    @unittest.skipIf(IS_JAMMY_DEBUG, "ld.mold is too old")
    def test_get_install_dir(self):
        self.assertIn("TEST_TMPDIR", os.environ)
        self.assertIn('installation', install_test_helper.get_install_dir())

    @unittest.skipIf(IS_JAMMY_DEBUG, "ld.mold is too old")
    def test_create_temporary_dir(self):
        subdirectory_name = "tmp"
        tmp_dir = install_test_helper.create_temporary_dir(subdirectory_name)
        self.assertIn(subdirectory_name, tmp_dir)
        self.assertTrue(os.path.isdir(tmp_dir))

    @unittest.skipIf(IS_JAMMY_DEBUG, "ld.mold is too old")
    def test_get_python_executable(self):
        self.assertIn("python3", install_test_helper.get_python_executable())

    @unittest.skipIf(IS_JAMMY_DEBUG, "ld.mold is too old")
    def test_run_and_kill(self):
        python = install_test_helper.get_python_executable()
        install_test_helper.run_and_kill([python, "-c",
                                         "import time; time.sleep(5)"], 0.5,
                                         from_install_dir=False)

    @unittest.skipIf(IS_JAMMY_DEBUG, "ld.mold is too old")
    def test_check_call(self):
        python = install_test_helper.get_python_executable()
        install_test_helper.check_call([python, "--help"])

    @unittest.skipIf(IS_JAMMY_DEBUG, "ld.mold is too old")
    def test_check_output(self):
        python = install_test_helper.get_python_executable()
        output = install_test_helper.check_output([python, "--help"])
        self.assertIn('PYTHONPATH', output)

    @unittest.skipIf(IS_JAMMY_DEBUG, "ld.mold is too old")
    def test_read_only(self):
        tmp_dir = os.environ['TEST_TMPDIR']
        tmp_file = os.path.join(tmp_dir, "test_file")
        with open(tmp_file, "w") as f:
            f.write("")
        install_test_helper._make_read_only(tmp_file)
        try:
            with open(tmp_file, "w") as f:
                f.write("")
            self.fail("File %s was writable!" % tmp_file)
        except IOError as e:
            # Test is suppose to raise an exception.
            if e.errno == 13:  # Permission denied
                pass
            else:
                raise e
