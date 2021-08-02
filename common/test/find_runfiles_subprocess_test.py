import os
import subprocess
import unittest


class TestFindRunfilesSubprocess(unittest.TestCase):

    def _check_output(self, env=os.environ):
        try:
            output = subprocess.check_output([
                "common/find_runfiles_test", "-spdlog_level=trace",
                "--gtest_filter=FindRunfilesTest.AcceptanceTest",
            ], stderr=subprocess.STDOUT, env=env)
            return output.decode("utf-8")
        except subprocess.CalledProcessError as e:
            e.output = e.output.decode("utf-8")
            print(e.output)
            raise

    def test_basic_child_process(self):
        # Check that a C++ program that uses FindRunfiles succeeds when called
        # from a python binary.
        output = self._check_output()
        # Check that the C++ program used the resource paths we wanted it to:
        # - It used the special-case TEST_SRCDIR mechanism.
        self.assertIn("FindRunfile mechanism = TEST_SRCDIR", output)
        # - It used find_runfiles_subprocess_test not find_runfiles_test, i.e.,
        # the runfiles of the parent process were imbued to the subprocess.
        self.assertIn("find_runfiles_subprocess_test.runfile", output)
        self.assertNotIn("find_runfiles_test.runfile", output)

    def test_sans_test_srcdir(self):
        # Repeat test_basic_child_subprocess but with TEST_SRCDIR unset.
        # from a python binary.
        new_env = dict(os.environ)
        del new_env["TEST_SRCDIR"]
        output = self._check_output(env=new_env)
        self.assertIn("FindRunfile mechanism = RUNFILES_{MANIFEST", output)
        self.assertIn("find_runfiles_subprocess_test.runfile", output)
        self.assertNotIn("find_runfiles_test.runfile", output)

    def test_sans_all_env(self):
        # Repeat test_basic_child_subprocess but with an empty environment.
        # This covers the "argv0 mechanism" codepaths.
        try:
            output = self._check_output(env={})
            status = 0
        except subprocess.CalledProcessError as e:
            output = e.output
            status = e.returncode
        # If the user-facing binary bazel-bin/common/find_runfiles_test happens
        # to already exist then the subprocess's AcceptanceTest case passes.
        #
        # If instead only bazel-out/k8-opt/bin/common/find_runfiles_test exists
        # (and it will always exist, because it is a data dependency of this
        # python test), then the subprocess's AcceptanceTest case fails.
        #
        # Either way, should report that its using argv0 and not segfault.
        self.assertIn("FindRunfile mechanism = argv0", output)
        self.assertIn(status, (0, 1))
