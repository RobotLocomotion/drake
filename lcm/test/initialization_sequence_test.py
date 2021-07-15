from subprocess import PIPE, STDOUT
import sys
import time
import unittest

from bazel_tools.tools.python.runfiles import runfiles

# On macOS, we don't want to support this (somewhat tricky) test case on an
# ongoing basis, we'll skip it.  (Thus, we also don't try to install psutil
# as a dependency on macOS.)
_SKIP = False
if "darwin" in sys.platform:
    _SKIP = True
else:
    import psutil


class InitializationSequenceTest(unittest.TestCase):
    """Checks for proper behavior of the `defer_initialization = true` argument
    to the DrakeLcm constructor.  Reliably testing this is a bit tricky, so
    we dedicate this entire test program for that purpose, distinct from all
    of the other functional tests.
    """

    def setUp(self):
        manifest = runfiles.Create()
        self._stub_path = manifest.Rlocation(
            "drake/lcm/initialization_sequence_test_stub")

        # We need a non-memq URL for this test to be meaningful.  (By default,
        # our configuration for the "bazel test" environment uses "memq://".)
        self._lcm_url = "udpm://239.255.76.67:7671"

    @unittest.skipIf(_SKIP, "Not supported on macOS")
    def test_worker_threads(self):
        # Launch the C++ stub.
        dut = psutil.Popen(
            [self._stub_path, f"--lcm_url={self._lcm_url}"],
            stdin=PIPE, stdout=PIPE, stderr=STDOUT)

        # Wait for it to complete constructing it's LCM instance and subscribe
        # to a channel.
        for line in dut.stdout:
            line = line.decode("utf-8").strip()
            print(f"dut says: {line}", flush=True)
            if "recv_parts_test_stub: construction is complete" in line:
                break
            time.sleep(0.1)

        # Check that it has not launched any threads, nor opened any sockets.
        self.assertEqual(dut.num_threads(), 1)
        self.assertListEqual(dut.connections(), [])

        # Prompt it to active the recv parts on the LCM instance and wait until
        # that is complete.
        dut.stdin.write("dummy\n".encode("utf-8"))
        dut.stdin.flush()
        for line in dut.stdout:
            line = line.decode("utf-8").strip()
            print(f"dut says: {line}", flush=True)
            if "recv_parts_test_stub: activation is complete" in line:
                break
            time.sleep(0.1)

        # Check that it has launched a thread and opened sockets.
        self.assertGreater(dut.num_threads(), 1)
        self.assertGreater(len(dut.connections()), 1)
