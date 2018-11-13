import signal
import subprocess
import sys
import time
import unittest


class TestMeshcatServer(unittest.TestCase):
    def test_run_meshcat_server_and_kill(self):
        self.assertEqual(len(sys.argv), 2)
        process = subprocess.Popen(sys.argv[1:])
        start = time.time()
        while time.time() - start < 2.0:
            time.sleep(0.25)
            process.poll()
            self.assertIsNone(process.returncode)
        process.terminate()
        process.wait()
        self.assertEqual(process.returncode, -signal.SIGTERM)
