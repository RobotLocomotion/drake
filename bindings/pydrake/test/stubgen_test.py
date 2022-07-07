import os
import subprocess
import unittest


from pydrake.common import FindResourceOrThrow


class TestStubgen(unittest.TestCase):
    def test_subprocess(self):
        """For now, just a simple smoke test to ensure pybind11-stubgen runs
        at all ever.
        """
        tool = FindResourceOrThrow("drake/bindings/pydrake/stubgen")
        nerfed = "--ignore-invalid=all"
        output_dir = os.environ['TEST_TMPDIR']
        subprocess.check_call([tool, f"--output-dir={output_dir}", nerfed])
        expected = f"{output_dir}/pydrake-stubs/all/__init__.pyi"
        self.assertTrue(os.path.exists(expected))
