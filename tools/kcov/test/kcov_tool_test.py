import importlib.machinery
import importlib.util
from pathlib import Path
import tempfile
import unittest
import zipfile

from python import runfiles


def _load_kcov_tool():
    """Loads the kcov_tool script (which has no .py suffix) as a module."""
    manifest = runfiles.Create()
    filename = manifest.Rlocation("drake/tools/kcov/kcov_tool")
    loader = importlib.machinery.SourceFileLoader("kcov_tool", filename)
    spec = importlib.util.spec_from_loader("kcov_tool", loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


class TestKcovTool(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory(prefix="kcov_tool_test_")
        self.addCleanup(self._tmp.cleanup)
        self.dut = _load_kcov_tool()
        self.outputs = Path(self._tmp.name) / "test.outputs"
        self.outputs.mkdir()

    def _make_zip(self):
        """Makes an outputs.zip like the one bazel makes, holding kcov data."""
        with zipfile.ZipFile(self.outputs / "outputs.zip", "w") as zf:
            zf.writestr("kcov/", "")
            zf.writestr("kcov/index.html", "hello")

    def _make_subtree(self, filename="index.html"):
        (self.outputs / "kcov").mkdir()
        (self.outputs / "kcov" / filename).write_text("hello")

    def test_remove_zip_only(self):
        # Bazel leaves the data only in the zip file until someone expands it.
        self._make_zip()
        self.assertTrue(self.dut.has_kcov_data(self.outputs))
        self.dut.remove_kcov_data(self.outputs)
        self.assertFalse(self.dut.has_kcov_data(self.outputs))
        self.assertFalse((self.outputs / "outputs.zip").exists())

    def test_remove_subtree_and_zip(self):
        self._make_zip()
        self._make_subtree()
        self.dut.remove_kcov_data(self.outputs)
        self.assertFalse((self.outputs / "kcov").exists())
        self.assertFalse((self.outputs / "outputs.zip").exists())

    def test_remove_subtree_only(self):
        self._make_subtree()
        self.dut.remove_kcov_data(self.outputs)
        self.assertFalse((self.outputs / "kcov").exists())

    def test_expand_from_zip(self):
        self._make_zip()
        self.dut.maybe_expand_zipped_kcov_data(self.outputs)
        index = self.outputs / "kcov" / "index.html"
        self.assertEqual(index.read_text(), "hello")

    def test_expand_without_zip_keeps_data(self):
        # A subtree that has no index.html and no zip to restore it from is
        # left alone, not deleted, and it is not an error.
        self._make_subtree(filename="other.dat")
        self.dut.maybe_expand_zipped_kcov_data(self.outputs)
        self.assertTrue((self.outputs / "kcov" / "other.dat").exists())
