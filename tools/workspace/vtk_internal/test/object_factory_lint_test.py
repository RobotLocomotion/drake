from pathlib import Path
import re
import unittest

from python import runfiles

_HOW_TO_FIX = """\n
If you're seeing this, Drake's copy of the generated VTK object factory is out
of date with the VTK source (typically after upgrading vtk_internal). Refresh
it by following the instructions in tools/workspace/vtk_internal/gen/README.md.
"""


class ObjectFactoryLintTest(unittest.TestCase):
    def _read(self, respath):
        """Returns the contents of the given resource path."""
        manifest = runfiles.Create()
        path = Path(manifest.Rlocation(respath))
        return path.read_text(encoding="utf-8")

    def _upstream_overrides(self):
        """Returns the set of override class names that upstream's CMake
        declares unconditionally for VTK::RenderingOpenGL2.
        """
        cmakelists = self._read("vtk_internal/Rendering/OpenGL2/CMakeLists.txt")
        match = re.search(r"set\(opengl_overrides\s+([^)]*)\)", cmakelists)
        self.assertIsNotNone(match, "Can't find opengl_overrides in VTK.")
        result = {f"vtkOpenGL{x}" for x in match.group(1).split()}
        # This one is declared outside of the opengl_overrides list.
        self.assertIn("OVERRIDE vtkDummyGPUInfoList", cmakelists)
        result.add("vtkDummyGPUInfoList")
        return result

    def _drake_overrides(self):
        """Returns the sets of override class names that Drake's copy of the
        generated factory registers, and that it has commented out.
        """
        factory = self._read(
            "drake/tools/workspace/vtk_internal/gen/"
            "vtkRenderingOpenGL2ObjectFactory.cxx"
        )
        pattern = r"^{}VTK_CREATE_CREATE_FUNCTION\((\w+)\)"
        registered = set(re.findall(pattern.format(""), factory, re.MULTILINE))
        removed = set(re.findall(pattern.format("// "), factory, re.MULTILINE))
        return registered, removed

    def test_overrides_up_to_date(self):
        """Checks that every override that VTK declares is either registered
        by Drake's copy of the generated object factory, or explicitly removed
        for Drake (and vice versa). A missing override is otherwise silent:
        VTK instantiates the (possibly no-op) base class instead.
        """
        upstream = self._upstream_overrides()
        registered, removed = self._drake_overrides()
        # The vtkRenderWindow override is declared conditionally (per platform)
        # by upstream, so it isn't part of the `upstream` set.
        removed.discard("vtkOpenGLRenderWindow")
        self.assertSetEqual(registered | removed, upstream, _HOW_TO_FIX)
        self.assertSetEqual(registered & removed, set(), _HOW_TO_FIX)
