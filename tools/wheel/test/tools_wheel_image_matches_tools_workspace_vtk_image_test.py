import unittest


class TestToolsWheelImageMatchesToolsWorkspaceVtkImage(unittest.TestCase):
    def setUp(self):
        self.maxDiff = None

    def test_dotfile_consistency(self):
        filenames = [
            "clone-vtk.sh",
            "common-definitions.sh",
            "vtk-cmake-args.sh",
        ]
        for f in filenames:
            with open(f"tools/wheel/image/{f}") as wheel:
                wheel_contents = wheel.read()
            with open(f"tools/workspace/vtk/image/{f}") as vtk:
                vtk_contents = vtk.read()
            self.assertEqual(
                wheel_contents,
                vtk_contents,
                msg=(
                    f"tools/wheel/image/{f} is not identical to "
                    f"tools/workspace/vtk/image/{f} but is expeted to be."))
