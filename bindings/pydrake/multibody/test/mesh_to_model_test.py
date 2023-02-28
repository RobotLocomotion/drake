"""Unit tests for mesh_to_model."""

from pydrake.multibody._mesh_model_maker import (
    MeshModelMaker,
    MeshFramePose,
    MeshMassSpec,
)

import os
import unittest

from pydrake.common import (
    FindResourceOrThrow,
    temp_directory,
)

# What do I need to test?
#
#  - Parameters
#    - obj_path
#       - valid
#       - invalid
#           - not obj
#           - not exists
#    - scale
#      - does the sign matter?
#      - Different scale --> different mass/inertia
#    - model_name
#      - None/empty --> obj name.
#      - non-empty string is used (body and model).
#    - mass_spec
#      - Initializing neither throws
#      - Define density & mass --> mass wins.
#      - Define mass only --> mass wins.
#       - Define density only --> density used.
#    - frame_pose
#      - at_com(True), p_GoBo(not None) --> throws
#      - at_com(False), p_GoBo(None): p_GoBo = [0, 0 ,0]
#      - at_com(True), p_GoBo(None): p_GoBo = p_GoGcm
#      - at_com(False, p_GoBo(not None): p_GoBo = p_GoBo
#    - no_collision
#       - simply on or off
#    - no_visual
#       - simply on or off
#    - encoded_package
#       - Valid (assuming obj can be parsed)
#         - "none"
#         - explicit "package.xml"
#           - Name different from directory, name is used.
#         - "auto"
#           - Existing package.xml
#               - same as explicit package.xml
#           - No package.xml
#               - File written with directory name.
#       - invalid
#         - explicit "package.xml"
#           - File can't be read
#           - isn't manifest/doesn't have <name>*</name> tag.
#         - "auto"
#           - failure modes of explicit package.xml


class TestObjToModel(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._obj_path = FindResourceOrThrow(
            "drake/geometry/render/test/meshes/box.obj")
        cls._temp_dir = temp_directory()

    def test_simple_valid_invocation(self):
        """Smoke test that a straightforward invocation works."""
        maker = MeshModelMaker()
        maker.scale = 1.0
        maker.model_name = None
        maker.mass_spec = MeshMassSpec(density=1)
        maker.frame_pose = MeshFramePose()
        maker.no_collision = False
        maker.no_visual = False
        maker.encoded_package = "none"

        sdf_result = os.path.join(self._temp_dir, "simple_invoke.sdf")
        maker.make_model(self._obj_path, sdf_result)

        with open(sdf_result) as f:
            xml = f.readline()
            self.assertRegex(xml, "<?xml.*")

# What do I need to test
# For each CLI parameter, confirm that they percolate through.
# - Mass spec - mutually exclusive
# - Frame pose - mutually exclusive
# Parameters get defaulted appropriately (i.e., it creates a file).
#   - this would largely be a smoke test that simply invoking it with an obj
#     does *something*.


class TestObjToModelWrapper(unittest.TestCase):
    """Tests the CLI to the functionality."""
    pass
