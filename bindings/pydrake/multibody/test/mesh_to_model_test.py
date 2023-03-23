"""Unit tests for mesh_to_model."""

from pydrake.multibody._mesh_model_maker import (
    MeshModelMaker,
    MeshFramePose,
    MeshMassSpec,
)

import logging
import os
import re
import unittest

from pydrake.common import (
    FindResourceOrThrow,
    temp_directory,
)


class LogRecorder(logging.Handler):
    """Simply records all logged messages."""
    def __init__(self):
        logging.Handler.__init__(self)
        self._records = []

    def clear(self):
        self._records = []

    def emit(self, record):
        self._records.append(record)

    def hasRecordRegex(self, level, regex):
        """
        Reports if there's a stored record of the given level whose message
        matches the given regex.
        """
        for record in self._records:
            if (record.levelno == level
                    and re.search(regex, record.getMessage())):
                return True
        return False

# What do I need to test?
#
#  - Parameters
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
#    - I need a test showing the resulting sdf gets "successfully" parsed.


class TestModelMaker(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._obj_stem = "box"
        cls._obj_path = FindResourceOrThrow(
            f"drake/geometry/render/test/meshes/{cls._obj_stem}.obj")
        cls._temp_dir = temp_directory()
        cls._logger = logging.getLogger("drake")
        cls._logger.setLevel(logging.INFO)

    def setUp(self):
        self.records = LogRecorder()
        self._logger.addHandler(self.records)

    def tearDown(self):
        self._logger.removeHandler(self.records)

    @staticmethod
    def _make_default_maker():
        """Returns a maker with a simple, valid configuration."""
        maker = MeshModelMaker()
        maker.scale = 1.0
        maker.model_name = None
        maker.mass_spec = MeshMassSpec(density=1)
        maker.frame_pose = MeshFramePose()
        maker.no_collision = False
        maker.no_visual = False
        maker.encoded_package = "none"
        return maker

    @staticmethod
    def _find_in_file(filename, regex):
        """Reports if the given regex matches any _line_ in the file."""
        print(f"Matching {regex}")
        with open(filename) as f:
            text = ''.join(f.readlines())
        print(text)
        return re.search(regex, text) is not None

    def test_simple_valid_invocation(self):
        """Smoke test that a straightforward invocation works."""
        maker = self._make_default_maker()

        sdf_result = os.path.join(self._temp_dir, "simple_invoke.sdf")
        maker.make_model(self._obj_path, sdf_result)

        self.assertTrue(os.path.exists(sdf_result))
        self.assertTrue(self._find_in_file(sdf_result, "<?xml.*"))

    def test_invalid_obj(self):
        maker = self._make_default_maker()
        sdf_result = os.path.join(self._temp_dir, "invalid_obj.sdf")

        # Invalid because it doesn't exist.
        with self.assertRaisesRegex(RuntimeError, "Cannot open file.*"):
            maker.make_model('invalid_path.obj', sdf_result)
        self.assertFalse(os.path.exists(sdf_result))

        # Invalid because it isn't an OBJ.
        not_an_obj = os.path.join(self._temp_dir, 'not_an_obj.txt')
        with open(not_an_obj, 'w') as f:
            f.write("Just some text\n")
        with self.assertRaisesRegex(RuntimeError, ".+obj data has no.+"):
            maker.make_model(not_an_obj, sdf_result)
        self.assertFalse(os.path.exists(sdf_result))

    def test_scale(self):
        """
        Confirm the scale parameter affects the output sdf. The scale factor
        should have three implications:

            - Reported bounding box changes.
            - Scale factor in SDFormat file changes.
            - Mass properties change.

        We don't explicitly test the inertia tensor because we are scaling the
        mesh directly and computing the mass properties on that mesh. Simply
        confirming the mass is correct should imply the inertia is also good.
        """
        maker = self._make_default_maker()
        sdf_result = os.path.join(self._temp_dir, "scaled.sdf")

        # Unit scale has a baseline 2x2x2 box.
        maker.scale = 1
        maker.make_model(self._obj_path, sdf_result)
        self.assertTrue(
            self.records.hasRecordRegex(logging.INFO, ".+2.0 x 2.0 x 2.0.+"))
        # maker density is 1.0.
        self.assertTrue(self._find_in_file(sdf_result, ".*<mass>8.0</mass>.*"))
        self.assertTrue(self._find_in_file(sdf_result,
                                           ".*<scale>1 1 1</scale>.*"))
        self.records.clear()

        # Non-unit scale.
        maker.scale = 2
        maker.make_model(self._obj_path, sdf_result)
        self.assertTrue(
            self.records.hasRecordRegex(logging.INFO, ".+4.0 x 4.0 x 4.0.+"))
        self.assertTrue(self._find_in_file(sdf_result,
                                           ".*<mass>64.0</mass>.*"))
        self.assertTrue(self._find_in_file(sdf_result,
                                           ".*<scale>2 2 2</scale>.*"))
        self.records.clear()

        # Invalid scale sizes logs an error and produces no file.
        maker.scale = -1
        os.remove(sdf_result)
        maker.make_model(self._obj_path, sdf_result)
        self.assertTrue(self.records.hasRecordRegex(
            logging.ERROR, ".*Scale .+ must be positive.+"))
        self.assertFalse(os.path.exists(sdf_result))

    def test_name(self):
        """
        Confirm the name parameter affects the output sdf. The name should have
        the following implications:

            - If no name is given, the stem name of the obj file is used.
            - If a name is given, the preferred name is used.
        """
        maker = self._make_default_maker()
        sdf_result = os.path.join(self._temp_dir, "named.sdf")

        # No name given uses the stem name.
        maker.model_name = None
        maker.make_model(self._obj_path, sdf_result)
        self.assertTrue(self._find_in_file(
            sdf_result, f".*<link name='{self._obj_stem}'>.*"))

        # Empty string given uses the stem name.
        maker.model_name = ''
        maker.make_model(self._obj_path, sdf_result)
        self.assertTrue(self._find_in_file(
            sdf_result, f".*<link name='{self._obj_stem}'>.*"))

        # Use given name
        maker.model_name = "frank"
        maker.make_model(self._obj_path, sdf_result)
        self.assertTrue(self._find_in_file(
            sdf_result, f".*<link name='{maker.model_name}'>.*"))

    def test_mesh_mass_spec(self):
        # Constructed without specifying either quantity throws.
        with self.assertRaisesRegex(ValueError, "Neither .+ were defined."):
            MeshMassSpec()

        # Construct with mass.
        has_mass = MeshMassSpec(mass=1.5)
        self.assertIsNone(has_mass.density)
        self.assertEqual(has_mass.mass, 1.5)

        # Construct with density.
        has_density = MeshMassSpec(density=1.5)
        self.assertEqual(has_density.density, 1.5)
        self.assertIsNone(has_density.mass)

        # Construct with both; specification doesn't reconcile both values.
        # MeshModelMaker does.
        has_density = MeshMassSpec(density=1.5, mass=2.5)
        self.assertEqual(has_density.density, 1.5)
        self.assertEqual(has_density.mass, 2.5)

    def test_mass(self):
        """
        Confirm the mass_spec parameter affects the output sdf. The definition
        of mass can specify mass or density.

            - If only density is specified, the total mass should be volume *
              density.
            - If mass is given (with or without density), the total mass should
              be exactly what is given.
            - In all cases, the center of mass and inertia tensor should remain
              unchanged.
        """
        pass

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
