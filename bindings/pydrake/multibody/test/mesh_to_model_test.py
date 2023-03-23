"""Unit tests for mesh_to_model."""

from pydrake.geometry import (
    ReadObjToTriangleSurfaceMesh,
)
from pydrake.math import (
    RigidTransform
)
from pydrake.multibody._mesh_model_maker import (
    MeshModelMaker,
    MeshFramePose,
    MeshMassSpec,
)
from pydrake.multibody.tree import (
    RotationalInertia,
    UnitInertia,
)

import logging
import os
import re
import shutil
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
        maker.collision = True
        maker.visual = True
        maker.encoded_package = "none"
        return maker

    @staticmethod
    def _find_in_file(filename, regex):
        """Reports if the given regex matches any _line_ in the file."""
        with open(filename) as f:
            text = ''.join(f.readlines())
        found = re.search(regex, text, re.MULTILINE) is not None
        if not found:
            print(f"Matching {regex}")
            print(text)
        return found

    @staticmethod
    def _extract_rotational_inertia(filename):
        """Extracts the rotational matrix from the named SDFormat file."""
        # This relies on the fact that all inertia values are on a single line
        # and all are present.
        with open(filename) as f:
            text = ''.join(f.readlines())
        matches = re.findall("<(i[xyz]{2})>(.+?)</i..>", text, re.MULTILINE)
        values = dict([(key.replace("i", "I"), float(value))
                       for key, value in matches])
        return RotationalInertia(**values)

    def test_simple_valid_invocation(self):
        """Smoke test that a straightforward invocation works."""
        maker = self._make_default_maker()

        sdf_result = os.path.join(self._temp_dir, "simple_invoke.sdf")
        maker.make_model(self._obj_path, sdf_result)

        self.assertTrue(os.path.exists(sdf_result))
        self.assertTrue(self._find_in_file(sdf_result, "<?xml"))

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
        self.assertTrue(self._find_in_file(sdf_result, "<mass>8.0</mass>"))
        self.assertTrue(self._find_in_file(sdf_result, "<scale>1 1 1</scale>"))
        self.records.clear()

        # Non-unit scale.
        maker.scale = 2
        maker.make_model(self._obj_path, sdf_result)
        self.assertTrue(
            self.records.hasRecordRegex(logging.INFO, ".+4.0 x 4.0 x 4.0.+"))
        self.assertTrue(self._find_in_file(sdf_result, "<mass>64.0</mass>"))
        self.assertTrue(self._find_in_file(sdf_result, "<scale>2 2 2</scale>"))
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
            sdf_result, f"<link name='{self._obj_stem}'>"))

        # Empty string given uses the stem name.
        maker.model_name = ''
        maker.make_model(self._obj_path, sdf_result)
        self.assertTrue(self._find_in_file(
            sdf_result, f"<link name='{self._obj_stem}'>"))

        # Use given name
        maker.model_name = "frank"
        maker.make_model(self._obj_path, sdf_result)
        self.assertTrue(self._find_in_file(
            sdf_result, f"<link name='{maker.model_name}'>"))

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
            - The inertia tensor should change proportionately with the mass;
              we'll test a single value, asserting equivalency.
        """
        maker = self._make_default_maker()
        sdf_result = os.path.join(self._temp_dir, "mass.sdf")

        # We expect full precision of the answers.
        kEps = 2e-16

        # Test object is a 2x2x2 box with volume V = 8 m³. Pick a density ρ and
        # a mass m such that V⋅ρ ≠ m.
        V = 8
        rho = 1.5
        density_mass = V * rho
        expected_mass = density_mass + 1
        G_GGo_G = UnitInertia.SolidBox(2, 2, 2)
        I_GGo_G_density = G_GGo_G * density_mass
        I_GGo_G_mass = G_GGo_G * expected_mass

        # Specify only density.
        maker.mass_spec = MeshMassSpec(density=rho)
        maker.make_model(self._obj_path, sdf_result)
        self.assertTrue(
            self._find_in_file(sdf_result, f"<mass>{density_mass}</mass>"))
        I_GGo_G = self._extract_rotational_inertia(sdf_result)
        self.assertTrue(I_GGo_G_density.IsNearlyEqualTo(I_GGo_G, kEps))

        # Specify only mass.
        maker.mass_spec = MeshMassSpec(mass=expected_mass)
        maker.make_model(self._obj_path, sdf_result)
        self.assertTrue(
            self._find_in_file(sdf_result, f"<mass>{expected_mass}</mass>"))
        I_GGo_G = self._extract_rotational_inertia(sdf_result)
        self.assertTrue(I_GGo_G_mass.IsNearlyEqualTo(I_GGo_G, kEps))

        # Specify both; mass wins.
        maker.mass_spec = MeshMassSpec(mass=expected_mass, density=rho)
        maker.make_model(self._obj_path, sdf_result)
        self.assertTrue(
            self._find_in_file(sdf_result, f"<mass>{expected_mass}</mass>"))
        I_GGo_G = self._extract_rotational_inertia(sdf_result)
        self.assertTrue(I_GGo_G_mass.IsNearlyEqualTo(I_GGo_G, kEps))

    def test_mesh_frame_pose(self):
        frame_pose = MeshFramePose()
        self.assertFalse(frame_pose.at_com)
        self.assertIsNone(frame_pose.p_GoBo)

        frame_pose = MeshFramePose(at_com=True)
        self.assertTrue(frame_pose.at_com)
        self.assertIsNone(frame_pose.p_GoBo)

        frame_pose = MeshFramePose(p_GoBo=(1, 2, 3))
        self.assertFalse(frame_pose.at_com)
        self.assertListEqual(list(frame_pose.p_GoBo), [1, 2, 3])

        with self.assertRaisesRegex(ValueError, "Specify either.*not both."):
            MeshFramePose(at_com=True, p_GoBo=[1, 2, 3])

    def test_body_origin(self):
        """
        Confirm the frame_pose parameter affects the output sdf. The definition
        of body frame pose can pose the geometry relative to the body in three
        ways:

            - Geometry and body frames coincident.
            - Geometry and body bases aligned, with geometry center of mass at
              body origin.
            - Arbitrary displacement between body origin Bo and geometry origin
              Go.

        We can easily test the first and third, but the box mesh has its
        center of mass at its origin, so we can't distinguish between the first
        two cases. For this, we need a custom mesh.
        """
        # Create a new box mesh whose center of mass is at (1, 1, 1).
        mesh = ReadObjToTriangleSurfaceMesh(self._obj_path)
        offset_obj = os.path.join(self._temp_dir, "offset_box.obj")
        with open(offset_obj, 'w') as f:
            for v in mesh.vertices():
                f.write(f"v {v[0] + 1} {v[1] + 1} {v[2] + 1}\n")
            for t in mesh.triangles():
                # In-memory triangles are zero indexed, OBJ is one indexed.
                f.write(f"f {t.vertex(0) + 1} {t.vertex(1) + 1} "
                        f"{t.vertex(2) + 1}\n")

        maker = self._make_default_maker()
        sdf_result = os.path.join(self._temp_dir, "origin.sdf")

        # Geometry and body frames are coincident.
        maker.frame_pose = MeshFramePose()
        maker.make_model(offset_obj, sdf_result)
        self.assertTrue(
            self._find_in_file(sdf_result,
                               "<visual .+?>\\n\\s+<pose>0 0 0 0 0 0</pose>"))
        self.assertTrue(
            self._find_in_file(
                sdf_result, "<collision .+?>\\n\\s+<pose>0 0 0 0 0 0</pose>"))

        # Geometry origin and body origin are coincident.
        maker.frame_pose = MeshFramePose(at_com=True)
        maker.make_model(offset_obj, sdf_result)
        self.assertTrue(
            self._find_in_file(
                sdf_result, "<visual .+?>\\n\\s+<pose>-1 -1 -1 0 0 0</pose>"))
        self.assertTrue(
            self._find_in_file(
                sdf_result, ("<collision .+?>\\n"
                             "\\s+<pose>-1 -1 -1 0 0 0</pose>")))

        # Geometry posed arbitrarily.
        maker.frame_pose = MeshFramePose(p_GoBo=[1, 2, 3])
        maker.make_model(offset_obj, sdf_result)
        self.assertTrue(
            self._find_in_file(
                sdf_result, "<visual .+?>\\n\\s+<pose>-1 -2 -3 0 0 0</pose>"))
        self.assertTrue(
            self._find_in_file(
                sdf_result, ("<collision .+?>\\n"
                             "\\s+<pose>-1 -2 -3 0 0 0</pose>")))
        
    def test_geometry_present(self):
        maker = self._make_default_maker()
        sdf_result = os.path.join(self._temp_dir, "origin.sdf")

        maker.collision = True
        maker.make_model(self._obj_path, sdf_result)
        self.assertTrue(
            self._find_in_file(sdf_result, "<collision[\s\S\r]+</collision>"))

        maker.collision = False
        maker.make_model(self._obj_path, sdf_result)
        self.assertFalse(
            self._find_in_file(sdf_result, "<collision[\s\S\r]+</collision>"))

        maker.visual = True
        maker.make_model(self._obj_path, sdf_result)
        self.assertTrue(
            self._find_in_file(sdf_result, "<visual[\s\S\r]+</visual>"))

        maker.visual = False
        maker.make_model(self._obj_path, sdf_result)
        self.assertFalse(
            self._find_in_file(sdf_result, "<visual[\s\S\r]+</visual>"))

    def test_package(self):
        """Tests the package encoding.
        
            - "none": the mesh urls have path stripped away.
            - "auto": 
                  Create package.xml in obj's directory.
                  Use existing package.xml if already exists in obj's dir.
            - "path/to/package.xml":
                  Throws if obj isn't "path/to/..../foo.obj.
                  Otherwise creates package://package_name/relative/foo.obj
        """
        # Prepare a small tree with a package.xml in the root and the mesh
        # as a descendant.
        package_dir = os.path.join(self._temp_dir, "package")
        relative_obj_dir = os.path.join("intermediate", "meshes")
        obj_dir = os.path.join(package_dir, relative_obj_dir)
        os.makedirs(obj_dir)
        package_obj_path = os.path.join(obj_dir, "box.obj")
        shutil.copy(self._obj_path, package_obj_path)

        def write_package(xml_path, name):
            with open(xml_path, 'w') as f:
                f.write(f"""
                    <?xml_version="1.0"?>
                    <package format="2">
                    <name>{name}</name>
                    </package>
                    """)

        package_xml_path = os.path.join(package_dir, "package.xml")
        write_package(package_xml_path, "test_package")

        maker = self._make_default_maker()
        sdf_result = os.path.join(obj_dir, "with_package.sdf")

        # Bad encoding dispatches an error.
        maker.encoded_package = 'junk'
        maker.make_model(package_obj_path, sdf_result)
        self.assertTrue(self.records.hasRecordRegex(logging.ERROR,
                                                    ".*Unrecognized.+junk.*"))
        self.assertFalse(os.path.exists(sdf_result))
        self.records.clear()
        
        # "none" gives us a relative path.
        maker.encoded_package = 'none'
        maker.make_model(package_obj_path, sdf_result)
        self.assertTrue(self._find_in_file(sdf_result, "<uri>box.obj</uri>"))
        
        # "auto" creates obj_dir/package.xml (named "meshes") and the uri
        # is defined w.r.t. that package.
        generated_xml = os.path.join(obj_dir, "package.xml")
        self.assertFalse(os.path.exists(generated_xml))
        maker.encoded_package = 'auto'
        maker.make_model(package_obj_path, sdf_result)
        self.assertTrue(os.path.exists(generated_xml))
        self.assertTrue(
            self._find_in_file(sdf_result,
                               "<uri>package://meshes/box.obj</uri>"))
        self.assertTrue(
            self._find_in_file(generated_xml,
                               "<name>meshes</name>"))
        
        # "auto" with pre-existing package.xml in obj file will use it.
        write_package(generated_xml, "unique")
        self.assertTrue(os.path.exists(generated_xml))
        maker.make_model(package_obj_path, sdf_result)
        self.assertTrue(
            self._find_in_file(sdf_result,
                               "<uri>package://unique/box.obj</uri>"))
        self.assertTrue(
            self._find_in_file(generated_xml,
                               "<name>unique</name>"))
        
        # path/to/package.xml logs an error if the mesh isn't a descendant of
        # the package root directory.
        maker.encoded_package = package_xml_path
        maker.make_model(self._obj_path, sdf_result)
        self.assertTrue(
            self.records.hasRecordRegex(
                logging.ERROR, ".*must be located in the file tree.*"))
        self.records.clear()
        
        # path/to/package.xml with obj in sub-tree.
        maker.encoded_package = package_xml_path
        maker.make_model(package_obj_path, sdf_result)
        package_path = os.path.join("test_package", relative_obj_dir)
        self.assertTrue(
            self._find_in_file(sdf_result,
                               f"<uri>package://{package_path}/box.obj</uri>"))
        
        # package.xml doesn't exist.
        maker.encoded_package = os.path.join(
            self._temp_dir, "p", "package.xml")
        maker.make_model(package_obj_path, sdf_result)

        # package.xml is malformed
        # TODO
        

        

# What do I need to test?
#
#  - Parameters
#    - encoded_package
#       - invalid
#         - explicit "package.xml"
#           - File can't be read
#           - isn't manifest/doesn't have <name>*</name> tag.
#         - "auto"
#           - failure modes of explicit package.xml
#    - I need a test showing the resulting sdf gets "successfully" parsed.


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
