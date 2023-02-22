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
from pydrake.multibody.parsing import (
    Parser,
)
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph,
)
from pydrake.multibody.tree import (
    RotationalInertia,
    UnitInertia,
)
from pydrake.systems.framework import (
    DiagramBuilder,
)

import filecmp
import logging
import os
import re
import shutil
import subprocess
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


def _parse_model_no_throw(sdf_file: str, package_xml: str = None):
    """
    Simply attempts to parse the given file, expecting no errors. This
    should be invoked for every interesting variation of an _expected_
    valid SDFormat file.

    If package_xml is given, it will be added to the parser.
    """
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
    parser = Parser(plant)
    if package_xml is not None:
        parser.package_map().AddPackageXml(package_xml)
    parser.AddModels(sdf_file)
    plant.Finalize()


def _make_offset_obj(target_obj_path: str, offset):
    """
    Writes a translated version of drake/geometry/render/test/meshes/box.obj
    to the given path. The vertices are all translated by the given offset.
    """
    source_path = FindResourceOrThrow(
            f"drake/geometry/render/test/meshes/box.obj")
    mesh = ReadObjToTriangleSurfaceMesh(source_path)
    with open(target_obj_path, 'w') as f:
        for v in mesh.vertices():
            f.write(f"v {v[0] + offset[0]} {v[1] + offset[1]} "
                    f"{v[2] + offset[2]}\n")
        for t in mesh.triangles():
            # In-memory triangles are zero indexed, OBJ is one indexed.
            f.write(f"f {t.vertex(0) + 1} {t.vertex(1) + 1} "
                    f"{t.vertex(2) + 1}\n")


class TestModelMaker(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._temp_dir = temp_directory()
        # We need to place the obj file in the temp_directory so that attempts
        # to parse the resulting SDF work.
        cls._obj_stem = "box"
        obj_source_path = FindResourceOrThrow(
            f"drake/geometry/render/test/meshes/{cls._obj_stem}.obj")
        cls._obj_path = os.path.join(cls._temp_dir, f"{cls._obj_stem}.obj")
        shutil.copy(obj_source_path, cls._obj_path)
        cls._logger = logging.getLogger("drake")
        cls._logger.setLevel(logging.INFO)

    def setUp(self):
        self.records = LogRecorder()
        self._logger.addHandler(self.records)

    def tearDown(self):
        self._logger.removeHandler(self.records)

    @staticmethod
    def _make_default_maker():
        """Returns a MeshModelmaker with a simple, valid configuration."""
        dut = MeshModelMaker()
        dut.scale = 1.0
        dut.model_name = None
        dut.mass_spec = MeshMassSpec(density=1)
        dut.frame_pose = MeshFramePose()
        dut.collision = True
        dut.visual = True
        dut.encoded_package = "none"
        return dut

    @staticmethod
    def _find_in_file(filename, regex):
        """Reports if the given regex matches any _line_ in the file."""
        with open(filename) as f:
            text = ''.join(f.readlines())
        found = re.search(regex, text, re.MULTILINE) is not None
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
        dut = self._make_default_maker()

        sdf_result = os.path.join(self._temp_dir, "simple_invoke.sdf")
        dut.make_model(self._obj_path, sdf_result)
        _parse_model_no_throw(sdf_result)

        self.assertTrue(os.path.exists(sdf_result))
        self.assertTrue(self._find_in_file(sdf_result, "<?xml"))

    def test_invalid_obj(self):
        dut = self._make_default_maker()
        sdf_result = os.path.join(self._temp_dir, "invalid_obj.sdf")

        # Invalid because it doesn't exist.
        with self.assertRaisesRegex(RuntimeError, "Cannot open file.*"):
            dut.make_model('invalid_path.obj', sdf_result)
        self.assertFalse(os.path.exists(sdf_result))

        # Invalid because it isn't an OBJ.
        not_an_obj = os.path.join(self._temp_dir, 'not_an_obj.txt')
        with open(not_an_obj, 'w') as f:
            f.write("Just some text\n")
        with self.assertRaisesRegex(RuntimeError, ".+obj data has no.+"):
            dut.make_model(not_an_obj, sdf_result)
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
        dut = self._make_default_maker()
        sdf_result = os.path.join(self._temp_dir, "scaled.sdf")

        # Unit scale has a baseline 2x2x2 box.
        dut.scale = 1
        dut.make_model(self._obj_path, sdf_result)
        _parse_model_no_throw(sdf_result)
        self.assertTrue(
            self.records.hasRecordRegex(logging.INFO, ".+2.0 x 2.0 x 2.0.+"))
        # Density is 1.0.
        self.assertTrue(self._find_in_file(sdf_result, "<mass>8.0</mass>"))
        self.assertTrue(self._find_in_file(sdf_result, "<scale>1 1 1</scale>"))
        self.records.clear()

        # Non-unit scale.
        dut.scale = 2
        dut.make_model(self._obj_path, sdf_result)
        _parse_model_no_throw(sdf_result)
        self.assertTrue(
            self.records.hasRecordRegex(logging.INFO, ".+4.0 x 4.0 x 4.0.+"))
        self.assertTrue(self._find_in_file(sdf_result, "<mass>64.0</mass>"))
        self.assertTrue(self._find_in_file(sdf_result, "<scale>2 2 2</scale>"))
        self.records.clear()

        # Invalid scale sizes logs an error and produces no file.
        dut.scale = -1
        os.remove(sdf_result)
        dut.make_model(self._obj_path, sdf_result)
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
        dut = self._make_default_maker()
        sdf_result = os.path.join(self._temp_dir, "named.sdf")

        # No name given uses the stem name.
        dut.model_name = None
        dut.make_model(self._obj_path, sdf_result)
        _parse_model_no_throw(sdf_result)
        self.assertTrue(self._find_in_file(
            sdf_result, f"<link name='{self._obj_stem}'>"))

        # Empty string given uses the stem name.
        dut.model_name = ''
        dut.make_model(self._obj_path, sdf_result)
        _parse_model_no_throw(sdf_result)
        self.assertTrue(self._find_in_file(
            sdf_result, f"<link name='{self._obj_stem}'>"))

        # Use given name
        dut.model_name = "frank"
        dut.make_model(self._obj_path, sdf_result)
        _parse_model_no_throw(sdf_result)
        self.assertTrue(self._find_in_file(
            sdf_result, f"<link name='{dut.model_name}'>"))

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
        dut = self._make_default_maker()
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
        dut.mass_spec = MeshMassSpec(density=rho)
        dut.make_model(self._obj_path, sdf_result)
        _parse_model_no_throw(sdf_result)
        self.assertTrue(
            self._find_in_file(sdf_result, f"<mass>{density_mass}</mass>"))
        I_GGo_G = self._extract_rotational_inertia(sdf_result)
        self.assertTrue(I_GGo_G_density.IsNearlyEqualTo(I_GGo_G, kEps))

        # Specify only mass.
        dut.mass_spec = MeshMassSpec(mass=expected_mass)
        dut.make_model(self._obj_path, sdf_result)
        _parse_model_no_throw(sdf_result)
        self.assertTrue(
            self._find_in_file(sdf_result, f"<mass>{expected_mass}</mass>"))
        I_GGo_G = self._extract_rotational_inertia(sdf_result)
        self.assertTrue(I_GGo_G_mass.IsNearlyEqualTo(I_GGo_G, kEps))

        # Specify both; mass wins.
        dut.mass_spec = MeshMassSpec(mass=expected_mass, density=rho)
        dut.make_model(self._obj_path, sdf_result)
        _parse_model_no_throw(sdf_result)
        self.assertTrue(
            self._find_in_file(sdf_result, f"<mass>{expected_mass}</mass>"))
        I_GGo_G = self._extract_rotational_inertia(sdf_result)
        self.assertTrue(I_GGo_G_mass.IsNearlyEqualTo(I_GGo_G, kEps))

    def test_mesh_frame_pose(self):
        default_pose = MeshFramePose()
        self.assertFalse(default_pose.at_com)
        self.assertIsNone(default_pose.p_GoBo)

        pose_at_com = MeshFramePose(at_com=True)
        self.assertTrue(pose_at_com.at_com)
        self.assertIsNone(pose_at_com.p_GoBo)

        arbitrary_pose = MeshFramePose(p_GoBo=(1, 2, 3))
        self.assertFalse(arbitrary_pose.at_com)
        self.assertListEqual(list(arbitrary_pose.p_GoBo), [1, 2, 3])

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
        # Create a new box mesh whose center of mass is at (3, 2, 1).
        offset_obj = os.path.join(self._temp_dir, "offset_box.obj")
        _make_offset_obj(offset_obj, [3, 2, 1])

        dut = self._make_default_maker()
        sdf_result = os.path.join(self._temp_dir, "origin.sdf")

        # Geometry and body frames are coincident.
        dut.frame_pose = MeshFramePose()
        dut.make_model(offset_obj, sdf_result)
        _parse_model_no_throw(sdf_result)
        self.assertTrue(
            self._find_in_file(sdf_result,
                               "<visual .+?>\\n\\s+<pose>0 0 0 0 0 0</pose>"))
        self.assertTrue(
            self._find_in_file(
                sdf_result, "<collision .+?>\\n\\s+<pose>0 0 0 0 0 0</pose>"))

        # Geometry origin and body origin are coincident.
        dut.frame_pose = MeshFramePose(at_com=True)
        dut.make_model(offset_obj, sdf_result)
        _parse_model_no_throw(sdf_result)
        self.assertTrue(
            self._find_in_file(
                sdf_result, "<visual .+?>\\n\\s+<pose>-3 -2 -1 0 0 0</pose>"))
        self.assertTrue(
            self._find_in_file(
                sdf_result, ("<collision .+?>\\n"
                             "\\s+<pose>-3 -2 -1 0 0 0</pose>")))

        # Geometry posed arbitrarily.
        dut.frame_pose = MeshFramePose(p_GoBo=[1, 2, 3])
        dut.make_model(offset_obj, sdf_result)
        _parse_model_no_throw(sdf_result)
        self.assertTrue(
            self._find_in_file(
                sdf_result, "<visual .+?>\\n\\s+<pose>-1 -2 -3 0 0 0</pose>"))
        self.assertTrue(
            self._find_in_file(
                sdf_result, ("<collision .+?>\\n"
                             "\\s+<pose>-1 -2 -3 0 0 0</pose>")))

    def test_geometry_present(self):
        dut = self._make_default_maker()
        sdf_result = os.path.join(self._temp_dir, "origin.sdf")

        dut.collision = True
        dut.make_model(self._obj_path, sdf_result)
        _parse_model_no_throw(sdf_result)
        self.assertTrue(
            self._find_in_file(sdf_result, R"<collision[\s\S\r]+</collision>"))

        dut.collision = False
        dut.make_model(self._obj_path, sdf_result)
        _parse_model_no_throw(sdf_result)
        self.assertFalse(
            self._find_in_file(sdf_result, R"<collision[\s\S\r]+</collision>"))

        dut.visual = True
        dut.make_model(self._obj_path, sdf_result)
        _parse_model_no_throw(sdf_result)
        self.assertTrue(
            self._find_in_file(sdf_result, R"<visual[\s\S\r]+</visual>"))

        dut.visual = False
        dut.make_model(self._obj_path, sdf_result)
        _parse_model_no_throw(sdf_result)
        self.assertFalse(
            self._find_in_file(sdf_result, R"<visual[\s\S\r]+</visual>"))

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

        dut = self._make_default_maker()
        sdf_result = os.path.join(obj_dir, "with_package.sdf")

        # Bad encoding dispatches an error.
        dut.encoded_package = 'junk'
        dut.make_model(package_obj_path, sdf_result)
        self.assertTrue(self.records.hasRecordRegex(logging.ERROR,
                                                    ".*Unrecognized.+junk.*"))
        self.assertFalse(os.path.exists(sdf_result))
        self.records.clear()

        # "none" gives us a relative path.
        dut.encoded_package = 'none'
        dut.make_model(package_obj_path, sdf_result)
        _parse_model_no_throw(sdf_result)
        self.assertTrue(self._find_in_file(sdf_result, "<uri>box.obj</uri>"))

        # "auto" creates obj_dir/package.xml (named "meshes") and the uri
        # is defined w.r.t. that package.
        generated_xml = os.path.join(obj_dir, "package.xml")
        self.assertFalse(os.path.exists(generated_xml))
        dut.encoded_package = 'auto'
        dut.make_model(package_obj_path, sdf_result)
        _parse_model_no_throw(sdf_result, generated_xml)
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
        dut.make_model(package_obj_path, sdf_result)
        _parse_model_no_throw(sdf_result, generated_xml)
        self.assertTrue(
            self._find_in_file(sdf_result,
                               "<uri>package://unique/box.obj</uri>"))
        self.assertTrue(
            self._find_in_file(generated_xml,
                               "<name>unique</name>"))

        # path/to/package.xml logs an error if the mesh isn't a descendant of
        # the package root directory.
        dut.encoded_package = package_xml_path
        dut.make_model(self._obj_path, sdf_result)
        self.assertTrue(
            self.records.hasRecordRegex(
                logging.ERROR, ".*must be located in the file tree.*"))
        self.records.clear()

        # path/to/package.xml with obj in sub-tree.
        dut.encoded_package = package_xml_path
        dut.make_model(package_obj_path, sdf_result)
        package_path = os.path.join("test_package", relative_obj_dir)
        self.assertTrue(
            self._find_in_file(sdf_result,
                               f"<uri>package://{package_path}/box.obj</uri>"))

        # package.xml doesn't exist.
        dut.encoded_package = os.path.join(
            self._temp_dir, "p", "package.xml")
        dut.make_model(package_obj_path, sdf_result)
        self.assertTrue(
            self.records.hasRecordRegex(logging.ERROR,
                                        ".*package.xml cannot be found.*"))
        self.records.clear()

        # package.xml is malformed; e.g., it has no name or isn't xml. We need
        # to set up an otherwise valid configuration. MeshModelMaker doesn't
        # distinguish between a valid xml file without a name field and a file
        # that isn't xml at all. They both produce the same result, so we'll
        # only test one.
        package_dir = os.path.join(self._temp_dir, "bad_package")
        os.makedirs(package_dir)
        package_obj_path = os.path.join(package_dir, "box.obj")
        shutil.copy(self._obj_path, package_obj_path)
        non_xml_path = os.path.join(package_dir, "package.xml")
        with open(non_xml_path, 'w') as f:
            f.write("Not really an xml file\n")

        dut.encoded_package = non_xml_path
        dut.make_model(package_obj_path, sdf_result)
        self.assertTrue(
            self.records.hasRecordRegex(logging.ERROR,
                                        ".*package.* not properly formed.*"))
        self.records.clear()


class TestMeshToModelProcess(unittest.TestCase):
    """
    Tests the command-line tool mesh_to_model by invoking the process
    directly.

    The above tests on the underlying class confirm correctness of the SDFormat
    file and logic for processing parameters. For these tests, we need to
    confirm that command-line parameters are passed as expected.

    We'll simply create two sdfs, one via subprocess and one via direct calls
    to ModelMeshMaker and compare the files. This will show that the
    command-line parameters are passed along as expected.
    """
    @classmethod
    def setUpClass(cls):
        cls._temp_dir = temp_directory()
        # We need to place the obj file in the temp_directory so that attempts
        # to parse the resulting SDF work.
        cls._obj_stem = "box"
        obj_source_path = FindResourceOrThrow(
            f"drake/geometry/render/test/meshes/box.obj")
        cls._obj_path = os.path.join(cls._temp_dir, "box.obj")
        shutil.copy(obj_source_path, cls._obj_path)
        cls._dut = FindResourceOrThrow(
            "drake/bindings/pydrake/multibody/mesh_to_model")

    def assert_files_equal(self, dut_path, ref_path):
        self.assertListEqual(list(open(dut_path)), list(open(ref_path)))

    def test_default_parameters(self):
        """
        Providing no command-line parameters should be equivalent to the
        default configuration for MeshModelMaker.
        """
        maker = MeshModelMaker()
        reference_sdf = os.path.join(self._temp_dir, "default_ref.sdf")
        maker.make_model(self._obj_path, reference_sdf)

        dut_sdf = os.path.join(self._temp_dir, "default_dut.sdf")
        subprocess.check_call([self._dut, self._obj_path, dut_sdf])

        self.assert_files_equal(dut_sdf, reference_sdf)

    def test_all_non_default(self):
        """
        Tweak all of the parameters that can be trivially confirmed by simply
        passing in different arguments. This isn't the exhaustive test; some
        parameters will need to be further probed, but this guarantees that
        we can see that each parameter is passed through as expected.
        """
        maker = MeshModelMaker()
        # All values are defined w.r.t. to the default values to guarnatee
        # they are different.
        maker.scale = maker.scale * 2
        self.assertIsNone(maker.model_name)
        maker.model_name = "alt_name"
        maker.mass_spec = MeshMassSpec(density=maker.mass_spec.density * 2)
        self.assertIsNone(maker.frame_pose.p_GoBo)
        maker.frame_pose = MeshFramePose(p_GoBo=[1, 2, 3])
        self.assertEqual(maker.encoded_package, "none")
        # For package, this is enough. We've confirmed the string gets passed
        # to MeshModelMaker.
        maker.encoded_package = "auto"
        # N.B. We're skipping collision and visual _here_ to make sure that
        # they work independently (see below).

        reference_sdf = os.path.join(self._temp_dir, "default_ref.sdf")
        maker.make_model(self._obj_path, reference_sdf)

        dut_sdf = os.path.join(self._temp_dir, "default_dut.sdf")
        parameters = ['--model-name', maker.model_name,
                      '--scale', str(maker.scale),
                      '--density', str(maker.mass_spec.density),
                      '--body-origin', '1,2,3',
                      '--package', maker.encoded_package,
                      self._obj_path, dut_sdf
                      ]
        subprocess.check_call([self._dut] + parameters)

        self.assert_files_equal(dut_sdf, reference_sdf)

    def test_specify_mass(self):
        """
        Test that specifying *mass* produces the expected result and that we
        can't pass both mass and density.
        """
        # Only one of mass/density can be specified.
        dut_sdf = os.path.join(self._temp_dir, "mass_dut.sdf")
        with self.assertRaisesRegex(subprocess.CalledProcessError,
                                    ".*non-zero exit status.*"):
            subprocess.check_call([self._dut, '--mass', '1', '--density', '1',
                                   self._obj_path, dut_sdf])

        maker = MeshModelMaker()
        maker.mass_spec = MeshMassSpec(mass=123)
        reference_sdf = os.path.join(self._temp_dir, "mass_ref.sdf")
        maker.make_model(self._obj_path, reference_sdf)

        subprocess.check_call([self._dut, '--mass', str(maker.mass_spec.mass),
                              self._obj_path, dut_sdf])

        self.assert_files_equal(dut_sdf, reference_sdf)

    def test_frame_pose(self):
        """
        We've already confirmed the default geometry-body frame alignment and
        arbitrary pose (--body-origin). Now we'll confirm at-com works and that
        specifying both --body-orign and --origin-at-com is invalid.
        """
        # Specifying both is an error.
        dut_sdf = os.path.join(self._temp_dir, "pose_dut.sdf")
        with self.assertRaisesRegex(subprocess.CalledProcessError,
                                    ".*non-zero exit status.*"):
            subprocess.check_call([self._dut, '--origin-at-com',
                                   '--body-origin', '1,2,3',
                                   self._obj_path, dut_sdf])

        # Make an obj whose center of mass is not at its frame origin.
        offset_obj = os.path.join(self._temp_dir, "offset_box.obj")
        _make_offset_obj(offset_obj, [1, 2, 3])

        maker = MeshModelMaker()
        maker.frame_pose = MeshFramePose(at_com=True)
        reference_sdf = os.path.join(self._temp_dir, "pose_ref.sdf")
        maker.make_model(offset_obj, reference_sdf)

        dut_sdf = os.path.join(self._temp_dir, "pose_dut.sdf")
        subprocess.check_call([self._dut, "--origin-at-com", offset_obj,
                               dut_sdf])

        self.assert_files_equal(dut_sdf, reference_sdf)

    def test_collision(self):
        """Omitting the collision geometry omits *only* the collision."""
        maker = MeshModelMaker()
        self.assertTrue(maker.collision)
        self.assertTrue(maker.visual)
        maker.collision = False
        reference_sdf = os.path.join(self._temp_dir, "no_collision_ref.sdf")
        maker.make_model(self._obj_path, reference_sdf)

        dut_sdf = os.path.join(self._temp_dir, "no_collision_dut.sdf")
        subprocess.check_call([self._dut, "--no-collision", self._obj_path,
                               dut_sdf])

        self.assert_files_equal(dut_sdf, reference_sdf)

    def test_visual(self):
        """Omitting the visual geometry omits *only* the visual."""
        maker = MeshModelMaker()
        self.assertTrue(maker.collision)
        self.assertTrue(maker.visual)
        maker.visual = False
        reference_sdf = os.path.join(self._temp_dir, "no_visual_ref.sdf")
        maker.make_model(self._obj_path, reference_sdf)

        dut_sdf = os.path.join(self._temp_dir, "no_visual_dut.sdf")
        subprocess.check_call([self._dut, "--no-visual", self._obj_path,
                               dut_sdf])

        self.assert_files_equal(dut_sdf, reference_sdf)
