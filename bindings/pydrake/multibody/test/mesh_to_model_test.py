"""Unit tests for mesh_to_model."""


import logging
import numpy as np
from pathlib import Path
import re
import shutil
import subprocess
import unittest
import xml.etree.ElementTree

from pydrake.common import (
    FindResourceOrThrow,
    temp_directory,
)
from pydrake.geometry import (
    ReadObjToTriangleSurfaceMesh,
)
from pydrake.multibody._mesh_model_maker import (
    MeshModelMaker,
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


def _parse_model_no_throw(sdf_file: Path, package_xml: Path = None):
    """
    Simply attempts to parse the given file, expecting no errors. This
    should be invoked for every interesting variation of an _expected_
    valid SDFormat file.

    If package_xml is given, it will be added to the parser.
    """
    builder = DiagramBuilder()
    plant, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
    parser = Parser(plant)
    if package_xml is not None:
        parser.package_map().AddPackageXml(str(package_xml))
    parser.AddModels(str(sdf_file))
    plant.Finalize()


def _make_offset_obj(target_obj_path: Path, offset):
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


def _file_contents(filename):
    with open(filename) as f:
        return f.read()


class TestModelMaker(unittest.TestCase):
    def setUp(self):
        self._temp_dir = Path(temp_directory())
        # We need to place the obj file in the temp_directory so that attempts
        # to parse the resulting SDF work.
        self._obj_stem = "box"
        obj_source_path = FindResourceOrThrow(
            f"drake/geometry/render/test/meshes/{self._obj_stem}.obj")
        self._obj_path = self._temp_dir / f"{self._obj_stem}.obj"
        shutil.copy(obj_source_path, self._obj_path)
        self._sdf_path = self._temp_dir / f"{self._obj_stem}.sdf"

    @staticmethod
    def _extract_rotational_inertia(filename):
        """Extracts the rotational matrix from the named SDFormat file."""
        # This relies on the fact that all inertia values are on a single line
        # and all are present.
        with open(filename) as f:
            text = f.read()
        matches = re.findall("<(i[xyz]{2})>(.+?)</i..>", text, re.MULTILINE)
        values = dict([(key.replace("i", "I"), float(value))
                       for key, value in matches])
        return RotationalInertia(**values)

    def test_simple_valid_invocation(self):
        """Smoke tests that a straightforward invocation works."""
        dut = MeshModelMaker(mesh_path=self._obj_path,
                             output_dir=self._temp_dir)
        dut.make_model()
        _parse_model_no_throw(self._sdf_path)
        self.assertRegex(_file_contents(self._sdf_path), "<?xml")

    def test_invalid_obj(self):
        dut = MeshModelMaker(mesh_path=Path("invalid_path.obj"),
                             output_dir=self._temp_dir)

        # Invalid because it doesn't exist.
        with self.assertRaisesRegex(RuntimeError, "Cannot open file.*"):
            dut.make_model()
        self.assertFalse(self._sdf_path.exists())

        # Invalid because it isn't an OBJ.
        not_an_obj = self._temp_dir / "not_an_obj.txt"
        with open(not_an_obj, 'w') as f:
            f.write("Just some text\n")
        with self.assertRaisesRegex(RuntimeError, ".+obj data has no.+"):
            dut.mesh_path = not_an_obj
            dut.make_model()
        self.assertFalse(self._sdf_path.exists())

    def test_scale(self):
        """
        Confirms the scale parameter affects the output sdf. The scale factor
        should have three implications:

            - Reported bounding box changes.
            - Scale factor in SDFormat file changes.
            - Mass properties change.

        We don't explicitly test the inertia tensor because we are scaling the
        mesh directly and computing the mass properties on that mesh. Simply
        confirming the mass is correct should imply the inertia is also good.
        """
        dut = MeshModelMaker(mesh_path=self._obj_path,
                             output_dir=self._temp_dir,
                             density=1.0)

        # Unit scale has a baseline 2x2x2 box.
        with self.assertLogs('drake', logging.INFO) as cm:
            dut.make_model()
        _parse_model_no_throw(self._sdf_path)
        self.assertRegex('\n'.join([r.getMessage() for r in cm.records]),
                         "2.0 x 2.0 x 2.0.")
        # Density is 1.0.
        self.assertRegex(_file_contents(self._sdf_path), "<mass>8.0</mass>")
        self.assertRegex(_file_contents(self._sdf_path),
                         "<scale>1.0 1.0 1.0</scale>")

        # Non-unit scale.
        dut.scale = 2
        with self.assertLogs('drake', logging.INFO) as cm:
            dut.make_model()
        _parse_model_no_throw(self._sdf_path)
        self.assertRegex('\n'.join([r.getMessage() for r in cm.records]),
                         "4.0 x 4.0 x 4.0")
        self.assertRegex(_file_contents(self._sdf_path), "<mass>64.0</mass>")
        self.assertRegex(_file_contents(self._sdf_path),
                         "<scale>2 2 2</scale>")

        # Invalid scale sizes logs an error and produces no file.
        dut.scale = -1
        self._sdf_path.unlink()
        with self.assertRaisesRegex(ValueError,
                                    ".*Scale .+ must be positive.+"):
            dut.make_model()
        self.assertFalse(self._sdf_path.exists())

    def test_name(self):
        """
        Confirms the name parameter affects the output sdf. The name should
        have the following implications:

            - If no name is given, the stem name of the obj file is used.
            - If a name is given, the preferred name is used.
        """
        dut = MeshModelMaker(mesh_path=self._obj_path,
                             output_dir=self._temp_dir)

        # No name given uses the stem name.
        dut.make_model()
        _parse_model_no_throw(self._sdf_path)
        self.assertRegex(_file_contents(self._sdf_path),
                         f"<link name='{self._obj_stem}'>")

        # Empty string given uses the stem name.
        dut.model_name = ""
        dut.make_model()
        _parse_model_no_throw(self._sdf_path)
        self.assertRegex(_file_contents(self._sdf_path),
                         f"<link name='{self._obj_stem}'>")

        # Use given name.
        dut.model_name = "frank"
        dut.make_model()
        _parse_model_no_throw(self._sdf_path)
        self.assertRegex(_file_contents(self._sdf_path),
                         f"<link name='{dut.model_name}'>")

    def test_mass(self):
        """
        Confirms the mass_spec parameter affects the output sdf. The definition
        of mass can specify mass or density.

            - If only density is specified, the total mass should be volume *
              density.
            - If mass is given (with or without density), the total mass should
              be exactly what is given.
            - The inertia tensor should change proportionately with the mass;
              we'll test a single value, asserting equivalency.
        """
        # We expect full precision of the answers.
        kEps = 2e-16

        # Test object is a 2x2x2 box with volume V = 8 m³. Pick a density ρ and
        # a mass m such that V⋅ρ ≠ m.
        V = 8
        rho = 1.5
        density_mass = V * rho
        expected_mass = density_mass + 1
        G_GGo_G = UnitInertia.SolidCube(2)
        I_GGo_G_density = G_GGo_G * density_mass
        I_GGo_G_mass = G_GGo_G * expected_mass

        # Specify only density.
        dut = MeshModelMaker(mesh_path=self._obj_path,
                             output_dir=self._temp_dir,
                             mass=None, density=rho)
        dut.make_model()
        _parse_model_no_throw(self._sdf_path)
        self.assertRegex(_file_contents(self._sdf_path),
                         f"<mass>{density_mass}</mass>")
        I_GGo_G = self._extract_rotational_inertia(self._sdf_path)
        self.assertTrue(I_GGo_G_density.IsNearlyEqualTo(I_GGo_G, kEps))

        # Specify only mass.
        dut.mass = expected_mass
        dut.density = None
        dut.make_model()
        _parse_model_no_throw(self._sdf_path)
        self.assertRegex(_file_contents(self._sdf_path),
                         f"<mass>{expected_mass}</mass>")
        I_GGo_G = self._extract_rotational_inertia(self._sdf_path)
        self.assertTrue(I_GGo_G_mass.IsNearlyEqualTo(I_GGo_G, kEps))

        # Specify both; mass wins; make sure we do it in the constructor!
        dut = MeshModelMaker(mesh_path=self._obj_path,
                             output_dir=self._temp_dir,
                             mass=expected_mass, density=rho)
        dut.make_model()
        _parse_model_no_throw(self._sdf_path)
        self.assertRegex(_file_contents(self._sdf_path),
                         f"<mass>{expected_mass}</mass>")
        I_GGo_G = self._extract_rotational_inertia(self._sdf_path)
        self.assertTrue(I_GGo_G_mass.IsNearlyEqualTo(I_GGo_G, kEps))

        # Specify neither; throws.
        with self.assertRaisesRegex(ValueError, ".*Neither.*provided.*"):
            MeshModelMaker(mesh_path='', output_dir='', density=None,
                           mass=None)

    def test_body_origin(self):
        """
        Confirms the frame_pose parameter affects the output sdf. The
        definition of body frame pose can pose the geometry relative to the
        body in three ways:

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
        offset_obj = self._temp_dir / "offset_box.obj"
        offset_sdf = self._temp_dir / "offset_box.sdf"
        _make_offset_obj(offset_obj, [3, 2, 1])

        dut = MeshModelMaker(mesh_path=offset_obj, output_dir=self._temp_dir)

        # Geometry and body frames are coincident.
        dut.make_model()
        _parse_model_no_throw(offset_sdf)
        self.assertRegex(_file_contents(offset_sdf),
                         "<visual .+?>\\n\\s+<pose>0.0 0.0 0.0 0 0 0</pose>")
        self.assertRegex(
            _file_contents(offset_sdf),
            ("<collision .+?>\\n\\s+<pose>0.0 0.0 0.0 0 0 0</pose>"))

        # Geometry center of mass and body origin are coincident.
        dut.at_com = True
        dut.make_model()
        _parse_model_no_throw(offset_sdf)
        self.assertRegex(
            _file_contents(offset_sdf),
            "<visual .+?>\\n\\s+<pose>-3.0 -2.0 -1.0 0 0 0</pose>")
        self.assertRegex(
            _file_contents(offset_sdf),
            "<collision .+?>\\n\\s+<pose>-3.0 -2.0 -1.0 0 0 0</pose>")

        # Geometry posed arbitrarily.
        dut.at_com = False
        dut.p_GoBo = np.array([1, 2, 3])
        dut.make_model()
        _parse_model_no_throw(offset_sdf)
        self.assertRegex(
            _file_contents(offset_sdf),
            "<visual .+?>\\n\\s+<pose>-1 -2 -3 0 0 0</pose>")
        self.assertRegex(
            _file_contents(offset_sdf),
            "<collision .+?>\\n\\s+<pose>-1 -2 -3 0 0 0</pose>")

        # Specifying both is an error.
        with self.assertRaisesRegex(ValueError,
                                    ".*Specify either.*not both.*"):
            MeshModelMaker(mesh_path='', output_dir='', at_com=True,
                           p_GoBo=(1, 2, 3))

    def test_package_none(self):
        # "none" gives us a relative path.
        dut = MeshModelMaker(mesh_path=self._obj_path,
                             output_dir=self._temp_dir,
                             encoded_package='none')

        dut.make_model()
        _parse_model_no_throw(self._sdf_path)
        self.assertRegex(_file_contents(self._sdf_path), "<uri>box.obj</uri>")

    def test_package_auto(self):
        # Using "auto" will create a package.xml in the same directory (if
        # missing), or use it if present.

        # We need a dedicated subtree so that this test's package.xml doesn't
        # interfere with other tests'; the obj needs to be in that folder.
        package_dir = self._temp_dir / "package_auto"
        package_dir.mkdir()
        package_obj_path = package_dir / "box.obj"
        shutil.copy(self._obj_path, package_obj_path)

        generated_xml = package_dir / "package.xml"
        self.assertFalse(generated_xml.exists())

        dut = MeshModelMaker(mesh_path=package_obj_path,
                             output_dir=package_dir,
                             encoded_package='auto')

        # "auto" creates obj_dir/package.xml (named "meshes") and the uri
        # is defined w.r.t. that package.
        dut.make_model()
        self.assertTrue(generated_xml.exists())
        sdf_result = package_dir / "box.sdf"
        _parse_model_no_throw(sdf_result, generated_xml)
        self.assertRegex(_file_contents(sdf_result),
                         "<uri>package://package_auto/box.obj</uri>")
        self.assertRegex(_file_contents(generated_xml),
                         "<name>package_auto</name>")

        # "auto" with pre-existing package.xml in obj file will use it.
        # We'll change the name in the generated xml to confirm it gets used.
        with open(generated_xml, 'w') as f:
            f.write(f"""<?xml version="1.0"?>
                <package format="2">
                <name>unique</name>
                </package>
                """)
        dut.make_model()
        _parse_model_no_throw(sdf_result, generated_xml)
        self.assertRegex(_file_contents(sdf_result),
                         "<uri>package://unique/box.obj</uri>")

    def test_package_xml(self):
        # Specify the xml; we'll test both the positive case (the mesh is in
        # the package subtree) and the error case (it isn't).

        # Prepare a small tree with a package.xml in the root and the mesh
        # as a descendant.
        package_dir = self._temp_dir / "package_xml"
        relative_obj_dir = Path("intermediate/meshes")
        obj_dir = package_dir / relative_obj_dir
        obj_dir.mkdir(parents=True)
        package_obj_path = obj_dir / "box.obj"
        shutil.copy(self._obj_path, package_obj_path)

        package_xml_path = package_dir / "package.xml"
        with open(package_xml_path, 'w') as f:
            f.write(f"""<?xml version="1.0"?>
                <package format="2">
                <name>test_package</name>
                </package>
                """)

        dut = MeshModelMaker(mesh_path=package_obj_path,
                             output_dir=obj_dir,
                             encoded_package=str(package_xml_path))

        # Success case: obj in package tree.
        dut.make_model()
        package_path = f"test_package/{relative_obj_dir}"
        sdf_result = obj_dir / "box.sdf"
        self.assertRegex(_file_contents(sdf_result),
                         f"<uri>package://{package_path}/box.obj</uri>")

        # Failure case: non-existent package.xml file.
        dut.encoded_package = str(package_dir / "intermediate" / "package.xml")
        with self.assertRaisesRegex(ValueError,
                                    ".*package.xml cannot be found.*"):
            dut.make_model()

        # Failure case: not in package tree.
        with self.assertRaisesRegex(ValueError,
                                    ".*must be located in the file tree.*"):
            MeshModelMaker(mesh_path=self._obj_path, output_dir=obj_dir,
                           encoded_package=str(package_xml_path)).make_model()

    def test_package_errors(self):
        # Bad encoding (not a valid encoding value) raises an error.
        dut = MeshModelMaker(mesh_path=self._obj_path,
                             output_dir=self._temp_dir,
                             encoded_package='junk')
        with self.assertRaisesRegex(ValueError, ".*Unrecognized.+junk.*"):
            dut.make_model()
        sdf_result = self._temp_dir / "box.sdf"
        self.assertFalse(sdf_result.exists())

        # Evaluate bad existing package.xml. First set up an isolated directory
        # to play in.
        package_dir = self._temp_dir / "bad_package"
        package_dir.mkdir()
        package_obj_path = package_dir / "box.obj"
        shutil.copy(self._obj_path, package_obj_path)
        non_xml_path = package_dir / "package.xml"

        # Failure scenario 1: package.xml lacks a name element.
        with open(non_xml_path, 'w') as f:
            # f.write("Not really an xml file\n")
            f.write('<?xml version="1.0"?><package format="2"/>')

        dut.mesh_path = package_obj_path
        dut.encoded_package = str(non_xml_path)
        with self.assertRaisesRegex(ValueError,
                                    ".*missing a 'name' element.*"):
            dut.make_model()

        # Failure scenario 2: package.xml isn't even xml.
        with open(non_xml_path, 'w') as f:
            f.write("Not really an xml file\n")

        dut.mesh_path = package_obj_path
        dut.encoded_package = str(non_xml_path)
        with self.assertRaisesRegex(xml.etree.ElementTree.ParseError,
                                    ".*syntax error.*"):
            dut.make_model()


class TestMeshToModelProcess(unittest.TestCase):
    """
    Tests the command-line tool mesh_to_model by invoking the process
    directly.

    The above tests on the underlying class confirm correctness of the SDFormat
    file and logic for processing parameters. For these tests, we need to
    confirm that command-line parameters are passed as expected.

    We'll simply create two SDFormat files, one via subprocess and one via
    direct calls to ModelMeshMaker and compare the files.
    """
    def setUp(self):
        self._temp_dir = Path(temp_directory())
        # We need to place the obj file in the temp_directory so that attempts
        # to parse the resulting SDF work.
        self._obj_stem = "box"
        obj_source_path = FindResourceOrThrow(
            f"drake/geometry/render/test/meshes/box.obj")
        self._obj_path = self._temp_dir / "box.obj"
        shutil.copy(obj_source_path, self._obj_path)
        self._dut = FindResourceOrThrow(
            "drake/bindings/pydrake/multibody/mesh_to_model")

    def assert_files_equal(self, dut_path, ref_path):
        self.assertEqual(open(dut_path).read(), open(ref_path).read())

    def test_default_parameters(self):
        # Providing no command-line parameters should be equivalent to the
        # default configuration for MeshModelMaker.
        ref_dir = self._temp_dir / "reference"
        ref_dir.mkdir()
        reference_sdf = ref_dir / "box.sdf"
        maker = MeshModelMaker(mesh_path=self._obj_path,
                               output_dir=ref_dir).make_model()

        subprocess.check_call([self._dut, self._obj_path])

        dut_sdf = self._obj_path.parent / "box.sdf"
        self.assert_files_equal(dut_sdf, reference_sdf)

    def test_non_default_parameters(self):
        # We don't exhaustively test all parameters. We know that the
        # command-line parameters are passed en masse to MeshModelMaker; if one
        # is passed, they are all passed. So, a single, non-default parameter
        # passed through provides sufficient evidence. This presupposes that
        # all of the defaulted MeshModelMaker's construction properties are
        # properly mapped to by the parsed arguments.
        ref_dir = self._temp_dir / "reference"
        ref_dir.mkdir()
        reference_sdf = ref_dir / "box.sdf"
        maker = MeshModelMaker(mesh_path=self._obj_path,
                               output_dir=ref_dir,
                               scale=1.5).make_model()

        dut_sdf = self._obj_path.parent / "box.sdf"
        subprocess.check_call([self._dut, '--scale', '1.5', self._obj_path])

        self.assert_files_equal(dut_sdf, reference_sdf)

    def test_reject_setting_both_mass_and_density(self):
        """
        Checks that we can't pass both mass and density.
        """
        with self.assertRaisesRegex(subprocess.CalledProcessError,
                                    ".*non-zero exit status.*"):
            subprocess.check_call([self._dut, '--mass', '1', '--density', '1',
                                   self._obj_path,  self._temp_dir])

    def test_reject_setting_both_com_and_pose(self):
        """
        Checks that we can't pass both --body-origin and --origin-at-com.
        """
        with self.assertRaisesRegex(subprocess.CalledProcessError,
                                    ".*non-zero exit status.*"):
            subprocess.check_call([self._dut, '--origin-at-com',
                                   '--body-origin', '1,2,3',
                                   self._obj_path,  self._temp_dir])
