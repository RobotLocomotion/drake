# -*- coding: utf-8 -*-

from pydrake.multibody.parsing import (
    AddCollisionFilterGroup,
    AddDirectives,
    AddFrame,
    AddModel,
    AddModelInstance,
    AddWeld,
    FlattenModelDirectives,
    GetScopedFrameByName,
    LoadModelDirectives,
    LoadModelDirectivesFromString,
    ModelDirective,
    ModelDirectives,
    ModelInstanceInfo,
    PackageMap,
    Parser,
    ProcessModelDirectives,
)

import copy
import os
import re
import unittest

from pydrake.common import FindResourceOrThrow
from pydrake.common.test_utilities.deprecation import catch_drake_warnings
from pydrake.geometry import SceneGraph
from pydrake.multibody.tree import (
    ModelInstanceIndex,
)
from pydrake.multibody.plant import (
    MultibodyPlant,
)


class TestParsing(unittest.TestCase):

    def test_package_map(self):
        # Simple coverage test for constructors.
        dut = PackageMap()
        self.assertEqual(dut.size(), 2)
        PackageMap(other=dut)
        copy.copy(dut)

        dut = PackageMap.MakeEmpty()
        dut2 = PackageMap.MakeEmpty()
        tmpdir = os.environ.get('TEST_TMPDIR')

        # Simple coverage test for Add, AddMap, Contains, size,
        # GetPackageNames, GetPath, AddPackageXml, Remove.
        dut.Add(package_name="root", package_path=tmpdir)
        dut2.Add(package_name="root", package_path=tmpdir)
        dut.AddMap(dut2)
        self.assertTrue(dut.Contains(package_name="root"))
        self.assertEqual(dut.size(), 1)
        self.assertEqual(dut.GetPackageNames(), ["root"])
        self.assertEqual(dut.GetPath(package_name="root"), tmpdir)
        dut.AddPackageXml(filename=FindResourceOrThrow(
            "drake/multibody/parsing/test/box_package/package.xml"))
        dut2.Remove(package_name="root")
        self.assertEqual(dut2.size(), 0)

        # Simple coverage test for folder and environment.
        dut.PopulateFromEnvironment(environment_variable='TEST_TMPDIR')
        dut.PopulateFromFolder(path=tmpdir)

    def test_package_map_remote_params(self):
        dut = PackageMap.RemoteParams(
            urls=["file:///tmp/missing.zip"],
            sha256="0" * 64,
            archive_type="zip",
            strip_prefix="prefix",)
        self.assertIn("missing.zip", dut.ToJson())
        copy.copy(dut)
        copy.deepcopy(dut)

    def test_package_map_add_remote(self):
        """Runs a full lifecycle of AddRemote + GetPath to check that Python
        bindings calling C++ code that shells out to Python all plays nice.
        """
        dut = PackageMap.MakeEmpty()
        zipfile = FindResourceOrThrow(
            "drake/multibody/parsing/test/package_map_test_packages/"
            "compressed.zip")
        dut.AddRemote(package_name="compressed",
                      params=PackageMap.RemoteParams(
                          urls=[f"file://{zipfile}"],
                          sha256=("b4bdbad313293ca61fe8f4ed1b5579da"
                                  "dadb3a5c08f0a6d06a8e39e5f97f1bd1"),
                          strip_prefix="compressed_prefix"))
        path = dut.GetPath("compressed")
        with open(f"{path}/README", encoding="utf-8") as f:
            self.assertEqual(f.read(), "This package is empty.\n")

    def test_parser_file(self):
        """Calls every combination of arguments for the Parser methods which
        use a file_name (not contents) and inspects their return type.
        """
        sdf_file = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf")
        urdf_file = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.urdf")
        for dut, file_name in (
                (Parser.AddModels, sdf_file),
                (Parser.AddModels, urdf_file),
                ):
            plant = MultibodyPlant(time_step=0.01)
            parser = Parser(plant=plant)
            result = dut(parser, file_name=file_name)
            self.assertIsInstance(result, list)
            self.assertIsInstance(result[0], ModelInstanceIndex)

    def test_parser_file_deprecated(self):
        sdf_file = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf")
        urdf_file = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.urdf")
        for dut, file_name, model_name, result_dim in (
                (Parser.AddModelFromFile, sdf_file, None, int),
                (Parser.AddModelFromFile, sdf_file, "", int),
                (Parser.AddModelFromFile, sdf_file, "a", int),
                (Parser.AddModelFromFile, urdf_file, None, int),
                (Parser.AddModelFromFile, urdf_file, "", int),
                (Parser.AddModelFromFile, urdf_file, "a", int),
                (Parser.AddAllModelsFromFile, sdf_file, None, list),
                (Parser.AddAllModelsFromFile, urdf_file, None, list),
                ):
            plant = MultibodyPlant(time_step=0.01)
            parser = Parser(plant=plant)

            with catch_drake_warnings(expected_count=1):
                if model_name is None:
                    result = dut(parser, file_name=file_name)
                else:
                    result = dut(parser, file_name=file_name,
                                 model_name=model_name)
            if result_dim is int:
                self.assertIsInstance(result, ModelInstanceIndex)
            else:
                assert result_dim is list
                self.assertIsInstance(result, list)
                self.assertIsInstance(result[0], ModelInstanceIndex)

    def test_parser_string(self):
        """Checks parsing from a string (not file_name)."""
        sdf_file = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf")
        with open(sdf_file, "r") as f:
            sdf_contents = f.read()

        plant = MultibodyPlant(time_step=0.01)
        parser = Parser(plant=plant)
        results = parser.AddModelsFromString(
            file_contents=sdf_contents, file_type="sdf")
        self.assertIsInstance(results[0], ModelInstanceIndex)

        # Check the related AddModel overload.
        plant = MultibodyPlant(time_step=0.01)
        parser = Parser(plant=plant)
        results = parser.AddModels(
            file_contents=sdf_contents, file_type="sdf")
        self.assertIsInstance(results[0], ModelInstanceIndex)

    def test_parser_url(self):
        """Tests for AddModelsFromUrl as well as its related AddModel overload.
        """
        sdf_url = "package://drake/multibody/benchmarks/acrobot/acrobot.sdf"

        plant = MultibodyPlant(time_step=0.01)
        results = Parser(plant).AddModelsFromUrl(url=sdf_url)
        self.assertIsInstance(results[0], ModelInstanceIndex)

        plant = MultibodyPlant(time_step=0.01)
        results = Parser(plant).AddModels(url=sdf_url)
        self.assertIsInstance(results[0], ModelInstanceIndex)

    def test_parser_prefix_constructors(self):
        model = "<robot name='r'><link name='a'/></robot>"
        plant = MultibodyPlant(time_step=0.0)
        scene_graph = SceneGraph()

        Parser(plant=plant).AddModelsFromString(model, "urdf")

        # Reload the same model, via a different parser constructor. Catch the
        # name collision.
        with self.assertRaisesRegex(RuntimeError, r'.*names must be unique.*'):
            Parser(plant=plant, scene_graph=scene_graph).AddModelsFromString(
                model, "urdf")

        # Reload the same model, but use model_name_prefix to avoid name
        # collisions.
        Parser(plant=plant, model_name_prefix="prefix1").AddModelsFromString(
            model, "urdf")
        Parser(plant=plant, scene_graph=scene_graph,
               model_name_prefix="prefix2").AddModelsFromString(model, "urdf")

    def test_strict(self):
        model = """<robot name='robot' version='0.99'>
            <link name='a'/>
            </robot>"""
        # Use lax parsing.
        plant = MultibodyPlant(time_step=0.01)
        parser = Parser(plant=plant)
        results = parser.AddModelsFromString(
            file_contents=model, file_type='urdf')
        self.assertIsInstance(results[0], ModelInstanceIndex)
        # Use strict parsing.
        plant = MultibodyPlant(time_step=0.01)
        parser = Parser(plant=plant)
        parser.SetStrictParsing()
        with self.assertRaisesRegex(RuntimeError, r'.*version.*ignored.*'):
            parser.AddModelsFromString(file_contents=model, file_type='urdf')

    def test_auto_renaming(self):
        model = """<robot name='robot' version='0.99'>
            <link name='a'/>
            </robot>"""
        plant = MultibodyPlant(time_step=0.01)
        parser = Parser(plant=plant)
        self.assertFalse(parser.GetAutoRenaming())
        results = parser.AddModelsFromString(
            file_contents=model, file_type='urdf')
        self.assertIsInstance(results[0], ModelInstanceIndex)
        # Reload without auto-renaming; fail.
        with self.assertRaisesRegex(RuntimeError, r''):
            parser.AddModelsFromString(model, 'urdf')
        # Enable renaming and subsequently succeed with reload.
        parser.SetAutoRenaming(value=True)
        results = parser.AddModelsFromString(model, 'urdf')
        self.assertTrue(plant.HasModelInstanceNamed('robot_1'))

    def test_model_instance_info(self):
        """Checks that ModelInstanceInfo bindings exist."""
        ModelInstanceInfo.model_name
        ModelInstanceInfo.model_path
        ModelInstanceInfo.parent_frame_name
        ModelInstanceInfo.child_frame_name
        ModelInstanceInfo.X_PC
        ModelInstanceInfo.model_instance

    def test_scoped_frame_names(self):
        plant = MultibodyPlant(time_step=0.01)
        GetScopedFrameByName(plant, "world")

    def _make_plant_parser_directives(self):
        """Returns a tuple (plant, parser, directives) for later testing."""
        model_dir = os.path.dirname(FindResourceOrThrow(
            "drake/multibody/parsing/test/"
            "process_model_directives_test/package.xml"))
        plant = MultibodyPlant(time_step=0.01)
        parser = Parser(plant=plant)
        parser.package_map().PopulateFromFolder(model_dir)
        directives_file = model_dir + "/add_scoped_top.dmd.yaml"
        directives = LoadModelDirectives(directives_file)
        return (plant, parser, directives)

    def test_load_model_directives_from_string(self):
        str = """
directives:
- add_model:
    name: new_model
    file: base.sdf
"""
        LoadModelDirectivesFromString(model_directives=str)

    def test_flatten_model_directives(self):
        (plant, parser, directives) = self._make_plant_parser_directives()
        added_models = ProcessModelDirectives(
            directives=directives, parser=parser)
        flat_directives = FlattenModelDirectives(
            directives=directives, package_map=parser.package_map()
        )
        self.assertGreater(len(flat_directives.directives), 0)

    def test_process_model_directives(self):
        """Check the Process... overload using a Parser."""
        (plant, parser, directives) = self._make_plant_parser_directives()
        added_models = ProcessModelDirectives(
            directives=directives, parser=parser)
        model_names = [model.model_name for model in added_models]
        self.assertIn("extra_model", model_names)
        plant.GetModelInstanceByName("extra_model")

    def test_process_model_directives_dispreferred(self):
        """Check the Process... overload that also passes a MbP."""
        (plant, parser, directives) = self._make_plant_parser_directives()
        added_models = ProcessModelDirectives(
            directives=directives, plant=plant, parser=parser)
        model_names = [model.model_name for model in added_models]
        self.assertIn("extra_model", model_names)
        plant.GetModelInstanceByName("extra_model")

    def test_add_collision_filter_group_struct(self):
        """Checks the bindings of the AddCollisionFilterGroup helper struct."""
        dut = AddCollisionFilterGroup(name="foo")
        self.assertIn("foo", repr(dut))
        copy.copy(dut)
        copy.deepcopy(dut)

    def test_add_directives_struct(self):
        """Checks the bindings of the AddDirectives helper struct."""
        dut = AddDirectives(file="package://foo/bar.dmd.yaml")
        self.assertIn("bar.dmd.yaml", repr(dut))
        copy.copy(dut)
        copy.deepcopy(dut)

    def test_add_frame_struct(self):
        """Checks the bindings of the AddFrame helper struct."""
        dut = AddFrame(name="foo")
        self.assertIn("foo", repr(dut))
        copy.copy(dut)
        copy.deepcopy(dut)

    def test_add_model_struct(self):
        """Checks the bindings of the AddModel helper struct."""
        dut = AddModel(file="package://foo/bar.sdf")
        self.assertIn("bar.sdf", repr(dut))
        copy.copy(dut)
        copy.deepcopy(dut)

    def test_add_model_instance_struct(self):
        """Checks the bindings of the AddModelInstance helper struct."""
        dut = AddModelInstance(name="foo")
        self.assertIn("foo", repr(dut))
        copy.copy(dut)
        copy.deepcopy(dut)

    def test_add_weld_struct(self):
        """Checks the bindings of the AddWeld helper struct."""
        dut = AddWeld(parent="foo")
        self.assertIn("foo", repr(dut))
        copy.copy(dut)
        copy.deepcopy(dut)

    def test_model_directive_struct(self):
        """Checks the bindings of the ModelDirective helper struct."""
        dut = ModelDirective(add_model=None)
        self.assertIn("add_model", repr(dut))
        copy.copy(dut)
        copy.deepcopy(dut)

    def test_model_directives_struct(self):
        """Checks the bindings of the ModelDirectives helper struct."""
        dut = ModelDirectives(directives=[ModelDirective()])
        self.assertIn("add_model", repr(dut))
        copy.copy(dut)
        copy.deepcopy(dut)
