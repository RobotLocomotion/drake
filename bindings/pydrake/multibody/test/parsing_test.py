# -*- coding: utf-8 -*-

from pydrake.multibody.parsing import (
    AddCollisionFilterGroup,
    AddDirectives,
    AddFrame,
    AddModel,
    AddModelInstance,
    AddWeld,
    GetScopedFrameByName,
    GetScopedFrameName,
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
from pydrake.multibody.tree import (
    ModelInstanceIndex,
)
from pydrake.multibody.plant import (
    MultibodyPlant,
)


class TestParsing(unittest.TestCase):

    def test_package_map(self):
        # Simple coverage test for default constructor
        dut = PackageMap()
        self.assertEqual(dut.size(), 1)

        dut = PackageMap.MakeEmpty()
        dut2 = PackageMap.MakeEmpty()
        tmpdir = os.environ.get('TEST_TMPDIR')
        model = FindResourceOrThrow(
            "drake/examples/atlas/urdf/atlas_minimal_contact.urdf")

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

    def test_parser_file(self):
        """Calls every combination of arguments for the Parser methods which
        use a file_name (not contents) and inspects their return type.
        """
        sdf_file = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf")
        urdf_file = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.urdf")
        for dut, file_name, model_name, result_dim in (
                # XXX rewrite?
                # (Parser.AddModelFromFile, sdf_file, None, int),
                # (Parser.AddModelFromFile, sdf_file, "", int),
                # (Parser.AddModelFromFile, sdf_file, "a", int),
                # (Parser.AddModelFromFile, urdf_file, None, int),
                # (Parser.AddModelFromFile, urdf_file, "", int),
                # (Parser.AddModelFromFile, urdf_file, "a", int),
                (Parser.AddAllModelsFromFile, sdf_file, None, list),
                (Parser.AddAllModelsFromFile, urdf_file, None, list),
                ):
            plant = MultibodyPlant(time_step=0.01)
            parser = Parser(plant=plant)
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
        self.assertEqual(parser.plant(), plant)
        with catch_drake_warnings(expected_count=1):
            result = parser.AddModelFromString(
                file_contents=sdf_contents, file_type="sdf")
        self.assertIsInstance(result, ModelInstanceIndex)

        plant = MultibodyPlant(time_step=0.01)
        parser = Parser(plant=plant)
        results = parser.AddModelsFromString(
            file_contents=sdf_contents, file_type="sdf")
        self.assertIsInstance(results[0], ModelInstanceIndex)

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
        with self.assertRaises(RuntimeError) as e:
            result = parser.AddModelsFromString(
                file_contents=model, file_type='urdf')
        pattern = r'.*version.*ignored.*'
        message = str(e.exception)
        match = re.match(pattern, message)
        self.assertTrue(match, f'"{message}" does not match "{pattern}"')

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
        frame = GetScopedFrameByName(plant, "world")
        self.assertIsNotNone(GetScopedFrameName(plant, frame))

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
