import argparse
import os
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
from xml.dom import minidom

from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import (
    BodyIndex,
    FrameIndex,
    ModelInstanceIndex,
    JointIndex,
)
from pydrake.common import FindResourceOrThrow
from pydrake.common.test_utilities.meta import (
    ValueParameterizedTest,
    run_with_multiple_values,
)

from pydrake.multibody.model_directives_to_sdformat import (
    _convert_directives,
)


def _get_plant_aggregate(num_func, get_func, index_cls, model_instances=None):
    items = []
    for i in range(num_func()):
        item = get_func(index_cls(i))
        if model_instances is None or item.model_instance() in model_instances:
            items.append(item)
    return items


def get_bodies(plant, model_instances=None):
    # TODO(eric.cousineau): Hoist this somewhere?
    return _get_plant_aggregate(
        plant.num_bodies, plant.get_body, BodyIndex, model_instances
    )


def get_frames(plant, model_instances=None):
    # TODO(eric.cousineau): Hoist this somewhere?
    return _get_plant_aggregate(
        plant.num_frames, plant.get_frame, FrameIndex, model_instances
    )


def get_frames_attached_to(plant, bodies):
    # TODO(eric.cousineau): Hoist this somewhere?
    frames = []
    for frame in get_frames(plant):
        if frame.body() in bodies:
            frames.append(frame)
    return frames


def get_joints(plant, model_instances=None):
    # TODO(eric.cousineau): Hoist this somewhere?
    return _get_plant_aggregate(
        plant.num_joints, plant.get_joint, JointIndex, model_instances
    )


def _strict_zip(*items):
    # TODO(eric.cousineau): Use `zip(*, strict=True)` once we use Python 3.10+
    # everywhere (i.e., once we drop Ubuntu 18.04).
    # WARNING: Items should be *iterable*, meaning they can be iterated
    # over multiple times. This will not work for *iterators* that can be
    # exhausted.
    assert len(items) >= 1
    lengths = tuple([len(x) for x in items])
    if len(set(lengths)) != 1:
        raise ValueError(f"Mismatch sizes {lengths} to zip() on {items}")
    return zip(*items)


class TestConvertModelDirectiveToSdformat(
    unittest.TestCase, metaclass=ValueParameterizedTest
):
    def setUp(self):
        self.maxDiff = None
        self.dmd_test_path = Path(
            "bindings/pydrake/multibody/test/"
            "model_directives_to_sdformat_files"
        )

    @run_with_multiple_values([dict(name=name) for name in [
        "deep_frame",
        "deep_weld",
        "frame_attached_to_frame",
        "hidden_frame",
        "inject_frames",
        "scoped_frame_name",
        "weld_frames_from_models",
    ]])
    def test_through_plant_comparison(self, *, name):
        """Checks that a MultibodyPlant created from the conversion tool's
        SDFormat output is identical to a MultibodyPlant created from the
        original *.dmd.yaml file.
        """
        # Convert, expecting exactly one output file.
        dmd_filename = self.dmd_test_path / f"{name}.dmd.yaml"
        output_files = _convert_directives(
            dmd_filename=dmd_filename,
            generate_world=False,
        )
        self.assertEqual(len(output_files), 1)
        _, sdf_str = output_files.popitem()

        # Helper packages that used by some of the test case inputs.
        package_xml_filenames = [
            FindResourceOrThrow(
                "drake/multibody/parsing/"
                "test/process_model_directives_test/package.xml"),
            FindResourceOrThrow(
                "drake/bindings/pydrake/multibody/"
                "test/model_directives_to_sdformat_files/package.xml"),
        ]

        # Load the original model directive file.
        dmd_plant = MultibodyPlant(time_step=0.0)
        dmd_parser = Parser(plant=dmd_plant)
        for package_xml_filename in package_xml_filenames:
            dmd_parser.package_map().AddPackageXml(package_xml_filename)
        dmd_parser.AddModels(str(dmd_filename))
        dmd_plant.Finalize()

        # Load the converted SDFormat file.
        sdf_plant = MultibodyPlant(time_step=0.0)
        sdf_parser = Parser(sdf_plant)
        for package_xml_filename in package_xml_filenames:
            sdf_parser.package_map().AddPackageXml(package_xml_filename)
        sdf_parser.AddModelsFromString(sdf_str, "sdf")
        sdf_plant.Finalize()

        # The conversion process creates an extra top level model instance.
        # (This could be avoided using the generated world model with merge
        # include.)
        self.assertEqual(
            dmd_plant.num_model_instances(),
            sdf_plant.num_model_instances() - 1)

        # Compare the two plants, one model instance at a time. (We don't
        # compare the world model nor the default model.)
        for i in range(2, dmd_plant.num_model_instances()):
            dmd_instance_index = ModelInstanceIndex(i)
            sdf_instance_index = sdf_plant.GetModelInstanceByName("::".join([
                name, dmd_plant.GetModelInstanceName(dmd_instance_index)]))

            # Check bodies.
            dmd_context = dmd_plant.CreateDefaultContext()
            sdf_context = sdf_plant.CreateDefaultContext()
            for sdf_body, dmd_body in _strict_zip(
                    get_bodies(sdf_plant, [sdf_instance_index]),
                    get_bodies(dmd_plant, [dmd_instance_index])):
                self.assertEqual(sdf_body.name(), dmd_body.name())
                sdf_X_WB = sdf_plant.EvalBodyPoseInWorld(sdf_context, sdf_body)
                dmd_X_WB = dmd_plant.EvalBodyPoseInWorld(dmd_context, dmd_body)
                self.assertTrue(sdf_X_WB.IsNearlyEqualTo(dmd_X_WB, 1e-10))

                # Check frames attached to this body. The converted file ends
                # up with some spurious extra frames that we'll ignore.
                sdf_frames = [
                    x for x in get_frames_attached_to(sdf_plant, [sdf_body])
                    if not x.name().startswith("_merged__")
                ]
                dmd_frames = get_frames_attached_to(dmd_plant, [dmd_body])
                for sdf_frame, dmd_frame in _strict_zip(
                        sdf_frames,
                        dmd_frames):
                    self.assertEqual(sdf_frame.name(), dmd_frame.name())

            # Check joints.
            for sdf_joint, dmd_joint in _strict_zip(
                    get_joints(sdf_plant, [sdf_instance_index]),
                    get_joints(dmd_plant, [dmd_instance_index])):
                self.assertEqual(sdf_joint.type_name(),
                                 dmd_joint.type_name())
                self.assertEqual(sdf_joint.parent_body().name(),
                                 dmd_joint.parent_body().name())
                self.assertEqual(sdf_joint.child_body().name(),
                                 dmd_joint.child_body().name())

    @run_with_multiple_values([dict(name=name) for name in [
        "deep_child_frame_weld",
        "deep_child_weld",
        "default_free_body_pose",
        "default_joint_positions",
        "different_scopes_frame",
        "frame_same_as_base_frame",
        "frames_same_name",
        "implicit_hidden_base_frame",
        "not_directives_first",
        "something_not_directives",
        "weld_same_parent_child",
        "world_base_frame",
    ]])
    def test_error(self, *, name):
        """Checks that a broken or unsupported dmd input file raises a useful
        error message.
        """
        dmd_filename = self.dmd_test_path / f"errors/{name}.dmd.yaml"
        with open(self.dmd_test_path / f"errors/{name}.error_regex") as f:
            expected_message_regex = f.read().strip()
        with self.assertRaisesRegex(Exception, expected_message_regex):
            _convert_directives(dmd_filename=dmd_filename)

    def test_add_model_instance_add_directives(self):
        dmd_filename = self.dmd_test_path / "add_directives.dmd.yaml"
        expected_filenames = [
            self.dmd_test_path / "add_directives.expected-sdf",
            self.dmd_test_path / "hidden_frame.expected-sdf",
        ]
        try:
            os.environ["ROS_PACKAGE_PATH"] = str(self.dmd_test_path)
            files = _convert_directives(
                dmd_filename=dmd_filename,
                expand_included=True,
                generate_world=False,
            )
        finally:
            del os.environ["ROS_PACKAGE_PATH"]
        self.assertEqual(len(files), len(expected_filenames))
        for (path, xml), expected in zip(files.items(), expected_filenames):
            self.assertEqual(path.name, expected.name.replace("expected-", ""))
            with open(expected, encoding="utf-8") as f:
                expected_xml = f.read()
            self.assertMultiLineEqual(xml, expected_xml)

    def test_resulting_xml(self):
        """Checks the literal xml output of a dmd conversion (instead of merely
        checking for MultibodyPlant equivalence).
        """
        dmd_filename = self.dmd_test_path / "inject_frames.dmd.yaml"
        expected_path = self.dmd_test_path / "inject_frames.expected-sdf"
        with open(expected_path, encoding="utf-8") as f:
            expected_xml = f.read()
        files = _convert_directives(
            dmd_filename=dmd_filename,
            generate_world=False,
        )
        self.assertEqual(len(files), 1)
        _, sdformat_str = files.popitem()
        self.maxDiff = None
        self.assertMultiLineEqual(expected_xml, sdformat_str)

    def test_error_wrong_file_extension(self):
        with self.assertRaisesRegex(Exception, "determine file format"):
            dmd_filename = Path("frame_same_as_base_frame.not_valid")
            _convert_directives(dmd_filename=dmd_filename)
