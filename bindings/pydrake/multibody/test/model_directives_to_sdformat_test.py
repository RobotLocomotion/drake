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


def get_model_instances(plant):
    # TODO(eric.cousineau): Hoist this somewhere?
    return _get_plant_aggregate(
        plant.num_model_instances, lambda x: x, ModelInstanceIndex
    )


def get_model_instances_names(plant):
    # TODO(eric.cousineau): Hoist this somewhere?
    return _get_plant_aggregate(
        plant.num_model_instances,
        plant.GetModelInstanceName,
        ModelInstanceIndex,
    )


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
    # TODO(eric.cousineau): Remove once `zip(*, strict=True)` is available:
    # https://www.python.org/dev/peps/pep-0618/
    # WARNING: Items should be *iterable*, meaning they can be iterated
    # over multiple times. This will not work for *iterators* that can be
    # exhausted.
    assert len(items) >= 1
    count = len(items[0])
    for i, x in enumerate(items):
        assert len(x) == count, f"len(items[{i}]) = {len(x)} != {count}"
    return zip(*items)


# Checks that body_a and body_b are effectively the same.
def assert_bodies_same(plant_a, context_a, body_a, plant_b, context_b, body_b):
    if body_a.name() != body_b.name():
        return False

    # Check that positions in the world frame reference are also the same.
    directives_body_transform = plant_a.EvalBodyPoseInWorld(context_a, body_a)
    sdformat_body_transform = plant_b.EvalBodyPoseInWorld(context_b, body_b)
    directives_body_transform.IsNearlyEqualTo(sdformat_body_transform, 1e-10)

    # Check frames attached to this body.
    frames_b = get_frames_attached_to(plant_b, [body_b])
    frames_a = get_frames_attached_to(plant_a, [body_a])

    # All frames created through model directives would have
    # been created when loading the SDFormat.
    if not all(
        frame_a.name() in [frame_b.name() for frame_b in frames_b]
        for frame_a in frames_a
    ):
        return False

    return True


# Compares if two lists of joints are similar. If they have the same
# child and parent, body name and the same type, we consider them the same.
def are_joints_same(joints_a, joints_b):
    for i in range(len(joints_a)):
        for j in range(len(joints_b)):
            if (
                joints_a[i].parent_body().name()
                == joints_b[j].parent_body().name()
                and joints_a[i].child_body().name()
                == joints_b[j].child_body().name()
                and joints_a[i].type_name() == joints_b[j].type_name()
            ):
                joints_a.pop(i)
                joints_b.pop(j)

    # All joints should have been verified.
    if len(joints_a) == 0 and len(joints_b) == 0:
        return True
    else:
        return False


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
            sdf_instance_index = sdf_plant.GetModelInstanceByName(
                f"{name}::{dmd_plant.GetModelInstanceName(dmd_instance_index)}")

            # Check bodies and corresponding frames.
            dmd_bodies = get_bodies(dmd_plant, [dmd_instance_index])
            sdf_bodies = get_bodies(sdf_plant, [sdf_instance_index])
            dmd_context = dmd_plant.CreateDefaultContext()
            sdf_context = sdf_plant.CreateDefaultContext()
            for sdf_body, dmd_body in _strict_zip(sdf_bodies, dmd_bodies):
                self.assertTrue(assert_bodies_same(
                    dmd_plant, dmd_context, dmd_body,
                    sdf_plant, sdf_context, sdf_body))

            # Check joints.
            self.assertTrue(are_joints_same(
                get_joints(dmd_plant, [dmd_instance_index]),
                get_joints(sdf_plant, [sdf_instance_index])))

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
        # TODO(jwnimmer-tri) Don't do this; changes to os.environ bleed into
        # other test cases.
        os.environ["ROS_PACKAGE_PATH"] = "bindings/pydrake/multibody"
        files = _convert_directives(
            dmd_filename=dmd_filename,
            expand_included=True,
            generate_world=False,
        )
        self.assertEqual(len(files), len(expected_filenames))
        for (path, xml), expected in zip(files.items(), expected_filenames):
            self.assertEqual(path.name, expected.name.replace("expected-", ""))
            with open(expected, encoding="utf-8") as f:
                expected_xml = f.read()
            self.assertMultiLineEqual(xml, expected_xml)

    def test_resulting_xml(self):
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
