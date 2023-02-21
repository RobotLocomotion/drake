import lxml.etree as ET
import os
import unittest

from pydrake.multibody.parsing import (
    Parser,
    LoadModelDirectives,
    ProcessModelDirectives,
)
from pydrake.multibody.plant import (
    MultibodyPlant,
)
from pydrake.multibody.tree import (
    BodyIndex,
    FrameIndex,
    ModelInstanceIndex,
    JointIndex,
)
from pydrake.common import GetDrakePath
from pydrake.common.test_utilities.meta import (
    ValueParameterizedTest,
    run_with_multiple_values,
)

import drake.multibody.parsing.model_directives_to_sdformat as \
    model_directives_to_sdformat
from drake.multibody.parsing.model_directives_to_sdformat import (
    convert_directive, ConversionError)

_SCOPE_DELIMITER = '::'


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
        ModelInstanceIndex
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


# Checks that body_a and body_b are effectively the same.
def assert_bodies_same(plant_a, context_a, body_a, plant_b, context_b, body_b):
    if (body_a.name() != body_b.name()):
        return False

    # Check that positions in the world frame reference are also the same
    directives_body_transform = \
        plant_a.EvalBodyPoseInWorld(
            context_a,
            body_a)
    sdformat_body_transform = plant_b.EvalBodyPoseInWorld(
        context_b, body_b)
    directives_body_transform.IsNearlyEqualTo(
        sdformat_body_transform, 1e-10)

    # Check frames attached to this body
    frames_b = get_frames_attached_to(
        plant_b, [body_b])
    frames_a = get_frames_attached_to(
        plant_a, [body_a])

    # All frames created through model directives would have
    # been created when loading the SDFormat
    if not all(
            frame_a.name()
            in
            [frame_b.name()
                for frame_b in frames_b]
            for frame_a in frames_a):
        return False

    return True


# Compares if two lists of joints are similar. If they have the same
# child and parent, body name and the same type, we consider them the same.
def are_joints_same(joints_a, joints_b):
    for i in range(len(joints_a)):
        for j in range(len(joints_b)):
            if joints_a[i].parent_body().name() == \
                joints_b[j].parent_body().name() \
                and joints_a[i].child_body().name() == \
                joints_b[j].child_body().name() \
                and joints_a[i].type_name() == \
                    joints_b[j].type_name():
                joints_a.pop(i)
                joints_b.pop(j)

    # All joints should have been verified
    if len(joints_a) == 0 and len(joints_b) == 0:
        return True
    else:
        return False


class TestConvertModelDirectiveToSDF(unittest.TestCase,
                                     metaclass=ValueParameterizedTest):

    files_to_test = [
        'multibody/parsing/test/model_directives_to_sdformat_files/'
        'inject_frames.yaml',
        'multibody/parsing/test/model_directives_to_sdformat_files/'
        'hidden_frame.yaml',
        'multibody/parsing/test/model_directives_to_sdformat_files/'
        'frame_attached_to_frame.yaml',
        'multibody/parsing/test/model_directives_to_sdformat_files/'
        'weld_frames_from_models.yaml',
        'multibody/parsing/test/model_directives_to_sdformat_files/'
        'scoped_frame_name.yaml',
    ]

    @run_with_multiple_values([dict(file_path=file_path)
                               for file_path in files_to_test])
    def test_through_plant_comparison(self, *, file_path):
        # Convert
        sdformat_tree = convert_directive(file_path, toplevel_entity='world')
        sfdormat_result = ET.tostring(
            sdformat_tree, pretty_print=True, encoding="unicode")

        # Load model directives
        directives_plant = MultibodyPlant(time_step=0.01)
        model_dir = os.path.dirname(os.path.join(
                GetDrakePath(),
                'multibody/parsing/test/'))
        parser = Parser(plant=directives_plant)
        parser.package_map().PopulateFromFolder(model_dir)
        directives = LoadModelDirectives(file_path)
        ProcessModelDirectives(directives=directives,
                               plant=directives_plant, parser=parser)
        directives_plant.Finalize()

        # Load converted SDFormat
        sdformat_plant = MultibodyPlant(time_step=0.01)
        sdformat_parser = Parser(sdformat_plant)
        sdformat_parser.package_map().PopulateFromFolder(model_dir)
        sdformat_parser.AddModelsFromString(sfdormat_result, "sdf")
        sdformat_plant.Finalize()

        # Compare plants
        self.assertEqual(sdformat_plant.num_model_instances(),
                         directives_plant.num_model_instances())

        file_name = os.path.splitext(os.path.basename(file_path))[0]

        for i in range(2, directives_plant.num_model_instances()):
            model_scoped_name = directives_plant.GetModelInstanceName(
                    ModelInstanceIndex(i))

            sdformat_model_instances = get_model_instances_names(
                sdformat_plant)
            model_found = False

            if model_scoped_name in sdformat_model_instances:
                model_found = True
                sdformat_model_index = sdformat_model_instances.index(
                    model_scoped_name)

            self.assertTrue(model_found)

            # Check Model Bodies and corresponding Frames
            model_instance = ModelInstanceIndex(i)
            directives_bodies = get_bodies(
                directives_plant, [model_instance])
            directives_context = directives_plant.CreateDefaultContext()
            sdformat_intance_index = ModelInstanceIndex(sdformat_model_index)
            sdformat_bodies = get_bodies(
                sdformat_plant, [sdformat_intance_index])
            sdformat_context = sdformat_plant.CreateDefaultContext()

            for sdformat_body, directives_body in zip(sdformat_bodies,
                                                      directives_bodies):
                self.assertTrue(assert_bodies_same(
                    directives_plant, directives_context, directives_body,
                    sdformat_plant, sdformat_context, sdformat_body))

            # Check Model Joints
            self.assertTrue(are_joints_same(
                get_joints(directives_plant, [model_instance]),
                get_joints(sdformat_plant, [sdformat_intance_index])))

    def test_error_no_directives(self):
        with self.assertRaisesRegex(
                ConversionError, r'\[directives\] must be the first keyword'
                ' in the yaml file, exiting.'):
            convert_directive('multibody/parsing/test/'
                              'model_directives_to_sdformat_files/'
                              'something_not_directives.yaml')

    def test_error_directives_not_frist(self):
        with self.assertRaisesRegex(
                ConversionError, r'\[directives\] must be the first keyword'
                ' in the yaml file, exiting.'):
            convert_directive('multibody/parsing/test/'
                              'model_directives_to_sdformat_files/'
                              'not_directives_first.yaml')

    def test_error_implicit_hidden_base_frame(self):
        with self.assertRaisesRegex(
                ConversionError, 'Unable to find scope for frame: '
                r'\[frame\] while adding frame: \[frame_name\].'):
            convert_directive('multibody/parsing/test/'
                              'model_directives_to_sdformat_files/'
                              'implicit_hidden_base_frame.yaml')

    def test_error_different_scopes_frame(self):
        with self.assertRaisesRegex(
                ConversionError, 'Frame named: '
                r'\[extra_model::sub_added_frame\] has a '
                'different scope in its name and its '
                r'base_frame: \[simple_model::frame\].'):
            convert_directive('multibody/parsing/test/'
                              'model_directives_to_sdformat_files/'
                              'different_scopes_frame.yaml')

    def test_error_world_base(self):
        with self.assertRaisesRegex(
                ConversionError,
                r'Adding a frame using base_frame=\[world\] is '
                'not supported.'):
            convert_directive(
                'multibody/parsing/test/'
                'model_directives_to_sdformat_files/world_base_frame.yaml')

    def test_error_frame_name_same_base_name(self):
        with self.assertRaisesRegex(
                ConversionError, r'Frame: \[frame\] has the same name as '
                'it\'s base frame. This case is not '
                'supported.'):
            convert_directive('multibody/parsing/test/'
                              'model_directives_to_sdformat_files/'
                              'frame_same_as_base_frame.yaml')

    def test_error_frames_same_name(self):
        with self.assertRaisesRegex(
                ConversionError, 'Found more than two frames with name: '
                r'\[frame_name\], could not resolve the '
                'scope.'):
            convert_directive(
                'multibody/parsing/test/'
                'model_directives_to_sdformat_files/frames_same_name.yaml')

    def test_error_too_many_model_scopes(self):
        with self.assertRaisesRegex(
                ConversionError, 'Too many nested models in frame: '
                r'\[top_level_model::inner_model::base\]. Only one level of '
                'nesting is allowed.'):
            convert_directive(
                'multibody/parsing/test/'
                'model_directives_to_sdformat_files/too_many_models.yaml')

    def test_add_model_instance_add_directives(self):
        os.environ['ROS_PACKAGE_PATH'] = 'multibody/parsing/'
        expected_xml = """<sdf version="1.9">
  <model name="add_directives">
    <model name="model_instance">
      <include merge="true">
        <uri>package://model_directives_to_sdformat_files/hidden_frame.sdf</uri>
      </include>
    </model>
  </model>
</sdf>
"""
        result = convert_directive(
            'multibody/parsing/test/'
            'model_directives_to_sdformat_files/add_directives.yaml', True)
        print(ET.tostring(result,
                          pretty_print=True,
                          encoding="unicode"))
        self.assertEqual(expected_xml, ET.tostring(result,
                                                   pretty_print=True,
                                                   encoding="unicode"))

    def test_resulting_xml_and_weld_structures(self):
        result = convert_directive(
            'multibody/parsing/test/'
            'model_directives_to_sdformat_files/inject_frames.yaml', True)
        expected_xml = """<sdf version="1.9">
  <model name="inject_frames">
    <model name="top_level_model">
      <include merge="true">
        <name>top_level_model</name>
        <uri>package://process_model_directives_test/simple_model.sdf</uri>
      </include>
      <frame name="top_injected_frame" attached_to="base">
        <pose degrees="true">1 2 3   10.0 20.0 30.0</pose>
      </frame>
    </model>
    <model name="mid_level_model" placement_frame="base">
      <pose relative_to="top_level_model::top_injected_frame"/>
      <include merge="true">
        <name>mid_level_model</name>
        <uri>package://process_model_directives_test/simple_model.sdf</uri>
      </include>
      <frame name="mid_injected_frame" attached_to="base">
        <pose degrees="true">1 2 3   0 0 0</pose>
      </frame>
    </model>
    <include>
      <name>bottom_level_model</name>
      <uri>package://process_model_directives_test/simple_model.sdf</uri>
      <placement_frame>base</placement_frame>
      <pose relative_to="mid_level_model::mid_injected_frame"/>
    </include>
    <joint name="top_level_model__top_injected_frame___to___mid_level_"""\
        """mid_level_model__base___weld_joint" type="fixed">
      <parent>top_level_model::top_injected_frame</parent>
      <child>mid_level_model::base</child>
    </joint>
    <joint name="mid_level_model__mid_injected_frame___to___"""\
        """bottom_level_model__base___weld_joint" type="fixed">
      <parent>mid_level_model::mid_injected_frame</parent>
      <child>bottom_level_model::base</child>
    </joint>
  </model>
</sdf>
"""

    def test_main(self):
        os.environ['ROS_PACKAGE_PATH'] = 'multibody/parsing/'
        self.assertEqual(0, model_directives_to_sdformat.main(
            ['-m',
             'multibody/parsing/test/'
             'model_directives_to_sdformat_files/hidden_frame.yaml',
             '-e', '-c']))

    def test_expand_directives_add_directives(self):
        os.environ['ROS_PACKAGE_PATH'] = 'multibody/parsing/'
        self.assertEqual(0, model_directives_to_sdformat.main(
            ['-m',
             'multibody/parsing/test/'
             'model_directives_to_sdformat_files/add_directives.yaml']))
