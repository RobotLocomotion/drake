# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import io
import lxml.etree as ET
import os
import string
import subprocess
import sys
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

import drake.multibody.parsing.model_directives_to_sdformat \
    as model_directives_to_sdformat

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
        converter = model_directives_to_sdformat.ModelDirectivesToSdf()
        sdformat_tree = converter.convert_directive(file_path)
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

        # Load converted SDFormat
        sdformat_plant = MultibodyPlant(time_step=0.01)
        sdformat_parser = Parser(sdformat_plant)
        sdformat_parser.package_map().PopulateFromFolder(model_dir)
        sdformat_parser.AddModelsFromString(sfdormat_result, "sdf")
        sdformat_plant.Finalize()

        # Compare plants
        # Note: SDFormat will create an extra top level model instance
        self.assertEqual(sdformat_plant.num_model_instances() - 1,
                         directives_plant.num_model_instances())

        # SDFormat will create an extra top level model instance with the
        # name of the file
        file_name = os.path.splitext(os.path.basename(file_path))[0]
        self.assertEqual(sdformat_plant.GetModelInstanceName(
            ModelInstanceIndex(2)), file_name)
        self.assertEqual(
            len(get_bodies(sdformat_plant, [ModelInstanceIndex(2)])), 0)

        for i in range(3, directives_plant.num_model_instances()):
            model_scoped_name = file_name \
                + model_directives_to_sdformat._SCOPE_DELIMITER \
                + directives_plant.GetModelInstanceName(
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
            directives_bodies = get_bodies(
                directives_plant, [ModelInstanceIndex(i)])
            sdformat_bodies = get_bodies(
                sdformat_plant, [
                    ModelInstanceIndex(sdformat_model_index)])
            for sdformat_body, directives_body in zip(
                    sdformat_bodies, directives_bodies):
                self.assertEqual(sdformat_body.name(), directives_body.name())
                sdformat_frames = get_frames_attached_to(
                    sdformat_plant, [sdformat_body])
                directives_frames = get_frames_attached_to(
                    directives_plant, [directives_body])
                # All frames created through model directives would have
                # been created when loading the SDFormat
                self.assertTrue(
                    all(
                        frame.name()
                        in
                        [sdformat_frame.name()
                         for sdformat_frame in sdformat_frames]
                        for frame in directives_frames))

            # Check Model Joints
            directives_joints = get_joints(
                directives_plant, [ModelInstanceIndex(i)])
            sdformat_joints = get_joints(
                sdformat_plant, [
                    ModelInstanceIndex(sdformat_model_index)])
            for i in range(len(directives_joints)):
                for j in range(len(sdformat_joints)):
                    # TODO (marcoag): If they have the same child and parent
                    # body name and the same type, we consider them the same
                    # joint. Anything else we should check?
                    if sdformat_joints[j].parent_body().name() == \
                       directives_joints[i].parent_body().name() \
                       and sdformat_joints[j].child_body().name() == \
                       directives_joints[i].child_body().name() \
                       and sdformat_joints[j].type_name() == \
                       directives_joints[i].type_name():
                        sdformat_joints.pop(j)
                        directives_joints.pop(i)

            # All joints should have been verified
            self.assertEqual(len(sdformat_joints), 0)
            self.assertEqual(len(directives_joints), 0)

    def test_error_no_directives(self):
        converter = model_directives_to_sdformat.ModelDirectivesToSdf()
        with self.assertRaisesRegex(
                model_directives_to_sdformat.ConversionError,
                r'\[directives\] must be the first keyword'
                ' in the yaml file, exiting.'):
            converter.convert_directive(
                    'multibody/parsing/test/'
                    'model_directives_to_sdformat_files/'
                    'something_not_directives.yaml')

    def test_error_directives_not_frist(self):
        converter = model_directives_to_sdformat.ModelDirectivesToSdf()
        with self.assertRaisesRegex(
                model_directives_to_sdformat.ConversionError,
                r'\[directives\] must be the first keyword'
                ' in the yaml file, exiting.'):
            result = converter.convert_directive(
                    'multibody/parsing/test/'
                    'model_directives_to_sdformat_files/'
                    'not_directives_first.yaml')

    def test_error_implicit_hidden_base_frame(self):
        converter = model_directives_to_sdformat.ModelDirectivesToSdf()
        with self.assertRaisesRegex(
                model_directives_to_sdformat.ConversionError,
                'Failed trying to find scope for frame: '
                r'\[frame\] when trying to add frame: '
                r'\[frame_name\].'):
            converter.convert_directive(
                    'multibody/parsing/test/'
                    'model_directives_to_sdformat_files/'
                    'implicit_hidden_base_frame.yaml')

    def test_error_different_scopes_frame(self):
        converter = model_directives_to_sdformat.ModelDirectivesToSdf()
        with self.assertRaisesRegex(
                model_directives_to_sdformat.ConversionError,
                'Frame named: '
                r'\[extra_model::sub_added_frame\] has a '
                'different scope in its name and its '
                r'base_frame: \[simple_model::frame\].'):
            converter.convert_directive(
                    'multibody/parsing/test/'
                    'model_directives_to_sdformat_files/'
                    'different_scopes_frame.yaml')

    def test_error_world_base(self):
        converter = model_directives_to_sdformat.ModelDirectivesToSdf()
        with self.assertRaisesRegex(
                model_directives_to_sdformat.ConversionError,
                r'Adding a frame using base_frame=\[world\] is '
                'not supported.'):
            converter.convert_directive(
                    'multibody/parsing/test/'
                    'model_directives_to_sdformat_files/world_base_frame.yaml')

    def test_error_frame_name_same_base_name(self):
        converter = model_directives_to_sdformat.ModelDirectivesToSdf()
        with self.assertRaisesRegex(
                model_directives_to_sdformat.ConversionError,
                r'Frame: \[frame\] has the same name as '
                'it\'s base frame. This case is not '
                'supported.'):
            converter.convert_directive(
                    'multibody/parsing/test/'
                    'model_directives_to_sdformat_files/'
                    'frame_same_as_base_frame.yaml')

    def test_error_frames_same_name(self):
        converter = model_directives_to_sdformat.ModelDirectivesToSdf()
        with self.assertRaisesRegex(
                model_directives_to_sdformat.ConversionError,
                'Found more than two frames with name: '
                r'\[frame_name\], could not resolve the '
                'scope.'):
            converter.convert_directive(
                    'multibody/parsing/test/'
                    'model_directives_to_sdformat_files/frames_same_name.yaml')

    def test_error_too_many_model_scopes(self):
        converter = model_directives_to_sdformat.ModelDirectivesToSdf()
        with self.assertRaisesRegex(
                model_directives_to_sdformat.ConversionError,
                'Too many nested models in frame: '
                r'\[top_level_model::inner_model::base\]. Only one level of '
                'nesting is allowed.'):
            converter.convert_directive(
                    'multibody/parsing/test/'
                    'model_directives_to_sdformat_files/too_many_models.yaml')

    def test_add_model_instance_add_directives(self):
        converter = model_directives_to_sdformat.ModelDirectivesToSdf()
        expected_xml = '<sdf version="1.9">'\
            '<model name="add_directives"><model name="model_instance">'\
            '<include merge="true">'\
            '<uri>package://model_directives_to_sdformat_files/'\
            'hidden_frame.sdf</uri>'\
            '</include></model></model></sdf>'
        result = converter.convert_directive(
                'multibody/parsing/test/'
                'model_directives_to_sdformat_files/add_directives.yaml',
                True)
        self.assertEqual(expected_xml, ET.tostring(result, encoding="unicode"))

    def test_resulting_xml_and_weld_structures(self):
        converter = model_directives_to_sdformat.ModelDirectivesToSdf()
        result = converter.convert_directive(
                'multibody/parsing/test/'
                'model_directives_to_sdformat_files/inject_frames.yaml',
                True)
        root = result.getroot()
        self.assertEqual(len(root.getchildren()), 1)
        root_model = root.getchildren()[0]

        # Check inject_frames model
        self.assertEqual(root_model.tag, 'model')
        self.assertEqual(root.getchildren()[0].attrib['name'], 'inject_frames')

        # Check fixed joints
        self.assertEqual(len(root_model.findall('joint')), 2)
        joint1 = root_model.findall('joint')[0]
        self.assertEqual(joint1.attrib['type'], 'fixed')
        self.assertEqual(joint1.find('parent').text,
                         'top_level_model::top_injected_frame')
        self.assertEqual(joint1.find('child').text, 'mid_level_model::base')
        joint2 = root_model.findall('joint')[1]
        self.assertEqual(joint2.attrib['type'], 'fixed')
        self.assertEqual(joint2.find('parent').text,
                         'mid_level_model::mid_injected_frame')
        self.assertEqual(joint2.find('child').text, 'bottom_level_model::base')

        # Check top_level_model
        self.assertEqual(len(root_model.findall('model')), 2)
        model1 = root_model.findall('model')[0]
        self.assertEqual(model1.attrib['name'], 'top_level_model')
        self.assertEqual(len(model1.findall('frame')), 1)
        model1_frame = model1.findall('frame')[0]
        self.assertEqual(model1_frame.attrib['name'], 'top_injected_frame')
        self.assertEqual(model1_frame.attrib['attached_to'], 'base')
        self.assertEqual(model1_frame.find('pose').attrib['degrees'], 'true')
        self.assertEqual(len(model1.findall('include')), 1)
        model1_include = model1.findall('include')[0]
        self.assertEqual(model1_include.attrib['merge'], 'true')
        self.assertEqual(model1_include.find('name').text, 'top_level_model')
        self.assertEqual(model1_include.find('uri').text,
                         'package://process_model_directives_test/'
                         'simple_model.sdf')

        # Check mid_level_model
        model2 = root_model.findall('model')[1]
        self.assertEqual(model2.attrib['name'], 'mid_level_model')
        self.assertEqual(model2.attrib['placement_frame'], 'base')
        self.assertEqual(len(model2.findall('frame')), 1)
        model2_frame = model2.findall('frame')[0]
        self.assertEqual(model2_frame.attrib['name'], 'mid_injected_frame')
        self.assertEqual(model2_frame.attrib['attached_to'], 'base')
        self.assertEqual(model2_frame.find('pose').attrib['degrees'], 'true')
        self.assertEqual(len(model2.findall('include')), 1)
        model2_include = model2.findall('include')[0]
        self.assertEqual(model2_include.attrib['merge'], 'true')
        self.assertEqual(model2_include.find('name').text, 'mid_level_model')
        self.assertEqual(model2_include.find('uri').text,
                         'package://process_model_directives_test/'
                         'simple_model.sdf')
        self.assertEqual(len(model2.findall('pose')), 1)
        model2_pose = model2.findall('pose')[0]
        self.assertEqual(model2_pose.attrib['relative_to'],
                         'top_level_model::top_injected_frame')

        # Check bottom_levelmodel
        self.assertEqual(len(root_model.findall('include')), 1)
        include = root_model.findall('include')[0]
        self.assertEqual(include.find('name').text, 'bottom_level_model')
        self.assertEqual(include.find('uri').text,
                         'package://process_model_directives_test/'
                         'simple_model.sdf')
        self.assertEqual(include.find('placement_frame').text, 'base')
        self.assertEqual(include.find('pose').attrib['relative_to'],
                         'mid_level_model::mid_injected_frame')

    def test_main(self):
        self.assertEqual(0, model_directives_to_sdformat.main(
            ['-m',
             'multibody/parsing/test/'
             'model_directives_to_sdformat_files/hidden_frame.yaml',
             '-n', '-c', '-p', '.']))
