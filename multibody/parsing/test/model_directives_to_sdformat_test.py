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
        sdformat_parser.AddModelFromString(sfdormat_result, "sdf")
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
                + model_directives_to_sdformat.SCOPE_DELIMITER \
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
