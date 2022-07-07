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

from pydrake.common import FindResourceOrThrow

from pydrake.common.test_utilities.meta import (
    ValueParameterizedTest,
    run_with_multiple_values,
)

import drake.multibody.parsing.model_directives_to_sdf \
    as model_directives_to_sdf

import lxml.etree as ET

import os
import subprocess
import unittest
import sys
import io


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
        'multibody/parsing/test/convert_model_directives_test/'
        'inject_frames.yaml',
        'multibody/parsing/test/convert_model_directives_test/'
        'hidden_frame.yaml',
        'multibody/parsing/test/convert_model_directives_test/'
        'frame_attached_to_frame.yaml',
        'multibody/parsing/test/convert_model_directives_test/'
        'weld_frames_from_models.yaml',
        'multibody/parsing/test/convert_model_directives_test/'
        'scoped_frame_name.yaml'
    ]

    @run_with_multiple_values([dict(file_path=file_path)
                               for file_path in files_to_test])
    def test_through_plant_comparation(self, *, file_path):
        # Convert
        converter = model_directives_to_sdf.ModelDirectivesToSdf()
        sdf_tree = converter.convert_directive(file_path)
        sdf_result = ET.tostring(
            sdf_tree, pretty_print=True, encoding="unicode")

        # Load model directives
        directives_plant = MultibodyPlant(time_step=0.01)
        model_dir = os.path.dirname(FindResourceOrThrow(
            'drake/multibody/parsing/test/'
            'convert_model_directives_test/package.xml'))
        parser = Parser(plant=directives_plant)
        parser.package_map().PopulateFromFolder(model_dir)
        directives = LoadModelDirectives(file_path)
        ProcessModelDirectives(directives=directives,
                               plant=directives_plant, parser=parser)

        # Load converted sdf
        sdf_plant = MultibodyPlant(time_step=0.01)
        sdf_parser = Parser(sdf_plant)
        sdf_parser.package_map().PopulateFromFolder(model_dir)
        sdf_parser.AddModelFromString(sdf_result, "sdf")
        sdf_plant.Finalize()

        # Compare plants
        # Note: SDF will create an extra top level model instance
        self.assertEqual(sdf_plant.num_model_instances()-1,
                         directives_plant.num_model_instances())

        # ModelInstanceIndex(0) should be the WorldModelInstance
        self.assertEqual(directives_plant.GetModelInstanceName(
            ModelInstanceIndex(0)), 'WorldModelInstance')
        self.assertEqual(directives_plant.GetModelInstanceName(
            ModelInstanceIndex(0)),
            sdf_plant.GetModelInstanceName(
            ModelInstanceIndex(0)))
        directives_world_model_bodies = get_bodies(
            directives_plant, [ModelInstanceIndex(0)])
        sdf_world_model_bodies = get_bodies(
            sdf_plant, [ModelInstanceIndex(0)])
        self.assertEqual(len(directives_world_model_bodies), 1)
        self.assertEqual(len(sdf_world_model_bodies), 1)
        self.assertEqual(directives_world_model_bodies[0].name(), 'WorldBody')
        self.assertEqual(sdf_world_model_bodies[0].name(), 'WorldBody')
        # Check world Frame
        directives_world_model_frames = get_frames_attached_to(
            directives_plant, directives_world_model_bodies)
        sdf_world_model_frames = get_frames_attached_to(
            sdf_plant, sdf_world_model_bodies)
        self.assertEqual(len(directives_world_model_frames), 1)
        self.assertEqual(len(sdf_world_model_frames), 1)
        self.assertEqual(directives_world_model_frames[0].name(), 'WorldBody')
        self.assertEqual(sdf_world_model_frames[0].name(), 'WorldBody')
        self.assertTrue(directives_world_model_frames[0].is_world_frame())
        self.assertTrue(sdf_world_model_frames[0].is_world_frame())

        # ModelInstanceIndex(1) should be the DefaultModelInstance
        self.assertEqual(directives_plant.GetModelInstanceName(
            ModelInstanceIndex(1)), 'DefaultModelInstance')
        self.assertEqual(directives_plant.GetModelInstanceName(
            ModelInstanceIndex(1)),
            sdf_plant.GetModelInstanceName(
            ModelInstanceIndex(1)))
        directives_default_model_bodies = get_bodies(
            directives_plant, [ModelInstanceIndex(1)])
        sdf_default_model_bodies = get_bodies(
            sdf_plant, [ModelInstanceIndex(1)])
        self.assertEqual(len(directives_default_model_bodies), 0)
        self.assertEqual(len(sdf_default_model_bodies), 0)

        # SDF will create an extra top level model instance with the
        # name of the file
        file_name = os.path.splitext(os.path.basename(file_path))[0]
        self.assertEqual(sdf_plant.GetModelInstanceName(
            ModelInstanceIndex(2)), file_name)
        self.assertEqual(
            len(get_bodies(sdf_plant, [ModelInstanceIndex(2)])), 0)

        for i in range(3, directives_plant.num_model_instances()):
            model_scoped_name = file_name \
                                + model_directives_to_sdf.SCOPE_DELIMITER \
                                + directives_plant.GetModelInstanceName(
                                    ModelInstanceIndex(i))

            sdf_model_instances = get_model_instances_names(sdf_plant)
            model_found = False

            if model_scoped_name in sdf_model_instances:
                model_found = True
                sdf_model_index = sdf_model_instances.index(model_scoped_name)

            self.assertTrue(model_found)

            # Check Model Bodies and corresponding Frames
            directives_bodies = get_bodies(
                directives_plant, [ModelInstanceIndex(i)])
            sdf_bodies = get_bodies(sdf_plant,
                                    [ModelInstanceIndex(sdf_model_index)])
            for sdf_body, directives_body in zip(
                    sdf_bodies, directives_bodies):
                self.assertEqual(sdf_body.name(), directives_body.name())
                sdf_frames = get_frames_attached_to(sdf_plant, [sdf_body])
                directives_frames = get_frames_attached_to(
                    directives_plant, [directives_body])
                # All frames created through model directives would have
                # been created when loading the sdf
                self.assertTrue(
                    all(
                        frame.name()
                        in
                        [sdf_frame.name()
                         for sdf_frame in sdf_frames]
                        for frame in directives_frames))

            # Check Model Joints
            directives_joints = get_joints(
                directives_plant, [ModelInstanceIndex(i)])
            sdf_joints = get_joints(sdf_plant,
                                    [ModelInstanceIndex(sdf_model_index)])
            for i in range(len(directives_joints)):
                for j in range(len(sdf_joints)):
                    # TODO (marcoag): If they have the same child and parent
                    # body name and the same type, we consider them the same
                    # joint. Anything else we should check?
                    if sdf_joints[j].parent_body().name() == \
                       directives_joints[i].parent_body().name() \
                       and sdf_joints[j].child_body().name() == \
                       directives_joints[i].child_body().name() \
                       and sdf_joints[j].type_name() == \
                       directives_joints[i].type_name():
                        sdf_joints.pop(j)
                        directives_joints.pop(i)

            # All joints should have been removed
            self.assertEqual(len(sdf_joints), 0)
            self.assertEqual(len(directives_joints), 0)

    def test_error_no_directives(self):
        s = io.StringIO()
        sys.stdout = s
        converter = model_directives_to_sdf.ModelDirectivesToSdf()
        result = converter.convert_directive(
            'multibody/parsing/test/convert_model_directives_test/'
            'something_not_directives.yaml')
        s.seek(0)
        self.assertEqual(
            s.read(),
            '[directives] must be the first keyword in the yaml file,'
            ' exiting.\n')

    def test_error_directives_not_frist(self):
        s = io.StringIO()
        sys.stdout = s
        converter = model_directives_to_sdf.ModelDirectivesToSdf()
        result = converter.convert_directive(
            'multibody/parsing/test/convert_model_directives_test/'
            'not_directives_first.yaml')
        s.seek(0)
        self.assertEqual(
            s.read(),
            '[directives] must be the first keyword in the yaml file,'
            ' exiting.\n')

    def test_error_implicit_hidden_base_frame(self):
        s = io.StringIO()
        sys.stdout = s
        converter = model_directives_to_sdf.ModelDirectivesToSdf()
        result = converter.convert_directive(
            'multibody/parsing/test/convert_model_directives_test/'
            'implicit_hidden_base_frame.yaml')
        s.seek(0)
        self.assertEqual(
            s.read(),
            'Failed trying to find scope for frame: [frame]. When trying'
            ' to add frame: [frame_name].\nFailed to perform add_frame '
            'directive.\n')

    def test_error_different_scopes_frame(self):
        s = io.StringIO()
        sys.stdout = s
        converter = model_directives_to_sdf.ModelDirectivesToSdf()
        result = converter.convert_directive(
            'multibody/parsing/test/convert_model_directives_test/'
            'different_scopes_frame.yaml')
        s.seek(0)
        self.assertEqual(
            s.read(),
            'Frame named: [extra_model::sub_added_frame] has a different'
            ' scope in its name and its base_frame: [simple_model::frame].\n'
            'Failed to perform add_frame directive.\n')
