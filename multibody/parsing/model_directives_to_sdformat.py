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

import argparse
import lxml.etree as ET
import os
import sys
from typing import Any, Dict

from pydrake.common.yaml import yaml_load_file
from pydrake.multibody.parsing import (
  PackageMap,
  Parser,
)
from pydrake.multibody.plant import MultibodyPlant

_SDF_VERSION = '1.9'
_SCOPE_DELIMITER = '::'
_WORLD_FRAME = 'world'


class ConversionError(Exception):
    pass


class ScopedName:

    def __init__(self, full_name: str):
        self.full_name = full_name
        pos = full_name.rfind(_SCOPE_DELIMITER)
        if pos == -1:
            self.name = full_name
            self.instance_name = ""
        else:
            self.instance_name = full_name[:pos]
            self.name = full_name[pos + len(_SCOPE_DELIMITER):]


def _join_name(instance_name: str, name: str):
    if instance_name != "":
        return instance_name + _SCOPE_DELIMITER + name
    else:
        return name


def _remove_root_scope(full_name: str):
    pos = full_name.find(_SCOPE_DELIMITER)
    if pos == -1:
        return full_name
    return full_name[pos + len(_SCOPE_DELIMITER):]


def _filter_directives(directives, cls):
    for dir_obj in directives:
        try:
            if any(isinstance(dir_obj, c) for c in iter(cls)):
                yield dir_obj
        except TypeError:
            if isinstance(dir_obj, cls):
                yield dir_obj


def _find_frame(name: str, other_directives) -> str:
    """Finds and returns a frame if it exists in all directives,
    if 0 or more than one are found it throws a ConversionError
    exception."""
    add_frame_gen = _filter_directives(other_directives, AddFrame)
    matching_frames = [
        dir_obj for dir_obj in add_frame_gen
        if name == dir_obj.scoped_name.name
    ]

    if len(matching_frames) > 1:
        raise ConversionError(
            'Found more than two frames with'
            f' name: [{name}], could not resolve the scope.')

    if matching_frames:
        return matching_frames[0]

    return None


def _resolve_and_scope_frame(frame_name: str, other_directives) -> str:
    """Returns the frame scoped, None if the scope could not be found."""
    frame = _find_frame(frame_name, other_directives)
    # Frame with the same name as base frame are not supported
    if frame is not None and frame_name == frame.base_frame:
        raise ConversionError(
                f'Frame: [{frame_name}] has the same name as'
                ' it\'s base frame. This case is not supported.')

    # Check if it's atached to other frame
    final_frame = frame
    while frame is not None:
        final_frame = frame
        frame = _find_frame(frame.base_frame, other_directives)

    if final_frame is None:
        return None

    return _join_name(final_frame.scoped_base_frame.instance_name, frame_name)


class AddFrame:
    def __init__(self, params):
        self.params = params
        x_pf = params['X_PF']
        self.name = params['name']
        self.base_frame = x_pf['base_frame']
        self.scoped_name = ScopedName(self.name)
        self.scoped_base_frame = ScopedName(self.base_frame)

        self.translation_str = '0 0 0'
        if 'translation' in x_pf:
            trans = x_pf['translation']
            self.translation_str = f'{trans[0]} {trans[1]} {trans[2]}'

        # Supporting !Rpy degree rotations for now.
        # See drake/common/schema/transform.h.
        self.rotation_str = '0 0 0'

        if 'rotation' in x_pf:
            rot = x_pf['rotation']
            if rot['_tag'] != '!Rpy':
                raise ConversionError(
                    f"Rotation of type {rot['_tag']} not suported.")
            deg = rot['deg']
            self.rotation_str = f"{deg[0]} {deg[1]} {deg[2]}"

    def resolve_names(self, other_directives):
        if self.base_frame == _WORLD_FRAME:
            raise ConversionError(
                f'Adding a frame using base_frame=[{_WORLD_FRAME}] is '
                'not supported.')

        if self.scoped_base_frame.instance_name == "":
            self.resolved_base_frame = self._resolve(self.base_frame,
                                                     other_directives)
        else:
            self.resolved_base_frame = self.scoped_base_frame

        # If name and base_frame are scoped both should have a common scope
        if (self.resolved_base_frame.instance_name != ""
                and self.scoped_name.instance_name != ""):
            if (self.scoped_name.instance_name
                    != self.resolved_base_frame.instance_name):
                raise ConversionError(
                    f'Frame named: [{self.name}] has a different '
                    'scope in its name and its base_frame: '
                    f'[{self.base_frame}].')

        # Construct the model scope if necessary.
        # Only one level of nesting is allowed for models.
        if len(self.base_frame.split(_SCOPE_DELIMITER)) > 2:
            raise ConversionError(
                f'Too many nested models in frame: [{self.base_frame}]. '
                'Only one level of nesting is allowed.')

    def _resolve(self, name, other_directives):
        resolved_name = _resolve_and_scope_frame(name, other_directives)
        if resolved_name is None:
            raise ConversionError(
                'Failed trying to find scope for frame: '
                f'[{name}] when trying to add frame: '
                f'[{self.name}].')
        return ScopedName(resolved_name)

    def to_sdf(self, root, other_directives):
        model_name = self.resolved_base_frame.instance_name
        model_root = root.find(f'./model[@name="{model_name}"]')
        if model_root is None:
            raise ConversionError(
                f'Failed to find model: [{model_name}] when trying to '
                f'add frame: [{self.name}].')
        frame_elem = ET.SubElement(model_root,
                                   'frame',
                                   name=self.scoped_name.name,
                                   attached_to=self.resolved_base_frame.name)
        pose_elem = ET.SubElement(
            frame_elem, 'pose', degrees='true')
        pose_elem.text = f'{self.translation_str}   {self.rotation_str}'


# For add_weld, we only support child frames that can be referred to
# within the same sdformat file that are nested only once, as we would
# require the placement_frame and pose combination for posturing.
#
# Supported example:
# add_model:
#   name: simple_model
#   file: ...
# add_weld:
#   parent: ...
#   child: simple_model::frame
#
# Unsupported example:
# add_model:
#   name: simple_model
#   file: ...
# add_weld:
#   parent: ...
#   child: simple_model::nested_model::frame
#
# Unsupported example:
# add_directives:
#   file: directive_that_contains_simple_model.yaml
# add_weld:
#   parent: ...
#   child: simple_model::frame
#
# Whenever the top model instance in the child of the weld cannot be
# referred to obviously, this conversion will fail. In the unsupported
# example above, it is unclear where simple_model is located, hence the
# placement_frame and pose combination cannot be applied for posturing.
class AddWeld:
    def __init__(self, params):
        self.params = params
        self.parent_name = params['parent']
        self.child_name = params['child']

    def resolve_names(self, other_directives):
        self.resolved_parent_name = self._resolve(self.parent_name,
                                                  other_directives)
        self.resolved_child_name = self._resolve(self.child_name,
                                                 other_directives)

    def _resolve(self, name, other_directives):
        scoped_name = ScopedName(name)
        # Resolve scope if the frame is implicit
        if scoped_name.instance_name != "":
            return scoped_name

        resolved_name = _resolve_and_scope_frame(name, other_directives)
        if resolved_name is None:
            raise ConversionError(
                f'Failed trying to find scope for '
                f'frame: [{self.params["parent"]}]. When tring to add '
                f'weld between: [{self.params["parent"]}] and: '
                f'[{self.params["child"]}].')
        return ScopedName(resolved_name)

    def to_sdf(self, root, other_directives):
        tmp_parent_name = self.resolved_parent_name.full_name.replace(
            _SCOPE_DELIMITER, "__")
        tmp_child_name = self.resolved_child_name.full_name.replace(
            _SCOPE_DELIMITER, "__")

        joint_name = f'{tmp_parent_name}___to___{tmp_child_name}___weld_joint'

        joint_elem = ET.SubElement(root, 'joint', name=joint_name)
        joint_elem.set('type', 'fixed')

        parent_elem = ET.SubElement(joint_elem, 'parent')
        parent_elem.text = self.resolved_parent_name.full_name

        child_elem = ET.SubElement(joint_elem, 'child')
        child_elem.text = self.resolved_child_name.full_name


class AddModel:
    def __init__(self, params):
        self.params = params
        self.name = params['name']
        self.file_name = params['file']

    def to_sdf(self, root, other_directives):
        merge_include = False

        for dir_obj in _filter_directives(other_directives, AddFrame):
            if dir_obj.resolved_base_frame.instance_name == self.name:
                merge_include = True

        if merge_include:
            model_root = ET.SubElement(root, 'model', name=self.name)
        else:
            model_root = root

        include_elem = ET.SubElement(model_root, 'include')
        if merge_include:
            include_elem.set("merge", "true")

        name_elem = ET.SubElement(include_elem, 'name')
        name_elem.text = self.name
        uri_elem = ET.SubElement(include_elem, 'uri')
        uri_elem.text = self.file_name

        for dir_obj in _filter_directives(other_directives, AddWeld):
            if dir_obj.resolved_child_name.instance_name == self.name:
                placement_frame_value = _remove_root_scope(
                    dir_obj.resolved_child_name.full_name)
                relative_to = dir_obj.resolved_parent_name.full_name

                if merge_include:
                    model_root.set('placement_frame', placement_frame_value)
                    model_root.insert(
                        0, ET.Element('pose', relative_to=relative_to))
                else:
                    placement_frame_elem = ET.SubElement(
                        include_elem, 'placement_frame')
                    placement_frame_elem.text = placement_frame_value
                    ET.SubElement(include_elem,
                                  'pose',
                                  relative_to=relative_to)


class AddModelInstance:
    def __init__(self, params: Dict):
        self.params = params
        self.name = params['name']


class AddDirectives:

    def __init__(self, params: Dict, nofollow: bool, packages_path: str):
        self.params = params
        self.model_ns = params.get('model_namespace')
        self.file_path = params['file']
        self.sdformat_uri = self.file_path.replace('.yaml', '.sdf')
        if not self.sdformat_uri:
            raise ConversionError(
                'Error trying to guess SDFormat file name '
                f'for add_directives: {self.params}')
        self.nofollow = nofollow
        self.packages_path = packages_path

    def to_sdf(self, root, other_directives):
        if self.model_ns is not None:
            # Check that the model instance has been created by
            # add_model_instance
            if not any(directive.name == self.model_ns for directive in
                       _filter_directives(other_directives, AddModelInstance)):
                raise ConversionError(
                    f'Requested model namespace {self.model_ns} '
                    'has not been constructed.')

            model_elem = ET.SubElement(root, 'model', name=self.model_ns)
            include_elem = ET.SubElement(model_elem, 'include', merge='true')
        else:
            include_elem = ET.SubElement(root, 'include', merge='true')

        uri_elem = ET.SubElement(include_elem, 'uri')
        uri_elem.text = self.sdformat_uri

        if not self.nofollow:
            self._convert_nested_directive()

    def _convert_nested_directive(self):
        if self.file_path.startswith("package://"):
            package_map = PackageMap()
            if self.packages_path:
                package_map.PopulateFromFolder(self.packages_path)
            else:
                package_map.PopulateFromFolder('.')
            suffix = self.file_path[len("package://"):]
            package, relative_path = suffix.split("/", maxsplit=1)
            if package_map.Contains(package):
                resolved_file_path = os.path.join(package_map.GetPath(package),
                                                  relative_path)
            else:
                raise ConversionError(f'Failed to find package [{package}] '
                                      'in the provided path:')

        result_tree = convert_directive(resolved_file_path, self.nofollow,
                                        self.packages_path)
        if result_tree is None:
            raise ConversionError(
                'Failed converting the file included through '
                f' add_directives: {self.params}')

        sdformat_file_path = resolved_file_path.replace('.yaml', '.sdf')

        result_tree.write(sdformat_file_path,
                          pretty_print=True,
                          encoding="unicode")


def _create_directive_from_yaml(directive: Dict, nofollow: bool,
                                packages_path: str):
    cmd, params = list(directive.items())[0]
    if 'add_model_instance' == cmd:
        return AddModelInstance(params)
    elif 'add_weld' == cmd:
        return AddWeld(params)
    elif 'add_frame' == cmd:
        return AddFrame(params)
    elif 'add_model' == cmd:
        return AddModel(params)
    elif 'add_directives' == cmd:
        return AddDirectives(params, nofollow, packages_path)
    else:
        raise ConversionError(f'Unknown directive {cmd}')


def convert_directive(input_path: str,
                      nofollow: bool = False,
                      packages_path: str = None,
                      check_sdf: bool = False) -> str:
    # Read the directives file
    yaml_dict = yaml_load_file(input_path)

    if list(yaml_dict.keys())[0] != 'directives':
        raise ConversionError('[directives] must be the first keyword in '
                              'the yaml file, exiting.')

    all_directives = [
        _create_directive_from_yaml(directive, nofollow, packages_path)
        for directive in yaml_dict['directives']
    ]

    for dir_obj in _filter_directives(all_directives, [AddFrame, AddWeld]):
        dir_obj.resolve_names(all_directives)

    # Initialize the sdformat XML root
    root = ET.Element('sdf', version=_SDF_VERSION)
    root_world_name = os.path.splitext(os.path.basename(input_path))[0]
    root_world_elem = ET.SubElement(root, 'world', name=root_world_name)

    # Note, we need to process the directives in the correct order
    for cls in [AddDirectives, AddModel, AddFrame, AddWeld]:
        for dir_obj in _filter_directives(all_directives, cls):
            dir_obj.to_sdf(root_world_elem, all_directives)

    # Check model validity by loading it through the SDFormat parser
    if check_sdf:
        try:
            directives_plant = MultibodyPlant(time_step=0.01)
            parser = Parser(plant=directives_plant)
            if packages_path:
                parser.package_map().PopulateFromFolder(packages_path)
            else:
                parser.package_map().PopulateFromFolder('.')
            parser.AddModelsFromString(ET.tostring(root), "sdf")
        except RuntimeError as e:
            raise ConversionError(
                'Failed to validate resulting SDFormat XML.') from e

    return ET.ElementTree(root)


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '-c', '--check-sdf', action='store_true',
        help='Chack and validate resulting SDFormat.')
    parser.add_argument(
        '-m', '--model-directives', type=str, required=True,
        help='Path to model directives file to be converted.')
    parser.add_argument(
        '-n', '--nofollow', action='store_true',
        help='Do not convert files included through [add_directives].')
    parser.add_argument(
        '-o', '--output', type=str,
        help='Output path for converted SDFormat file.')
    parser.add_argument(
        '-p', '--packages-path', type=str, help='Path used to find packages.')
    args = parser.parse_args(argv)

    result_tree = convert_directive(
        args.model_directives, args.nofollow, args.packages_path,
        args.check_sdf)

    if result_tree is not None:
        if args.output:
            result_tree.write(args.output,
                              pretty_print=True)
        else:
            print(ET.tostring(result_tree,
                              pretty_print=True,
                              encoding="unicode"))
    else:
        raise ConversionError('Failed to convert model directives.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
