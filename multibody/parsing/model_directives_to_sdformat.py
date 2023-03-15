import argparse
import copy
import dataclasses as dc
import lxml.etree as ET
import os
import sys
from typing import List

from pydrake.common import schema
from pydrake.common.yaml import yaml_load_typed
from pydrake.multibody.parsing import (
  ModelDirectives,
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
        pos = full_name.find(_SCOPE_DELIMITER)
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
        if isinstance(dir_obj, cls):
            yield dir_obj


def _find_frame(name: str, directives) -> str:
    """Finds and returns a frame if it exists in all directives,
    if 0 or more than one are found it throws a ConversionError
    exception."""
    add_frame_gen = _filter_directives(directives, AddFrame)
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


def _get_most_distal_parent_frame_directive(frame, directives):
    aux_frame = frame
    while frame is not None:
        aux_frame = frame
        frame = _find_frame(frame.base_frame, directives)

    return aux_frame


def _resolve_and_scope_frame(frame_name: str, directives) -> str:
    """Returns the frame scoped, None if the scope could not be found."""
    frame = _find_frame(frame_name, directives)
    # Frame with the same name as base frame are not supported
    if frame is not None and frame_name == frame.base_frame:
        raise ConversionError(
                f'Frame: [{frame_name}] has the same name as'
                ' it\'s base frame. This case is not supported.')

    # Traverse frame graph to see if this frame is transitively attached
    final_frame = _get_most_distal_parent_frame_directive(frame, directives)

    if final_frame is None:
        return None

    return _join_name(final_frame.scoped_base_frame.instance_name, frame_name)


def _levels_of_nesting(scope: str) -> int:
    return len(scope.split(_SCOPE_DELIMITER))


def _remove_suffix(input: str, suffix: str) -> str:
    if suffix and input.endswith(suffix):
        return input[:-len(suffix)]
    return input


class AddFrame:
    def __init__(self, directive):
        x_pf = directive.X_PF
        self.name = directive.name
        self.base_frame = x_pf.base_frame
        self.scoped_name = ScopedName(self.name)
        self.scoped_base_frame = ScopedName(self.base_frame)

        self.translation_str = '0 0 0'
        trans = x_pf.translation
        self.translation_str = f'{trans[0]} {trans[1]} {trans[2]}'

        # Supporting !Rpy degree rotations for now.
        # See drake/common/schema/transform.h.
        self.rotation_str = '0 0 0'

        rot = x_pf.rotation.value
        if not isinstance(rot, schema.Rotation.Identity):
            if not isinstance(rot, schema.Rotation.Rpy):
                raise ConversionError(
                    f"Rotation of type {type(rot).__name__} not suported.")
            deg = rot.deg
            self.rotation_str = f"{deg[0]} {deg[1]} {deg[2]}"

    def resolve_names(self, directives):
        if self.base_frame == _WORLD_FRAME:
            raise ConversionError(
                f'Adding a frame using base_frame=[{_WORLD_FRAME}] is '
                'not supported.')

        if self.scoped_base_frame.instance_name == "":
            self.resolved_base_frame = self._resolve(self.base_frame,
                                                     directives)
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

    def _resolve(self, name, directives):
        resolved_name = _resolve_and_scope_frame(name, directives)
        if resolved_name is None:
            raise ConversionError(
                f'Unable to find scope for frame: [{name}]'
                f' while adding frame: [{self.name}].')
        return ScopedName(resolved_name)

    def insert_into_root_sdformat_node(self, root, directives):
        model_name = self.resolved_base_frame.instance_name
        model_root = root.find(f'./model[@name="{model_name}"]')
        if model_root is None:
            raise ConversionError(
                f'Failed to find model: [{model_name}]'
                f' while adding frame: [{self.name}].')
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
#
# TODO(marcoag): Try to simplify this logic by flattening the model
# directives. Maybe check implicit model instance scoping rules by
# directly parsing each model directive individually, and checking
# the resultant change against the MbP.
class AddWeld:
    def __init__(self, directive):
        self.parent_name = directive.parent
        self.child_name = directive.child

    def resolve_names(self, directives):
        self.resolved_parent_name = self._resolve(self.parent_name,
                                                  directives)
        self.resolved_child_name = self._resolve(self.child_name,
                                                 directives)

    def _resolve(self, name, directives):
        scoped_name = ScopedName(name)
        # Resolve scope if the frame is implicit
        if scoped_name.instance_name != "":
            return scoped_name

        resolved_name = _resolve_and_scope_frame(name, directives)
        if resolved_name is None:
            raise ConversionError(
                f'Unable to resolve scope for '
                f'frame: [{self.parent_name}]. When tring to add '
                f'weld between: [{self.parent_name}] and: '
                f'[{self.child_name}].')
        return ScopedName(resolved_name)

    def insert_into_root_sdformat_node(self, root, directives):
        tmp_parent_name = self.resolved_parent_name.full_name.replace(
            _SCOPE_DELIMITER, "__")
        tmp_child_name = self.resolved_child_name.full_name.replace(
            _SCOPE_DELIMITER, "__")

        joint_name = f'{tmp_parent_name}__to__{tmp_child_name}__weld_joint'

        joint_elem = ET.SubElement(root, 'joint', name=joint_name)
        joint_elem.set('type', 'fixed')

        parent_elem = ET.SubElement(joint_elem, 'parent')
        parent_elem.text = self.resolved_parent_name.full_name

        child_elem = ET.SubElement(joint_elem, 'child')
        child_elem.text = self.resolved_child_name.full_name


class AddModel:
    def __init__(self, directive):
        self.name = directive.name
        self.file_name = directive.file

    def insert_into_root_sdformat_node(self, root, directives):
        merge_include = False

        for dir_obj in _filter_directives(directives, AddFrame):
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

        for dir_obj in _filter_directives(directives, AddWeld):
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
    def __init__(self, directive):
        self.name = directive.name


class AddDirectives:
    def __init__(self, directive: bool):
        self.model_ns = directive.model_namespace
        self.file_path = directive.file
        self.sdformat_uri = self.file_path.replace('.dmd.yaml', '.sdf')
        if not self.sdformat_uri:
            raise ConversionError(
                'Unable to guess guess SDFormat file name '
                f'for add_directives: {self.params}')

    def insert_into_root_sdformat_node(self, root, directives, args):
        if self.model_ns is not None:
            # Check that the model instance has been created by
            # add_model_instance
            if not any(directive.name == self.model_ns for directive in
                       _filter_directives(directives, AddModelInstance)):
                raise ConversionError(
                    f'Requested model namespace {self.model_ns} '
                    'has not been constructed.')

            model_elem = ET.SubElement(root, 'model', name=self.model_ns)
            include_elem = ET.SubElement(model_elem, 'include', merge='true')
        else:
            include_elem = ET.SubElement(root, 'include', merge='true')

        uri_elem = ET.SubElement(include_elem, 'uri')
        uri_elem.text = self.sdformat_uri

        if args.expand_included:
            self._convert_nested_directive(args)

    def _convert_nested_directive(self, args):
        if self.file_path.startswith("package://"):
            package_map = PackageMap()
            package_map.PopulateFromRosPackagePath()
            suffix = self.file_path[len("package://"):]
            package_name, relative_path = suffix.split("/", maxsplit=1)
            if package_map.Contains(package_name):
                resolved_file_path = os.path.join(
                        package_map.GetPath(package_name), relative_path)
            else:
                raise ConversionError('Failed to find package '
                                      f'[{package_name}] ')
        else:
            raise ConversionError('The provided file path is invalid. It must'
                                  ' be of the form: [package://path_to_file/'
                                  'file.dmd.yaml]')

        # Update args with new target file
        expanded_file_args = copy.deepcopy(args)
        expanded_file_args.model_directives = resolved_file_path
        result_tree = convert_directives(expanded_file_args)
        if result_tree is None:
            raise ConversionError(
                'Failed converting the file included through '
                f' add_directives: {self.params}')

        sdformat_file_path = resolved_file_path.replace('.dmd.yaml', '.sdf')

        if args.output_path:
            expanded_included_path = os.path.join(
                    expanded_file_args.output_path, package_name)
        else:
            expanded_included_path = None

        generate_output(result_tree, relative_path, expanded_included_path,
                        generate_world=False)


def _create_object_from_directive(directive, expand_included: bool):
    if directive.add_model_instance:
        return AddModelInstance(directive.add_model_instance)
    elif directive.add_weld:
        return AddWeld(directive.add_weld)
    elif directive.add_frame:
        return AddFrame(directive.add_frame)
    elif directive.add_model:
        return AddModel(directive.add_model)
    elif directive.add_directives:
        return AddDirectives(directive.add_directives)
    else:
        raise ConversionError(f'Unknown directive')


def convert_directives(args):
    # Read the directives file
    directives_data = yaml_load_typed(schema=ModelDirectives,
                                      filename=args.model_directives)

    all_directives = [
        _create_object_from_directive(directive, args.expand_included)
        for directive in directives_data.directives
    ]

    for dir_obj in _filter_directives(all_directives, (AddFrame, AddWeld)):
        dir_obj.resolve_names(all_directives)

    # Initialize the sdformat XML root
    root = ET.Element('sdf', version=_SDF_VERSION)
    root_name = os.path.basename(_remove_suffix(
        args.model_directives, '.dmd.yaml'))

    root_elem = ET.SubElement(root, 'model', name=root_name)

    for dir_obj in all_directives:
        if isinstance(dir_obj, AddDirectives):
            dir_obj.insert_into_root_sdformat_node(
                    root_elem, all_directives, args)
        elif not isinstance(dir_obj, AddModelInstance):
            dir_obj.insert_into_root_sdformat_node(
                    root_elem, all_directives)

    # Check model validity by loading it through the SDFormat parser
    if args.check_sdf:
        try:
            directives_plant = MultibodyPlant(time_step=0.01)
            parser = Parser(plant=directives_plant)
            parser.package_map().PopulateFromRosPackagePath()
            parser.AddModelsFromString(ET.tostring(root), "sdf")
        except RuntimeError as e:
            raise ConversionError(
                'Failed to validate resulting SDFormat XML.') from e

    return ET.ElementTree(root)


def generate_output(result_tree: ET.ElementTree,
                    input_path: str = '',
                    output_path: str = '',
                    generate_world: bool = True):

    if output_path:
        filename = os.path.basename(input_path)
        output_filename = os.path.join(
                output_path, _remove_suffix(filename, '.dmd.yaml') + '.sdf')

        # Create path if it does not exist
        if not os.path.exists(output_path):
            os.makedirs(output_path)

        if generate_world:
            # Saving a world with a merge include to the model
            # TODO(marcoag): Add a validty check that ensures the model
            # is world-mergeable.
            world_output_path = output_path + \
                    _remove_suffix(filename, '.dmd.yaml') + '_world.sdf'
            world_root = ET.Element('sdf', version=_SDF_VERSION)
            root_world_name = result_tree.find('model').get('name')
            root_world_elem = ET.SubElement(world_root, 'world',
                                            name=root_world_name)
            root_world_elem.append(
                    ET.Comment('Provides a direct inclusion of '
                               'a given model, with no nesting'))
            include_elem = ET.SubElement(root_world_elem, 'include',
                                         merge='true')
            uri_elem = ET.SubElement(include_elem, 'uri')
            uri_elem.text = 'package://' + output_filename
            world_tree = ET.ElementTree(world_root)
            world_tree.write(world_output_path,
                             pretty_print=True)

        # Saving the converted model
        result_tree.write(output_filename,
                          pretty_print=True)
    else:
        print(ET.tostring(result_tree,
                          pretty_print=True,
                          encoding="unicode"))


def _create_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '-c', '--check-sdf', action='store_true',
        help='Check and validate resulting SDFormat.')
    parser.add_argument(
        '-e', '--expand-included', action='store_true',
        help='Convert files included through [add_directives].')
    parser.add_argument(
        '-m', '--model-directives', type=str, required=True,
        help='Path to model directives file to be converted.')
    parser.add_argument(
        '-o', '--output-path', type=str,
        help='Output path of directory where SDFormat files will be written.')
    return parser


def main(argv=None) -> None:
    parser = _create_parser()
    args = parser.parse_args(argv)

    result_tree = convert_directives(args)

    if result_tree is None:
        raise ConversionError('Failed to convert model directives.')

    generate_output(result_tree, args.model_directives, args.output_path)

    return 0


if __name__ == '__main__':
    sys.exit(main())
