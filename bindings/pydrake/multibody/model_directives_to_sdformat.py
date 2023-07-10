r"""
Utility for converting Drake Model Directives files (*.dmd.yaml) to SDFormat.

Preconditions on the model directives file(s):

  - No deep nested welds are present (see issue #19090).
  - It is assumed that the model directives files are well formed.
  - References to elements in subsequently included files are correct (the
    tool will not verify them).
  - It does not contain the ``default_joint_positions`` or
    the ``default_free_body_pose`` directive since they are still not
    supported.

Properties of the resulting SDFormat file(s):

  - The SDFormat files are generated alongside the original "dmd.yaml" files,
    with the only difference of having an ``.sdf`` extension (unless the
    option --output-path is provided).
  - An SDFormat file is created containing the conversion of the model
    directives with the addition of a top level model using the name of
    the file itself without the extension.
  - An SDFormat file is created with the same file name but ending in
    ``_world.sdf``. This file contains a top level ``<world>`` element that
    merge includes the previous ``.sdf`` file.

**Running**:

    From a Drake source build, run this as::

            bazel run //tools:model_directives_to_sdformat -- --help

    From a Drake binary release (including pip releases), run this as::

            python3 -m pydrake.multibody.model_directives_to_sdformat \
                    -m path/to/file.dmd.yaml

    For binary releases (except for pip) there is also a shortcut available
    as::
            /opt/drake/bin/model_directives_to_sdformat -m \
                    path/to/file.dmd.yaml

**Expand Included**:

    ``dmd.yaml`` included files will only be converted if the
    ``--expand-included`` or ``-e`` flag is selected:

            bazel run //tools:model_directives_to_sdformat -- \
                    -m path/to/file.dmd.yaml --expand-included

    SDFormat resulting files from these included model directives will be
    created alongside the original files.

**Output Path**:

    By default converted SDFormat files are created alongside original files
    with the only difference of having the ``.sdf`` extension instead of the
    ``dmd.yaml`` one. In order to generate them in a specific location the
    `--output-path`` or ``-o`` option can be used, followed by the desired
    output path:

            bazel run //tools:model_directives_to_sdformat -- \
                    -m path/to/file.dmd.yaml --output-path path/to/output_dir/
"""

import argparse
import os
from pathlib import Path
import sys
from xml.dom import minidom
import xml.etree.ElementTree as ET

from pydrake.common import schema
from pydrake.common.yaml import yaml_load_typed
from pydrake.multibody.parsing import (
    ModelDirectives,
    PackageMap,
)

# TODO(jwnimmer-tri) Once we drop support for Python 3.8 (and thus can rely
# on PEP-585), switch to using `dict` throughout instead of `Dict`.
if sys.version_info[:2] >= (3, 9):
    Dict = dict
else:
    from typing import Dict


_SDF_VERSION = "1.9"
_SCOPE_DELIMITER = "::"
_WORLD_FRAME = "world"


class _ConversionError(Exception):
    pass


# TODO(marcoag): Replace this with the newer pydrake.multibody.tree.ScopedName
class _ScopedName:
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
    if 0 or more than one are found it throws a _ConversionError
    exception."""
    add_frame_gen = _filter_directives(directives, _AddFrame)
    matching_frames = [
        dir_obj
        for dir_obj in add_frame_gen
        if name == dir_obj.scoped_name.name
    ]

    if len(matching_frames) > 1:
        raise _ConversionError(
            "Found more than two frames with"
            f" name: [{name}], could not resolve the scope."
        )

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
    """Returns the frame scoped and if it is deeply nested,"""
    """None is returned if the scope could not be found."""
    deeply_nested = False
    frame = _find_frame(frame_name, directives)
    # Frame with the same name as base frame are not supported.
    if frame is not None and frame_name == frame.base_frame:
        raise _ConversionError(
            f"Frame: [{frame_name}] has the same name as"
            " it's base frame. This case is not supported."
        )

    # Traverse frame graph to see if this frame is transitively attached.
    final_frame = _get_most_distal_parent_frame_directive(frame, directives)

    if final_frame is None:
        return None, deeply_nested

    if _SCOPE_DELIMITER in final_frame.scoped_base_frame.name:
        deeply_nested = True

    return (
        _join_name(final_frame.scoped_base_frame.instance_name, frame_name),
        deeply_nested,
    )


def _remove_suffix(input: str, suffix: str) -> str:
    if suffix and input.endswith(suffix):
        return input[: -len(suffix)]
    return input


class _AddFrame:
    def __init__(self, directive):
        x_pf = directive.X_PF
        self.name = directive.name
        self.base_frame = x_pf.base_frame
        self.scoped_name = _ScopedName(self.name)
        self.scoped_base_frame = _ScopedName(self.base_frame)

        self.translation_str = "0 0 0"
        trans = x_pf.translation
        self.translation_str = f"{trans[0]} {trans[1]} {trans[2]}"

        # Supporting !Rpy degree rotations for now.
        # See drake/common/schema/transform.h.
        self.rotation_str = "0 0 0"

        rot = x_pf.rotation.value
        if not isinstance(rot, schema.Rotation.Identity):
            if not isinstance(rot, schema.Rotation.Rpy):
                raise _ConversionError(
                    f"Rotation of type {type(rot).__name__} not suported."
                )
            deg = rot.deg
            self.rotation_str = f"{deg[0]} {deg[1]} {deg[2]}"

    def resolve_names(self, directives):
        if self.base_frame == _WORLD_FRAME:
            raise _ConversionError(
                f"Adding a frame using base_frame=[{_WORLD_FRAME}]"
                " is not supported."
            )

        if self.scoped_base_frame.instance_name == "":
            self.resolved_base_frame = self._resolve(
                self.base_frame, directives
            )
        else:
            self.resolved_base_frame = self.scoped_base_frame

        # If name and base_frame are scoped both should have a common scope.
        if (
            self.resolved_base_frame.instance_name != ""
            and self.scoped_name.instance_name != ""
        ):
            if (
                self.scoped_name.instance_name
                != self.resolved_base_frame.instance_name
            ):
                raise _ConversionError(
                    f"Frame named: [{self.name}] has a different "
                    "scope in its name and its base_frame: "
                    f"[{self.base_frame}]."
                )

    def _resolve(self, name, directives):
        resolved_name, _ = _resolve_and_scope_frame(
            name, directives
        )
        if resolved_name is None:
            raise _ConversionError(
                f"Unable to find scope for frame: [{name}]"
                f" while adding frame: [{self.name}]."
            )
        return _ScopedName(resolved_name)

    def insert_into_root_sdformat_node(self, root, directives):
        model_name = self.resolved_base_frame.instance_name
        model_root = root.find(f'./model[@name="{model_name}"]')
        if model_root is None:
            raise _ConversionError(
                f"Failed to find model: [{model_name}]"
                f" while adding frame: [{self.name}]."
            )
        frame_elem = ET.SubElement(
            model_root,
            "frame",
            name=self.scoped_name.name,
            attached_to=self.resolved_base_frame.name,
        )
        pose_elem = ET.SubElement(frame_elem, "pose", degrees="true")
        pose_elem.text = f"{self.translation_str}   {self.rotation_str}"


# For add_weld, we only support child frames that can be referred to within
# the same sdformat file that are nested only once, as we would require the
# placement_frame and pose combination for posturing.
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
class _AddWeld:
    def __init__(self, directive):
        self.parent_name = directive.parent
        self.child_name = directive.child

    def resolve_names(self, directives):
        self.resolved_parent_name, deeply_nested = self._resolve(
            self.parent_name, directives
        )
        self.resolved_child_name, deeply_nested = self._resolve(
            self.child_name, directives
        )
        if deeply_nested:
            raise _ConversionError(
                "Found weld with deeply nested child frame"
                f" [{self.resolved_child_name.full_name}]. "
                "Welds with deeply nested childs are not "
                "suported."
            )

        if (
            self.resolved_parent_name.full_name
            == self.resolved_child_name.full_name
        ):
            raise _ConversionError(
                "Weld must specify different name for "
                "parent and child, while "
                f"[{self.resolved_child_name.full_name}] "
                "was specified for both."
            )

    def _resolve(self, name, directives):
        scoped_name = _ScopedName(name)
        deeply_nested = False
        # Resolve scope if the frame is implicit
        if scoped_name.instance_name != "":
            if _SCOPE_DELIMITER in scoped_name.name:
                deeply_nested = True
            return scoped_name, deeply_nested

        resolved_name, deeply_nested = _resolve_and_scope_frame(
            name, directives
        )
        if resolved_name is None:
            raise _ConversionError(
                "Unable to resolve scope for "
                f"frame: [{self.parent_name}]. When tring to add "
                f"weld between: [{self.parent_name}] and: "
                f"[{self.child_name}]."
            )
        return _ScopedName(resolved_name), deeply_nested

    def insert_into_root_sdformat_node(self, root, directives):
        tmp_parent_name = self.resolved_parent_name.full_name.replace(
            _SCOPE_DELIMITER, "__"
        )
        tmp_child_name = self.resolved_child_name.full_name.replace(
            _SCOPE_DELIMITER, "__"
        )

        joint_name = f"{tmp_parent_name}__to__{tmp_child_name}__weld_joint"

        joint_elem = ET.SubElement(root, "joint", name=joint_name)
        joint_elem.set("type", "fixed")

        parent_elem = ET.SubElement(joint_elem, "parent")
        parent_elem.text = self.resolved_parent_name.full_name

        child_elem = ET.SubElement(joint_elem, "child")
        child_elem.text = self.resolved_child_name.full_name


class _AddModel:
    def __init__(self, directive):
        self.name = directive.name
        self.file_name = directive.file
        if directive.default_free_body_pose:
            raise _ConversionError(
                f"default_free_body_pose is not supported yet."
            )
        if directive.default_joint_positions:
            raise _ConversionError(
                f"default_joint_positions is not supported yet."
            )

    def insert_into_root_sdformat_node(self, root, directives):
        merge_include = False

        for dir_obj in _filter_directives(directives, _AddFrame):
            if dir_obj.resolved_base_frame.instance_name == self.name:
                merge_include = True

        if merge_include:
            model_root = ET.SubElement(root, "model", name=self.name)
        else:
            model_root = root

        include_elem = ET.SubElement(model_root, "include")
        if merge_include:
            include_elem.set("merge", "true")

        name_elem = ET.SubElement(include_elem, "name")
        name_elem.text = self.name
        uri_elem = ET.SubElement(include_elem, "uri")
        uri_elem.text = self.file_name

        for dir_obj in _filter_directives(directives, _AddWeld):
            if dir_obj.resolved_child_name.instance_name == self.name:
                placement_frame_value = _remove_root_scope(
                    dir_obj.resolved_child_name.full_name
                )
                relative_to = dir_obj.resolved_parent_name.full_name

                if merge_include:
                    model_root.set("placement_frame", placement_frame_value)
                    model_root.insert(
                        0, ET.Element("pose", relative_to=relative_to)
                    )
                else:
                    placement_frame_elem = ET.SubElement(
                        include_elem, "placement_frame"
                    )
                    placement_frame_elem.text = placement_frame_value
                    ET.SubElement(
                        include_elem, "pose", relative_to=relative_to
                    )


class _AddModelInstance:
    def __init__(self, directive):
        self.name = directive.name


class _AddDirectives:
    def __init__(self, directive: bool):
        self.model_ns = directive.model_namespace
        self.file_path = directive.file
        self.sdformat_uri = self.file_path.replace(".dmd.yaml", ".sdf")
        if not self.sdformat_uri:
            raise _ConversionError(
                "Unable to guess guess SDFormat file name "
                f"for add_directives: {self.params}"
            )

    def insert_into_root_sdformat_node(self, root, directives):
        if self.model_ns is not None:
            # Check that the model instance has been created by
            # add_model_instance.
            if not any(
                directive.name == self.model_ns
                for directive in _filter_directives(
                    directives, _AddModelInstance
                )
            ):
                raise _ConversionError(
                    f"Requested model namespace {self.model_ns} "
                    "has not been constructed."
                )

            model_elem = ET.SubElement(root, "model", name=self.model_ns)
            include_elem = ET.SubElement(model_elem, "include", merge="true")
        else:
            include_elem = ET.SubElement(root, "include", merge="true")

        uri_elem = ET.SubElement(include_elem, "uri")
        uri_elem.text = self.sdformat_uri


def _create_object_from_directive(directive):
    if directive.add_model_instance:
        return _AddModelInstance(directive.add_model_instance)
    elif directive.add_weld:
        return _AddWeld(directive.add_weld)
    elif directive.add_frame:
        return _AddFrame(directive.add_frame)
    elif directive.add_model:
        return _AddModel(directive.add_model)
    elif directive.add_directives:
        return _AddDirectives(directive.add_directives)
    else:
        raise _ConversionError(f"Unknown directive")


def _convert_one_dmd(*, dmd_filename: Path):
    """Given a foo.dmd.yaml filename, returns the semantically equivalent
    SDFormat content as an XML string, plus the list of included dmd.yaml
    filenames.
    """
    # Check that the file is a .dmd.yaml file.
    if not dmd_filename.name.endswith('.dmd.yaml'):
        raise _ConversionError(
            "Unable to determine file format. Make sure the provided file has "
            "the drake model directives '.dmd.yaml' extension"
        )
    root_name = _remove_suffix(dmd_filename.name, ".dmd.yaml")

    # Read the directives file.
    dmd_data = yaml_load_typed(schema=ModelDirectives, filename=dmd_filename)
    commands = [
        _create_object_from_directive(directive)
        for directive in dmd_data.directives
    ]

    # Resolve the names in `add_frame` and `add_weld` commands.
    for command in _filter_directives(commands, (_AddFrame, _AddWeld)):
        command.resolve_names(commands)

    # Populate the XML document.
    sdf_root = ET.Element("sdf", version=_SDF_VERSION)
    model_root = ET.SubElement(sdf_root, "model", name=root_name)
    for command in commands:
        if isinstance(command, _AddModelInstance):
            # TODO(jwnimmer-tri) It doesn't seem like this should be a no-op?
            continue
        command.insert_into_root_sdformat_node(model_root, commands)

    # Gather the list of includes.
    includes = []
    for command in commands:
        if isinstance(command, _AddDirectives):
            includes.append(command.file_path)

    # Return the document as a pretty string.
    xml = minidom.parseString(ET.tostring(sdf_root)).toprettyxml(indent="  ")
    return (xml, includes)


def _convert_directives(*,
                        dmd_filename: Path,
                        output_path: Path = None,
                        expand_included: bool = False,
                        generate_world: bool = True) -> dict[Path, str]:
    """Given a foo.dmd.yaml filename, returns the semantically equivalent
    SDFormat content(s). The return value is dictionary of output filenames
    and their desired contents.

    If an output_path is given, the filenames will be based on that path;
    otherwise the filenames will be siblings to the dmd_filename.
    """
    result = dict()

    # Convert this file.
    sdf_str, included_dmd_uris = _convert_one_dmd(dmd_filename=dmd_filename)
    sdf_filename = Path(str(dmd_filename).replace(".dmd.yaml", ".sdf"))
    if output_path is not None:
        sdf_filename = output_path / sdf_filename.name
    result[sdf_filename] = sdf_str

    # Add a world file, if requested.
    if generate_world:
        world_filename = Path(str(sdf_filename).replace(".sdf", "_world.sdf"))
        world_str = _generate_world(sdf_filename)
        result[world_filename] = world_str

    # Recurse, if requested.
    if expand_included:
        worklist = included_dmd_uris
        finished = set()
        while len(worklist) > 0:
            # Grab the first uri and resolve it to a path.
            uri = worklist.pop(0)
            if not uri.startswith("package://"):
                raise _ConversionError(
                    "The provided file path is invalid. It must be of the "
                    "form: [package://path_to_file/file.dmd.yaml]"
                )
            suffix = uri[len("package://"):]
            package_name, relative_path = suffix.split("/", maxsplit=1)
            package_map = PackageMap()
            package_map.PopulateFromRosPackagePath()
            if not package_map.Contains(package_name):
                raise _ConversionError(
                    f"Failed to find package [{package_name}]"
                )
            package_path = Path(package_map.GetPath(package_name))
            sub_filename = package_path / relative_path

            # Convert dmd -> sdf.
            sub_str, sub_uris = _convert_one_dmd(dmd_filename=sub_filename)
            sub_filename = Path(str(sub_filename).replace(".dmd.yaml", ".sdf"))
            if output_path is not None:
                sub_filename = output_path / sub_filename.name
            result[sub_filename] = sub_str

            # Recursively expand.
            finished.add(uri)
            for sub_uri in sub_uris:
                if sub_uri not in finished:
                    worklist.append(sub_uri)

    return result


# Returns the world file XML string with a merge include to the model.
# TODO(marcoag): Add a validity check that ensures the model is
# world-mergeable. The package name for the include is inferred
# from the directory that contains the output file.
def _generate_world(sdf_filename: Path):
    world_root = ET.Element("sdf", version=_SDF_VERSION)
    root_world_name = sdf_filename.stem
    root_world_elem = ET.SubElement(world_root, "world", name=root_world_name)
    root_world_elem.append(
        ET.Comment(
            "Provides a direct inclusion of a given model, with no nesting"
        )
    )
    include_elem = ET.SubElement(root_world_elem, "include", merge="true")
    uri_elem = ET.SubElement(include_elem, "uri")
    uri_elem.text = (
        "package://" + sdf_filename.parent.name + "/" + sdf_filename.name
    )
    world_tree = ET.ElementTree(world_root)
    world_tree = world_tree.getroot()
    xmlstr = minidom.parseString(ET.tostring(world_tree)).toprettyxml(
        indent="  "
    )
    return xmlstr


def _main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-e",
        "--expand-included",
        action="store_true",
        help="Convert files included through [add_directives].",
    )
    parser.add_argument(
        "-m",
        "--model-directives",
        type=Path,
        required=True,
        help="Path to model directives file to be converted.",
    )
    parser.add_argument(
        "-o",
        "--output-path",
        type=Path,
        help="Output path of directory where SDFormat files will be written. "
        "When not provided, it will be the same directory as the input file.",
    )
    args = parser.parse_args(argv)

    # Do the conversion in memory.
    converted = _convert_directives(
        dmd_filename=args.model_directives,
        output_path=args.output_path,
        expand_included=args.expand_included,
    )

    # Once that succeeds, write everything to disk.
    if args.output_path is not None:
        os.makedirs(args.output_path, exist_ok=True)
    for out_path, out_string in converted.items():
        with open(out_path, "w", encoding="utf-8") as f:
            f.write(out_string)

    return 0


if __name__ == "__main__":
    _main()
