"""
Fix broken (non-physical) inertias in SDFormat/URDF,
by writing a revised file.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import xml.parsers.expat as expat

import numpy as np

from pydrake.geometry import (Role, SceneGraph)
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.multibody.tree import (BodyIndex, CalcSpatialInertia, RigidBody,
                                    SpatialInertia, UnitInertia)


@dataclass
class SourceLocation:
    index: int
    line: int
    column: int


@dataclass
class ElementFacts:
    name: str
    attributes: list
    depth: int
    start: SourceLocation
    end: SourceLocation = None
    parent: ElementFacts = None
    # Need facts about models, for SDFormat file/plant association???


class UrdfDriver:
    def __init__(self):
        pass

    def associate_plant_models(self, inertials, plant, models):
        assert len(models) >= 1
        assert len(inertials) == plant.num_bodies() - 1, (
            inertials, plant.num_bodies())
        mapping = {}
        for k in range(1, plant.num_bodies()):
            mapping[k] = inertials[k - 1]
            # TODO assert more sanity.
        return mapping

    def format_inertia(self, inertial_facts, fixed_inertia, indentation):
        depth = inertial_facts.depth
        indent = " " * indentation
        mass = fixed_inertia.get_mass()
        rot = fixed_inertia.CalcRotationalInertia()
        mom = rot.get_moments()
        prod = rot.get_products()

        def d(indentation):
            return indent * (depth + indentation)

        return f"""\
<inertial>
{d(1)}<mass value="{mass}"/>
{d(1)}<origin rpy=" 0 0 0" xyz="0 0 0"/>
{d(1)}<inertia ixx="{mom[0]}" ixy="{prod[0]}" ixz="{prod[1]}"\
 iyy="{mom[1]}" iyz="{prod[2]}" izz"{mom[2]}"/>
{d(0)}</inertial>"""


class SdformatDriver:
    def __init__(self):
        pass

    def associate_plant_models(self, inertials, plant, models):
        assert len(models) >= 1
        # XXX This assertion is likely violated by files that use include tags.
        assert len(inertials) == plant.num_bodies() - 1, (
            inertials, plant.num_bodies())
        mapping = {}
        for k in range(1, plant.num_bodies()):
            mapping[k] = inertials[k - 1]
            # TODO assert more sanity.
        return mapping

    def format_inertia(self, inertial_facts, fixed_inertia, indentation):
        depth = inertial_facts.depth
        indent = " " * indentation
        mass = fixed_inertia.get_mass()
        rot = fixed_inertia.CalcRotationalInertia()
        mom = rot.get_moments()
        prod = rot.get_products()

        def d(indentation):
            return indent * (depth + indentation)

        return f"""\
<inertial>
{d(1)}<pose>0 0 0 0 0 0</pose>
{d(1)}<mass>{mass}</mass>
{d(1)}<inertia>
{d(2)}<ixx>{mom[0]}</ixx>
{d(2)}<ixy>{prod[0]}</ixy>
{d(2)}<ixz>{prod[1]}</ixz>
{d(2)}<iyy>{mom[1]}</iyy>
{d(2)}<iyz>{prod[2]}</iyz>
{d(2)}<izz>{mom[2]}</izz>
{d(1)}</inertia>
{d(0)}</inertial>"""


class XmlInertiaMapper:
    def __init__(self, input_text):
        self._input_text = input_text

        # Build a line array for later use.  Add an unused "line" to simulate
        # 1-based indexing.
        self._input_lines = [""] + self._input_text.split('\n')

        # Configure the parser.
        self._parser = expat.ParserCreate()
        self._parser.StartElementHandler = self._start_el
        self._parser.EndElementHandler = self._end_el
        self._parser.ordered_attributes = True

        # Declare some state to remember during parsing.
        self._depth = 0
        self._links = []
        self._inertials = []
        self._format_driver = None

        # Eventually build a mapping from body_index to inertial_facts.
        self._mapping = {}

    def _make_el_facts(self, name, attributes):
        return ElementFacts(
            name,
            attributes,
            self._depth,
            SourceLocation(
                self._parser.CurrentByteIndex,
                self._parser.CurrentLineNumber,
                self._parser.CurrentColumnNumber))

    def _start_el(self, name, attributes):
        if not self._format_driver and self._depth == 0:
            if name == "robot":
                self._format_driver = UrdfDriver()
            elif name == "sdf":
                self._format_driver = SdformatDriver()
            else:
                raise RuntimeError("unknown file format!")

        if name == "link":
            self._links.append(self._make_el_facts(name, attributes))
        if name == "inertial":
            element = self._make_el_facts(name, attributes)
            element.parent = self._links[-1]
            self._inertials.append(element)
        self._depth += 1

    def _end_el(self, name):
        self._depth -= 1
        if name == "inertial":
            self._inertials[-1].end = SourceLocation(
                self._parser.CurrentByteIndex,
                self._parser.CurrentLineNumber,
                self._parser.CurrentColumnNumber)
            print(f"inertial facts: {self._inertials[-1]}")

    def parse(self):
        got = self._parser.Parse(self._input_text)

    def associate_plant_models(self, plant, models):
        self._mapping = self._format_driver.associate_plant_models(
            self._inertials, plant, models)

    def mapping(self):
        return self._mapping

    def _adjusted_el_end_index(self, raw_end_index):
        input_text = self._input_text
        if input_text[raw_end_index] == '<':
            # Typical case:
            # `<inertial>...</inertial>`.
            #               ^                # Raw index points here.
            #                          ^     # Adjusted index points here.
            return raw_end_index + input_text[raw_end_index:].find('>') + 1
        else:
            # `<inertial/>` corner case.
            #             ^     # Raw index points here; good to go
            return raw_end_index

    def build_output(self, new_inertias_mapping):
        input_text = self._input_text
        output = ""
        index = 0
        for body_index, new_inertia in sorted(new_inertias_mapping.items()):
            facts = self._mapping[body_index]
            output += input_text[index:facts.start.index]
            output += self.format_inertia(body_index, new_inertia)
            index = self._adjusted_el_end_index(facts.end.index)
        output += input_text[index:]
        return output

    def format_inertia(self, body_index, spatial_inertia):
        inertial_facts = self._mapping[body_index]

        # Compute indentation based on the first line of the target element.
        line = self._input_lines[inertial_facts.start.line]
        spaces = len(line) - len(line.lstrip())
        indentation = spaces // inertial_facts.depth

        return self._format_driver.format_inertia(
            inertial_facts, spatial_inertia, indentation)


class InertiaProcessor:
    def __init__(self, args, plant, scene_graph, models, mapper):
        self._args = args
        self._plant = plant
        self._scene_graph = scene_graph
        self._models = models
        self._mapper = mapper

    def _maybe_fix_inertia(self, body_index):
        body = self._plant.get_body(BodyIndex(body_index))
        # XXX is this the analog of downcast in python?
        if not isinstance(body, RigidBody):
            # Only rigid bodies have constant inertia, for which model file
            # fixups make sense.
            return
        maybe_frame_id = self._plant.GetBodyFrameIdIfExists(
            BodyIndex(body_index))
        if not maybe_frame_id:
            # No geometry to fix inertia from.
            return
        old_inertia = body.default_spatial_inertia()
        if self._args.invalid_only and old_inertia.IsPhysicallyValid():
            # Skip valid inertias by user preference.
            return
        inspector = self._scene_graph.model_inspector()
        geoms = inspector.GetGeometries(maybe_frame_id, Role.kProximity)
        if not geoms:
            # No geometry to fix inertia from.
            return

        # Collect some density==1 inertias for all geometries.
        M_BBo_B_one = SpatialInertia(0, np.zeros(3), UnitInertia(0, 0, 0))
        for geom in geoms:
            M_GG_G_one = CalcSpatialInertia(inspector.GetShape(geom), 1.0)
            X_BG = inspector.GetPoseInFrame(geom)
            M_GBo_B_one = M_GG_G_one.ReExpress(
                X_BG.rotation()).Shift(-X_BG.translation())
            M_BBo_B_one += M_GBo_B_one

            # Rebuild the final inertia using the mass found in the input.
            mass = old_inertia.get_mass()
            M_BBo_B = SpatialInertia(mass, M_BBo_B_one.get_com(),
                                     M_BBo_B_one.get_unit_inertia())

        return M_BBo_B

    def process(self):
        mapping = self._mapper.mapping()
        new_inertias_mapping = {}
        for body_index in sorted(mapping.keys()):
            maybe_inertia = self._maybe_fix_inertia(body_index)
            if maybe_inertia:
                new_inertias_mapping[body_index] = maybe_inertia
        return self._mapper.build_output(new_inertias_mapping)


def main():
    # Prepare to parse arguments.
    args_parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    args_parser.add_argument(
        "input_file", type=str,
        help="Filesystem path to an SDFormat or URDF file.")
    args_parser.add_argument(
        "output_file", type=str, nargs='?',
        help="[Optional] Filesystem path to write output with repaired"
        " inertias. If missing, output will go to stdout.")
    args_parser.add_argument(
        "--invalid_only", action="store_true",
        help="only fix physically invalid inertias.")
    args_parser.add_argument(
        "--in_place", action="store_true",
        help="modify the input file in-place. Any output_file argument"
        " will be ignored.")
    args = args_parser.parse_args()

    with open(args.input_file) as fo:
        input_text = fo.read()

    # parse with drake to build mbp and confirm sanity
    plant = MultibodyPlant(time_step=0.0)
    scene_graph = SceneGraph()
    plant.RegisterAsSourceForSceneGraph(scene_graph)

    parser = Parser(plant)
    parser.package_map().PopulateFromRosPackagePath()

    if '<sdf' in input_text:
        file_type = 'sdf'
    elif '<robot' in input_text:
        file_type = 'urdf'
    else:
        raise RuntimeError(f"Input file '{args.input_file}' is of unknown"
                           " type; only SDFormat and URDF are supported'")
    models = parser.AddModelsFromString(input_text, file_type)

    # parse with expat to build index
    mapper = XmlInertiaMapper(input_text)
    mapper.parse()
    mapper.associate_plant_models(plant, models)

    # fix indicated inertias
    processor = InertiaProcessor(args, plant, scene_graph, models, mapper)
    output_text = processor.process()

    # write output
    if args.output_file:
        output_file = args.output_file
    else:
        output_file = "/dev/stdout"
    if args.in_place:
        output_file = args.input_file
    with open(output_file, 'w') as fo:
        fo.write(output_text)


if __name__ == '__main__':
    main()
