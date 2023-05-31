#!/usr/bin/python3 -B
"""
Fix broken (non-physical) inertias in SDFormat/URDF,
by writing a revised file.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import sys
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
    # Need facts about models, for SDFormat???


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
        return f"""<inertial>
{indent * (depth + 1)}<mass value="{mass}"/>
{indent * (depth + 1)}<origin rpy=" 0 0 0" xyz="0 0 0"/>
{indent * (depth + 1)}<inertia ixx="{mom[0]}" ixy="{prod[0]}" ixz="{prod[1]}" iyy="{mom[1]}" iyz="{prod[2]}" izz"{mom[2]}"/>
{indent * (depth + 0)}</inertial>"""


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
        return f"""<inertial>
{indent * (depth + 1)}<pose>0 0 0 0 0 0</pose>
{indent * (depth + 1)}<mass>{mass}</mass>
{indent * (depth + 1)}<inertia>
{indent * (depth + 2)}<ixx>{mom[0]}</ixx>
{indent * (depth + 2)}<ixy>{prod[0]}</ixy>
{indent * (depth + 2)}<ixz>{prod[1]}</ixz>
{indent * (depth + 2)}<iyy>{mom[1]}</iyy>
{indent * (depth + 2)}<iyz>{prod[2]}</iyz>
{indent * (depth + 2)}<izz>{mom[2]}</izz>
{indent * (depth + 1)}</inertia>
{indent * (depth + 0)}</inertial>"""


class XmlInertiaMapper:
    def __init__(self, input_text):
        self._input_text = input_text

        # Infer file global indentation. If you like literal tabs, you lose.
        # XXX This algorithm is forgiving of a few typos, but could go wrong in
        # corner cases. Maybe just infer indentation from the element start
        # line and depth instead.
        lines = self._input_text.split('\n')
        num_bins = min(5, len(lines))
        bins = [[] for _ in range(num_bins)]
        for k, line in enumerate(lines):
            bins[k % num_bins].append(line)
        self._indentation = 0
        for k in range(num_bins):
            bin_lines = bins[k]
            indents = [len(l) - len(l.lstrip()) for l in bin_lines]
            self._indentation = max(self._indentation, math.gcd(*indents))
        print(f"indentation: {self._indentation}", file=sys.stderr)

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

        self._mapping = {}

    def _start_el(self, name, attributes):
        if not self._format_driver and self._depth == 0:
            if name == "robot":
                self._format_driver = UrdfDriver()
            elif name == "sdf":
                self._format_driver = SdformatDriver()
            else:
                raise RuntimeError("unknown file format!")

        element = ElementFacts(
            name,
            attributes,
            self._depth,
            SourceLocation(
                self._parser.CurrentByteIndex,
                self._parser.CurrentLineNumber,
                self._parser.CurrentColumnNumber))
        if name == "link":
            self._links.append(element)
        if name == "inertial":
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

    def build_output(self, fixed_inertias_mapping):
        assert self._mapping
        input_text = self._input_text
        preserved_inputs = []
        output = ""
        index = 0
        for body_index, fixed_inertia in sorted(fixed_inertias_mapping.items()):
            inertial_facts = self._mapping[body_index]
            preserved_inputs.append(
                input_text[index:inertial_facts.start.index])
            if input_text[inertial_facts.end.index] == '<':
                # Typical case:
                # `<inertial>...</inertial>`.
                index = (
                    inertial_facts.end.index +
                    input_text[inertial_facts.end.index:].find('>') + 1)
            else:
                # `<inertial/>` corner case.
                index = inertial_facts.end.index
            output += preserved_inputs[-1]
            output += self.format_inertia(body_index, fixed_inertia)
        preserved_inputs.append(input_text[index:])
        output += preserved_inputs[-1]
        return output

    def format_inertia(self, body_index, spatial_inertia):
        inertial_facts = self._mapping[body_index]
        return self._format_driver.format_inertia(
            inertial_facts, spatial_inertia, self._indentation)


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
        fixed_inertias_mapping = {}
        for body_index in sorted(mapping.keys()):
            maybe_inertia = self._maybe_fix_inertia(body_index)
            if maybe_inertia:
                fixed_inertias_mapping[body_index] = maybe_inertia
        return self._mapper.build_output(fixed_inertias_mapping)


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
