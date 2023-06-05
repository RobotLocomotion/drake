"""
Fix broken (non-physical) inertias in SDFormat/URDF,
by writing a revised file.
"""
from __future__ import annotations

import abc
import argparse
from dataclasses import dataclass
from typing import Callable
import xml.parsers.expat as expat

import numpy as np

from pydrake.geometry import (Role, SceneGraph)
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.multibody.tree import (BodyIndex, CalcSpatialInertia, RigidBody,
                                    SpatialInertia, UnitInertia)


@dataclass
class SourceLocation:
    """The location of an XML syntax item as reported by expat parsing."""

    index: int
    "0-based byte index from start of input stream."

    line: int
    "1-based line number from start of input stream."


@dataclass
class ElementFacts:
    """Facts about an XML element as collecting by expat parsing."""

    name: str
    "The name of the XML element."

    attributes: dict
    "The attributes of the XML element."

    depth: int
    "The nesting depth of the XML element; 0 for the root element."

    start: SourceLocation
    "The location of the start of the XML element text."

    end: SourceLocation = None
    """The location of the end of the XML element text; may require
    adjustments, depending on the input syntax used.
    """

    parent: ElementFacts = None
    "Facts for the parent of this element (optional)."

    serial_number: int = None
    """An opinionated index of this element within the sequence of similar
    elements; see XmlInertiaMapper's implementation for details."""


class FormatDriver(object, metaclass=abc.ABCMeta):

    """This interface encapsulates the differences between URDF and SDFormat
    processing, for purposes of the implementation of XmlInertiaMapper.
    """

    @abc.abstractmethod
    def is_model_element(self, name: str) -> bool:
        """Return True if the element directly encloses link elements."""
        raise NotImplementedError

    @abc.abstractmethod
    def is_element_ignored(self, name: str, attributes: dict) -> bool:
        """Return True if the element is ignored by the Drake parser."""
        raise NotImplementedError

    # TODO: support some per-format element indexing.

    @abc.abstractmethod
    def associate_plant_models(self, models: list, links: list,
                               inertials: list, plant: MultibodyPlant) -> dict:
        """Return a mapping of body_index to inertial ElementFacts."""
        raise NotImplementedError

    # TODO: adjust interface to allow passing inertia pose.
    # Or, maybe split this into mass/pose/inertia APIs?
    @abc.abstractmethod
    def format_inertia(self, mass: float, moments: list, products: list,
                       indenter: Callable[[int], str]) -> str:
        """Return a formatted XML string for the repaired inertial tag.

        The provided products and moments of inertia should be expressed in the
        body frame.

        The indenter function takes an indentation level relative to the
        indentation depth of the element to be replaced, and returns a string
        of white space necessary to reach that indentation level.

        Args:
            mass: the mass of the body (kg).
            moments: the principle moments of inertia -- ixx, iyy, izz.
            products: the products of inertia -- ixy, ixz, iyz.
            indenter: the indenter function (see above).

        """
        raise NotImplementedError


class UrdfDriver(FormatDriver):
    """Format driver for URDF files."""

    def is_model_element(self, name: str) -> bool:
        return name == "robot"

    def is_element_ignored(self, name: str, attributes: dict) -> bool:
        # TODO(rpoyner_tri): The 'drake_ignore' attribute is regrettable legacy
        # cruft that should be removed when the associated URDF parser cruft is
        # removed.
        return ((attributes.get("drake_ignore") == "true")
                or (attributes.get("name") == "world"))

    def associate_plant_models(self, models: list, links: list,
                               inertials: list, plant: MultibodyPlant) -> dict:
        # This assertion is weakened for files that use 'drake_ignore'.
        assert len(links) >= plant.num_bodies() - 1, (
            links, plant.num_bodies())
        mapping = {}
        for inertial in inertials:
            link = inertial.parent
            if link.serial_number is None:
                continue
            k = 1 + link.serial_number
            assert k < plant.num_bodies()
            mapping[BodyIndex(k)] = inertial
            # TODO assert more sanity.
        return mapping

    def format_inertia(self, mass: float, moments: list, products: list,
                       indenter: Callable[[int], str]) -> str:
        d = indenter
        return f"""\
<inertial>
{d(1)}<mass value="{mass}"/>
{d(1)}<origin xyz="0 0 0" rpy="0 0 0"/>
{d(1)}<inertia ixx="{moments[0]}" ixy="{products[0]}" ixz="{products[1]}"\
 iyy="{moments[1]}" iyz="{products[2]}" izz="{moments[2]}"/>
{d(0)}</inertial>"""


class SdformatDriver(FormatDriver):
    """Format driver for SDFormat files."""

    def is_model_element(self, name: str) -> bool:
        return name == "model"

    def is_element_ignored(self, name: str, attributes: dict) -> bool:
        return False

    def associate_plant_models(self, models: list, links: list,
                               inertials: list, plant: MultibodyPlant) -> dict:
        # Because SDFormat has both nested models and inclusions, we will have
        # to rummage around in the plant, finding body indices by using name
        # strings.
        mapping = {}
        for inertial in inertials:
            link = inertial.parent
            model = link.parent
            enclosing_models = [model]
            while model.parent:
                model = model.parent
                enclosing_models.append(model)
            enclosing_models.reverse()
            model_name = '::'.join(
                [m.attributes["name"] for m in enclosing_models])
            model_instance = plant.GetModelInstanceByName(model_name)
            body = plant.GetBodyByName(link.attributes["name"], model_instance)
            mapping[body.index()] = inertial
            # TODO assert more sanity.
        return mapping

    def format_inertia(self, mass: float, moments: list, products: list,
                       indenter: Callable[[int], str]) -> str:
        d = indenter
        return f"""\
<inertial>
{d(1)}<pose>0 0 0 0 0 0</pose>
{d(1)}<mass>{mass}</mass>
{d(1)}<inertia>
{d(2)}<ixx>{moments[0]}</ixx>
{d(2)}<ixy>{products[0]}</ixy>
{d(2)}<ixz>{products[1]}</ixz>
{d(2)}<iyy>{moments[1]}</iyy>
{d(2)}<iyz>{products[2]}</iyz>
{d(2)}<izz>{moments[2]}</izz>
{d(1)}</inertia>
{d(0)}</inertial>"""


class XmlInertiaMapper:
    """Handles the parsing, indexing, and output generation details for editing
    of inertias in some formats of XML robot model files.
    """

    def __init__(self, input_text: str):
        self._input_text = input_text

        # Build a line array for later use.  Add an unused "line" to simulate
        # 1-based indexing.
        self._input_lines = [""] + self._input_text.split('\n')

        # Configure the parser.
        self._parser = expat.ParserCreate()
        self._parser.StartElementHandler = self._start_element
        self._parser.EndElementHandler = self._end_element

        # Declare some state to remember during parsing.
        self._depth = 0
        self._models = []
        self._model_stack = []
        self._links = []
        self._ignored_links = 0
        self._inertials = []
        self._format_driver = None

        # Eventually build a mapping from body_index to inertial_facts.
        self._mapping = {}

    def _make_location(self) -> SourceLocation:
        return SourceLocation(self._parser.CurrentByteIndex,
                              self._parser.CurrentLineNumber)

    def _make_element_facts(self, name: str, attributes: dict) -> ElementFacts:
        return ElementFacts(name, attributes,
                            self._depth, self._make_location())

    def _start_element(self, name: str, attributes: dict):
        if not self._format_driver and self._depth == 0:
            if name == "robot":
                self._format_driver = UrdfDriver()
            elif name == "sdf":
                self._format_driver = SdformatDriver()
            else:
                raise RuntimeError("unknown file format!")

        if self._format_driver.is_model_element(name):
            element = self._make_element_facts(name, attributes)
            if self._model_stack:
                element.parent = self._model_stack[-1]
            self._models.append(element)
            self._model_stack.append(element)

        if name == "link":
            element = self._make_element_facts(name, attributes)
            element.parent = self._model_stack[-1]
            element.serial_number = len(self._links) - self._ignored_links

            if self._format_driver.is_element_ignored(name, attributes):
                self._ignored_links += 1
                element.serial_number = None

            self._links.append(element)

        if name == "inertial":
            element = self._make_element_facts(name, attributes)
            element.parent = self._links[-1]
            self._inertials.append(element)

        self._depth += 1

    def _end_element(self, name: str):
        self._depth -= 1

        if self._format_driver.is_model_element(name):
            self._model_stack.pop()

        if name == "inertial":
            self._inertials[-1].end = self._make_location()

    def parse(self):
        """Execute the parsing of the XML text."""
        self._parser.Parse(self._input_text)

    def associate_plant_models(self, plant: MultibodyPlant):
        """Match body indices to inertial elements in the input text."""
        assert self._format_driver is not None
        self._mapping = self._format_driver.associate_plant_models(
            self._models, self._links, self._inertials, plant)

    def mapping(self) -> dict:
        """Return a mapping from body indices to inertial element facts."""
        return self._mapping

    def _adjusted_element_end_index(self, raw_end_index: int) -> int:
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

    def build_output(self, new_inertias_mapping: dict) -> str:
        input_text = self._input_text
        output = ""
        index = 0
        for body_index, new_inertia in sorted(new_inertias_mapping.items()):
            facts = self._mapping[body_index]
            output += input_text[index:facts.start.index]
            output += self._format_inertia(body_index, new_inertia)
            index = self._adjusted_element_end_index(facts.end.index)
        output += input_text[index:]
        return output

    def _format_inertia(self, body_index: BodyIndex,
                        spatial_inertia: SpatialInertia) -> str:
        inertial_facts = self._mapping[body_index]

        # Compute indentation based on the first line of the target element.
        line = self._input_lines[inertial_facts.start.line]
        spaces = len(line) - len(line.lstrip())
        indentation = spaces // inertial_facts.depth

        # Extract the mass properties.
        mass = spatial_inertia.get_mass()
        rot = spatial_inertia.CalcRotationalInertia()
        mom = rot.get_moments()
        prod = rot.get_products()

        # Build the indenter.
        depth = inertial_facts.depth
        indent = " " * indentation

        def d(more_depth):
            return indent * (depth + more_depth)

        return self._format_driver.format_inertia(mass, mom, prod, d)


class InertiaProcessor:
    """Handles selection, repair, and replacement of inertial properties,
    in model files pre-processed by drake parsing and XmlInertiaMapper.
    """

    def __init__(self, args: argparse.Namespace, plant: MultibodyPlant,
                 scene_graph: SceneGraph, mapper: XmlInertiaMapper):
        self._args = args
        self._plant = plant
        self._scene_graph = scene_graph
        self._mapper = mapper

    def _maybe_fix_inertia(
            self, body_index: BodyIndex) -> SpatialInertia | None:
        assert int(body_index) < self._plant.num_bodies(), (
            body_index, self._plant.num_bodies())
        body = self._plant.get_body(body_index)
        if not isinstance(body, RigidBody):
            # Only rigid bodies have constant inertia, for which model file
            # fixups make sense.
            return
        maybe_frame_id = self._plant.GetBodyFrameIdIfExists(body_index)
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

        # TODO: shift the summed-up inertia back to Bcm, and return that.

        return M_BBo_B

    def process(self) -> str:
        """Return a new model file text, with selected inertial properties
        replaced by new ones calculated from supplied geometry.
        """
        mapping = self._mapper.mapping()
        new_inertias_mapping = {}
        for body_index in sorted(mapping.keys()):
            maybe_inertia = self._maybe_fix_inertia(body_index)
            if maybe_inertia:
                new_inertias_mapping[body_index] = maybe_inertia
        return self._mapper.build_output(new_inertias_mapping)


def main():
    # Parse arguments.
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

    # Parse with drake to build mbp and confirm sanity.
    plant = MultibodyPlant(time_step=0.0)
    scene_graph = SceneGraph()
    plant.RegisterAsSourceForSceneGraph(scene_graph)

    parser = Parser(plant)
    parser.package_map().PopulateFromRosPackagePath()

    # Read from the disk file here, to get more lenient processing of URIs,
    # better error messages, etc.
    parser.AddModels(args.input_file)

    # Slurp input file for indexing and editing.
    with open(args.input_file) as fo:
        input_text = fo.read()

    # Parse with expat to build index.
    mapper = XmlInertiaMapper(input_text)
    mapper.parse()
    mapper.associate_plant_models(plant)

    # Fix indicated inertias.
    processor = InertiaProcessor(args, plant, scene_graph, mapper)
    output_text = processor.process()

    # Write output.
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
