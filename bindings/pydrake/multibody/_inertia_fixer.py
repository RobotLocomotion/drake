"""
Fix broken (non-physical) inertias in SDFormat/URDF,
by writing a revised file.
"""
from __future__ import annotations

import abc
import argparse
from dataclasses import dataclass, field
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

    prior_data: str
    "Character data immediately preceding the element text."

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
    adjustments, depending on the input syntax used (optional).
    """

    parent: ElementFacts = None
    "Facts for the parent of this element (optional)."

    children: list[ElementFacts] = field(default_factory=list)
    "Facts about relevant children of this element (optional)."

    parent_model: ElementFacts = None
    """Facts for the enclosing model of this model element (optional).
    The definition of a model element is determined by each file format;
    see FormatDriver.is_model_element().
    """


class FormatDriver(object, metaclass=abc.ABCMeta):

    """This interface encapsulates the differences between URDF and SDFormat
    processing, for purposes of the implementation of XmlInertiaMapper.
    """

    @abc.abstractmethod
    def is_model_element(self, name: str) -> bool:
        """Return True if the element directly encloses link elements."""
        raise NotImplementedError

    @abc.abstractmethod
    def associate_plant_models(self, models: list, links: list,
                               plant: MultibodyPlant) -> dict:
        """Return a mapping of body_index to inertial ElementFacts."""
        raise NotImplementedError

    @abc.abstractmethod
    def format_inertia(self, input_text: str, inertial_facts: ElementFacts,
                       pose: list,
                       mass: float, moments: list, products: list) -> str:
        """Return a formatted XML string for the repaired inertial tag.

        The provided products and moments of inertia should be expressed in the
        body frame.

        Args:
            input_text: full contents of the input file
            inertial_facts: facts about the element to reformat
            pose: Pose of the center-of-mass frame relative to the body frame.
            mass: the mass of the body (kg).
            moments: the principle moments of inertia -- ixx, iyy, izz.
            products: the products of inertia -- ixy, ixz, iyz.

        """
        raise NotImplementedError


def is_synth_element(el: ElementFacts) -> bool:
    """Return true if the ElementFacts is synthetic.

    Synthetic elements are placeholders for later expansion into output XML.
    """
    return el.end.index == el.start.index


def make_synth_element(parent: ElementFacts, name: str) -> ElementFacts:
    """Make a new synthetic child ElementFacts for a given parent ElementFacts.
    The parent must already have at least one child.

    The returned synthetic element will be used for later expansion into the
    output XML.
    """
    assert parent.children, parent
    kid0 = parent.children[0]
    synth_el = ElementFacts(kid0.prior_data, name, {},
                            kid0.depth, kid0.start)
    synth_el.end = synth_el.start
    return synth_el


def find_inertial_facts_for_link(link: ElementFacts) -> ElementFacts:
    """Fetch or synthesize a child inertial ElementFacts for a link
    ElementFacts.
    """
    inertial_kids = [x for x in link.children if x.name == "inertial"]
    if inertial_kids:
        return inertial_kids[0]
    return make_synth_element(link, "inertial")


def maybe_synthesize_children(parent: ElementFacts, names: list[str]):
    """Given a parent ElementFacts and a list of required child elements,
    synthesize ElementFacts for any children that are missing. The parent must
    already have at least one child.

    The new synthetic elements will be used for later expansion into the output
    XML.
    """
    assert parent.children, parent
    kid_names = [x.name for x in parent.children]
    kid0 = parent.children[0]
    for needed_name in names:
        if needed_name in kid_names:
            continue
        parent.children.insert(
            0, make_synth_element(parent, needed_name))


def adjusted_element_end_index(input_text: str, facts: ElementFacts) -> int:
    """Returns the index of the end of all of the text (including the element
    closure), when given an ElementFacts produced by the expat parse.
    """
    if is_synth_element(facts):
        # Empty, synthetic, "pseudo" element.
        return facts.end.index

    if input_text[facts.end.index:].startswith(f"</{facts.name}"):
        # Typical case:
        # `<inertial>...</inertial>`.
        #               ^                # Raw index points here.
        #                          ^     # Adjusted index points here.
        return facts.end.index + input_text[facts.end.index:].find('>') + 1

    # `<inertial/>` corner case.
    #             ^     # Raw index points here; good to go
    return facts.end.index


def make_format_helpers(facts: ElementFacts) -> (
        Callable[[int], str], Callable[[], str]):
    """Return two callable functions that help format white space for output
    XML elements.

    The returned callables are intended to be convenient to invoke in Python
    f-strings; callers typically assign them as `d` and `coda`.

    d(int) -> str
        indents n levels in addition to the current depth of the element
         described by `facts`.

    coda() -> str
        provides appropriate trailing white space to maintain typical
        formatting:
            * newline plus d(0) if `facts` is synthetic,
            * otherwise, empty string.

    """
    spaces = len(facts.prior_data)
    indentation = spaces // facts.depth

    # Build an element-specific indenter.
    depth = facts.depth
    indent = " " * indentation

    def d(more_depth):
        return indent * (depth + more_depth)

    synth = facts.end.index == facts.start.index

    def coda():
        return '\n' + d(0) if synth else ""

    return (d, coda)


def bug():
    import pdb
    pdb.set_trace()


class UrdfDriver(FormatDriver):
    """Format driver for URDF files."""

    def is_model_element(self, name: str) -> bool:
        return name == "robot"

    def _is_element_ignored(self, facts: ElementFacts) -> bool:
        # TODO(rpoyner_tri): The 'drake_ignore' attribute is regrettable legacy
        # cruft that should be removed when the associated URDF parser cruft is
        # removed.
        return ((facts.attributes.get("drake_ignore") == "true")
                or (facts.attributes.get("name") == "world"))

    def associate_plant_models(self, models: list, links: list,
                               plant: MultibodyPlant) -> dict:
        # This assertion is weakened for files that use 'drake_ignore'.
        assert len(links) >= plant.num_bodies() - 1, (
            links, plant.num_bodies())
        mapping = {}
        serial_number = 0
        for link in links:
            if self._is_element_ignored(link):
                continue
            serial_number += 1
            if not link.children:
                # Without inertia (mass) or geometry, there is nothing to do.
                continue
            link_name = link.attributes.get("name")
            assert serial_number < plant.num_bodies()
            bix = BodyIndex(serial_number)
            mapping[bix] = find_inertial_facts_for_link(link)
            # TODO assert more sanity.
            assert plant.get_body(bix).name() == link.attributes.get("name"), (
                bug(),
                plant.get_body(bix).name(), bix, link.attributes.get("name"))
        return mapping

    def format_inertia(self, input_text: str, inertial_facts: ElementFacts,
                       pose: list,
                       mass: float, moments: list, products: list) -> str:
        end = adjusted_element_end_index(input_text, inertial_facts)
        inertia_output = f"""\
<inertia ixx="{moments[0]}" ixy="{products[0]}" ixz="{products[1]}"\
 iyy="{moments[1]}" iyz="{products[2]}" izz="{moments[2]}"/>"""
        xyz_output = ' '.join([str(x) for x in pose[:3]])
        rpy_output = ' '.join([str(x) for x in pose[3:]])
        # * self-closing or no inertial tag: pave it all.
        if end == inertial_facts.end.index:
            d, coda = make_format_helpers(inertial_facts)
            return f"""\
<inertial>
{d(1)}{inertia_output}
{d(1)}<mass value="{mass}"/>
{d(1)}<origin rpy="{rpy_output}" xyz="{xyz_output}"/>
{d(0)}</inertial>{coda()}"""

        maybe_synthesize_children(
            inertial_facts, ["origin", "mass", "inertia"])
        # Now we get to slice up the inertial element, and only rewrite the
        # parts that we must.
        output = ""
        index = inertial_facts.start.index
        for facts in inertial_facts.children:
            output += input_text[index:facts.start.index]
            d, coda = make_format_helpers(facts)
            if facts.name == "inertia":
                output += f"{inertia_output}{coda()}"
            if facts.name == "mass":
                output += f'<mass value="{mass}"/>{coda()}'
            if facts.name == "origin":
                output += f"""\
<origin rpy="{rpy_output}" xyz="{xyz_output}"/>{coda()}"""
            index = adjusted_element_end_index(input_text, facts)
        output += input_text[index:end]
        return output


class SdformatDriver(FormatDriver):
    """Format driver for SDFormat files."""

    def is_model_element(self, name: str) -> bool:
        return name == "model"

    def associate_plant_models(self, models: list, links: list,
                               plant: MultibodyPlant) -> dict:
        # Because SDFormat has both nested models and inclusions, we will have
        # to rummage around in the plant, finding body indices by using name
        # strings.
        mapping = {}
        for link in links:
            if not link.children:
                continue
            model = link.parent
            enclosing_models = [model]
            while model.parent_model:
                model = model.parent_model
                enclosing_models.append(model)
            enclosing_models.reverse()
            model_name = '::'.join(
                [m.attributes["name"] for m in enclosing_models])
            model_instance = plant.GetModelInstanceByName(model_name)
            body = plant.GetBodyByName(link.attributes["name"], model_instance)
            bix = body.index()
            mapping[bix] = find_inertial_facts_for_link(link)
            # TODO assert more sanity.
            assert plant.get_body(bix).name() == link.attributes.get("name"), (
                plant.get_body(bix).name(), bix, link.attributes.get("name"))
        return mapping

    def format_inertia(self, input_text: str, inertial_facts: ElementFacts,
                       pose: list,
                       mass: float, moments: list, products: list) -> str:
        end = adjusted_element_end_index(input_text, inertial_facts)
        pose_output = ' '.join([str(x) for x in pose])
        # * self-closing or no inertial tag: pave it all.
        if end == inertial_facts.end.index:
            d, coda = make_format_helpers(inertial_facts)
            return f"""\
<inertial>
{d(1)}<inertia>
{d(2)}<ixx>{moments[0]}</ixx>
{d(2)}<ixy>{products[0]}</ixy>
{d(2)}<ixz>{products[1]}</ixz>
{d(2)}<iyy>{moments[1]}</iyy>
{d(2)}<iyz>{products[2]}</iyz>
{d(2)}<izz>{moments[2]}</izz>
{d(1)}</inertia>
{d(1)}<mass>{mass}</mass>
{d(1)}<pose>{pose_output}</pose>
{d(0)}</inertial>{coda()}"""

        maybe_synthesize_children(
            inertial_facts, ["pose", "mass", "inertia"])
        # Now we get to slice up the inertial element, and only rewrite the
        # parts that we must.
        output = ""
        index = inertial_facts.start.index
        for facts in inertial_facts.children:
            output += input_text[index:facts.start.index]
            d, coda = make_format_helpers(facts)
            if facts.name == "inertia":
                output += f"""\
<inertia>
{d(1)}<ixx>{moments[0]}</ixx>
{d(1)}<ixy>{products[0]}</ixy>
{d(1)}<ixz>{products[1]}</ixz>
{d(1)}<iyy>{moments[1]}</iyy>
{d(1)}<iyz>{products[2]}</iyz>
{d(1)}<izz>{moments[2]}</izz>
{d(0)}</inertia>{coda()}"""
            if facts.name == "mass":
                output += f"<mass>{mass}</mass>{coda()}"
            if facts.name == "pose":
                output += f"<pose>{pose_output}</pose>{coda()}"
            index = adjusted_element_end_index(input_text, facts)
        end = adjusted_element_end_index(input_text, inertial_facts)
        output += input_text[index:end]
        return output


class XmlInertiaMapper:
    """Handles the parsing, indexing, and output generation details for editing
    of inertias in some formats of XML robot model files.
    """

    def __init__(self, input_text: str):
        self._input_text = input_text

        # Configure the parser.
        self._parser = expat.ParserCreate()
        self._parser.StartElementHandler = self._handle_start_element
        self._parser.EndElementHandler = self._handle_end_element
        self._parser.CharacterDataHandler = self._handle_char_data

        # Declare some state to remember during parsing.
        self._depth = 0
        self._element_stack = []
        self._models = []
        self._model_stack = []
        self._links = []
        self._format_driver = None
        self._last_char_data = ""

        # Eventually build a mapping from body_index to inertial_facts.
        self._mapping = {}

    def _make_location(self) -> SourceLocation:
        return SourceLocation(self._parser.CurrentByteIndex,
                              self._parser.CurrentLineNumber)

    def _make_element_facts(self, name: str, attributes: dict) -> ElementFacts:
        return ElementFacts(self._last_char_data, name, attributes,
                            self._depth, self._make_location())

    def _handle_char_data(self, data: str):
        self._last_char_data = data

    def _handle_start_element(self, name: str, attributes: dict):
        if not self._format_driver and self._depth == 0:
            if name == "robot":
                self._format_driver = UrdfDriver()
            elif name == "sdf":
                self._format_driver = SdformatDriver()
            else:
                raise RuntimeError("unknown file format!")

        element = self._make_element_facts(name, attributes)
        if self._element_stack:
            element.parent = self._element_stack[-1]
            element.parent.children.append(element)

        if self._format_driver.is_model_element(name):
            if self._model_stack:
                element.parent_model = self._model_stack[-1]
            self._models.append(element)
            self._model_stack.append(element)

        if name == "link":
            self._links.append(element)

        self._depth += 1
        self._element_stack.append(element)

    def _handle_end_element(self, name: str):
        self._element_stack[-1].end = self._make_location()
        self._element_stack.pop()
        self._depth -= 1

        if self._format_driver.is_model_element(name):
            self._model_stack.pop()

    def parse(self):
        """Execute the parsing of the XML text."""
        self._parser.Parse(self._input_text)

    def associate_plant_models(self, plant: MultibodyPlant):
        """Match body indices to inertial elements in the input text."""
        assert self._format_driver is not None
        self._mapping = self._format_driver.associate_plant_models(
            self._models, self._links, plant)

    def mapping(self) -> dict:
        """Return a mapping from body indices to inertial element facts."""
        return self._mapping

    def build_output(self, new_inertias_mapping: dict) -> str:
        output = ""
        input_text = self._input_text
        input_text_index = 0
        for body_index, new_inertia in sorted(new_inertias_mapping.items()):
            inertial_facts = self._mapping[body_index]
            output += input_text[input_text_index:inertial_facts.start.index]
            output += self._format_inertia(inertial_facts, *new_inertia)
            input_text_index = adjusted_element_end_index(
                input_text, inertial_facts)
        output += input_text[input_text_index:]
        return output

    def _format_inertia(self, inertial_facts: ElementFacts,
                        spatial_inertia: SpatialInertia, com: list) -> str:
        # Extract the mass properties.
        mass = spatial_inertia.get_mass()
        rot = spatial_inertia.CalcRotationalInertia()
        mom = rot.get_moments()
        prod = rot.get_products()
        pose = [x for x in com] + [0.0, 0.0, 0.0]

        return self._format_driver.format_inertia(
            self._input_text, inertial_facts,
            pose, mass, mom, prod)


class InertiaProcessor:
    """Handles selection, repair, and replacement of inertial properties,
    in model files pre-processed by drake parsing and XmlInertiaMapper.
    """

    def __init__(self, plant: MultibodyPlant, scene_graph: SceneGraph,
                 mapper: XmlInertiaMapper, *, invalid_only: bool = False):
        self._plant = plant
        self._scene_graph = scene_graph
        self._mapper = mapper
        self._invalid_only = invalid_only

    def _maybe_fix_inertia(
            self, body_index: BodyIndex) -> [SpatialInertia, list] | None:
        assert int(body_index) < self._plant.num_bodies(), (
            body_index, self._plant.num_bodies())
        body = self._plant.get_body(body_index)
        if not isinstance(body, RigidBody):
            # Only rigid bodies have constant inertia, for which model file
            # fixups make sense.
            return
        frame_id = self._plant.GetBodyFrameIdOrThrow(body_index)
        old_inertia = body.default_spatial_inertia()
        if self._invalid_only and old_inertia.IsPhysicallyValid():
            # Skip valid inertias by user preference.
            return
        inspector = self._scene_graph.model_inspector()
        for role in [Role.kProximity, Role.kIllustration, Role.kPerception]:
            geoms = inspector.GetGeometries(frame_id, role)
            if geoms:
                break
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

        # TODO: rotate to zero out products of inertia, and return both inertia
        # matrix and pose.
        M_BBcm_B = M_BBo_B.Shift(M_BBo_B.get_com())

        return M_BBo_B, M_BBo_B.get_com()

    def process(self) -> str:
        """Return a new model file text, with selected inertial properties
        replaced by new ones calculated from supplied geometry.
        """
        mapping = self._mapper.mapping()
        new_inertias_mapping = {}
        for body_index in sorted(mapping.keys()):
            maybe_inertia = self._maybe_fix_inertia(body_index)
            if maybe_inertia is not None:
                new_inertias_mapping[body_index] = maybe_inertia
        return self._mapper.build_output(new_inertias_mapping)


def fix_inertia_from_string(input_text: str, file_type: str, *,
                            invalid_only: bool = False):
    """A pure string-processing embodiment, for testing.
    """
    # Parse with drake to build mbp and confirm sanity.
    plant = MultibodyPlant(time_step=0.0)
    scene_graph = SceneGraph()
    plant.RegisterAsSourceForSceneGraph(scene_graph)

    parser = Parser(plant)
    parser.SpoilInvalidInertia()
    parser.package_map().PopulateFromRosPackagePath()

    parser.AddModelsFromString(input_text, file_type)

    # Parse with expat to build index.
    mapper = XmlInertiaMapper(input_text)
    mapper.parse()
    mapper.associate_plant_models(plant)

    # Fix indicated inertias.
    processor = InertiaProcessor(plant, scene_graph, mapper,
                                 invalid_only=invalid_only)
    return processor.process()


class InertiaFixer:
    def __init__(self, *,
                 input_file: Path,
                 output_file: Path = None,
                 invalid_only: bool = False,
                 in_place: bool = False):
        self.input_file = input_file
        self.output_file = output_file
        self.invalid_only = invalid_only
        self.in_place = in_place

    def fix_inertia(self):
        # Parse with drake to build mbp and confirm sanity.
        plant = MultibodyPlant(time_step=0.0)
        scene_graph = SceneGraph()
        plant.RegisterAsSourceForSceneGraph(scene_graph)

        parser = Parser(plant)
        parser.package_map().PopulateFromRosPackagePath()

        # Read from the disk file here, to get more lenient processing of URIs,
        # better error messages, etc.
        parser.AddModels(str(self.input_file))

        # Slurp input file for indexing and editing.
        with open(self.input_file) as fo:
            input_text = fo.read()

        # Parse with expat to build index.
        mapper = XmlInertiaMapper(input_text)
        mapper.parse()
        mapper.associate_plant_models(plant)

        # Fix indicated inertias.
        processor = InertiaProcessor(plant, scene_graph, mapper,
                                     invalid_only=self.invalid_only)
        output_text = processor.process()

        # Write output.
        if self.output_file:
            output_file = self.output_file
        else:
            output_file = "/dev/stdout"
        if self.in_place:
            output_file = self.input_file
        with open(output_file, 'w') as fo:
            fo.write(output_text)
