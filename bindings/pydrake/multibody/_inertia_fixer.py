"""Fix broken (non-physical) inertias in SDFormat/URDF, by writing a revised
file.

The common data structure being repaired in both formats is the content of the
`<inertial>` tag, which contains the mass, the inertia tensor, and also defines
a central inertia frame in which the tensor is expressed.

By definitions used in both URDF and SDFormat documentation, the relevant
frames are the link origin frame (L) and the central inertia frame (C). Drake
software often uses the terminology "body frame" (B), which is equivalent to L.
Therefore, in Drake monogram notation, the inertia tensor is expressed in B,
and the `<inertial>` tag includes the pose of C relative to B, X_BC.

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


# This function has an aggressively short name for embedding into f-strings.
def gfmt(x: float) -> str:
    """Converts a floating point number to a string, using some rounding to
    avoid platform differences in results.
    """
    return f"{x:.5g}"


class FormatDriver(object, metaclass=abc.ABCMeta):
    """This interface encapsulates the differences between URDF and SDFormat
    processing, for purposes of the implementation of XmlInertiaMapper.
    """

    @abc.abstractmethod
    def is_model_element(self, name: str) -> bool:
        """Returns True if the element directly encloses link elements.
        Args:
            name: the name of the XML element
        """
        raise NotImplementedError

    @abc.abstractmethod
    def associate_plant_models(self, models: list, links: list,
                               plant: MultibodyPlant) -> dict:
        """Returns a mapping of body_index to inertial ElementFacts.

        Args:
            models: a list of ElementFacts for model elements
            links: a list of ElementFacts for link elements
            plant: a MultibodyPlant containing the parsed models
        """
        raise NotImplementedError

    @abc.abstractmethod
    def format_inertia(self, input_bytes: bytes, inertial_facts: ElementFacts,
                       pose: list, mass: float, moments: list,
                       products: list) -> bytes:
        """Returns formatted XML bytes for the repaired inertial tag.

        See above for definitions of the frames involved.

        Args:
            input_bytes: full contents of the input file
            inertial_facts: facts about the element to reformat
            pose: Pose of the central inertia frame (C) relative to the body
                  frame (B), as a 6-element list: [x, y, z, r, p, y].
            mass: the mass of the body (kg).
            moments: the principle moments of inertia, expressed in C --
                     [ixx, iyy, izz].
            products: the products of inertia, expressed in C --
                      [ixy, ixz, iyz].
        """
        raise NotImplementedError


def is_synth_element(el: ElementFacts) -> bool:
    """Returns True if the ElementFacts is synthetic.

    Synthetic elements are placeholders for later expansion into output XML.
    """
    return el.end.index == el.start.index


def make_synth_element(parent: ElementFacts, name: str) -> ElementFacts:
    """Makes a new synthetic child ElementFacts for a given parent
    ElementFacts. The parent must already have at least one child.

    The returned synthetic element will be used for later expansion into the
    output XML.

    Args:
        parent: the ElementFacts that will get a new synthetic child
        name: the XML element name of the new child

    """
    assert parent.children, parent
    kid0 = parent.children[0]
    synth_el = ElementFacts(kid0.prior_data, name, {},
                            kid0.depth, kid0.start)
    synth_el.end = synth_el.start
    return synth_el


def find_inertial_facts_for_link(link: ElementFacts) -> ElementFacts:
    """Fetches or synthesizes a child inertial ElementFacts for a link
    ElementFacts.
    """
    inertial_kids = [x for x in link.children if x.name == "inertial"]
    if inertial_kids:
        return inertial_kids[0]
    return make_synth_element(link, "inertial")


def maybe_synthesize_children(parent: ElementFacts, names: list[str]):
    """Given a parent ElementFacts and a list of required child elements,
    synthesizes ElementFacts for any children that are missing. The parent must
    already have at least one child.

    The new synthetic elements will be used for later expansion into the output
    XML. Note that synthetic elements are inserted at the beginning of the list
    of child elements (for formatting convenience), so they will appear in the
    formatted output in the reverse order of that given by the input list of
    names.

    Args:
        parent: the ElementFacts that will get new synthetic children
        names: a list of XML element names to insert if missing
    """
    assert parent.children, parent
    kid_names = [x.name for x in parent.children]
    for needed_name in names:
        if needed_name in kid_names:
            continue
        parent.children.insert(
            0, make_synth_element(parent, needed_name))


def adjusted_element_end_index(input_bytes: bytes, facts: ElementFacts) -> int:
    """Returns the index of the end of all of the text (including the element
    closure), when given an ElementFacts produced by the expat parse.

    Args:
        input_bytes: full contents of the input file
        facts: an ElementFacts structure
    """
    if is_synth_element(facts):
        # Empty, synthetic, "pseudo" element.
        return facts.end.index

    if input_bytes[facts.end.index:].startswith(
            f"</{facts.name}".encode('utf-8')):
        # Typical case:
        # `<inertial>...</inertial>`.
        #               ^                # Raw index points here.
        #                          ^     # Adjusted index points here.
        return facts.end.index + input_bytes[facts.end.index:].find(b'>') + 1

    # `<inertial/>` corner case.
    #             ^     # Raw index points here; good to go
    return facts.end.index


def make_format_helpers(facts: ElementFacts) -> (
        Callable[[int], str], Callable[[], str]):
    """Returns two callable functions that help format white space for output
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

    Args:
        facts: an ElementFacts structure

    Returns:
        d: the indent function described above
        coda: the trailing space function described above
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
            assert plant.get_body(bix).name() == link.attributes.get("name"), (
                plant.get_body(bix).name(), bix, link.attributes.get("name"))
        return mapping

    def format_inertia(self, input_bytes: bytes, inertial_facts: ElementFacts,
                       pose: list, mass: float, moments: list,
                       products: list) -> bytes:
        end = adjusted_element_end_index(input_bytes, inertial_facts)
        inertia_output = f"""\
<inertia ixx="{gfmt(moments[0])}" ixy="{gfmt(products[0])}"\
 ixz="{gfmt(products[1])}" iyy="{gfmt(moments[1])}"\
 iyz="{gfmt(products[2])}" izz="{gfmt(moments[2])}"/>"""
        xyz_output = ' '.join([gfmt(x) for x in pose[:3]])
        rpy_output = ' '.join([gfmt(x) for x in pose[3:]])
        # * self-closing or no inertial tag: pave it all.
        if end == inertial_facts.end.index:
            d, coda = make_format_helpers(inertial_facts)
            return f"""\
<inertial>
{d(1)}{inertia_output}
{d(1)}<mass value="{gfmt(mass)}"/>
{d(1)}<origin rpy="{rpy_output}" xyz="{xyz_output}"/>
{d(0)}</inertial>{coda()}""".encode('utf-8')

        maybe_synthesize_children(
            inertial_facts, ["origin", "mass", "inertia"])
        # Now we get to slice up the inertial element, and only rewrite the
        # parts that we must.
        output = bytes()
        index = inertial_facts.start.index
        for facts in inertial_facts.children:
            output += input_bytes[index:facts.start.index]
            d, coda = make_format_helpers(facts)
            if facts.name == "inertia":
                output += f"{inertia_output}{coda()}".encode('utf-8')
            if facts.name == "mass":
                output += f'<mass value="{gfmt(mass)}"/>{coda()}'.encode(
                    'utf-8')
            if facts.name == "origin":
                output += f"""\
<origin rpy="{rpy_output}" xyz="{xyz_output}"/>{coda()}""".encode('utf-8')
            index = adjusted_element_end_index(input_bytes, facts)
        output += input_bytes[index:end]
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
            assert plant.get_body(bix).name() == link.attributes.get("name"), (
                plant.get_body(bix).name(), bix, link.attributes.get("name"))
        return mapping

    def format_inertia(self, input_bytes: bytes, inertial_facts: ElementFacts,
                       pose: list, mass: float, moments: list,
                       products: list) -> bytes:
        end = adjusted_element_end_index(input_bytes, inertial_facts)
        pose_output = ' '.join([gfmt(x) for x in pose])
        # * self-closing or no inertial tag: pave it all.
        if end == inertial_facts.end.index:
            d, coda = make_format_helpers(inertial_facts)
            return f"""\
<inertial>
{d(1)}<inertia>
{d(2)}<ixx>{gfmt(moments[0])}</ixx>
{d(2)}<ixy>{gfmt(products[0])}</ixy>
{d(2)}<ixz>{gfmt(products[1])}</ixz>
{d(2)}<iyy>{gfmt(moments[1])}</iyy>
{d(2)}<iyz>{gfmt(products[2])}</iyz>
{d(2)}<izz>{gfmt(moments[2])}</izz>
{d(1)}</inertia>
{d(1)}<mass>{gfmt(mass)}</mass>
{d(1)}<pose>{pose_output}</pose>
{d(0)}</inertial>{coda()}""".encode('utf-8')

        maybe_synthesize_children(
            inertial_facts, ["pose", "mass", "inertia"])
        # Now we get to slice up the inertial element, and only rewrite the
        # parts that we must.
        output = bytes()
        index = inertial_facts.start.index
        for facts in inertial_facts.children:
            output += input_bytes[index:facts.start.index]
            d, coda = make_format_helpers(facts)
            if facts.name == "inertia":
                output += f"""\
<inertia>
{d(1)}<ixx>{gfmt(moments[0])}</ixx>
{d(1)}<ixy>{gfmt(products[0])}</ixy>
{d(1)}<ixz>{gfmt(products[1])}</ixz>
{d(1)}<iyy>{gfmt(moments[1])}</iyy>
{d(1)}<iyz>{gfmt(products[2])}</iyz>
{d(1)}<izz>{gfmt(moments[2])}</izz>
{d(0)}</inertia>{coda()}""".encode('utf-8')
            if facts.name == "mass":
                output += f"<mass>{gfmt(mass)}</mass>{coda()}".encode('utf-8')
            if facts.name == "pose":
                output += f"<pose>{pose_output}</pose>{coda()}".encode('utf-8')
            index = adjusted_element_end_index(input_bytes, facts)
        end = adjusted_element_end_index(input_bytes, inertial_facts)
        output += input_bytes[index:end]
        return output


class XmlInertiaMapper:
    """Handles the parsing, indexing, and output generation details for editing
    of inertias in some formats of XML robot model files.
    """

    def __init__(self, input_text: str):
        """Initialize an XmlInertiaMapper.

        Args:
            input_text: the full contents of the input file
        """
        self._input_text = input_text
        self._input_bytes = input_text.encode('utf-8')

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

    def build_plant_models_association(self, plant: MultibodyPlant):
        """Matches body indices to inertial elements in the input text.

        Args:
            plant: a MultibodyPlant containing the parsed models
        """
        assert self._format_driver is not None
        self._mapping = self._format_driver.associate_plant_models(
            self._models, self._links, plant)

    def mapping(self) -> dict:
        """Returns a mapping from body indices to inertial element facts."""
        return self._mapping

    def build_output(self, new_inertias_mapping: dict) -> str:
        """Builds and returns the complete output text.

        Args:
            new_inertias_mapping: a dictionary whose keys are BodyIndex values,
               and whose values are a tuple of the body's spatial inertia and
               6D inertial pose as [x, y, z, r, p, y].
        """
        output = bytes()
        input_bytes = self._input_bytes
        input_bytes_index = 0
        for body_index, new_inertia in sorted(new_inertias_mapping.items()):
            inertial_facts = self._mapping[body_index]
            output += input_bytes[input_bytes_index:inertial_facts.start.index]
            output += self._format_inertia(inertial_facts, *new_inertia)
            input_bytes_index = adjusted_element_end_index(
                input_bytes, inertial_facts)
        output += input_bytes[input_bytes_index:]
        return output.decode('utf-8')

    def _format_inertia(self, inertial_facts: ElementFacts,
                        spatial_inertia: SpatialInertia, pose: list) -> str:
        # Extract the mass properties.
        mass = spatial_inertia.get_mass()
        rot = spatial_inertia.CalcRotationalInertia()
        mom = rot.get_moments()
        prod = rot.get_products()

        return self._format_driver.format_inertia(
            self._input_bytes, inertial_facts,
            pose, mass, mom, prod)


class InertiaProcessor:
    """Handles selection, repair, and replacement of inertial properties,
    in model files pre-processed by drake parsing and XmlInertiaMapper.
    """

    def __init__(self, plant: MultibodyPlant, scene_graph: SceneGraph,
                 mapper: XmlInertiaMapper):
        self._plant = plant
        self._scene_graph = scene_graph
        self._mapper = mapper

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
        inspector = self._scene_graph.model_inspector()
        for role in [Role.kProximity, Role.kIllustration, Role.kPerception]:
            geoms = inspector.GetGeometries(frame_id, role)
            if geoms:
                break
        if not geoms:
            # No geometry to fix inertia from.
            return

        # Collect some density==1 inertias for all geometries. The relevant
        # frames named below are:
        # * B: the body frame
        # * G: the frame of a single attached geometry
        # * Bcm: the body frame, translated to the center of mass, but not
        #   rotated.
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

        # TODO(rpoyner-tri): rotate to zero out products of inertia, and return
        # both inertia matrix and pose.
        M_BBcm_B = M_BBo_B.Shift(M_BBo_B.get_com())
        pose = [x for x in M_BBo_B.get_com()] + [0.0, 0.0, 0.0]

        return M_BBcm_B, pose

    def process(self) -> str:
        """Returns a new model file text, with selected inertial properties
        replaced by new ones calculated from supplied geometry.
        """
        mapping = self._mapper.mapping()
        new_inertias_mapping = {}
        for body_index in sorted(mapping.keys()):
            maybe_inertia = self._maybe_fix_inertia(body_index)
            if maybe_inertia is not None:
                new_inertias_mapping[body_index] = maybe_inertia
        return self._mapper.build_output(new_inertias_mapping)


def fix_inertia_from_string(input_text: str, file_type: str) -> str:
    """A pure string-processing wrapper for the tool, for testing.
    """
    # Parse with drake to build mbp and confirm sanity.
    plant = MultibodyPlant(time_step=0.0)
    scene_graph = SceneGraph()
    plant.RegisterAsSourceForSceneGraph(scene_graph)

    parser = Parser(plant)
    parser.package_map().PopulateFromRosPackagePath()

    parser.AddModelsFromString(input_text, file_type)

    # Parse with expat to build index.
    mapper = XmlInertiaMapper(input_text)
    mapper.parse()
    mapper.build_plant_models_association(plant)

    # Fix indicated inertias.
    processor = InertiaProcessor(plant, scene_graph, mapper)
    return processor.process()


class InertiaFixer:
    """Fixes specified inertias in a URDF or SDFormat input file, writing the
    new file contents to one of: stdout, a new file, or the original file.
    """
    def __init__(self, *,
                 input_file: Path,
                 output_file: Path = None,
                 in_place: bool = False):
        """Initialize an InertiaFixer.

        Args:
            input_file: the file to fix.
            output_file: a file to be created or overwritten with the new file
                         contents. If None, the new contents will be written to
                         stdout.
            in_place: if True, write output back to the input file.
        """
        self.input_file = input_file
        self.output_file = output_file
        self.in_place = in_place

    def fix_inertia(self):
        """Executes the inertia fixing processing and write the output as
        specified by the initialization arguments.
        """
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
        with open(self.input_file, encoding='utf-8') as fo:
            input_text = fo.read()

        # Parse with expat to build index.
        mapper = XmlInertiaMapper(input_text)
        mapper.parse()
        mapper.build_plant_models_association(plant)

        # Fix indicated inertias.
        processor = InertiaProcessor(plant, scene_graph, mapper)
        output_text = processor.process()

        # Write output.
        if self.output_file:
            output_file = self.output_file
        else:
            output_file = "/dev/stdout"
        if self.in_place:
            output_file = self.input_file
        with open(output_file, 'w', encoding='utf-8') as fo:
            fo.write(output_text)
