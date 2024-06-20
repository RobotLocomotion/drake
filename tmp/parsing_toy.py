"""
Add a toy example trying to indicate difficulties of parsing phase decoupling
pursuant to:

- http://sdformat.org/tutorials?tut=composition_proposal#1-5-minimal-libsdformat-interface-types-for-non-sdformat-models
- https://github.com/RobotLocomotion/drake/pull/16727#issuecomment-1332820283

Goal:

Part of the issue is a coupling of context across parsing that happens across
to distinct formats (e.g. SDFormat and Drake's URDF) as well as
context-sensitive semantics via frame graphs (e.g. posturing objects relative
to one another).

To cover this, we create our toy dataclass format, and denote the (parse,
operate) steps among the two modes as is currently done (via resolution
callbacks).


Issues:
1. Poses of Drake frames cannot be relative to ther frames without requiring
   attachment.
"""

import os
from collections import defaultdict
import dataclasses as dc
from typing import Union
import unittest

import numpy as np


@dc.dataclass
class PoseSchema:
    relative_to: str
    # Position relative to frame specified by `relative_to`.
    position: np.ndarray

    def __post_init__(self):
        self.position = np.asarray(self.position)


def prefix_pose(prefix: str, pose: PoseSchema):
    return PoseSchema(
        relative_to=scope(prefix, pose.relative_to), position=pose.position
    )


@dc.dataclass
class LinkSchema:
    name: str
    pose: PoseSchema


@dc.dataclass
class FrameSchema:
    name: str
    attached_to: str
    pose: PoseSchema


@dc.dataclass
class IncludeSchema:
    name: str
    file: str
    pose: PoseSchema


MOCK_FILESYSTEM = {
    "top.model": [
        LinkSchema(
            name="L1",
            pose=PoseSchema(
                relative_to="__model__",
                position=[0.0, 0.0, 10.0],
            ),
        ),
        LinkSchema(
            name="L2",
            pose=PoseSchema(
                relative_to="__model__",
                position=[1.0, 2.0, 3.0],
            ),
        ),
        FrameSchema(
            name="F1",
            attached_to="L2",
            pose=PoseSchema(
                relative_to="L1",
                position=[1.0, 2.0, 3.0],
            ),
        ),
        IncludeSchema(
            name="included",
            file="sub.drake_model",
            pose=PoseSchema(
                relative_to="L1",
                position=[0, 0, 0],
            ),
        ),
        FrameSchema(
            name="top_frame",
            attached_to="__model__",
            pose=PoseSchema(
                relative_to="included::included_frame",
                position=[10.0, 20.0, 30.0],
            ),
        ),
    ],
    "sub.drake_model": [
        LinkSchema(
            name="base",
            pose=PoseSchema(
                relative_to="__model__",
                position=[0.0, 0.0, 0.0],
            ),
        ),
        FrameSchema(
            name="included_frame",
            attached_to="base",
            pose=PoseSchema(
                relative_to="base",
                position=[0.5, 0.25, 0.125],
            ),
        ),
    ],
}


def scope(prefix, name=None, *args):
    if name == None:
        return prefix
    if name == "world":
        return name
    elif prefix == "":
        return name
    else:
        return scope(f"{prefix}::{name}", *args)

def unscope(prefix, name):
    assert (prefix+"::") in name
    return name[len(prefix)+2:]


# This is a lightweight Model used by libsdformat when interfacing with custom
# parsers. Only frame bearing elements (models, links, frames, and joints) are
# contained in libsdformat's InterfaceModel. Here, for simplicity, we only use
# links and frames.
class InterfaceModel:
    def __init__(self, model_name, schema, pose, reposture_function):
        self.name = model_name
        self.pose = pose
        self.reposture_function = reposture_function
        self.interface_links = []
        self.interface_frames = []

        for directive in schema:
            if isinstance(directive, FrameSchema):
                assert directive.name != "world"
                self.interface_frames.append(directive)
            elif isinstance(directive, LinkSchema):
                self.interface_links.append(directive)


# This class emulates libsdformat's sdf::Model class.
class SdfModel:
    def __init__(self, name, schema, pose, custom_parser_cb=None):
        """
        Load model from a given schema

        Args:
            model_name: Name of model, since schema doesn't contain model name.
            schema: Schema to load.
            pose: Pose of model.
            custom_parser_cb: Callback function for custom parser
        """
        self.name = name
        self.pose = pose

        self.links = []
        self.frames = []
        self.models = []

        for directive in schema:
            if isinstance(directive, FrameSchema):
                assert directive.name != "world"
                self.frames.append(directive)
            elif isinstance(directive, LinkSchema):
                self.links.append(directive)
            elif isinstance(directive, IncludeSchema):
                extension = os.path.splitext(directive.file)[1]
                if extension == ".model":
                    sub_model = SdfModel(
                        directive.name, directive.file, directive.pose
                    )
                elif extension == ".drake_model":
                    if custom_parser_cb is None:
                        raise RuntimeError("Custom parser not set")
                    # Simulate the custom parser callback used in the
                    # Interface API of libsdformat.
                    sub_model = custom_parser_cb(
                        directive.name, directive.file, directive.pose
                    )
                else:
                    raise RuntimeError(f"Unknown file extension {extension}")

                self.models.append(sub_model)


# Emulates libsdformat's pose graph functionality as found in
# https://github.com/gazebosim/sdformat/blob/6af4be6b2dbd13271ef88b2088ac91cea695dfa6/src/FrameSemantics.cc
class PoseGraph:
    def __init__(self, model: SdfModel):
        self.graph = {}
        self.graph["world"] = PoseSchema(
            relative_to="__root__", position=[0, 0, 0]
        )
        self.graph[model.name] = PoseSchema(
            relative_to="world", position=[0, 0, 0]
        )
        self.add_model(model, model.name)

    def add_model(self, model: Union[SdfModel, InterfaceModel], prefix: str):

        self._add_to_graph(
            scope(prefix, "__model__"),
            PoseSchema(relative_to=prefix, position=[0, 0, 0]),
        )

        if isinstance(model, SdfModel):
            for link in model.links:
                self._add_to_graph(
                    scope(prefix, link.name), prefix_pose(prefix, link.pose)
                )

            for frame in model.frames:
                self._add_to_graph(
                    scope(prefix, frame.name), prefix_pose(prefix, frame.pose)
                )

            for nested_model in model.models:
                self._add_to_graph(
                    scope(prefix, nested_model.name),
                    prefix_pose(prefix, nested_model.pose),
                )
                self.add_model(nested_model, scope(prefix, nested_model.name))

        elif isinstance(model, InterfaceModel):
            for link in model.interface_links:
                self._add_to_graph(
                    scope(prefix, link.name), prefix_pose(prefix, link.pose)
                )

            for frame in model.interface_frames:
                self._add_to_graph(
                    scope(prefix, frame.name), prefix_pose(prefix, frame.pose)
                )

    def resolve_pose(self, frame, relative_to):
        # 1. Resolve `frame` relative_to `__model__`: X_MF
        # 2. Resolve `relative_to` relative_to `__model`: X_MR
        # 3. Compute X_RF = X_MR^-1 * X_MF

        X_MF = self._resolve_to_root(frame)
        X_MR = self._resolve_to_root(relative_to)
        return X_MF - X_MR

    def _add_to_graph(self, key, val):
        if key in self.graph:
            raise RuntimeError(f"Frame {key} is already in graph")
        self.graph[key] = val

    def _resolve_to_root(self, frame):
        pose = self.graph[frame]
        if pose.relative_to == "__root__":
            return pose.position
        else:
            return self._resolve_to_root(pose.relative_to) + pose.position

    def __str__(self):
        return str(self.graph)

    def to_dot(self):
        out = "digraph {\n"
        for key in self.graph.keys():
            out += f'  "{key}"\n'

        for key, val in self.graph.items():
            out += f'  "{val.relative_to}" -> "{key}"'
            out += f' [label="{val.position}"]\n'
        out += "}"
        return out

class FrameGraph:
    def __init__(self, model: SdfModel):
        self.graph = {}
        self.add_model(model, model.name)

    def add_model(self, model: Union[SdfModel, InterfaceModel], prefix: str):

        if isinstance(model, SdfModel):
            assert len(model.links) > 0
            self._add_to_graph(
                scope(prefix,  "__model__"),
                scope(prefix, model.links[0].name),
            )

            for link in model.links:
                self._add_to_graph(scope(prefix, link.name), "__link__")

            for frame in model.frames:
                self._add_to_graph(scope(prefix, frame.name),
                                   scope(prefix, frame.attached_to))

            for nested_model in model.models:
                self._add_to_graph(
                    scope(prefix, nested_model.name),
                    scope(prefix, nested_model.name, "__model__"),
                )
                self.add_model(nested_model, scope(prefix, nested_model.name))

        elif isinstance(model, InterfaceModel):
            for link in model.interface_links:
                self._add_to_graph(scope(prefix, link.name), "__link__")

            for frame in model.interface_frames:
                self._add_to_graph(scope(prefix, frame.name),
                                   scope(prefix, frame.attached_to))

    def resolve_to_link(self, frame):
        parent_frame = self.graph[frame]
        if parent_frame  == "__link__":
            return frame
        else:
            return self.resolve_to_link(parent_frame)

    def _add_to_graph(self, key, val):
        if key in self.graph:
            raise RuntimeError(f"Frame {key} is already in graph")
        self.graph[key] = val

    def __str__(self):
        return str(self.graph)

    def to_dot(self):
        out = "digraph {\n"
        for key in self.graph.keys():
            out += f'  "{key}"\n'

        for key, val in self.graph.items():
            out += f'  "{key}" -> "{val}"\n'
        out += "}"
        return out


# Emulates sdf::Root in libsdformat.
class SdfRoot:
    def __init__(self, filepath, custom_parser=None):
        self.model = SdfModel(
            "top",
            MOCK_FILESYSTEM[filepath],
            PoseSchema("", [0, 0, 0]),
            custom_parser_cb=custom_parser,
        )
        self._pose_graph = PoseGraph(self.model)
        self._frame_graph = FrameGraph(self.model)

        for model in self.model.models:
            if isinstance(model, InterfaceModel):
                model.reposture_function(self._pose_graph)


    # In libsdformat, each DOM element has a `SemanticPose` member function
    # that can be used to resolve poses. We're simplifying the API here by
    # providing a single function for resolving poses. The main difference in
    # functionality is here we have to pass fully scoped names whereas in
    # libsdformat, the frame lookup happens relative to the element's own
    # scope.
    def resolve_pose(self, frame, relative_to):
        return self._pose_graph.resolve_pose(frame, relative_to)

    def resolve_attached_to_link(self, scoped_frame_name):
        return self._frame_graph.resolve_to_link(scoped_frame_name)


# Emulates Drake's MultibodyPlant. Methods for adding model instances, bodies,
# and frames are defined here to reflect the API available in MultibodyPlant,
# but their functionality is limited to simple bookkeeping.
class MockMultibodyPlant:

    _id_counter = 0

    def __init__(self):
        self._model_instances = {}

    def _next_id(self):
        self._id_counter += 1
        return self._id_counter

    def add_model_instance(self, name, canonical_link, X_LcM):
        model_instance = self._next_id()
        self._model_instances[model_instance] = defaultdict(dict)
        self._model_instances[model_instance]["name"] = name
        self.add_frame("__model__", model_instance, canonical_link, X_LcM)
        return model_instance

    # Drake MBP's add_body equivalent doesn't take X_WB as an argument, but it
    # is used here to emulate the function of calling SetDefaultFreeBodyPose
    # after a body has been added to the MBP
    def add_body(self, name, model_instance, X_WB):
        model = self._model_instances[model_instance]
        model["bodies"][name] = X_WB
        self.add_frame(name, model_instance, name, [0, 0, 0])

    def add_frame(self, name, model_instance, parent_body, X_PF):
        model = self._model_instances[model_instance]
        model["frames"][name] = [parent_body, X_PF]

    def __str__(self):
        return f"{self._model_instances}"

    def num_bodies(self):
        count = 0
        for _, model in self._model_instances.items():
            count += len(model["bodies"])
        return count

    def num_frames(self):
        count = 0
        for _, model in self._model_instances.items():
            count += len(model["frames"])
        return count

    def bodies(self, model_instance):
        return self._model_instances[model_instance]["bodies"]

    def resolve_frame_pose_to_world(self, scoped_frame_name):
        if scoped_frame_name == "world":
            return [0, 0, 0]
        for _, model in self._model_instances.items():
            for frame_name, (parent_body, X_PF) in model["frames"].items():
                if scope(model["name"], frame_name) == scoped_frame_name:
                    return model["bodies"][parent_body] + X_PF

        raise RuntimeError(f"Frame '{scoped_frame_name}' not found")


# Emulates the functionality of //multibody/parsing:parser and
# //multibody/parsing:detail_sdf_parser
def mock_drake_sdf_parser(model_name, sdf_file):

    plant = MockMultibodyPlant()

    # We make assumptions about poses encountered in the custom parser such
    # that we don't need pose resolution. This assumption holds, for example,
    # in Drake's URDF parser as all the poses there are expressed relative to
    # the parent body of a joint. Thus, they can be added into the
    # MultibodyPlant without having to be resolved relative to some other
    # frame.
    def custom_parser(nested_model_name, file_name, pose):
        schema = MOCK_FILESYSTEM[file_name]

        canonical_link = ""
        for directive in schema:
            if isinstance(directive, LinkSchema):
                canonical_link = directive.name
                X_PLc = directive.pose.position
                X_PM = pose.position
                X_LcM = - X_PLc + X_PM
                break
        assert canonical_link != ""
        nested_model_instance = plant.add_model_instance(
            scope(model_name, nested_model_name), canonical_link, X_LcM
        )

        for directive in schema:
            if isinstance(directive, FrameSchema):
                # All poses of frames parsed by the custom parser are expected
                # to be expressed relative to the parent frame. This is similar
                # to how the Drake URDF parser is used as a custom parser via
                # libsdformat's Interface API.
                assert directive.pose.relative_to == directive.attached_to

                plant.add_frame(
                    directive.name,
                    nested_model_instance,
                    directive.attached_to,
                    directive.pose.position,
                )
            elif isinstance(directive, LinkSchema):
                # The pose of the body will be updated during reposture, so we 
                # could just use identity here.
                plant.add_body(
                    directive.name,
                    nested_model_instance,
                    directive.pose.position,
                )

        def reposture_function(pose_graph):
            bodies = plant.bodies(nested_model_instance)
            for body_name,_ in bodies.items():
                X_WB = pose_graph.resolve_pose(
                    scope(model_name, nested_model_name, body_name), "world")
                bodies[body_name] = X_WB

        return InterfaceModel(nested_model_name, schema, pose,
                              reposture_function)

    root = SdfRoot(sdf_file, custom_parser=custom_parser)
    model = root.model
    assert len(model.links) > 0

    canonical_link = model.links[0].name
    X_WLc = root.resolve_pose(scope(model.name, canonical_link), model.name)
    X_WM = [0, 0, 0]
    X_LcM = -X_WLc + X_WM
    model_instance = plant.add_model_instance(model_name, canonical_link,
                                              X_LcM)

    for link in model.links:
        X_WB = root.resolve_pose(scope(model.name, link.name), model.name)
        plant.add_body(link.name, model_instance, X_WB)

    for frame in model.frames:
        # NOTE: It is necessary to resolve the pose of the frame relative to
        # the parent (attached_to) frame because the pose given in the schema
        # might be relative to some other frame. This resolution capability is
        # missing in Drake, thus, we rely on libsdformat to do it for us. But
        # for libsdformat to have this capability, it needs to know about all
        # the frames in the model including nested and custom parsed models.
        # This is why callbacks are needed for implementing custom parsers in
        # libsdformat.
        # The frame resolution capability is emulated here in the `PoseGraph`
        # class.
        attached_to_link = root.resolve_attached_to_link(
            scope(model.name, frame.attached_to))
        attached_to_link_local_name = unscope(model.name, attached_to_link)
        X_PF = root.resolve_pose(scope(model.name, frame.name),
                                 attached_to_link)
        plant.add_frame(frame.name, model_instance,
                        attached_to_link_local_name, X_PF)

    return plant


EXPECTED_POSITION_W_MAP = {
    "world": [0, 0, 0],
    "top::__model__": [0, 0, 0],
    "top::L1": [0.0, 0.0, 10.0],
    "top::L2": [1.0, 2.0, 3.0],
    "top::F1": [1.0, 2.0, 13.0],
    "top::included::__model__": [0, 0, 10],
    "top::included::base": [0, 0, 10],
    "top::included::included_frame": [0.5, 0.25, 10.125],
    "top::top_frame": [10.5, 20.25, 40.125],
}

class Test(unittest.TestCase):
    def test_expected(self):
        plant = mock_drake_sdf_parser("top", "top.model")
        self.assertEqual(plant.num_bodies(), 3)
        self.assertEqual(plant.num_frames(), 8)
        position_W_map = dict(
            (frame, plant.resolve_frame_pose_to_world(frame))
            for frame in EXPECTED_POSITION_W_MAP.keys()
        )
        np.testing.assert_equal(position_W_map, EXPECTED_POSITION_W_MAP)


if __name__ == "__main__":
    unittest.main()
