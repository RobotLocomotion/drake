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

AT PRESENT: This doesn't capture the nuance, because we're not using
`placement_frame` which I (Eric) think induces the parsing complexity we're
facing?
"""

import dataclasses as dc
import unittest

import numpy as np


@dc.dataclass
class PoseSchema:
    relative_to: str
    # Position relative to frame specified by `relative_to`.
    position: np.ndarray

    def __post_init__(self):
        self.position = np.asarray(self.position)


@dc.dataclass
class FrameSchema:
    name: str
    pose: PoseSchema


@dc.dataclass
class IncludeSchema:
    name: str
    file: str
    pose: PoseSchema


MOCK_FILESYSTEM = {
    "top.model": [
        FrameSchema(
            name="base",
            pose=PoseSchema(
                relative_to="world",
                position=[1.0, 2.0 ,3.0],
            ),
        ),
        IncludeSchema(
            name="included",
            file="sub.model",
            pose=PoseSchema(
                relative_to="base",
                position=[0, 0, 0],
            ),
        ),
        FrameSchema(
            name="top_frame",
            pose=PoseSchema(
                relative_to="included::included_frame",
                position=[10.0, 20.0, 30.0],
            ),
        ),
    ],
    "sub.model": [
        FrameSchema(
            name="included_frame",
            pose=PoseSchema(
                relative_to="__model__",
                position=[0.5, 0.25, 0.125],
            ),
        ),
    ],
}

EXPECTED_POSITION_W_MAP = {
    "world": [0, 0, 0],
    "top::__model__": [0, 0, 0],
    "top::base": [1.0, 2.0, 3.0],
    "top::included::__model__": [1.0, 2.0, 3.0],
    "top::included::included_frame": [1.5, 2.25, 3.125],
    "top::top_frame": [11.5, 22.25, 33.125],

}


# TODO: Add in placement_frame functionality to show (or maybe resolve?)
# parsing difficulty?


class Model:
    def __init__(self):
        # Denotes frames, stored w.r.t. "world".
        # We do this to exacerbate the parsing phases for posturing models, at
        # least as of drake==v1.10.0.
        self.position_W_map = {
            "world": np.zeros(3),
        }

    def add_frame(self, name, position):
        # N.B. We don't need cycle checks since we're greedily consuming frames
        # as they're added (vs libsdformat parsing them in separate pass).
        assert name not in self.position_W_map
        self.position_W_map[name] = position

    def resolve_position_W(self, relative_to, position):
        return self.position_W_map[relative_to] + position


def scope(prefix, name):
    if name == "world":
        return name
    elif prefix == "":
        return name
    else:
        return f"{prefix}::{name}"


def mock_parse(model, schema, model_name, model_position_W):
    model.add_frame(scope(model_name, "__model__"), model_position_W)

    def resolve_position_W(pose):
        return model.resolve_position_W(
            relative_to=scope(model_name, pose.relative_to),
            position=pose.position,
        )

    for directive in schema:
        if isinstance(directive, FrameSchema):
            assert directive.name != "world"
            name = scope(model_name, directive.name)
            position_W = resolve_position_W(directive.pose)
            model.add_frame(name, position_W)
        elif isinstance(directive, IncludeSchema):
            # Using a separate schema at this level (rather than distinct
            # schemas) *kinda of* simulates the separation between libsdformat,
            # separate formats, and how they are rendered upon MultibodyPlant.
            sub_schema = MOCK_FILESYSTEM[directive.file]
            sub_position_W = resolve_position_W(directive.pose)
            sub_name = scope(model_name, directive.name)
            mock_parse(model, sub_schema, sub_name, sub_position_W)


class Test(unittest.TestCase):
    def test_expected(self):
        model= Model()
        top_schema = MOCK_FILESYSTEM["top.model"]
        mock_parse(model, top_schema, "top", [0, 0, 0])
        np.testing.assert_equal(model.position_W_map, EXPECTED_POSITION_W_MAP)


if __name__ == "__main__":
    unittest.main()
