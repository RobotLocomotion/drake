"""Rough starting prototype to do quick regression tests on kinematic models.

Supports:

- Generating a fresh benchmark from a Python expression.
- Generating a benchmark against an existing benchmark.
- Comparing benchmarks.
- Saving/loading benchmarks to/from files.
"""

import argparse
import os
import sys

import numpy as np
import yaml

from pydrake.systems.framework import DiagramBuilder
from pydrake.geometry import (
    ConnectDrakeVisualizer, SceneGraph, DispatchLoadMessage)
from pydrake.lcm import DrakeLcm

from pydrake.multibody.parsing import (
    Parser,
)
from pydrake.multibody.tree import (
    FrameIndex, JointIndex,
    PrismaticJoint, RevoluteJoint,
)
from pydrake.multibody.plant import (
    MultibodyPlant,
    AddMultibodyPlantSceneGraph,
)
from pydrake.common.eigen_geometry import Isometry3, AngleAxis


def indent(prefix, s):
    out = [prefix + line for line in s.split("\n")]
    return "\n".join(out)


class Comparison(object):
    """Provides comparison parameters and results (errors)."""
    def __init__(self, tol=1e-6, error_ignore=None):
        """
        Args:
            tol: Tolerance used for comparisons.
        """
        self.tol = tol
        self.errors = []
        self._error_ignore = error_ignore

    def error(self, err):
        """Registers a warning.

        Returns:
            True if registered, False if ignored.
        """
        assert isinstance(err, Error)
        if self._error_ignore and self._error_ignore(err):
            print("<Ignore {}>".format(type(err).__name__))
            return False
        print(err)
        self.errors.append(err)
        return True


class Error(object):
    def __init__(self, path):
        self.path = path

    def _get_message(self):
        raise NotImplemented

    def __str__(self):
        return "{}: {}:\n{}".format(
            type(self).__name__, self.path, indent('  ', self._get_message()))


class KinematicError(Error):
    def __init__(self, path, a, b, rot_err, tr_err):
        Error.__init__(self, path)
        self.a = a
        self.b = b
        self.rot_err = rot_err
        self.tr_err = tr_err

    def _get_message(self):
        return ("translation: {}\nrotation: {}").format(
            self.tr_err, self.rot_err)


class Frame(object):
    """Contains frame name and pose w.r.t. world at given configuration."""
    def __init__(self, name, X_WF):
        self.name = str(name)
        self.X_WF = Isometry3(X_WF)

    def compare(self, b, cmp_, parent_path):
        """Compares to another frame `b` given `cmp_` metrics."""
        path = parent_path + "/frame[@name='{}']".format(self.name)
        assert self.name == b.name, path
        X_AB = self.X_WF.inverse().multiply(b.X_WF)
        rot_err = AngleAxis(X_AB.rotation()).angle()
        if np.abs(rot_err - np.pi) < np.abs(rot_err):
            rot_err -= np.pi
        tr_err = np.linalg.norm(X_AB.translation())
        if np.abs(tr_err) > cmp_.tol or np.abs(rot_err) > cmp_.tol:
            return not cmp_.error(
                KinematicError(path, self, b, rot_err, tr_err))
        return True

    @classmethod
    def from_yaml(cls, d):
        """Creates from a dict (from YAML)."""
        return cls(
            name=d["name"],
            X_WF=Isometry3(
                rotation=d["X_WF"]["rotation"],
                translation=d["X_WF"]["translation"],
            ),
        )

    def to_yaml(self):
        """Serializes to YAML-friendly dict."""
        return dict(
            name=str(self.name),
            X_WF=dict(
                translation=self.X_WF.translation().tolist(),
                rotation=self.X_WF.rotation().tolist(),
            ),
        )


class ListMismatch(Error):
    def __init__(self, path, a, b):
        Error.__init__(self, path)
        self.a = a
        self.b = b
        self.not_in_a = set(b) - set(a)
        self.not_in_b = set(a) - set(b)

    def _get_message(self):
        # TODO(eric.cousineau): Catch duplicates / ordering?
        if self.not_in_a or self.not_in_b:
            return "Not in A: {}\nNot in B: {}".format(
                repr(self.not_in_a), repr(self.not_in_b))
        else:
            return "Order mismatch:\n  A: {}\n  B: {}".format(
                repr(self.a), repr(self.b))


class PositionNameMismatch(ListMismatch):
    pass


class PositionValueMismatch(Error):
    def __init__(self, path, a, b, q_err):
        Error.__init__(self, path)
        self.a = a
        self.b = b
        self.q_err = q_err

    def _get_message(self):
        return "error: {}\nnames: {}\na = {}\nb = {}".format(
            self.q_err, self.a.names, self.a.get_values(self.a.names),
            self.b.get_values(self.a.names))


class SemanticQ(object):
    """Provides semantic meaning for position values, and means of interop."""
    def __init__(self, names, values):
        self.names = [str(x) for x in names]
        self._values = np.array(values)
        assert len(self.names) == len(self._values)

    def get_values(self, names_out):
        """Gets raw value according to order in `names_out`. If `names_out` is
        None, returns order given the current order."""
        if names_out is None:
            return self._values.copy()
        else:
            # Reorder to the order of `names_out`.
            assert len(self.names) == len(names_out)
            idx_to_out = [self.names.index(x) for x in names_out]
            return self._values[idx_to_out]

    def compare(self, b, cmp_, parent_path):
        """Compares to another SemanticQ `b` given `cmp_` metrics."""
        path = parent_path + "/q"
        # WARNING: At present, our code assumes a fixed instance ordering.
        # For now, require the exact same joint / dof order.
        if self.names != b.names:
            if cmp_.error(PositionNameMismatch(path, self.names, b.names)):
                return False
        values_b = b.get_values(self.names)
        q_err = np.linalg.norm(self._values - values_b)
        if q_err > cmp_.tol:
            err = PositionValueMismatch(
                path, a=self, b=b, q_err=q_err)
            if cmp_.error(err):
                # Do not compare frames based on bad values.
                return False
        return True

    def __str__(self):
        out = []
        for name, value in zip(self.names, self._values):
            out.append("{}={}".format(name, value))
        return "[{}]".format(", ".join(out))

    @classmethod
    def from_yaml(cls, d):
        """Creates from a dict (from YAML)."""
        return cls(names=d["names"], values=np.array(d["values"]))

    def to_yaml(self):
        """Serializes to YAML-friendly dict."""
        return dict(names=list(self.names), values=self._values.tolist())


class FrameNameMismatch(ListMismatch):
    pass


class Configuration(object):
    """Contains configuration (semantics and values), human-readable note, and
    frames."""
    def __init__(self, note, q, frames):
        self.note = note
        self.q = q
        self.frames = frames
        self._sort()

    def compare(self, b, cmp_, parent_path):
        """Compares to another configuration `b` given `cmp_` metrics."""
        self._sort()
        b._sort()
        path = parent_path + "/configuration[@note='{}']".format(self.note)
        if not self.q.compare(b.q, cmp_, path):
            # Do not compare frames based on bad values.
            return False
        frame_map = {frame.name: frame for frame in self.frames}
        a_names = [frame.name for frame in self.frames]
        b_names = [frame.name for frame in b.frames]
        good = True
        for frame_b in b.frames:
            frame = frame_map.get(frame_b.name)
            if frame is None:
                continue
            if not frame.compare(frame_b, cmp_, path):
                good = False
        return good

    def _sort(self):
        # TODO(eric.cousineau): Control access to make this more succinct.
        self.frames = sorted(self.frames, key=lambda x: x.name)

    @classmethod
    def from_yaml(cls, d):
        """Creates from a dict (from YAML)."""
        out = cls(
            note=str(d["note"]),
            q=SemanticQ.from_yaml(d["q"]),
            frames=[Frame.from_yaml(x) for x in d["frames"]],
        )
        out._sort()
        return out

    def to_yaml(self):
        """Serializes to YAML-friendly dict."""
        self._sort()
        return dict(
            note=str(self.note),
            q=self.q.to_yaml(),
            frames=[x.to_yaml() for x in self.frames],
        )


class ConfigurationMismatch(ListMismatch):
    pass


class Benchmark(object):
    """Benchmarks a set of configurations and frames for a given tree
    representation."""
    def __init__(self, expression, type_name, configurations=None):
        """
        Args:
            expression: Python expression used to create tree. Evaluated in
                scope of this module.
            type_name: Direct type name (no module) of tree.
            configurations: List[Configuration]
        """
        self.expression = expression
        self.type_name = type_name
        self.configurations = configurations or []
        # Only set when loading
        self.filename = None

    @classmethod
    def from_yaml(cls, d):
        """Creates from a dict (from YAML)."""
        return cls(
            expression=d["expression"],
            type_name=d["type_name"],
            configurations=[
                Configuration.from_yaml(x) for x in d["configurations"]],
        )

    def to_yaml(self):
        """Serializes to YAML-friendly dict."""
        return dict(
            expression=self.expression,
            type_name=self.type_name,
            configurations=[x.to_yaml() for x in self.configurations],
        )

    def _info(self):
        # Provides brief overview of this benchmark for traceability.
        return dict(
            expression=self.expression,
            type_name=self.type_name,
            filename=self.filename or "<new>",
        )

    def compare(self, b, cmp_=None):
        """Compares to another benchmark `b` given `tol`
        Returns:
            Comparison with errors (and tolerance).
        """
        if cmp_ is None:
            cmp_ = Comparison()
        print(yaml.dump(
            dict(
                compare=dict(a=self._info(), b=b._info()),
            ), default_flow_style=False))
        a_notes = [config.note for config in self.configurations]
        b_notes = [config.note for config in b.configurations]
        path = ''
        if a_notes != b_notes:
            if cmp_.error(ConfigurationMismatch(path, a_notes, b_notes)):
                return cmp_
        for config_b in b.configurations:
            for config in self.configurations:
                if config.note == config_b.note:
                    break
            else:
                continue
            # Check all configurations.
            if not config.compare(config_b, cmp_, path):
                break
        return cmp_

    @classmethod
    def load_yaml(cls, filename):
        """Loads benchmark from YAML file."""
        print("Load: {}".format(filename))
        with open(filename) as f:
            benchmark = cls.from_yaml(yaml.load(f))
        benchmark.filename = filename
        return benchmark

    def save_yaml(self, filename):
        """Saves benchmark to YAML file."""
        print("Save: {}".format(filename))
        with open(filename, 'w') as f:
            f.write("# GENERATED FILE - DO NOT EDIT BY HAND\n")
            yaml.dump(self.to_yaml(), f)
        self.filename = filename


class Adaptor(object):
    """Base class for tree representation wrapper.
    Note: `MultibodyPlantWrapper` may be overkill for this problem.
    """
    @property
    def position_names(self):
        """Names of position degrees of freedom."""
        raise NotImplemented()

    def _do_calc_frames(self, q_raw):
        """Implements frame computations given a `q_raw` that is relevant to
        the concrete tree representation."""
        raise NotImplemented()

    def calc_frames(self, q):
        """Computes all frames given configuration with semantics."""
        return self._do_calc_frames(q.get_values(self.position_names))


def add_new_configurations(benchmark, adaptor):
    """Adds new configurations (zeros, ones, and incrementing values) to
    benchmark."""
    nq = len(adaptor.position_names)

    def add(q_raw, note):
        q = SemanticQ(adaptor.position_names, q_raw.copy())
        frames = adaptor.calc_frames(q)
        benchmark.configurations.append(Configuration(note, q, frames))

    q_raw = np.array([0.] * nq)
    add(q_raw, "Zeros")
    q_raw[:] = 1
    add(q_raw, "Ones")
    q_raw[:] = np.arange(nq) * 0.1 + 0.1
    add(q_raw, "Incrementing")


def add_old_configurations(benchmark, adaptor, benchmark_old):
    """Adds configurations from `benchmark_old` to `benchmark`, computing
    kinematics using `adaptor`."""
    for config_old in benchmark_old.configurations:
        a_names = adaptor.position_names
        b_names = config_old.q.names
        # See warning in `SemanticQ.compare`.
        if a_names != b_names:
            # Print information to give more context.
            print(yaml.dump(
                dict(
                    compare=dict(a=benchmark._info(), b=benchmark_old._info()),
                ), default_flow_style=False))
            raise RuntimeError(str(ListMismatch('', a_names, b_names)))
        frames = adaptor.calc_frames(config_old.q)
        benchmark.configurations.append(
            Configuration(config_old.note, config_old.q, frames))


def create_benchmark(
        expr, benchmark_old=None, return_adaptor=False, locals_=None):
    """Creates a benchmark from (str) `expr`.
    Args:
        expr: Expression to evaluate tree.
        benchmark_old: Benchmark that shall be compared against.
            If None, uses new values (for generating a fresh benchmark).
            If not None, uses `benchmark_old`s semantic configuration values.
    """
    value = eval(expr, globals(), locals_ or locals())
    if isinstance(value, tuple) and isinstance(value[0], MultibodyPlant):
        adaptor = MbpAdaptor(*value)
    else:
        raise RuntimeError("Cannot handle {}".format(value))
    benchmark = Benchmark(expr, type(value).__name__)
    if benchmark_old is None:
        add_new_configurations(benchmark, adaptor)
    else:
        add_old_configurations(benchmark, adaptor, benchmark_old)
    if return_adaptor:
        return benchmark, adaptor
    else:
        return benchmark


class MbpAdaptor(Adaptor):
    _acceptable_joints = (
        PrismaticJoint,
        RevoluteJoint,
    )

    def __init__(self, plant, scene_graph, diagram):
        self.plant = plant
        self.scene_graph = scene_graph
        self.diagram = diagram
        # Assumes single DOF per Joint.
        joints = []
        for i in range(plant.num_joints()):
            joint = plant.get_joint(JointIndex(i))
            assert joint.name(), joint.name()
            if joint.num_positions() == 0:
                continue
            assert joint.num_positions() == 1, (joint, joint.name())
            assert isinstance(joint, self._acceptable_joints)
            joints.append(joint)
        self._joint_map = {}
        self._joint_names = []
        joints = sorted(joints, key=lambda x: x.position_start())
        for joint in joints:
            self._joint_names.append(joint.name())
            self._joint_map[joint.name()] = joint
        # Get frames.
        nf = plant.num_frames()
        self._frames = []
        for i in range(nf):
            frame = plant.get_frame(FrameIndex(i))
            if frame.name() and not frame.name().startswith("_"):
                self._frames.append(frame)
        assert len(self._frames) > 0

    @property
    def position_names(self):
        return self._joint_names

    def _do_calc_frames(self, q_raw):
        context = self.plant.CreateDefaultContext()
        for name_i, q_i in zip(self._joint_names, q_raw):
            joint = self._joint_map[name_i]
            # Is there a better way than this?
            if isinstance(joint, RevoluteJoint):
                joint.set_angle(context, q_i)
            elif isinstance(joint, PrismaticJoint):
                joint.set_translation(context, q_i)
        config_frames = []
        for frame in self._frames:
            X_WF = self.plant.CalcRelativeTransform(
                context, self.plant.world_frame(), frame)
            instance = frame.model_instance()
            if instance != self.plant.world_frame().model_instance():
                name = "{}::{}".format(
                    self.plant.GetModelInstanceName(instance), frame.name())
            else:
                name = frame.name()
            config_frames.append(Frame(name, X_WF))
        return config_frames


def make_plant(model_directive_path):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder)
    InsertCorrectLoadingMethodHere(model_directive_path, plant)
    plant.Finalize()
    ConnectDrakeVisualizer(
        builder=builder, scene_graph=scene_graph)
    diagram = builder.Build()
    return plant, scene_graph, diagram


# Re-do RBT stuff


def simple_mbp():
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder)
    Parser(plant).AddModelFromFile(
        "common/test/models/double_pendulum.sdf")
    plant.Finalize()
    ConnectDrakeVisualizer(
        builder=builder, scene_graph=scene_graph)
    diagram = builder.Build()
    return plant, scene_graph, diagram


def main():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command")
    create = subparsers.add_parser("create")
    create.add_argument("--output", type=str, required=True)
    create.add_argument("--expr", type=str, required=True)
    create.add_argument(
        "--old_expr", type=str, default=None, help="For debugging")
    args = parser.parse_args()

    if args.command == "create":
        benchmark_old = None
        if args.old_expr:
            benchmark_old = create_benchmark(args.old_expr)
        benchmark = create_benchmark(args.expr, benchmark_old)
        benchmark.save_yaml(args.output)
    else:
        exit(1)


if __name__ == "__main__":
    main()
