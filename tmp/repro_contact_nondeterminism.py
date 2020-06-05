"""
Captures non-determinism with MBP and (MBP, SG) with (unfortunately) complex
models.

Example run:

    bazel run //tmp:repro_contact_nondeterminism

Then look at results:

    $EDITOR ./tmp/repro_contact_nondeterminism.py.*.txt

Tests the following:
- MBP w/ IIWA, either Continuous or Discrete
- W/ and w/o SceneGraph (geometry)
- W/ and w/o Anzu WSG or Drake WSG
- Different re-simulation styles:
  - reuse: Clone initial context, then use Context.SetTimeStateAndParametersFrom
    to restore initial conditions.
  - recreate: Recreate everything from scratch (but in same order)
- In a given setup, will resimulate for M meta-trials x S sim-trials.
  - Each sim is run for 1s, with an output recorded every 1ms.
    - Geometry:
        - No: Output is str version of MBP position vector, q
          (easy to check for equality)
        - With: Output is q and contact results from (MBP, SG)
    - All floats are printed with 4 digits of precision.
    - For each time t, the value is recorded in a string, and added to the
      trial's "Frames" for the sim.
    - After each trial, the Frames are checked against a set of all frames for
      the simulation trials within the setup. Unique is BAD (unless the set is
      empty). Non-unique is good.
      - If it's the first trial, good.
      - If the trial frames are non-unique, that's good.
      - If the trial frames are unique, that's BAD. Non-deterministic.
  - After the sim trials are done, results are aggregated, and included in
    summary.
"""

from collections import namedtuple
from difflib import Differ
from enum import Enum
import glob
import os
import sys
from textwrap import dedent, indent

import numpy as np

# TODO(eric): Meh.
from pydrake.all import (
    Role, ResetIntegratorFromFlags, BodyIndex, MultibodyPlant,
    set_log_level)
from pydrake.common import FindResourceOrThrow
from pydrake.geometry import ConnectDrakeVisualizer
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph,
    ConnectContactResultsToDrakeVisualizer,
)
from pydrake.systems.analysis import Simulator
from pydrake.systems.lcm import LcmInterfaceSystem
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import ConstantVectorSource

VISUALIZE = False


def rpy_deg(rpy_deg):
    return RollPitchYaw(np.deg2rad(rpy_deg))


def no_control(builder, plant, model):
    nu = plant.num_actuated_dofs(model)
    constant = builder.AddSystem(ConstantVectorSource(np.zeros(nu)))
    builder.Connect(
        constant.get_output_port(0),
        plant.get_actuation_input_port(model))


def contact_results_to_str(plant, contact_results, prec=10):
    out = ""
    count = contact_results.num_point_pair_contacts()
    if count == 0:
        return "<no contacts>"
    infos = [contact_results.point_pair_contact_info(i) for i in range(count)]
    # TODO(eric.cousineau): Are these contact pairs being out of order an
    # issue?
    infos.sort(key=lambda x: (x.bodyA_index(), x.bodyB_index()))
    for i, info in enumerate(infos):
        bodyA = plant.get_body(BodyIndex(info.bodyA_index()))
        bodyB = plant.get_body(BodyIndex(info.bodyB_index()))
        # N.B. Use np.array() always to ensure we use the same formatting for
        # floating-point numbers.
        out += dedent(f"""\
        point_pair_contact_info({i}):
          bodyA: {bodyA.name()}
          bodyB: {bodyB.name()}
          contact_force: {np.asarray(info.contact_force())}
          contact_point: {np.asarray(info.contact_point())}
          slip_speed: {np.asarray([info.slip_speed()])}
          separation_speed: {np.asarray([info.separation_speed()])}
        """.rstrip())
        out += "\n"
    return out.rstrip()


"""Defines simulation setup."""
Setup = namedtuple(
    "Setup",
    (
        "plant_time_step",
        "has_geometry",
        "gripper",
    ),
)


class SetupEnum(Enum):
    """Different setups."""
    Continuous_NoGeometry = Setup(
        plant_time_step=0., has_geometry=False, gripper=None)
    Discrete_NoGeometry = Setup(
        plant_time_step=0.001, has_geometry=False, gripper=None)
    Continuous_WithGeometry_NoGripper = Setup(
        plant_time_step=0., has_geometry=True, gripper=None)
    Discrete_WithGeometry_NoGripper = Setup(
        plant_time_step=0.001, has_geometry=True, gripper=None)
    # Grippers.
    Discrete_WithGeometry_AnzuWsg = Setup(
        plant_time_step=0., has_geometry=True,
        gripper="drake/tmp/schunk_wsg_50_anzu.sdf")
    Discrete_WithGeometry_AnzuWsgWelded = Setup(
        plant_time_step=0.001, has_geometry=True,
        gripper="drake/tmp/schunk_wsg_50_anzu_welded.sdf")
    Discrete_WithGeometry_DrakeWsg = Setup(
        plant_time_step=0.001, has_geometry=True,
        gripper=(
            "drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50.sdf"))
    Discrete_WithGeometry_DrakeWsgWelded = Setup(
        plant_time_step=0.001, has_geometry=True,
        gripper="drake/tmp/schunk_wsg_50_drake_welded.sdf")

    def __repr__(self):
        return self.name


def make_simulator(setup):
    setup = setup.value
    max_step_size = 0.01

    builder = DiagramBuilder()
    if setup.has_geometry:
        plant, scene_graph = AddMultibodyPlantSceneGraph(
            builder, time_step=setup.plant_time_step)
    else:
        plant = builder.AddSystem(MultibodyPlant(
            time_step=setup.plant_time_step))
        scene_graph = None
    parser = Parser(plant)

    control_models = []

    iiwa = parser.AddModelFromFile(FindResourceOrThrow(
        "drake/manipulation/models/iiwa_description/urdf/"
        "iiwa14_spheres_dense_elbow_collision.urdf"))
    plant.WeldFrames(
        plant.world_frame(), plant.GetFrameByName("base"),
        RigidTransform([0.6, 0., 0.]))
    control_models.append(iiwa)

    if setup.gripper is None:
        wsg = None
    else:
        if "anzu" in setup.gripper:
            X_7G = RigidTransform(rpy_deg([180., 0., 158.]), [0, 0, 0.053])
            body = "gripper"
        else:
            X_7G = RigidTransform(rpy_deg([90., 0., 90.]), [0, 0, 0.114])
            body = "body"
        wsg = parser.AddModelFromFile(FindResourceOrThrow(setup.gripper))
        plant.WeldFrames(
            plant.GetFrameByName("iiwa_link_7"),
            plant.GetFrameByName(body),
            X_7G)
        control_models.append(wsg)

    table = parser.AddModelFromFile(FindResourceOrThrow("drake/tmp/fake_table.sdf"))
    plant.WeldFrames(
        plant.world_frame(), plant.GetFrameByName("fake_table"),
        RigidTransform([0, 0, -0.5]))
    plant.Finalize()

    no_control(builder, plant, iiwa)
    if wsg is not None:
        no_control(builder, plant, wsg)

    if VISUALIZE and scene_graph:
        role = Role.kProximity
        ConnectDrakeVisualizer(builder, scene_graph, role=role)
        ConnectContactResultsToDrakeVisualizer(builder, plant)

    diagram = builder.Build()

    d_context = diagram.CreateDefaultContext()
    context = plant.GetMyContextFromRoot(d_context)
    q0_iiwa = [0.02, 0.085, -0.285, 1.43, 0.284, -1.07, 0.13]
    plant.SetPositions(context, iiwa, q0_iiwa)
    if wsg is not None and plant.num_positions(wsg) > 0:
        q0_wsg = [-0.05, 0.05]
        plant.SetPositions(context, wsg, q0_wsg)
    contact_results_port = plant.get_contact_results_output_port()

    def calc_output(d_context):
        context = plant.GetMyContextFromRoot(d_context)
        q = plant.GetPositions(context)
        if setup == SetupEnum.Discrete_NoGeometry:
            return f"q: {q}"
        else:
            contact_results = contact_results_to_str(
                plant, contact_results_port.Eval(context))
            return f"q: {q}\ncontact_results: {indent(contact_results, '  ')}"

    simulator = Simulator(diagram, d_context)
    if VISUALIZE and scene_graph:
        simulator.set_target_realtime_rate(1.)
    ResetIntegratorFromFlags(simulator, "runge_kutta2", max_step_size)
    return simulator, calc_output


def text_diff(a, b):
    diff = Differ()
    a = f"{a.rstrip()}\n".splitlines(keepends=True)
    b = f"{b.rstrip()}\n".splitlines(keepends=True)
    if len(a) == len(b):
        # Show lines directly interleaved.
        out = []
        for ai, bi in zip(a, b):
            out += diff.compare([ai], [bi])
        return "".join(out)
    else:
        # Show diff w/ chunks.
        # TODO(eric.cousineau): How to show full text, but keep context chunks
        # small?
        return("".join(diff.compare(a, b)))


class Frames:
    """Records a set of text-diff'able values at a known set of times.

    Can report the first time there is divergence w/ another Frames object,
    as well as show the diff between first diff time and last time in sim."""
    def __init__(self):
        self._times = []
        self._values = []

    def add(self, t, value):
        index = len(self._times)
        self._times.append(t)
        self._values.append(value)
        return index

    def __len__(self):
        return len(self._times)

    def __getitem__(self, i):
        return self._times[i], self._values[i]

    def first_diff(self, other):
        # Return index for both items at first difference, or None.
        assert len(self) == len(other)
        for i in range(len(self)):
            t, value = self[i]
            t_other, value_other = other[i]
            assert t == t_other, (t, t_other)
            if value != value_other:
                return i
        return None

    def __eq__(self, other):
        return self.first_diff(other) is None

    def __hash__(self):
        return hash((tuple(self._times), tuple(self._values)))

    def __str__(self):
        # Abbrev. to first and last.
        lines = []
        assert len(self) > 1
        times = [self._times[0], self._times[-1]]
        values = [self._values[0], self._values[-1]]
        for t, value in zip(times, values):
            lines += [
                f"t: {float_fmt(t)}",
                indent(value, '  '),
            ]
        return "\n".join(lines)

    def text_diff(self, other):
        index = self.first_diff(other)
        assert index is not None, f"\nself:\n{self}\n\nother:\n{other}"
        t, value = self[index]
        _, value_other = other[index]
        lines = [
            f"diff[0], t: {float_fmt(t)}",
            indent(text_diff(value, value_other), '  '),
        ]
        t, value = self[-1]
        _, value_other = other[-1]
        lines += [
            f"diff[-1], t: {float_fmt(t)}",
            indent(text_diff(value, value_other), '  '),
        ]
        return "\n".join(lines)


class SimulationChecker:
    """
    Collects simulation results to then diff across:
    - previous runs of the simulation
    - along each previous simulation's time t

    This uses a set to record the stuff.
    """
    def __init__(self):
        self._frames_baseline = None
        self._frames_set = set()

    def run(self, simulator, calc_output):
        dt = 0.001
        end_time = 1.

        prefix = '    '
        # TODO(eric.cousineau): Use monitor... somehow?
        d_context = simulator.get_context()

        frames = Frames()
        while d_context.get_time() + dt / 2 < end_time:
            simulator.AdvanceTo(d_context.get_time() + dt)
            output = calc_output(d_context)
            frames.add(d_context.get_time(), output)

        prev_count = len(self._frames_set)
        self._frames_set.add(frames)
        count = len(self._frames_set)
        if self._frames_baseline is None:
            print(f"{prefix}good (first)")
            assert count == 1
            self._frames_baseline = frames
        elif count == prev_count:
            print(f"{prefix}good (not unique)")
        else:
            print(f"{prefix}BAD")
            print(indent(
                self._frames_baseline.text_diff(frames), prefix + '  '))
        self._frames_set.add(frames)

    def summary(self):
        count = len(self._frames_set)
        assert count > 0
        if count == 1:
            return f"good (num_unique = 1)"
        else:
            return f"BAD  (num_unique = {count})"


def run_simulations(num_sim_trials, with_wsg):
    print("resimulate_mode: reuse")
    simulator, calc_output = make_simulator(with_wsg)
    simulator.Initialize()
    d_context = simulator.get_mutable_context()
    d_context_initial = d_context.Clone()
    reuse = SimulationChecker()
    for index in range(num_sim_trials):
        # Run multiple simulations, resetting the context.
        print(f"  index: {index}")
        d_context.SetTimeStateAndParametersFrom(d_context_initial)
        reuse.run(simulator, calc_output)

    print()
    print("resimulate_mode: recreate")
    recreate = SimulationChecker()
    for index in range(num_sim_trials):
        # Run multiple simulations, recreating from scratch each time.
        print(f"  index: {index}")
        simulator, calc_output = make_simulator(with_wsg)
        recreate.run(simulator, calc_output)

    return "\n".join([
        f"reuse:    {reuse.summary()}",
        f"recreate: {recreate.summary()}",
    ])


def reexecute_if_unbuffered():
    """Ensures that output is immediately flushed (e.g. for segfaults).
    ONLY use this at your entrypoint. Otherwise, you may have code be
    re-executed that will clutter your console."""
    import os
    import shlex
    import sys
    if os.environ.get("PYTHONUNBUFFERED") in (None, ""):
        os.environ["PYTHONUNBUFFERED"] = "1"
        argv = list(sys.argv)
        if argv[0] != sys.executable:
            argv.insert(0, sys.executable)
        cmd = " ".join([shlex.quote(arg) for arg in argv])
        sys.stdout.flush()
        # Redirect stderr to stdout?
        os.execv(argv[0], argv)


def main():
    # To see if our determinism check is repeatable... (or whatever the right
    # wording is ;)
    num_meta_trials = 2
    num_sim_trials = 4
    tally = []
    for setup in SetupEnum:
        da_printer.redirect(f"{base_file}.{setup}.txt")
        tally.append(f"* setup = {setup}")
        # Repeat same number of trials.
        for meta_trial in range(num_meta_trials):
            print(
                f"\n\n[ meta_trial = {meta_trial}, "
                f"num_sim_trials = {num_sim_trials}, setup = {setup} ]\n")
            summary = run_simulations(num_sim_trials, setup)
            tally.append(
                f"  * meta_trial = {meta_trial}, "
                f"num_sim_trials = {num_sim_trials}")
            tally.append(indent(summary, '      '))
        tally.append("")
    text = "\n".join(tally).rstrip()
    print()
    da_printer.redirect(f"{base_file}.summary.txt")
    print(text)


def float_fmt(x):
    return f"{x:.4g}"


print_ = print


class DaPrinter:
    def __init__(self):
        self._f = None

    def close(self):
        if self._f is not None:
            self._f.close()
            self._f = None

    def redirect(self, file):
        self.close()
        print(f"redirect: {file}")
        self._f = open(file, "w")

    def print(self, x=""):
        print_(x)
        if self._f is not None:
            self._f.write(x + "\n")

    def __enter__(self):
        pass

    def __exit__(self, *args):
        self.close()


if __name__ == "__main__":
    reexecute_if_unbuffered()

    # HACK: Break through `bazel run` sandbox.
    base_file = os.path.realpath(__file__)
    for x in glob.glob(f"{base_file}.*.txt"):
        print(f"remove: {x}")
        os.unlink(x)

    da_printer = DaPrinter()
    print = da_printer.print
    with da_printer:
        np.set_printoptions(formatter={'all': float_fmt})
        # Suppress warnings about joint limits plz.
        set_log_level("err")
        if "--visualize" in sys.argv:
            VISUALIZE = True
            sys.argv.remove("--visualize")
        if "--test" in sys.argv:
            run_simulations(4, SetupEnum.Discrete_WithGeometry_DrakeWsgWelded)
        else:
            main()
