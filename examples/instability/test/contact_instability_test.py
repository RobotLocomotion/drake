"""
Shows simple box collision going unstable based on time step.
"""

from contextlib import contextmanager
from types import SimpleNamespace
import unittest

import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.geometry import DrakeVisualizer, DrakeVisualizerParams, Role
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.math import SpatialVelocity
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlant, MultibodyPlantConfig
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, EventStatus

VISUALIZE = True


def add_basic_simulation_components(ns, time_step, solver=None):
    ns.builder = DiagramBuilder()
    config = MultibodyPlantConfig(time_step=time_step)
    if solver is not None:
        config.discrete_contact_solver = solver
    else:
        assert time_step == 0.0
    ns.plant, ns.scene_graph = AddMultibodyPlant(config, ns.builder)
    ns.parser = Parser(ns.plant)
    DrakeVisualizer.AddToBuilder(
        ns.builder,
        ns.scene_graph,
        params=DrakeVisualizerParams(role=Role.kIllustration),
    )
    ns.plant.mutable_gravity_field().set_gravity_vector([0.0, 0.0, 0.0])


def add_floating_contact_sim(ns):
    a_file = FindResourceOrThrow(
        "drake/examples/instability/test/contact_object_a.sdf"
    )
    b_file = FindResourceOrThrow(
        "drake/examples/instability/test/contact_object_b.sdf"
    )
    ns.model_a = ns.parser.AddModelFromFile(a_file, "a")
    ns.model_b = ns.parser.AddModelFromFile(b_file, "b")
    ns.plant.Finalize()
    ns.frame_W = ns.plant.world_frame()
    ns.frame_A = ns.plant.GetFrameByName("__model__", ns.model_a)
    ns.frame_B = ns.plant.GetFrameByName("__model__", ns.model_b)
    return ns


def total_energy(plant, context):
    return plant.EvalKineticEnergy(context) + plant.EvalPotentialEnergy(
        context
    )


def monitor_large_energy_delta(simulator, t, plant, max_energy_gain):
    diagram_context = simulator.get_context()
    context = plant.GetMyContextFromRoot(diagram_context)
    energy_init = total_energy(plant, context)
    max_bad_energy_delta = None

    def monitor(diagram_context):
        nonlocal max_bad_energy_delta
        context = plant.GetMyContextFromRoot(diagram_context)
        energy_now = total_energy(plant, context)
        energy_delta = energy_now - energy_init
        if energy_delta > max_energy_gain:
            if (
                max_bad_energy_delta is None
                or energy_delta > max_bad_energy_delta
            ):
                max_bad_energy_delta = energy_delta
        return EventStatus.DidNothing()

    simulator.set_monitor(monitor)
    simulator.AdvanceTo(t)
    if max_bad_energy_delta is not None:
        raise RuntimeError(f"Too much energy gained: {max_bad_energy_delta} J")


class Test(unittest.TestCase):
    @contextmanager
    def assert_raises_message(self, pieces, cls=RuntimeError):
        with self.assertRaises(cls) as cm:
            yield
        for piece in pieces:
            self.assertIn(piece, str(cm.exception))

    def run_floating_contact(
        self, time_step, *, solver=None, max_energy_gain,
    ):
        print((time_step, solver))
        ns = SimpleNamespace()
        add_basic_simulation_components(ns, time_step, solver)
        add_floating_contact_sim(ns)

        diagram = ns.builder.Build()
        diagram_context = diagram.CreateDefaultContext()
        context = ns.plant.GetMyContextFromRoot(diagram_context)

        X_WA0 = RigidTransform([0, -0.18, 0])
        X_WB0 = RigidTransform(
            rpy=RollPitchYaw(np.deg2rad([0, 0, -90.0])), p=[0, 0.0, 0],
        )
        ns.plant.SetFreeBodyPose(context, ns.frame_A.body(), X_WA0)
        ns.plant.SetFreeBodyPose(context, ns.frame_B.body(), X_WB0)

        V_WA0 = SpatialVelocity(w=[0, 0, 0], v=[0, 0.1, 0.0])
        ns.plant.SetFreeBodySpatialVelocity(ns.frame_A.body(), V_WA0, context)

        simulator = Simulator(diagram, diagram_context)
        if VISUALIZE:
            simulator.set_target_realtime_rate(1.0)
        t_max = 1.0
        monitor_large_energy_delta(simulator, t_max, ns.plant, max_energy_gain)

    def test_floating_contact(self):
        time_step_continuous = 0.0
        time_step_unstable = 0.0005
        time_step_stable = 0.0004

        self.run_floating_contact(
            time_step_continuous, max_energy_gain=0.0,
        )
        with self.assert_raises_message(["Too much energy gained"]):
            self.run_floating_contact(
                time_step_unstable, solver="sap", max_energy_gain=0.15,
            )
        self.run_floating_contact(
            time_step_stable, solver="sap", max_energy_gain=0.0,
        )
