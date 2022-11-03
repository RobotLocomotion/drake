"""
Shows simple box collision going unstable based on time step.
"""

from types import SimpleNamespace
import unittest

import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.math import SpatialVelocity
from pydrake.systems.analysis import Simulator

from drake.examples.instability.test.instability_common import (
    TestBase,
    add_basic_simulation_components,
    monitor_large_energy_delta,
)

VISUALIZE = False


def add_floating_contact_sim(ns):
    # "a" and "b" are objects that are representative of one version of the
    # haptic simulation where contact instability occurred (anzu#9395).
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


class Test(TestBase):
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
