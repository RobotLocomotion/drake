"""
Shows simulated controllers going unstable. These are purposeful "over stiff"
controllers. Concretely, these controllers do not attempt to cancel out any
dynamics.
"""

import dataclasses as dc
from types import SimpleNamespace
import unittest

import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.common.cpp_param import List
from pydrake.common.value import Value
from pydrake.multibody.math import SpatialForce, SpatialVelocity
from pydrake.multibody.plant import ExternallyAppliedSpatialForce
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import LeafSystem

from drake.examples.instability.test.instability_common import (
    TestBase,
    add_basic_simulation_components,
    get_frame_spatial_velocity,
    lstsq,
    make_force_for_frame,
    monitor_large_energy_delta,
    simple_jacobian,
)

VISUALIZE = False


def add_floating_sim(ns):
    model_file = FindResourceOrThrow(
        "drake/examples/instability/test/floating_body.sdf"
    )
    ns.model = ns.parser.AddModelFromFile(model_file, "model")
    ns.plant.Finalize()
    ns.frame_W = ns.plant.world_frame()
    ns.frame_F = ns.plant.GetFrameByName("__model__", ns.model)


def add_articulated_sim(ns):
    model_file = FindResourceOrThrow(
        "drake/manipulation/models/franka_description/urdf/panda_arm.urdf"
    )
    ns.model = ns.parser.AddModelFromFile(model_file, "model")
    ns.plant.WeldFrames(
        ns.plant.world_frame(),
        ns.plant.GetFrameByName("panda_link0", ns.model),
    )
    ns.plant.Finalize()
    ns.frame_W = ns.plant.world_frame()
    ns.frame_F = ns.plant.GetFrameByName("panda_link8", ns.model)
    # Bent elbow.
    q0 = np.deg2rad([0.0, 45.0, 0.0, -45.0, 0.0, 90.0, 0.0])
    ns.plant.SetDefaultPositions(ns.model, q0)
    return ns


@dc.dataclass
class RotationalVelocityDamper:
    """Should generally remove energy from the system."""

    kd: float

    def __call__(self, V_WF):
        V_WFdes = SpatialVelocity.Zero()
        V_FFdes_W = V_WFdes - V_WF
        tau = self.kd * V_FFdes_W.rotational()
        F_F_W = SpatialForce(tau=tau, f=np.zeros(3))
        return F_F_W


class FloatingController(LeafSystem):
    """Applies RotationalVelocityDamper to a floating body."""

    def __init__(self, plant, frame_F, damper):
        super().__init__()
        frame_W = plant.world_frame()

        nx = plant.num_positions() + plant.num_velocities()
        context = plant.CreateDefaultContext()

        def control_math():
            V_WF = get_frame_spatial_velocity(plant, context, frame_W, frame_F)
            F_F_W = damper(V_WF)
            external_force = make_force_for_frame(frame_F, F_F_W)
            return external_force

        self.plant_state_input = self.DeclareVectorInputPort("plant_state", nx)

        def control_calc(sys_context, output):
            x = self.plant_state_input.Eval(sys_context)
            plant.SetPositionsAndVelocities(context, x)
            external_force = control_math()
            output.set_value([external_force])

        forces_cls = Value[List[ExternallyAppliedSpatialForce]]
        self.forces_output = self.DeclareAbstractOutputPort(
            "forces_output", alloc=forces_cls, calc=control_calc,
        )
        self._plant = plant

    def AddToBuilder(self, builder):
        plant = self._plant
        builder.AddSystem(self)
        builder.Connect(
            plant.get_state_output_port(), self.plant_state_input,
        )
        builder.Connect(
            self.forces_output, plant.get_applied_spatial_force_input_port(),
        )


class ArticulatedController(LeafSystem):
    """
    Applies RotationalVelocityDamper to an articulated model instance.

    Note: This may be expected to dampen energy; however, since we are
    injecting energy to remove rotation velocity, that energy may go towards
    translation.

    TODO(eric.cousineau): Add translation damping.
    """

    def __init__(self, plant, frame_F, damper):
        super().__init__()
        frame_W = plant.world_frame()
        nx = plant.num_positions() + plant.num_velocities()
        nu = plant.num_actuated_dofs()
        context = plant.CreateDefaultContext()

        def control_math():
            v = plant.GetVelocities(context)
            Jv_WF = simple_jacobian(plant, context, frame_W, frame_F)
            V_WF = SpatialVelocity(Jv_WF @ v)
            F_F_W = damper(V_WF)
            u = Jv_WF.T @ F_F_W.get_coeffs()
            return u

        self.plant_state_input = self.DeclareVectorInputPort("plant_state", nx)

        def control_calc(sys_context, output):
            x = self.plant_state_input.Eval(sys_context)
            plant.SetPositionsAndVelocities(context, x)
            u = control_math()
            output.set_value(u)

        self.torques_output = self.DeclareVectorOutputPort(
            "torques_output", size=nu, calc=control_calc,
        )
        self._plant = plant

    def AddToBuilder(self, builder):
        plant = self._plant
        builder.AddSystem(self)
        builder.Connect(
            plant.get_state_output_port(), self.plant_state_input,
        )
        builder.Connect(
            self.torques_output, plant.get_actuation_input_port(),
        )


class Test(TestBase):
    def run_floating(
        self, time_step, *, kd, solver=None, max_energy_gain,
    ):
        print((time_step, kd, solver))
        ns = SimpleNamespace()
        add_basic_simulation_components(ns, time_step, solver)
        add_floating_sim(ns)
        controller = FloatingController(
            ns.plant, ns.frame_F, RotationalVelocityDamper(kd),
        )
        controller.AddToBuilder(ns.builder)
        diagram = ns.builder.Build()
        diagram_context = diagram.CreateDefaultContext()
        context = ns.plant.GetMyContextFromRoot(diagram_context)
        V0 = SpatialVelocity(w=[0.5, -0.3, 0.2], v=np.zeros(3))
        ns.plant.SetFreeBodySpatialVelocity(ns.frame_F.body(), V0, context)

        simulator = Simulator(diagram, diagram_context)
        if VISUALIZE:
            simulator.set_target_realtime_rate(1.0)
        t_max = 1.0
        if max_energy_gain is not None:
            monitor_large_energy_delta(
                simulator, t_max, ns.plant, max_energy_gain
            )
        else:
            simulator.AdvanceTo(t_max)

    def run_articulated(
        self, time_step, *, kd, solver=None, max_energy_gain,
    ):
        print((time_step, kd, solver))
        ns = SimpleNamespace()
        add_basic_simulation_components(ns, time_step, solver)
        add_articulated_sim(ns)

        controller = ArticulatedController(
            ns.plant, ns.frame_F, RotationalVelocityDamper(kd),
        )
        controller.AddToBuilder(ns.builder)
        diagram = ns.builder.Build()
        diagram_context = diagram.CreateDefaultContext()
        context = ns.plant.GetMyContextFromRoot(diagram_context)

        V0 = SpatialVelocity(w=[0.5, -0.3, 0.2], v=np.zeros(3))
        Jv_WF = simple_jacobian(ns.plant, context, ns.frame_W, ns.frame_F)
        v0 = lstsq(Jv_WF, V0.get_coeffs())
        ns.plant.SetVelocities(context, ns.model, v0)

        simulator = Simulator(diagram, diagram_context)
        if VISUALIZE:
            simulator.set_target_realtime_rate(1.0)
        t_max = 1.0
        if max_energy_gain is not None:
            monitor_large_energy_delta(
                simulator, t_max, ns.plant, max_energy_gain
            )
        else:
            simulator.AdvanceTo(t_max)

    def test_floating(self):
        print("[ Floating ]")
        kd_stable = 0.3
        # kd_stable = 0.0001  # Can see visible evolution.
        kd_unstable = 0.35

        time_step_continuous = 0.0
        time_step_unstable = 0.001
        time_step_stable = 0.0008

        # (kd_stable, time_step_unstable): Things are happy.
        self.run_floating(
            time_step_unstable,
            kd=kd_stable,
            solver="sap",
            max_energy_gain=0.0,
        )
        self.run_floating(
            time_step_unstable,
            kd=kd_stable,
            solver="tamsi",
            max_energy_gain=0.0,
        )
        self.run_floating(
            time_step_continuous, kd=kd_stable, max_energy_gain=0.0,
        )

        # (kd_unstable, time_step_unstable): Makes SAP and TAMSI go unstable.
        with self.assert_raises_message(
            ["Encountered singular articulated body hinge inertia"]
        ):
            self.run_floating(
                time_step_unstable,
                kd=kd_unstable,
                solver="sap",
                # Let error message come through.
                max_energy_gain=None,
            )
        with self.assert_raises_message(["Spatial force", "contains NaN"]):
            self.run_floating(
                time_step_unstable,
                kd=kd_unstable,
                solver="tamsi",
                # Let error message come through.
                max_energy_gain=None,
            )
        # (kd_unstable, time_step_stable): Good.
        self.run_floating(
            time_step_stable,
            kd=kd_unstable,
            solver="sap",
            max_energy_gain=0.0,
        )
        self.run_floating(
            time_step_stable,
            kd=kd_unstable,
            solver="tamsi",
            max_energy_gain=0.0,
        )
        # - Continuous is good (but "stiff").
        self.run_floating(
            time_step_continuous, kd=kd_unstable, max_energy_gain=0.0,
        )

    def test_articulated(self):
        print("[ Articulated ]")
        kd_stable = 1.6
        kd_unstable = 1.7

        time_step_continuous = 0.0
        time_step_unstable = 0.001
        time_step_stable = 0.0008

        # (kd_stable, time_step_unstable): Things are happy.
        self.run_articulated(
            time_step_unstable,
            kd=kd_stable,
            solver="sap",
            max_energy_gain=0.001,
        )
        self.run_articulated(
            time_step_unstable,
            kd=kd_stable,
            solver="tamsi",
            max_energy_gain=0.001,
        )
        self.run_articulated(
            time_step_continuous, kd=kd_stable, max_energy_gain=0.0,
        )

        # kd_unstable: Makes SAP and TAMSI go unstable.
        with self.assert_raises_message(["Too much energy gained"]):
            # N.B. No solver error by default, but goes unstable.
            self.run_articulated(
                time_step_unstable,
                kd=kd_unstable,
                solver="sap",
                max_energy_gain=15059.0,
            )
        with self.assert_raises_message(
            ["MultibodyPlant's discrete update solver failed to converge"]
        ):
            self.run_articulated(
                time_step_unstable,
                kd=kd_unstable,
                solver="tamsi",
                # Let error message come through.
                max_energy_gain=None,
            )
        self.run_articulated(
            time_step_continuous, kd=kd_unstable, max_energy_gain=0.0,
        )

        # Using same gain but decreasing time step makes it stable.
        self.run_articulated(
            time_step_stable,
            kd=kd_unstable,
            solver="sap",
            max_energy_gain=0.0,
        )
        self.run_articulated(
            time_step_stable,
            kd=kd_unstable,
            solver="tamsi",
            max_energy_gain=0.0,
        )
