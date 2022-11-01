"""
Shows simulated controllers going unstable.
"""

from contextlib import contextmanager
import dataclasses as dc
from types import SimpleNamespace
import unittest

import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.common.cpp_param import List
from pydrake.common.value import Value
from pydrake.geometry import DrakeVisualizer, DrakeVisualizerParams, Role
from pydrake.multibody.math import SpatialForce, SpatialVelocity
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import (
    AddMultibodyPlant,
    ExternallyAppliedSpatialForce,
    MultibodyPlantConfig,
)
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, EventStatus, LeafSystem

VISUALIZE = False


def simple_jacobian(plant, context, frame_W, frame_F):
    Jv_WF = plant.CalcJacobianSpatialVelocity(
        context,
        with_respect_to=JacobianWrtVariable.kV,
        frame_B=frame_F,
        p_BP=[0, 0, 0],
        frame_A=frame_W,
        frame_E=frame_W,
    )
    return Jv_WF


def get_frame_spatial_velocity(plant, context, frame_W, frame_F):
    Jv_WF = simple_jacobian(plant, context, frame_W, frame_F)
    v = plant.GetVelocities(context)
    V_WF = SpatialVelocity(Jv_WF @ v)
    return V_WF


def make_force_for_frame(frame_F, F_F_W):
    external_force = ExternallyAppliedSpatialForce()
    external_force.body_index = frame_F.body().index()
    external_force.F_Bq_W = F_F_W
    external_force.p_BoBq_B = frame_F.GetFixedPoseInBodyFrame().translation()
    return external_force


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


def lstsq(A, b):
    return np.linalg.lstsq(A, b, rcond=None)[0]


def norm(x):
    return np.linalg.norm(x)


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
