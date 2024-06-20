from contextlib import contextmanager
import unittest

import numpy as np

from pydrake.geometry import DrakeVisualizer, DrakeVisualizerParams, Role
from pydrake.multibody.math import SpatialVelocity
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import (
    AddMultibodyPlant,
    ExternallyAppliedSpatialForce,
    MultibodyPlantConfig,
)
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import DiagramBuilder, EventStatus


class TestBase(unittest.TestCase):
    @contextmanager
    def assert_raises_message(self, pieces, cls=RuntimeError):
        with self.assertRaises(cls) as cm:
            yield
        for piece in pieces:
            self.assertIn(piece, str(cm.exception))


def add_basic_simulation_components(ns, time_step, solver=None):
    ns.builder = DiagramBuilder()
    config = MultibodyPlantConfig(time_step=time_step)
    if solver is not None:
        config.discrete_contact_solver = solver
    else:
        assert time_step == 0.0
    ns.plant, ns.scene_graph = AddMultibodyPlant(config, ns.builder)
    ns.parser = Parser(ns.plant)
    # Disable gravity so that we do not need to worry about gravity feedforward
    # control terms.
    ns.plant.mutable_gravity_field().set_gravity_vector([0.0, 0.0, 0.0])


def total_energy(plant, context):
    return plant.EvalKineticEnergy(context) + plant.EvalPotentialEnergy(
        context
    )


def monitor_large_energy_delta(simulator, t, plant, max_energy_gain):
    """
    Assumes simulator currently has desired intial state.

    This will simulate the system forward to time t, and check if the
    max_energy_gain (change from initial energy) is ever exceeded. If so, the
    maximum violation is recorded and an exception is thrown.
    """
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


def lstsq(A, b):
    return np.linalg.lstsq(A, b, rcond=None)[0]


def norm(x):
    return np.linalg.norm(x)
