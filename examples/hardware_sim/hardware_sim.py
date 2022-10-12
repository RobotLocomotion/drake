"""
This program serves as an example of a simulator for hardware, i.e., a
simulator for robots that one might have in their lab. There is no controller
built-in to this program -- it merely sends status and sensor messages, and
listens for command messages.

It is intended to operate in the "no ground truth" regime, i.e, the only LCM
messages it knows about are the ones used by the actual hardware. The one
messaging difference from real life is that we emit visualization messages (for
meldis or drake-visualizer) so that you can watch a simulation on your screen
while some (separate) controller operates the robot, without extra hassle.

Drake maintainers should keep this file in sync with both hardware_sim.cc and
scenario.h.
"""

import dataclasses as dc
import math
import typing

from pydrake.common import RandomGenerator
from pydrake.lcm import DrakeLcmParams
from pydrake.manipulation import ApplyDriverConfigs
from pydrake.manipulation.kuka_iiwa import IiwaDriver
from pydrake.manipulation.schunk_wsg import SchunkWsgDriver
from pydrake.manipulation.util import ZeroForceDriver
from pydrake.multibody.plant import (
    AddMultibodyPlant,
    MultibodyPlantConfig,
)
from pydrake.multibody.parsing import (
    ModelDirective,
    ModelDirectives,
    ProcessModelDirectives,
)
from pydrake.systems.analysis import (
    ApplySimulatorConfig,
    Simulator,
    SimulatorConfig,
)
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.lcm import ApplyLcmBusConfig
from pydrake.systems.sensors import (
    ApplyCameraConfig,
    CameraConfig,
)
from pydrake.visualization import (
    ApplyVisualizationConfig,
    VisualizationConfig,
)


@dc.dataclass
class Scenario:
    """Defines the YAML format for a (possibly stochastic) scenario to be
    simulated.
    """

    # Random seed for any random elements in the scenario. The seed is always
    # deterministic in the `Scenario`; a caller who wants randomness must
    # populate this value from their own randomness.
    random_seed: int = 0

    # The maximum simulation time (in seconds).  The simulator will attempt to
    # run until this time and then terminate.
    simulation_duration: float = math.inf

    # Simulator configuration (integrator and publisher parameters).
    simulator_config: SimulatorConfig = SimulatorConfig(
        max_step_size=1e-3,
        accuracy=1.0e-2,
        target_realtime_rate=1.0)

    # Plant configuration (timestep and contact parameters).
    plant_config: MultibodyPlantConfig = MultibodyPlantConfig()

    # All of the fully deterministic elements of the simulation.
    directives: typing.List[ModelDirective] = dc.field(default_factory=list)

    # A map of {bus_name: lcm_params} for LCM transceivers to be used by
    # drivers, sensors, etc.
    lcm_buses: typing.Mapping[str, DrakeLcmParams] = dc.field(
        default_factory=lambda: dict(default=DrakeLcmParams()))

    # For actuated models, specifies where each model's actuation inputs come
    # from, keyed on the ModelInstance name.
    model_drivers: typing.Mapping[str, typing.Union[
        IiwaDriver,
        SchunkWsgDriver,
        ZeroForceDriver,
    ]] = dc.field(default_factory=dict)

    # Cameras to add to the scene (and broadcast over LCM). The key for each
    # camera is a helpful mnemonic, but does not serve a technical role. The
    # CameraConfig::name field is still the name that will appear in the
    # Diagram artifacts.
    cameras: typing.Mapping[str, CameraConfig] = dc.field(default_factory=dict)

    visualization: VisualizationConfig = VisualizationConfig()


def run(*, scenario):
    """Runs a simulation of the given scenario.
    """
    builder = DiagramBuilder()

    # Create the multibody plant and scene graph.
    sim_plant, scene_graph = AddMultibodyPlant(
        config=scenario.plant_config,
        builder=builder)

    # Add model directives.
    added_models = ProcessModelDirectives(
        directives=ModelDirectives(directives=scenario.directives),
        plant=sim_plant)

    # Now the plant is complete.
    sim_plant.Finalize()

    # Add LCM buses. (The simulator will handle polling the network for new
    # messages and dispatching them to the receivers, i.e., "pump" the bus.)
    lcm_buses = ApplyLcmBusConfig(
        lcm_buses=scenario.lcm_buses,
        builder=builder)

    # Add actuation inputs.
    ApplyDriverConfigs(
        driver_configs=scenario.model_drivers,
        sim_plant=sim_plant,
        models_from_directives=added_models,
        lcm_buses=lcm_buses,
        builder=builder)

    # Add scene cameras.
    camera_lcm = lcm_buses.Find("Cameras", "default")
    for _, camera in scenario.cameras.items():
        ApplyCameraConfig(
            config=camera,
            plant=sim_plant,
            builder=builder,
            scene_graph=scene_graph,
            lcm=camera_lcm)

    # Add visualization.
    ApplyVisualizationConfig(scenario.visualization, builder, lcm_buses)

    # Build the diagram and its simulator.
    diagram = builder.Build()
    simulator = Simulator(diagram)
    ApplySimulatorConfig(scenario.simulator_config, simulator)

    # Sample the random elements of the context.
    random = RandomGenerator(scenario.random_seed)
    diagram.SetRandomContext(simulator.get_mutable_context(), random)

    # Simulate.
    simulator.AdvanceTo(scenario.simulation_duration)


# TODO(jwnimmer-tri) Add __main__ program here as well.
