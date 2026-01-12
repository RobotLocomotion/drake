# This file is licensed under the MIT-0 License.
# See LICENSE-MIT-0.txt in the current directory.

"""
This program serves as an example of a simulator for hardware, i.e., a
simulator for robots that one might have in their lab. There is no controller
built-in to this program -- it merely sends status and sensor messages, and
listens for command messages.

It is intended to operate in the "no ground truth" regime, i.e, the only LCM
messages it knows about are the ones used by the actual hardware. The one
messaging difference from real life is that we emit visualization messages (for
Meldis) so that you can watch a simulation on your screen while some (separate)
controller operates the robot, without extra hassle.

Drake maintainers should keep this file in sync with both hardware_sim.cc and
scenario.h.
"""

import argparse
import dataclasses as dc
import math
import typing

import numpy as np

from pydrake.common import RandomGenerator
from pydrake.common.yaml import yaml_load_typed
from pydrake.geometry import SceneGraphConfig
from pydrake.lcm import DrakeLcmParams
from pydrake.manipulation import (
    ApplyDriverConfigs,
    ApplyNamedPositionsAsDefaults,
    IiwaDriver,
    SchunkWsgDriver,
    ZeroForceDriver,
)
from pydrake.multibody.parsing import (
    ModelDirective,
    ModelDirectives,
    ProcessModelDirectives,
)
from pydrake.multibody.plant import (
    AddMultibodyPlant,
    MultibodyPlantConfig,
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
        max_step_size=1e-3, accuracy=1.0e-2, target_realtime_rate=1.0
    )

    # Plant configuration (time step and contact parameters).
    plant_config: MultibodyPlantConfig = MultibodyPlantConfig()

    # SceneGraph configuration.
    scene_graph_config: SceneGraphConfig = SceneGraphConfig()

    # All of the fully deterministic elements of the simulation.
    directives: typing.List[ModelDirective] = dc.field(default_factory=list)

    # A map of {bus_name: lcm_params} for LCM transceivers to be used by
    # drivers, sensors, etc.
    lcm_buses: typing.Mapping[str, DrakeLcmParams] = dc.field(
        default_factory=lambda: dict(default=DrakeLcmParams())
    )

    # For actuated models, specifies where each model's actuation inputs come
    # from, keyed on the ModelInstance name.
    model_drivers: typing.Mapping[
        str,
        typing.Union[
            IiwaDriver,
            SchunkWsgDriver,
            ZeroForceDriver,
        ],
    ] = dc.field(default_factory=dict)

    # Cameras to add to the scene (and broadcast over LCM). The key for each
    # camera is a helpful mnemonic, but does not serve a technical role. The
    # CameraConfig::name field is still the name that will appear in the
    # Diagram artifacts.
    cameras: typing.Mapping[str, CameraConfig] = dc.field(default_factory=dict)

    visualization: VisualizationConfig = VisualizationConfig()

    # A map-of-maps {model_instance_name: {joint_name: np.ndarray}} that
    # defines the initial state of some joints in the scene. Joints not
    # mentioned will remain in their configurations as applied by the model
    # directives.
    initial_position: dict[
        str,  # model_instance_name ->
        dict[str, np.ndarray],  # joint_name -> positions
    ] = dc.field(default_factory=dict)


def _load_scenario(*, filename, scenario_name, scenario_text):
    """Implements the command-line handling logic for scenario data.
    Returns a `Scenario` object loaded from the given input arguments.
    """
    result = yaml_load_typed(
        schema=Scenario,
        filename=filename,
        child_name=scenario_name,
        defaults=Scenario(),
    )
    result = yaml_load_typed(
        schema=Scenario, data=scenario_text, defaults=result
    )
    return result


def run(*, scenario, graphviz=None):
    """Runs a simulation of the given scenario."""
    builder = DiagramBuilder()

    # Create the multibody plant and scene graph.
    sim_plant, scene_graph = AddMultibodyPlant(
        plant_config=scenario.plant_config,
        scene_graph_config=scenario.scene_graph_config,
        builder=builder,
    )

    # Add model directives.
    added_models = ProcessModelDirectives(
        directives=ModelDirectives(directives=scenario.directives),
        plant=sim_plant,
    )

    # Override or supplement initial positions.
    ApplyNamedPositionsAsDefaults(
        input=scenario.initial_position, plant=sim_plant
    )

    # Now the plant is complete.
    sim_plant.Finalize()

    # Add LCM buses. (The simulator will handle polling the network for new
    # messages and dispatching them to the receivers, i.e., "pump" the bus.)
    lcm_buses = ApplyLcmBusConfig(lcm_buses=scenario.lcm_buses, builder=builder)

    # Add actuation inputs.
    ApplyDriverConfigs(
        driver_configs=scenario.model_drivers,
        sim_plant=sim_plant,
        models_from_directives=added_models,
        lcm_buses=lcm_buses,
        builder=builder,
    )

    # Add scene cameras.
    for _, camera in scenario.cameras.items():
        ApplyCameraConfig(config=camera, builder=builder, lcm_buses=lcm_buses)

    # Add visualization.
    ApplyVisualizationConfig(scenario.visualization, builder, lcm_buses)

    # Build the diagram and its simulator.
    diagram = builder.Build()
    simulator = Simulator(diagram)
    ApplySimulatorConfig(scenario.simulator_config, simulator)

    # Sample the random elements of the context.
    random = RandomGenerator(scenario.random_seed)
    diagram.SetRandomContext(simulator.get_mutable_context(), random)

    # Visualize the diagram, when requested.
    if graphviz is not None:
        with open(graphviz, "w", encoding="utf-8") as f:
            options = {"plant/split": "I/O"}
            f.write(diagram.GetGraphvizString(options=options))

    # Simulate.
    simulator.AdvanceTo(scenario.simulation_duration)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario_file",
        required=True,
        help="Scenario filename, e.g., "
        "drake/examples/hardware_sim/example_scenarios.yaml",
    )
    parser.add_argument(
        "--scenario_name",
        required=True,
        help="Scenario name within the scenario_file, e.g., Demo in the "
        "example_scenarios.yaml; scenario names appears as the keys of "
        "the YAML document's top-level mapping item",
    )
    parser.add_argument(
        "--scenario_text",
        default="{}",
        help="Additional YAML scenario text to load, in order to override "
        "values in the scenario_file, e.g., timeouts",
    )
    parser.add_argument(
        "--graphviz",
        metavar="FILENAME",
        help="Dump the Simulator's Diagram to this file in Graphviz format "
        "as a debugging aid",
    )
    args = parser.parse_args()
    scenario = _load_scenario(
        filename=args.scenario_file,
        scenario_name=args.scenario_name,
        scenario_text=args.scenario_text,
    )
    run(scenario=scenario, graphviz=args.graphviz)


if __name__ == "__main__":
    main()
