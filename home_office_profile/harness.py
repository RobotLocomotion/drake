#!/usr/bin/env python3
"""
Standalone Drake benchmark of a cluttered home-office scene (instance
node_88dda9bd): furniture welded to the world and 24 free manipulands
resting on a desk under gravity, ~90 contacts at steady state. Intended as
a contact-cost profiling target; imports nothing outside pydrake.

Physics configuration: 10 ms discrete steps, hydroelastic_with_fallback
contact, all other MultibodyPlantConfig fields at their defaults (in
particular the default "lagged" SAP-family discrete contact approximation).

Run:  python3 harness.py [--duration 60] [--time_step 0.01]
"""

import argparse
import time
from pathlib import Path

from pydrake.geometry import Role
from pydrake.multibody.parsing import (
    LoadModelDirectives,
    Parser,
    ProcessModelDirectives,
)
from pydrake.multibody.plant import AddMultibodyPlant, MultibodyPlantConfig
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder

HERE = Path(__file__).resolve().parent
PACKAGES = HERE / "packages"
DMD = (
    PACKAGES
    / "scenesmith_scenes/home_office_desk_tidy/instances/node_88dda9bd.dmd.yaml"
)


def main():
    argp = argparse.ArgumentParser(description=__doc__)
    argp.add_argument("--duration", type=float, default=60.0)
    argp.add_argument("--time_step", type=float, default=0.01)
    args = argp.parse_args()

    config = MultibodyPlantConfig(
        time_step=args.time_step,
        contact_model="hydroelastic_with_fallback",
    )
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlant(config, builder)
    parser = Parser(plant)
    for pkg in ["scenesmith_scenes", "scenesmith_models"]:
        parser.package_map().Add(pkg, str(PACKAGES / pkg))
    ProcessModelDirectives(LoadModelDirectives(str(DMD)), plant, parser)
    plant.Finalize()
    diagram = builder.Build()

    inspector = scene_graph.model_inspector()
    print(
        f"nq={plant.num_positions()} nv={plant.num_velocities()} "
        f"bodies={plant.num_bodies()} "
        f"proximity_geoms={inspector.NumGeometriesWithRole(Role.kProximity)}"
    )

    simulator = Simulator(diagram)
    simulator.Initialize()

    t_wall_start = time.perf_counter()
    chunk = 5.0
    t = 0.0
    while t < args.duration:
        t = min(t + chunk, args.duration)
        t0 = time.perf_counter()
        simulator.AdvanceTo(t)
        dt_wall = time.perf_counter() - t0
        print(
            f"sim t={t:6.1f} s  chunk wall={dt_wall:6.2f} s  "
            f"rtr={chunk / dt_wall:5.2f}x",
            flush=True,
        )
    wall = time.perf_counter() - t_wall_start

    plant_context = plant.GetMyContextFromRoot(simulator.get_context())
    results = plant.get_contact_results_output_port().Eval(plant_context)
    print(
        f"done: {args.duration} sim-s in {wall:.1f} wall-s "
        f"(realtime rate {args.duration / wall:.2f}x); final contacts: "
        f"{results.num_hydroelastic_contacts()} hydroelastic, "
        f"{results.num_point_pair_contacts()} point-pair"
    )


if __name__ == "__main__":
    main()
