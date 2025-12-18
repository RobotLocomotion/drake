"""
Run with:
bazel run //examples/hardware_sim:memory_leak
"""

import gc

import psutil

from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.sensors import ApplyCameraConfig, CameraConfig


def get_memory_mb() -> float:
    """Get current process RSS in MB."""
    return psutil.Process().memory_info().rss / (1024 * 1024)


def run_memory_leak_test(num_iterations: int = 5):
    """Run a memory leak test with the homecart model and camera."""
    print("=" * 60)
    print("Memory Leak Test")
    print("=" * 60)
    print(f"Model: drake_models/tri_homecart/homecart.dmd.yaml")
    print(f"Iterations: {num_iterations}")
    print("=" * 60)

    initial_memory = get_memory_mb()
    print(f"Initial memory: {initial_memory:.2f} MB")

    readings = []

    for i in range(num_iterations):
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.01)

        # Load the homecart model using model directives
        # Load a smaller model to see differences in memory leaked.
        parser = Parser(plant)
        parser.AddModels(
            url="package://drake_models/tri_homecart/homecart.dmd.yaml"
        )
        plant.Finalize()

        # Add camera (RgbdSensor) which triggers RenderEngine creation
        camera_config = CameraConfig()
        camera_config.name = "test_camera"
        camera_config.width = 640
        camera_config.height = 480
        camera_config.X_PB.base_frame = "world"
        ApplyCameraConfig(
            config=camera_config,
            builder=builder,
            plant=plant,
            scene_graph=scene_graph,
        )

        diagram = builder.Build()
        simulator = Simulator(diagram)
        simulator.AdvanceTo(0.1)

        del diagram, builder
        gc.collect()

        current_memory = get_memory_mb()
        readings.append(current_memory)
        print(f"  Iteration {i}: {current_memory:.2f} MB")

    growth = readings[-1] - readings[0]
    per_iteration = growth / (num_iterations - 1) if num_iterations > 1 else 0

    print(f"\nMemory growth: {growth:.2f} MB")
    print(f"Per-iteration growth: {per_iteration:.2f} MB")


def main():
    run_memory_leak_test(num_iterations=5)


if __name__ == "__main__":
    main()

