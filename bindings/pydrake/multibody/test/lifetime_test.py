import gc
import unittest
import weakref

from pydrake.geometry import SceneGraph, SceneGraphConfig
from pydrake.multibody.plant import (
    AddMultibodyPlant,
    AddMultibodyPlantSceneGraph,
    MultibodyPlant,
    MultibodyPlantConfig,
)
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.test.test_util import call_build_from_cpp


class TestAddSystemWrappers(unittest.TestCase):
    def call_build_from(self, diagram_builder, language):
        assert language in ["python", "c++"]
        if language == "python":
            # The pydrake binding ensures object lifetimes properly.
            return diagram_builder.Build()
        else:
            # Calling build on a python-populated diagram builder from c++ is
            # dodgy at best, but don't worry -- all will be well if something
            # retains ownership of the diagram.
            return call_build_from_cpp(diagram_builder)

    def test_diagram_and_systems_lifetimes(self):
        """DiagramBuilder's management of object lifetimes has proven tricky to
        get right. Past implementations have invited either crashes or memory
        leaks.

        https://github.com/RobotLocomotion/drake/issues/14355 -- crash
        https://github.com/RobotLocomotion/drake/issues/14387 -- leak
        """

        def make_diagram(add_plant_wrapper, call_build_from_language):
            # Use a nested function to ensure that all locals get garbage
            # collected quickly.
            builder = DiagramBuilder()
            plant, scene_graph = add_plant_wrapper(builder)
            plant.Finalize()
            diagram = self.call_build_from(builder, call_build_from_language)
            return diagram

        def make_from_plant_scene_graph(builder):
            return AddMultibodyPlantSceneGraph(
                builder, MultibodyPlant(0.01), SceneGraph()
            )

        def make_from_plant(builder):
            return AddMultibodyPlantSceneGraph(builder, MultibodyPlant(0.01))

        def make_from_time_step_scene_graph(builder):
            return AddMultibodyPlantSceneGraph(builder, 0.01, SceneGraph())

        def make_from_time_step(builder):
            return AddMultibodyPlantSceneGraph(builder, 0.01)

        def make_from_plant_config(builder):
            return AddMultibodyPlant(
                MultibodyPlantConfig(time_step=0.01), builder
            )

        def make_from_plant_config_scene_graph_config(builder):
            return AddMultibodyPlant(
                MultibodyPlantConfig(time_step=0.01),
                SceneGraphConfig(),
                builder,
            )

        for add_plant_wrapper in [
            make_from_plant_scene_graph,
            make_from_plant,
            make_from_time_step_scene_graph,
            make_from_time_step,
            make_from_plant_config,
            make_from_plant_config_scene_graph_config,
        ]:
            for call_build_from_language in ["python", "c++"]:
                diagram = make_diagram(
                    add_plant_wrapper, call_build_from_language
                )
                gc.collect()
                # Without necessary lifetime annotations, we get a segfault
                # when creating the context.
                context = diagram.CreateDefaultContext()

                # With too-conservative lifetime annotations, objects become
                # immortal, and leak memory.
                spy = weakref.finalize(diagram, lambda: None)
                del diagram
                del context
                gc.collect()
                self.assertFalse(spy.alive)
