import pydrake.visualization as mut

import copy
import unittest

from pydrake.lcm import (
    DrakeLcm,
)
from pydrake.systems.lcm import (
    LcmBuses,
)
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph,
)
from pydrake.systems.framework import (
    DiagramBuilder,
)


class TestConfig(unittest.TestCase):

    def test_apply_visualizer_config(self):
        """Exercises VisualizerConfig and ApplyVisualizerConfig.
        """
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
        plant.Finalize()
        lcm_buses = LcmBuses()
        lcm_buses.Add("default", DrakeLcm())

        config = mut.VisualizerConfig(publish_period=0.25)
        self.assertIn("publish_period", repr(config))
        copy.copy(config)
        mut.ApplyVisualizerConfig(
            config=config,
            plant=plant,
            scene_graph=scene_graph,
            lcm_buses=lcm_buses,
            builder=builder)

    def test_add_default_visualizer(self):
        """Exercises AddDefaultVisualizer.
        """
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
        plant.Finalize()
        config = mut.AddDefaultVisualizer(builder=builder)
