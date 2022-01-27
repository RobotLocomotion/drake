import pydrake.visualization.meldis as mut

import unittest

from pydrake.common import (
    FindResourceOrThrow,
)
from pydrake.geometry import (
    DrakeVisualizer,
)
from pydrake.multibody.parsing import (
    Parser,
)
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph,
)
from pydrake.systems.framework import (
    DiagramBuilder,
)


class TestMeldis(unittest.TestCase):

    def test_viewer_applet(self):
        """Check that _ViewerApplet doesn't crash when receiving messages.
        Note that many geometry types are not yet covered by this test.
        """
        # Create the device under test.
        dut = mut.Meldis()
        meshcat = dut.meshcat
        lcm = dut._lcm

        # The path is created by the constructor.
        self.assertEqual(meshcat.HasPath("/DRAKE_VIEWER"), True)

        # Enqueue the load + draw messages.
        sdf_file = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf")
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
        parser = Parser(plant=plant)
        parser.AddModelFromFile(sdf_file)
        plant.Finalize()
        DrakeVisualizer.AddToBuilder(builder=builder, scene_graph=scene_graph,
                                     lcm=lcm)
        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        diagram.Publish(context)

        # The geometry isn't registered until the load is processed.
        link_path = "/DRAKE_VIEWER/2/plant/acrobot/Link2/0"
        self.assertEqual(meshcat.HasPath(link_path), False)

        # Process the load + draw; make sure the geometry exists now.
        lcm.HandleSubscriptions(timeout_millis=0)
        self.assertEqual(meshcat.HasPath(link_path), True)
