import pydrake.visualization.meldis as mut

import unittest

from pydrake.common import (
    FindResourceOrThrow,
)
from pydrake.multibody.parsing import (
    Parser,
)
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph,
    ConnectContactResultsToDrakeVisualizer,
)
from pydrake.multibody.tree import (
    PrismaticJoint,
)
from pydrake.systems.framework import (
    DiagramBuilder,
)


class TestMeldis(unittest.TestCase):

    def test_contact_applet(self):
        """Check that _ContactApplet doesn't crash when receiving messages.
        """
        # Create the device under test.
        dut = mut.Meldis()
        meshcat = dut.meshcat
        lcm = dut._lcm

        # Enqueue a point contact result message.
        sdf_file = FindResourceOrThrow(
            "drake/examples/manipulation_station/models/sphere.sdf")
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.001)
        parser = Parser(plant=plant)
        sphere1_model = parser.AddModelFromFile(sdf_file, "sphere1")
        sphere2_model = parser.AddModelFromFile(sdf_file, "sphere2")
        body1 = plant.GetBodyByName("base_link", sphere1_model)
        body2 = plant.GetBodyByName("base_link", sphere2_model)
        plant.AddJoint(PrismaticJoint(
            name="sphere1_x", frame_on_parent=plant.world_body().body_frame(),
            frame_on_child=body1.body_frame(), axis=[1, 0, 0]))
        plant.AddJoint(PrismaticJoint(
            name="sphere2_x", frame_on_parent=plant.world_body().body_frame(),
            frame_on_child=body2.body_frame(), axis=[1, 0, 0]))
        plant.Finalize()
        ConnectContactResultsToDrakeVisualizer(
            builder=builder, plant=plant, scene_graph=scene_graph, lcm=lcm)
        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        plant.SetPositions(plant.GetMyMutableContextFromRoot(context),
                           [-0.03, 0.03])
        diagram.Publish(context)

        # The geometry isn't registered until the load is processed.
        pair_path = "/CONTACT_RESULTS/point/base_link(2)+base_link(3)"
        self.assertEqual(meshcat.HasPath(pair_path), False)

        # Process the load + draw; make sure the geometry exists now.
        lcm.HandleSubscriptions(timeout_millis=0)
        self.assertEqual(meshcat.HasPath(pair_path), True)
