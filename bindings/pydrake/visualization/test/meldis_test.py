import pydrake.visualization.meldis as mut

import unittest

from pydrake.common import (
    FindResourceOrThrow,
)
from pydrake.geometry import (
    DrakeVisualizer,
    DrakeVisualizerParams,
    Role,
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

    def _make_diagram(self, *, sdf_filename, visualizer_params, lcm):
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
        parser = Parser(plant=plant)
        parser.AddModelFromFile(FindResourceOrThrow(sdf_filename))
        plant.Finalize()
        DrakeVisualizer.AddToBuilder(builder=builder, scene_graph=scene_graph,
                                     params=visualizer_params, lcm=lcm)
        diagram = builder.Build()
        return diagram

    def test_viewer_applet(self):
        """Check that _ViewerApplet doesn't crash when receiving messages.
        Note that many geometry types are not yet covered by this test.
        """
        # Create the device under test.
        dut = mut.Meldis()
        meshcat = dut.meshcat
        lcm = dut._lcm

        # Polling before any messages doesn't crash.
        dut._invoke_poll()

        # Enqueue the load + draw messages.
        diagram = self._make_diagram(
            sdf_filename="drake/multibody/benchmarks/acrobot/acrobot.sdf",
            visualizer_params=DrakeVisualizerParams(),
            lcm=lcm)
        context = diagram.CreateDefaultContext()
        diagram.Publish(context)

        # The geometry isn't registered until the load is processed.
        self.assertEqual(meshcat.HasPath("/DRAKE_VIEWER"), False)
        link_path = "/DRAKE_VIEWER/2/plant/acrobot/Link2/0"
        self.assertEqual(meshcat.HasPath(link_path), False)

        # Process the load + draw; make sure the geometry exists now.
        lcm.HandleSubscriptions(timeout_millis=0)
        dut._invoke_subscriptions()
        self.assertEqual(meshcat.HasPath("/DRAKE_VIEWER"), True)
        self.assertEqual(meshcat.HasPath(link_path), True)

    def test_hydroelastic_geometry(self):
        """Check that _ViewerApplet doesn't crash when receiving
        hydroelastic geometry.
        """
        dut = mut.Meldis()
        lcm = dut._lcm
        diagram = self._make_diagram(
            sdf_filename="drake/examples/hydroelastic/"
                         "spatula_slip_control/models/spatula.sdf",
            visualizer_params=DrakeVisualizerParams(
                show_hydroelastic=True,
                role=Role.kProximity),
            lcm=lcm)
        context = diagram.CreateDefaultContext()
        diagram.Publish(context)
        lcm.HandleSubscriptions(timeout_millis=0)
        dut._invoke_subscriptions()

    def test_contact_applet_point_pair(self):
        """Check that _ContactApplet doesn't crash when receiving point
           contact messages.
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
        dut._invoke_subscriptions()
        self.assertEqual(meshcat.HasPath(pair_path), True)

    def test_contact_applet_hydroelastic(self):
        """Check that _ContactApplet doesn't crash when receiving hydroelastic
           messages.
        """
        # Create the device under test.
        dut = mut.Meldis()
        meshcat = dut.meshcat
        lcm = dut._lcm

        # Enqueue a hydroelastic contact message.
        sdf_file = FindResourceOrThrow(
            "drake/multibody/meshcat/test/hydroelastic.sdf")
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.001)
        parser = Parser(plant=plant)
        parser.AddModelFromFile(sdf_file)
        body1 = plant.GetBodyByName("body1")
        body2 = plant.GetBodyByName("body2")
        plant.AddJoint(PrismaticJoint(
            name="sphere1", frame_on_parent=plant.world_body().body_frame(),
            frame_on_child=body1.body_frame(), axis=[1, 0, 0]))
        plant.AddJoint(PrismaticJoint(
            name="sphere2", frame_on_parent=plant.world_body().body_frame(),
            frame_on_child=body2.body_frame(), axis=[1, 0, 0]))
        plant.Finalize()
        ConnectContactResultsToDrakeVisualizer(
            builder=builder, plant=plant, scene_graph=scene_graph, lcm=lcm)
        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        plant.SetPositions(plant.GetMyMutableContextFromRoot(context),
                           [-0.05, 0.1])
        diagram.Publish(context)

        # The geometry isn't registered until the load is processed.
        hydro_path = "/CONTACT_RESULTS/hydroelastic/body1+body2"
        self.assertEqual(meshcat.HasPath(hydro_path), False)

        # Process the load + draw; contact results should now exist.
        lcm.HandleSubscriptions(timeout_millis=0)
        dut._invoke_subscriptions()
        self.assertEqual(meshcat.HasPath(hydro_path), True)
