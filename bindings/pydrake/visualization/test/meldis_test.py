"""Unit tests for MeLDiS.

You can also visualize the LCM test data like so:

1. In a separate terminal,
    bazel run //tools:meldis -- -w

2. In another terminal, pass the name of a specific test to bazel. For example,
    bazel run //bindings/pydrake/visualization:py/meldis_test \\
        TestMeldis.test_contact_applet_hydroelastic
"""

import pydrake.visualization as mut

import functools
import numpy as np
import unittest

from drake import (
    lcmt_point_cloud,
    lcmt_viewer_geometry_data,
    lcmt_viewer_link_data,
)

from pydrake.common import (
    FindResourceOrThrow,
)
from pydrake.common.value import (
    Value,
)
from pydrake.geometry import (
    DrakeVisualizer,
    DrakeVisualizerParams,
    Role,
)
from pydrake.lcm import (
    DrakeLcm,
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
from pydrake.perception import (
    BaseField,
    Fields,
    PointCloud,
    PointCloudToLcm,
)
from pydrake.systems.framework import (
    DiagramBuilder,
)
from pydrake.systems.lcm import (
    LcmPublisherSystem,
)


class TestMeldis(unittest.TestCase):

    def _make_diagram(self, *, resource, visualizer_params, lcm):
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
        parser = Parser(plant=plant)
        parser.AddModels(FindResourceOrThrow(resource))
        plant.Finalize()
        DrakeVisualizer.AddToBuilder(builder=builder, scene_graph=scene_graph,
                                     params=visualizer_params, lcm=lcm)
        diagram = builder.Build()
        return diagram

    def _create_point_cloud(self, num_fields):
        if num_fields == 3:
            cloud_fields = Fields(BaseField.kXYZs)
        elif num_fields == 4:
            cloud_fields = Fields(BaseField.kXYZs | BaseField.kRGBs)
        else:
            assert num_fields == 7
            cloud_fields = Fields(
                BaseField.kXYZs | BaseField.kRGBs | BaseField.kNormals
            )

        num_points = 10
        cloud = PointCloud(num_points, cloud_fields)
        xyzs = np.random.uniform(-0.1, 0.1, (3, num_points))
        cloud.mutable_xyzs()[:] = xyzs
        if num_fields > 3:
            rgbs = np.random.randint(0, 255, (3, num_points), dtype=np.uint8)
            cloud.mutable_rgbs()[:] = rgbs
        if num_fields > 4:
            normals = np.random.uniform(-0.1, 0.1, (3, num_points))
            cloud.mutable_normals()[:] = normals
        return cloud

    def _publish_lcm_point_cloud(self, lcm, channel, cloud):
        """Given the complexity of the lcmt_point_cloud message format, the
        easiest way to feed the DUT with a sample point cloud is to use
        PointCloudToLcmSystem for the point cloud conversion and
        LcmPublisherSystem to publish the LCM message onto the DUT's LCM bus.
        """
        builder = DiagramBuilder()
        cloud_to_lcm = builder.AddSystem(PointCloudToLcm(frame_name="world"))
        cloud_lcm_publisher = builder.AddSystem(
            LcmPublisherSystem.Make(
                channel=channel,
                lcm_type=lcmt_point_cloud,
                lcm=lcm,
                publish_period=1.0,
                use_cpp_serializer=True))
        builder.Connect(
            cloud_to_lcm.get_output_port(),
            cloud_lcm_publisher.get_input_port())
        diagram = builder.Build()

        # Set input and publish the point cloud.
        context = diagram.CreateDefaultContext()
        cloud_context = cloud_to_lcm.GetMyContextFromRoot(context)
        cloud_to_lcm.get_input_port().FixValue(
            cloud_context, Value[PointCloud](cloud))
        diagram.ForcedPublish(context)

    def test_viewer_applet(self):
        """Checks that _ViewerApplet doesn't crash when receiving messages.
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
            resource="drake/multibody/benchmarks/acrobot/acrobot.sdf",
            visualizer_params=DrakeVisualizerParams(),
            lcm=lcm)
        context = diagram.CreateDefaultContext()
        diagram.ForcedPublish(context)

        # The geometry isn't registered until the load is processed.
        self.assertEqual(meshcat.HasPath("/DRAKE_VIEWER"), False)
        link_path = "/DRAKE_VIEWER/2/plant/acrobot/Link2/0"
        self.assertEqual(meshcat.HasPath(link_path), False)

        # Process the load + draw; make sure the geometry exists now.
        lcm.HandleSubscriptions(timeout_millis=0)
        dut._invoke_subscriptions()
        self.assertEqual(meshcat.HasPath("/DRAKE_VIEWER"), True)
        self.assertEqual(meshcat.HasPath(link_path), True)

    def test_viewer_applet_robot_meshes(self):
        """Checks _ViewerApplet support for meshes.
        """
        # Create the device under test.
        dut = mut.Meldis()
        lcm = dut._lcm

        # Process the load + draw messages.
        diagram = self._make_diagram(
            resource="drake/manipulation/models/iiwa_description/urdf/"
                     "iiwa14_no_collision.urdf",
            visualizer_params=DrakeVisualizerParams(),
            lcm=lcm)
        diagram.ForcedPublish(diagram.CreateDefaultContext())
        lcm.HandleSubscriptions(timeout_millis=0)
        dut._invoke_subscriptions()
        self.assertEqual(dut.meshcat.HasPath("/DRAKE_VIEWER"), True)

    def test_viewer_applet_reload_optimization(self):
        """Checks that loading the identical scene twice is efficient.
        """
        # Create the device under test.
        dut = mut.Meldis()
        meshcat = dut.meshcat

        # Initialize an identical simulation twice in a row. The second time
        # around should be more efficient.
        for i in range(2):
            # To parcel out messages one at a time, we'll have the Diagram send
            # its messages to a temporary location and then forward them along
            # one at a time to Meldis.
            temp_lcm = DrakeLcm()
            temp_lcm_queue = []   # Tuples of (channel, buffer).
            for name in ["DRAKE_VIEWER_LOAD_ROBOT", "DRAKE_VIEWER_DRAW"]:
                def _on_message(channel, data):
                    temp_lcm_queue.append((channel, data))
                temp_lcm.Subscribe(channel=name, handler=functools.partial(
                    _on_message, name))

            # Create the plant + visualizer.
            diagram = self._make_diagram(
                resource="drake/multibody/benchmarks/acrobot/acrobot.sdf",
                visualizer_params=DrakeVisualizerParams(),
                lcm=temp_lcm)

            # Capture the LOAD and DRAW messages via temp_lcm.
            diagram.ForcedPublish(diagram.CreateDefaultContext())
            temp_lcm.HandleSubscriptions(timeout_millis=0)
            load, draw = temp_lcm_queue
            assert load[0] == "DRAKE_VIEWER_LOAD_ROBOT"
            assert draw[0] == "DRAKE_VIEWER_DRAW"

            # Forward the LOAD message to Meldis.
            dut._lcm.Publish(*load)
            dut._lcm.HandleSubscriptions(timeout_millis=0)
            dut._invoke_subscriptions()

            # The first time around, the link won't be added until the DRAW
            # message arrives. The second time around, the link should still
            # be intact from the first time (i.e., no `Delete` occurred).
            link_path = "/DRAKE_VIEWER/2/plant/acrobot/Link2/0"
            if i == 0:
                self.assertFalse(meshcat.HasPath(link_path))
            else:
                self.assertTrue(meshcat.HasPath(link_path))

            # Forward the DRAW message to Meldis.
            dut._lcm.Publish(*draw)
            dut._lcm.HandleSubscriptions(timeout_millis=0)
            dut._invoke_subscriptions()

            # The link always exists after a DRAW.
            self.assertTrue(meshcat.HasPath(link_path))

    def test_hydroelastic_geometry(self):
        """Check that _ViewerApplet doesn't crash when receiving
        hydroelastic geometry.
        """
        dut = mut.Meldis()
        lcm = dut._lcm
        diagram = self._make_diagram(
            resource="drake/examples/hydroelastic/"
                     "spatula_slip_control/models/spatula.sdf",
            visualizer_params=DrakeVisualizerParams(
                show_hydroelastic=True,
                role=Role.kProximity),
            lcm=lcm)
        context = diagram.CreateDefaultContext()
        diagram.ForcedPublish(context)
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
        sphere1_model, = Parser(plant, "sphere1").AddModels(sdf_file)
        sphere2_model, = Parser(plant, "sphere2").AddModels(sdf_file)
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
        diagram.ForcedPublish(context)

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
        parser.AddModels(sdf_file)
        body1 = plant.GetBodyByName("body1")
        body2 = plant.GetBodyByName("body2")
        plant.AddJoint(PrismaticJoint(
            name="body1", frame_on_parent=plant.world_body().body_frame(),
            frame_on_child=body1.body_frame(), axis=[0, 0, 1]))
        plant.AddJoint(PrismaticJoint(
            name="body2", frame_on_parent=plant.world_body().body_frame(),
            frame_on_child=body2.body_frame(), axis=[1, 0, 0]))
        plant.Finalize()
        ConnectContactResultsToDrakeVisualizer(
            builder=builder, plant=plant, scene_graph=scene_graph, lcm=lcm)
        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        plant.SetPositions(plant.GetMyMutableContextFromRoot(context),
                           [0.1, 0.3])
        diagram.ForcedPublish(context)

        # The geometry isn't registered until the load is processed.
        hydro_path = "/CONTACT_RESULTS/hydroelastic/" + \
                     "body1.two_bodies::body1_collision+body2"
        hydro_path2 = "/CONTACT_RESULTS/hydroelastic/" + \
                      "body1.two_bodies::body1_collision2+body2"
        self.assertEqual(meshcat.HasPath(hydro_path), False)
        self.assertEqual(meshcat.HasPath(hydro_path2), False)

        # Process the load + draw; contact results should now exist.
        lcm.HandleSubscriptions(timeout_millis=1)
        dut._invoke_subscriptions()

        self.assertEqual(meshcat.HasPath("/CONTACT_RESULTS/hydroelastic"),
                         True)
        self.assertEqual(meshcat.HasPath(hydro_path), True)
        self.assertEqual(meshcat.HasPath(hydro_path2), True)

    def test_deformable(self):
        """Check that _ViewerApplet doesn't crash for deformable geometries
        in DRAKE_VIEWER_DEFORMABLE channel.
        """

        # Create the device under test.
        dut = mut.Meldis()

        # Prepare a deformable-geometry message. It is a triangle surface mesh
        # of a tetrahedron.
        geom0 = lcmt_viewer_geometry_data()
        geom0.type = lcmt_viewer_geometry_data.MESH
        geom0.position = [0.0, 0.0, 0.0]
        q_wxyz = [1.0, 0.0, 0.0, 0.0]
        geom0.quaternion = q_wxyz
        geom0.color = [0.9, 0.9, 0.9, 1.0]
        geom0.string_data = "tetrahedron"
        geom0.float_data = [
            # 4 vertices and 4 triangles
            4.0, 4.0,
            # 4 vertices at the origin and on the three axes.
            0.0, 0.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
            # 4 triangles, use float for integer vertex indices
            0., 2., 1.,
            0., 1., 3.,
            0., 3., 2.,
            1., 2., 3.]
        geom0.num_float_data = len(geom0.float_data)
        message = lcmt_viewer_link_data()
        message.name = "test_deformable_geometry"
        message.robot_num = 0
        message.num_geom = 1
        message.geom = [geom0]

        dut._lcm.Publish(channel="DRAKE_VIEWER_DEFORMABLE",
                         buffer=message.encode())

        meshcat_path = f"/DRAKE_VIEWER/{message.robot_num}/{message.name}"
        # Before the subscribed handlers are called, there is no meshcat path
        # from the published lcm message.
        self.assertEqual(dut.meshcat.HasPath(meshcat_path), False)

        dut._lcm.HandleSubscriptions(timeout_millis=1)
        dut._invoke_subscriptions()

        # After the handlers are called, we have the expected meshcat path.
        self.assertEqual(dut.meshcat.HasPath(meshcat_path), True)

    def test_point_cloud(self):
        """Check that _PointCloudApplet doesn't crash when receiving point
        cloud messages.
        """
        # Create the device under test.
        dut = mut.Meldis()

        test_tuples = (
           (3, "DRAKE_POINT_CLOUD", "/POINT_CLOUD/default"),
           (4, "DRAKE_POINT_CLOUD_XYZRGB", "/POINT_CLOUD/XYZRGB"),
           (7, "DRAKE_POINT_CLOUD_12345", "/POINT_CLOUD/12345"),
        )
        for num_fields, channel, meshcat_path in test_tuples:
            with self.subTest(num_fields=num_fields):
                self.assertEqual(dut.meshcat.HasPath(meshcat_path), False)
                cloud = self._create_point_cloud(num_fields=num_fields)
                self._publish_lcm_point_cloud(dut._lcm, channel, cloud)

                dut._lcm.HandleSubscriptions(timeout_millis=1)
                dut._invoke_subscriptions()
                self.assertEqual(dut.meshcat.HasPath(meshcat_path), True)
