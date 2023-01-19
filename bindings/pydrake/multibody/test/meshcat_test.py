from pydrake.multibody.meshcat import (
    ContactVisualizer_,
    ContactVisualizerParams,
    JointSliders,
    _HydroelasticContactVisualizer,
    _PointContactVisualizer,
)

import copy
import unittest

import numpy as np

from pydrake.common import (
    FindResourceOrThrow,
)
from pydrake.common.test_utilities import (
    numpy_compare,
)
from pydrake.geometry import (
    Meshcat,
    Rgba,
)
from pydrake.multibody.parsing import (
    Parser,
)
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph,
)
from pydrake.systems.framework import (
    DiagramBuilder,
    DiagramBuilder_,
)


class TestMeshcat(unittest.TestCase):

    @numpy_compare.check_nonsymbolic_types
    def test_contact_visualizer(self, T):
        meshcat = Meshcat()
        params = ContactVisualizerParams()
        params.publish_period = 0.123
        params.default_color = Rgba(0.5, 0.5, 0.5)
        params.prefix = "py_visualizer"
        params.delete_on_initialization_event = False
        params.force_threshold = 0.2
        params.newtons_per_meter = 5
        params.radius = 0.1
        self.assertIn("publish_period", repr(params))
        copy.copy(params)
        params2 = ContactVisualizerParams(publish_period=0.4)
        self.assertEqual(params2.publish_period, 0.4)
        vis = ContactVisualizer_[T](meshcat=meshcat, params=params)
        vis.Delete()
        vis.contact_results_input_port()
        vis.query_object_input_port()

        builder = DiagramBuilder_[T]()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.01)
        plant.Finalize()
        ContactVisualizer_[T].AddToBuilder(
            builder=builder, plant=plant, meshcat=meshcat, params=params)
        ContactVisualizer_[T].AddToBuilder(
            builder=builder,
            contact_results_port=plant.get_contact_results_output_port(),
            meshcat=meshcat,
            params=params)
        ContactVisualizer_[T].AddToBuilder(
            builder=builder,
            contact_results_port=plant.get_contact_results_output_port(),
            query_object_port=scene_graph.get_query_output_port(),
            meshcat=meshcat,
            params=params)

    def test_joint_sliders(self):
        # Load up an acrobot.
        acrobot_file = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.urdf")
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
        parser = Parser(plant)
        parser.AddModels(acrobot_file)
        plant.Finalize()

        # Construct a sliders system, using every available option.
        meshcat = Meshcat()
        dut = JointSliders(
            meshcat=meshcat,
            plant=plant,
            initial_value=[0.1, 0.2],
            lower_limit=[-3.0, -6.0],
            upper_limit=[3.0, 6.0],
            step=[0.25, 0.50],
            decrement_keycodes=["ArrowLeft", "ArrowDown"],
            increment_keycodes=["ArrowRight", "ArrowUp"],
        )

        # Various methods should not crash.
        dut.get_output_port()
        dut.Delete()

        # The constructor also accepts single values for broadcast (except for
        # the initial value).
        dut = JointSliders(
            meshcat=meshcat,
            plant=plant,
            initial_value=[0.1, 0.2],
            lower_limit=-3.0,
            upper_limit=3.0,
            step=0.1,
        )
        dut.Delete()

        # The constructor also accepts None directly, for optionals.
        dut = JointSliders(
            meshcat=meshcat,
            plant=plant,
            initial_value=None,
            lower_limit=None,
            upper_limit=None,
            step=None,
        )
        dut.Delete()

        # The constructor has default values, in any case.
        dut = JointSliders(meshcat, plant)

        # The Run function doesn't crash.
        builder.AddSystem(dut)
        diagram = builder.Build()
        q = dut.Run(diagram=diagram,
                    timeout=1.0,
                    stop_button_keycode="ArrowLeft")
        np.testing.assert_equal(q, [0, 0])

        # The SetPositions function doesn't crash (Acrobot has two positions).
        dut.SetPositions(q=[1, 2])

    def test_internal_point_contact_visualizer(self):
        """A very basic existence test, since this class is internal use only.
        The pydrake-internal user (meldis) has additional acceptance tests.
        """
        meshcat = Meshcat()
        params = ContactVisualizerParams()
        dut = _PointContactVisualizer(meshcat=meshcat, params=params)

    def test_internal_hydroelastic_contact_visualizer(self):
        """A very basic existence test, since this class is internal use only.
        The pydrake-internal user (meldis) has additional acceptance tests.
        """
        meshcat = Meshcat()
        params = ContactVisualizerParams()
        dut = _HydroelasticContactVisualizer(meshcat=meshcat, params=params)
