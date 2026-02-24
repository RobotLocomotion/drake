# -*- coding: utf-8 -*-

import collections
import copy
import itertools
import pickle
import unittest
import weakref

import numpy as np

from pydrake.autodiffutils import AutoDiffXd
from pydrake.common import FindResourceOrThrow, Parallelism
from pydrake.common.cpp_param import List
from pydrake.common.deprecation import install_numpy_warning_filters
from pydrake.common.eigen_geometry import Quaternion_
from pydrake.common.test_utilities import numpy_compare
from pydrake.common.test_utilities.deprecation import catch_drake_warnings
from pydrake.common.test_utilities.pickle_compare import assert_pickle
from pydrake.common.value import AbstractValue, Value
from pydrake.geometry import (
    Box,
    GeometryId,
    GeometryInstance,
    GeometrySet,
    HydroelasticContactRepresentation,
    PenetrationAsPointPair_,
    PolygonSurfaceMesh_,
    ProximityProperties,
    Role,
    SceneGraphConfig,
    SignedDistancePair_,
    SignedDistanceToPoint_,
    Sphere,
    SurfaceTriangle,
    TriangleSurfaceMesh,
)
from pydrake.lcm import DrakeLcm
from pydrake.math import (
    RigidTransform,
    RigidTransform_,
    RollPitchYaw_,
    RotationMatrix_,
)
from pydrake.multibody.benchmarks.acrobot import (
    AcrobotParameters,
    MakeAcrobotPlant,
)
from pydrake.multibody.fem import (
    DeformableBodyConfig_,
    ForceDensityType,
)
from pydrake.multibody.math import (
    SpatialAcceleration_,
    SpatialForce_,
    SpatialMomentum_,
    SpatialVelocity_,
)
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import (
    AddMultibodyPlant,
    AddMultibodyPlantSceneGraph,
    ApplyMultibodyPlantConfig,
    CalcContactFrictionFromSurfaceProperties,
    ConnectContactResultsToDrakeVisualizer,
    ContactModel,
    ContactResults_,
    ContactResultsToLcmSystem,
    CoulombFriction_,
    DeformableContactInfo_,
    DiscreteContactApproximation,
    DistanceConstraintParams,
    ExternallyAppliedSpatialForce_,
    ExternallyAppliedSpatialForceMultiplexer_,
    MultibodyPlant,
    MultibodyPlant_,
    MultibodyPlantConfig,
    PointPairContactInfo_,
    Propeller_,
    PropellerInfo,
    Wing,
)
from pydrake.multibody.tree import (
    BallRpyJoint_,
    Body_,  # dispreferred alias for RigidBody_
    BodyIndex,
    CalcSpatialInertia,
    DoorHinge_,
    DoorHingeConfig,
    FixedOffsetFrame_,
    ForceDensityField_,
    ForceElement_,
    ForceElementIndex,
    Frame_,
    FrameIndex,
    GravityForceField_,
    JacobianWrtVariable,
    Joint_,
    JointActuator_,
    JointActuatorIndex,
    JointIndex,
    LinearBushingRollPitchYaw_,
    LinearSpringDamper_,
    ModelInstanceIndex,
    MultibodyForces_,
    PdControllerGains,
    PlanarJoint_,
    PrismaticJoint_,
    PrismaticSpring_,
    QuaternionFloatingJoint_,
    RevoluteJoint_,
    RevoluteSpring_,
    RigidBody,
    RigidBody_,
    RotationalInertia_,
    RpyFloatingJoint_,
    ScopedName,
    ScrewJoint_,
    SpatialInertia_,
    UniformGravityFieldElement_,
    UnitInertia_,
    UniversalJoint_,
    WeldJoint_,
    default_model_instance,
    world_frame_index,
    world_index,
    world_model_instance,
)
from pydrake.symbolic import Expression, Variable
from pydrake.systems.analysis import Simulator_
from pydrake.systems.framework import (
    AbstractParameterIndex,
    DiagramBuilder,
    DiagramBuilder_,
    DiscreteStateIndex,
    InputPort_,
    LeafSystem_,
    OutputPort_,
    System_,
)
from pydrake.systems.lcm import LcmPublisherSystem
from pydrake.systems.scalar_conversion import TemplateSystem


def get_index_class(cls, T):
    # Maps a class to its corresponding index class, accommdating inheritance.
    class_to_index_class_map = {
        RigidBody_[T]: BodyIndex,
        ForceElement_[T]: ForceElementIndex,
        Frame_[T]: FrameIndex,
        Joint_[T]: JointIndex,
        JointActuator_[T]: JointActuatorIndex,
    }
    for key_cls, index_cls in class_to_index_class_map.items():
        if issubclass(cls, key_cls):
            return index_cls
    raise RuntimeError("Unknown class: {}".format(cls))


# Permits parametric scalar type conversion.
def to_type(system, T):
    assert isinstance(system, System_[float])
    if T is float:
        return system
    else:
        return system.ToScalarType[T]()


class TestPlant(unittest.TestCase):
    def setUp(self):
        unittest.TestCase.setUp(self)
        # For some reason, something in how `unittest` tries to scope warnings
        # causes the previous filters to be lost. Re-install here.
        install_numpy_warning_filters(force=True)

    def test_type_safe_indices(self):
        self.assertEqual(world_index(), BodyIndex(0))
        self.assertEqual(repr(world_index()), "BodyIndex(0)")
        self.assertEqual(world_frame_index(), FrameIndex(0))
        self.assertEqual(repr(world_frame_index()), "FrameIndex(0)")
        self.assertEqual(world_model_instance(), ModelInstanceIndex(0))
        self.assertEqual(repr(world_model_instance()), "ModelInstanceIndex(0)")
        self.assertEqual(default_model_instance(), ModelInstanceIndex(1))
        self.assertTrue(ModelInstanceIndex(0) < ModelInstanceIndex(1))

    def assert_sane(self, x, nonzero=True):
        self.assertTrue(np.all(np.isfinite(numpy_compare.to_float(x))))
        if nonzero:
            numpy_compare.assert_float_not_equal(x, 0.0)

    @numpy_compare.check_all_types
    def test_multibody_plant_construction_api(self, T):
        # SceneGraph does not support `Expression` type.
        DiagramBuilder = DiagramBuilder_[T]
        SpatialInertia = SpatialInertia_[float]
        RigidTransform = RigidTransform_[T]
        CoulombFriction = CoulombFriction_[T]

        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
        plant.SetUseSampledOutputPorts(use_sampled_output_ports=False)
        self.assertEqual(plant.has_sampled_output_ports(), False)
        self.assertEqual(plant.time_step(), 0.0)
        spatial_inertia = SpatialInertia.NaN()
        body = plant.AddRigidBody(name="new_body", M_BBo_B=spatial_inertia)
        body.default_mass()
        body.default_com()
        body.default_unit_inertia()
        body.default_rotational_inertia()
        body.default_spatial_inertia()
        new_model_instance = plant.AddModelInstance("new_model_instance")
        plant.RenameModelInstance(
            model_instance=new_model_instance, name="something"
        )
        plant.RenameModelInstance(
            model_instance=new_model_instance, name="new_model_instance"
        )
        body = plant.AddRigidBody(
            name="new_body_2",
            M_BBo_B=spatial_inertia,
            model_instance=new_model_instance,
        )
        box = Box(width=0.5, depth=1.0, height=2.0)
        body_X_BG = RigidTransform()
        body_friction = CoulombFriction(
            static_friction=0.6, dynamic_friction=0.5
        )
        self.assertEqual(
            numpy_compare.to_float(body_friction.static_friction()), 0.6
        )
        self.assertEqual(
            numpy_compare.to_float(body_friction.dynamic_friction()), 0.5
        )
        body_friction2 = CoulombFriction(
            static_friction=0.2, dynamic_friction=0.1
        )
        CalcContactFrictionFromSurfaceProperties(body_friction, body_friction2)

        if T is float:
            plant.RegisterVisualGeometry(
                body=body,
                X_BG=body_X_BG,
                shape=box,
                name="new_body_visual",
                diffuse_color=[1.0, 0.64, 0.0, 0.5],
            )
            plant.RegisterVisualGeometry(
                body=body,
                geometry_instance=GeometryInstance(
                    X_PG=body_X_BG, shape=Sphere(1.0), name="from_instance"
                ),
            )
            plant.RegisterCollisionGeometry(
                body=body,
                X_BG=body_X_BG,
                shape=box,
                name="new_body_collision",
                coulomb_friction=body_friction,
            )
            self.assertGreater(plant.num_collision_geometries(), 0)
            body0_props = scene_graph.model_inspector().GetProximityProperties(
                plant.GetCollisionGeometriesForBody(body)[0]
            )
            body0_friction = body0_props.GetProperty(
                "material", "coulomb_friction"
            )
            self.assertEqual(body0_friction.static_friction(), 0.6)
            self.assertEqual(body0_friction.dynamic_friction(), 0.5)
            explicit_props = ProximityProperties()
            explicit_props.AddProperty(
                "material", "coulomb_friction", CoulombFriction(1.1, 0.8)
            )
            plant.RegisterCollisionGeometry(
                body=body,
                X_BG=body_X_BG,
                shape=box,
                name="new_body_collision2",
                properties=explicit_props,
            )
            body1_props = scene_graph.model_inspector().GetProximityProperties(
                plant.GetCollisionGeometriesForBody(body)[1]
            )
            body1_friction = body1_props.GetProperty(
                "material", "coulomb_friction"
            )
            self.assertGreater(plant.num_collision_geometries(), 1)
            self.assertEqual(body1_friction.static_friction(), 1.1)
            self.assertEqual(body1_friction.dynamic_friction(), 0.8)

        # Confirm that passing the diagram context into the plant (a common
        # user error) throws a RuntimeError instead of crashing.
        plant.Finalize()
        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        with self.assertRaises(RuntimeError):
            plant.EvalBodyPoseInWorld(context, body)

    @numpy_compare.check_all_types
    def test_joint_actuator_remodeling(self, T):
        """
        Tests joint actuator APIs related to remodeling: `RemoveJointActuator`,
        `has_joint_actuator` and the 0 argument `GetJointActuatorIndices()`.
        """
        plant = MultibodyPlant_[T](0)
        instance = plant.AddModelInstance("instance")
        body = plant.AddRigidBody(name="body", model_instance=instance)
        joint = plant.AddJoint(
            PrismaticJoint_[T](
                "joint", plant.world_frame(), body.body_frame(), [0, 0, 1]
            )
        )
        actuator = plant.AddJointActuator("actuator", joint, 1)
        actuator_index = actuator.index()
        self.assertEqual(plant.GetJointActuatorIndices(), [actuator.index()])
        self.assertTrue(plant.has_joint_actuator(actuator_index=actuator_index))
        plant.RemoveJointActuator(actuator=actuator)
        self.assertFalse(
            plant.has_joint_actuator(actuator_index=actuator_index)
        )
        plant.Finalize()
        self.assertEqual(plant.GetJointActuatorIndices(), [])

    @numpy_compare.check_all_types
    def test_joint_remodeling(self, T):
        """
        Tests joint  APIs related to remodeling: `RemoveJoint`,
        `has_joint` and the 0 argument `GetJointIndices()`.
        """
        plant = MultibodyPlant_[T](0)
        instance = plant.AddModelInstance("instance")
        body = plant.AddRigidBody(name="body", model_instance=instance)
        joint = plant.AddJoint(
            PrismaticJoint_[T](
                "joint", plant.world_frame(), body.body_frame(), [0, 0, 1]
            )
        )
        joint_index = joint.index()
        self.assertEqual(plant.GetJointIndices(), [joint_index])
        self.assertTrue(plant.has_joint(joint_index=joint_index))
        plant.RemoveJoint(joint=joint)
        self.assertFalse(plant.has_joint(joint_index=joint_index))
        plant.Finalize()
        # 6 dof joint will be added for the now free body.
        self.assertEqual(plant.GetJointIndices(), [JointIndex(1)])

    def test_multibody_plant_config(self):
        MultibodyPlantConfig()
        config = MultibodyPlantConfig(time_step=0.01)
        self.assertEqual(config.time_step, 0.01)
        self.assertIn("time_step", repr(config))
        copy.copy(config)

        builder = DiagramBuilder_[float]()
        plant, scene_graph = AddMultibodyPlant(config=config, builder=builder)
        self.assertIsNotNone(plant)
        self.assertIsNotNone(scene_graph)

        self.assertNotEqual(
            plant.get_contact_model(), ContactModel.kHydroelasticsOnly
        )
        config.contact_model = "hydroelastic"
        ApplyMultibodyPlantConfig(config=config, plant=plant)
        self.assertEqual(
            plant.get_contact_model(), ContactModel.kHydroelasticsOnly
        )

    def test_add_multibody_plant_config_3args(self):
        plant_config = MultibodyPlantConfig(time_step=0.01)
        scene_graph_config = SceneGraphConfig()

        builder = DiagramBuilder_[float]()
        plant, scene_graph = AddMultibodyPlant(
            plant_config=plant_config,
            scene_graph_config=scene_graph_config,
            builder=builder,
        )
        self.assertIsNotNone(plant)
        self.assertIsNotNone(scene_graph)

    @numpy_compare.check_all_types
    def test_get_bodies_welded_to_keep_alive(self, T):
        """
        Explicitly tests casting behavior for GetBodiesWeldedTo(). The model
        has a link welded to the world.

        Prior implementations would inadvertently attempt to copy Body
        objects, leading to run-time errors.
        We additionally check keep-alive behavior on constituent elemenets,
        similar to the more generic form in `pydrake_pybind_test`.
        """

        def test_casting_and_get_weakref_and_last_body():
            # Define scoped function so that we are confident on reference
            # counting.
            # TODO(eric.cousineau): Figure out why `plant_f` has to be garbage
            # collected to allow `plant` to be freed.
            plant_f = MultibodyPlant_[float](time_step=0.0)
            Parser(plant_f).AddModels(
                url="package://drake/examples/acrobot/Acrobot.urdf"
            )
            plant_f.Finalize()
            plant = to_type(plant_f, T)
            world_body = plant.world_body()
            base_link = plant.GetBodyByName("base_link")
            first_body, last_body = plant.GetBodiesWeldedTo(world_body)
            self.assertIs(first_body, world_body)
            self.assertIs(last_body, base_link)
            ref = weakref.ref(plant)
            return ref, last_body

        ref, last_body = test_casting_and_get_weakref_and_last_body()
        # Show that `last_body` keeps `plant` alive.
        self.assertIsNot(ref(), None)
        del last_body
        # Show that deleting reference to `last_body` allows `plant` to be
        # garbage collected.
        self.assertIs(ref(), None)

    @numpy_compare.check_all_types
    def test_multibody_plant_api_via_parsing(self, T):
        Body = Body_[T]  # check that dispreferred alias works
        Frame = Frame_[T]
        JointActuator = JointActuator_[T]
        InputPort = InputPort_[T]
        OutputPort = OutputPort_[T]

        def check_repr(element, expected):
            if T is float:
                self.assertEqual(repr(element), expected)

        # TODO(eric.cousineau): Decouple this when construction can be done
        # without parsing.
        # This a subset of `multibody_plant_sdf_parser_test.cc`.
        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf"
        )
        # N.B. `Parser` only supports `MultibodyPlant_[float]`.
        plant_f = MultibodyPlant_[float](time_step=0.01)
        (model_instance,) = Parser(plant_f).AddModels(file_name)
        self.assertIsInstance(model_instance, ModelInstanceIndex)
        check_repr(model_instance, "ModelInstanceIndex(2)")
        plant_f.Finalize()
        plant = to_type(plant_f, T)

        benchmark = MakeAcrobotPlant(AcrobotParameters(), True)
        self.assertEqual(plant.num_bodies(), benchmark.num_bodies())
        self.assertEqual(plant.num_joints(), benchmark.num_joints())
        self.assertEqual(plant.num_actuators(), benchmark.num_actuators())
        self.assertEqual(
            plant.num_actuators(model_instance), benchmark.num_actuators()
        )
        self.assertEqual(
            plant.num_model_instances(), benchmark.num_model_instances() + 1
        )
        self.assertEqual(plant.num_positions(), benchmark.num_positions())
        self.assertEqual(
            plant.num_positions(model_instance=model_instance),
            benchmark.num_positions(),
        )
        self.assertEqual(plant.num_velocities(), benchmark.num_velocities())
        self.assertEqual(
            plant.num_velocities(model_instance=model_instance),
            benchmark.num_velocities(),
        )
        self.assertEqual(
            plant.num_multibody_states(), benchmark.num_multibody_states()
        )
        self.assertEqual(
            plant.num_multibody_states(model_instance=model_instance),
            benchmark.num_multibody_states(),
        )
        self.assertEqual(
            plant.num_actuated_dofs(), benchmark.num_actuated_dofs()
        )
        self.assertEqual(
            plant.num_actuated_dofs(model_instance=model_instance),
            benchmark.num_actuated_dofs(),
        )
        self.assertTrue(plant.is_finalized())
        self.assertTrue(plant.HasBodyNamed(name="Link1"))
        self.assertTrue(
            plant.HasBodyNamed(name="Link1", model_instance=model_instance)
        )
        self.assertTrue(plant.HasJointNamed(name="ShoulderJoint"))
        self.assertTrue(
            plant.HasJointNamed(
                name="ShoulderJoint", model_instance=model_instance
            )
        )
        shoulder = plant.GetJointByName(name="ShoulderJoint")
        self._test_joint_api(T, shoulder)
        check_repr(
            shoulder,
            "<RevoluteJoint name='ShoulderJoint' index=0 model_instance=2>",
        )
        np.testing.assert_array_equal(
            shoulder.position_lower_limits(), [-np.inf]
        )
        np.testing.assert_array_equal(
            shoulder.position_upper_limits(), [np.inf]
        )
        self.assertIs(
            shoulder,
            plant.GetJointByName(
                name="ShoulderJoint", model_instance=model_instance
            ),
        )
        self._test_joint_actuator_api(
            T, plant.GetJointActuatorByName(name="ElbowJoint")
        )
        self.assertEqual(
            plant.GetJointActuatorByName(name="ElbowJoint"),
            plant.GetJointActuatorByName(
                name="ElbowJoint", model_instance=model_instance
            ),
        )
        link1 = plant.GetBodyByName(name="Link1")
        self._test_body_api(T, link1)
        self.assertIs(
            link1,
            plant.GetBodyByName(name="Link1", model_instance=model_instance),
        )
        link1 = plant.GetRigidBodyByName(name="Link1")
        self._test_body_api(T, link1)
        self.assertIs(
            link1,
            plant.GetRigidBodyByName(
                name="Link1", model_instance=model_instance
            ),
        )
        self.assertEqual(len(plant.GetBodyIndices(model_instance)), 2)
        check_repr(link1, "<RigidBody name='Link1' index=1 model_instance=2>")
        self.assertTrue(plant.HasFrameNamed(name="Link1"))
        self.assertTrue(
            plant.HasFrameNamed(name="Link1", model_instance=model_instance)
        )
        self._test_frame_api(T, plant.GetFrameByName(name="Link1"))
        link1_frame = plant.GetFrameByName(name="Link1")
        check_repr(
            link1_frame,
            "<RigidBodyFrame name='Link1' index=1 model_instance=2>",
        )
        self.assertIs(
            link1_frame,
            plant.GetFrameByName(name="Link1", model_instance=model_instance),
        )
        self.assertEqual(
            len(plant.GetFrameIndices(model_instance=model_instance)), 7
        )
        self.assertIs(link1.GetParentPlant(), plant)
        self.assertTrue(plant.HasModelInstanceNamed(name="acrobot"))
        self.assertEqual(
            model_instance, plant.GetModelInstanceByName(name="acrobot")
        )
        self.assertEqual(
            plant.GetBodiesWeldedTo(plant.world_body()), [plant.world_body()]
        )
        link2 = plant.GetBodyByName(name="Link2")
        self.assertEqual(
            plant.GetBodiesKinematicallyAffectedBy([shoulder.index()]),
            [link1.index(), link2.index()],
        )
        self.assertIsInstance(plant.get_actuation_input_port(), InputPort)
        self.assertIsInstance(plant.get_geometry_pose_output_port(), OutputPort)
        self.assertIsInstance(plant.get_net_actuation_output_port(), OutputPort)
        self.assertIsInstance(
            plant.get_net_actuation_output_port(model_instance), OutputPort
        )
        self.assertIsInstance(plant.get_state_output_port(), OutputPort)
        self.assertIsInstance(
            plant.get_generalized_acceleration_output_port(), OutputPort
        )
        self.assertIsInstance(
            plant.get_reaction_forces_output_port(), OutputPort
        )
        self.assertIsInstance(
            plant.get_contact_results_output_port(), OutputPort
        )
        self.assertIsInstance(plant.num_frames(), int)
        self.assertIsInstance(plant.get_body(body_index=world_index()), Body)
        self.assertEqual(
            plant.IsAnchored(plant.get_body(body_index=world_index())), True
        )
        self.assertEqual(plant.NumBodiesWithName("Link1"), 1)
        self.assertIs(shoulder, plant.get_joint(joint_index=JointIndex(0)))
        self.assertIs(
            shoulder, plant.get_mutable_joint(joint_index=JointIndex(0))
        )
        self.assertEqual(
            [JointIndex(0), JointIndex(1)],
            plant.GetJointIndices(model_instance=model_instance),
        )
        elbow = plant.GetJointByName(name="ElbowJoint")
        self.assertEqual(
            [elbow.index()],
            plant.GetActuatedJointIndices(model_instance=model_instance),
        )
        joint_actuator = plant.get_joint_actuator(
            actuator_index=JointActuatorIndex(0)
        )
        self.assertIsInstance(joint_actuator, JointActuator)
        self.assertEqual(
            [joint_actuator.index()],
            plant.GetJointActuatorIndices(model_instance=model_instance),
        )
        check_repr(
            joint_actuator,
            "<JointActuator name='ElbowJoint' index=0 model_instance=2>",
        )
        self.assertIsInstance(
            plant.get_frame(frame_index=world_frame_index()), Frame
        )
        self.assertEqual(
            "acrobot", plant.GetModelInstanceName(model_instance=model_instance)
        )
        self.assertIn("acrobot", plant.GetTopologyGraphvizString())

    def test_scoped_name(self):
        ScopedName()
        ScopedName(namespace_name="foo", element_name="bar")
        ScopedName.Make(namespace_name="foo", element_name="bar")
        ScopedName.Join(name1="foo", name2="bar")
        dut = ScopedName.Parse(scoped_name="foo::bar")
        self.assertEqual(dut.get_namespace(), "foo")
        self.assertEqual(dut.get_element(), "bar")
        self.assertEqual(dut.get_full(), "foo::bar")
        self.assertEqual(dut.to_string(), "foo::bar")
        dut.set_namespace("robot1")
        dut.set_element("torso")
        self.assertEqual(str(dut), "robot1::torso")
        self.assertEqual(repr(dut), "ScopedName('robot1', 'torso')")
        copy.copy(dut)

    def _test_multibody_tree_element_mixin(self, T, element):
        cls = type(element)
        self.assertIsInstance(element.index(), get_index_class(cls, T))
        self.assertIsInstance(element.model_instance(), ModelInstanceIndex)
        self.assertFalse(element.is_ephemeral())
        element.GetParentPlant()

    def _test_frame_api(self, T, frame):
        Frame = Frame_[T]

        self.assertIsInstance(frame, Frame)
        self._test_multibody_tree_element_mixin(T, frame)

        self.assertIsInstance(frame.body(), RigidBody_[T])
        self.assertIsInstance(frame.is_world_frame(), bool)
        self.assertIsInstance(frame.is_body_frame(), bool)
        self.assertIsInstance(frame.name(), str)
        self.assertIsInstance(frame.scoped_name(), ScopedName)

        self.assertIsInstance(
            frame.GetFixedPoseInBodyFrame(), RigidTransform_[T]
        )
        self.assertIsInstance(
            frame.GetFixedRotationMatrixInBodyFrame(), RotationMatrix_[T]
        )

    def _test_body_api(self, T, body):
        Body = RigidBody_[T]

        self.assertIsInstance(body, Body)
        self._test_multibody_tree_element_mixin(T, body)
        self.assertIsInstance(body.name(), str)
        self.assertIsInstance(body.scoped_name(), ScopedName)
        self.assertIsInstance(body.is_floating_base_body(), bool)
        self.assertIsInstance(body.has_quaternion_dofs(), bool)
        self.assertIsInstance(body.default_mass(), float)
        with catch_drake_warnings(expected_count=1) as w:
            self.assertIsInstance(body.is_floating(), bool)
            self.assertIn("Use is_floating_base_body", str(w[0].message))
        # Other APIs can't be called on a Body that isn't part of
        # a multibody system.

    @numpy_compare.check_all_types
    def test_body_context_methods(self, T):
        plant = MultibodyPlant_[T](0.0)
        dut = plant.AddRigidBody(
            name="name",
            M_BBo_B=SpatialInertia_[float](
                mass=1.0,
                p_PScm_E=[0.0, 0.0, 0.0],
                G_SP_E=UnitInertia_[float](Ixx=1.0, Iyy=1.0, Izz=1.0),
            ),
        )
        plant.Finalize()
        context = plant.CreateDefaultContext()
        self.assertIsInstance(dut.get_mass(context=context), T)
        self.assertIsInstance(
            dut.CalcCenterOfMassInBodyFrame(context=context), np.ndarray
        )
        self.assertIsInstance(
            dut.CalcCenterOfMassTranslationalVelocityInWorld(context=context),
            np.ndarray,
        )
        self.assertIsInstance(
            dut.CalcCenterOfMassTranslationalAccelerationInWorld(
                context=context
            ),
            np.ndarray,
        )
        self.assertIsInstance(
            dut.CalcSpatialInertiaInBodyFrame(context=context),
            SpatialInertia_[T],
        )
        self.assertIsInstance(
            dut.EvalPoseInWorld(context=context), RigidTransform_[T]
        )
        self.assertIsInstance(
            dut.EvalSpatialVelocityInWorld(context=context), SpatialVelocity_[T]
        )
        self.assertIsInstance(
            dut.EvalSpatialAccelerationInWorld(context=context),
            SpatialAcceleration_[T],
        )
        # The test_multibody_dynamics already covers GetForceInWorld,
        # AddInForce, AddInForceInWorld.

        # Also test these body api methods which require a finalized plant.
        for i in range(7):
            self.assertIsNotNone(dut.floating_position_suffix(i))
        for i in range(6):
            self.assertIsNotNone(dut.floating_velocity_suffix(i))

    def _test_joint_api(self, T, joint):
        Joint = Joint_[T]
        Body = RigidBody_[T]
        Frame = Frame_[T]

        self.assertIsInstance(joint, Joint)
        self._test_multibody_tree_element_mixin(T, joint)
        self.assertIsInstance(joint.name(), str)
        self.assertIsInstance(joint.type_name(), str)
        self.assertEqual(joint.type_name(), joint.kTypeName)
        self.assertIsInstance(joint.parent_body(), Body)
        self.assertIsInstance(joint.child_body(), Body)
        self.assertIsInstance(joint.frame_on_parent(), Frame)
        self.assertIsInstance(joint.frame_on_child(), Frame)
        self.assertIsInstance(joint.num_positions(), int)
        self.assertIsInstance(joint.num_velocities(), int)
        self.assertIsInstance(joint.position_start(), int)
        self.assertIsInstance(joint.velocity_start(), int)

        nq = joint.num_positions()

        q_default = joint.default_positions()
        self.assertEqual(len(q_default), nq)
        joint.set_default_positions(default_positions=q_default)

        q_lower = joint.position_lower_limits()
        self.assertEqual(len(q_lower), nq)
        q_upper = joint.position_upper_limits()
        self.assertEqual(len(q_upper), nq)
        joint.set_position_limits(lower_limits=q_lower, upper_limits=q_upper)

        nv = joint.num_velocities()

        v_lower = joint.velocity_lower_limits()
        self.assertEqual(len(v_lower), nv)
        v_upper = joint.velocity_upper_limits()
        self.assertEqual(len(v_upper), nv)
        joint.set_velocity_limits(lower_limits=v_lower, upper_limits=v_upper)

        a_lower = joint.acceleration_lower_limits()
        self.assertEqual(len(a_lower), nv)
        a_upper = joint.acceleration_upper_limits()
        self.assertEqual(len(a_upper), nv)
        joint.set_acceleration_limits(
            lower_limits=a_lower, upper_limits=a_upper
        )

    def _test_joint_actuator_api(self, T, joint_actuator):
        JointActuator = JointActuator_[T]
        Joint = Joint_[T]
        self.assertIsInstance(joint_actuator, JointActuator)
        self._test_multibody_tree_element_mixin(T, joint_actuator)
        self.assertIsInstance(joint_actuator.name(), str)
        self.assertIsInstance(joint_actuator.joint(), Joint)
        self.assertIsInstance(joint_actuator.effort_limit(), float)
        joint_actuator.set_effort_limit(effort_limit=22.0)
        self.assertIsInstance(joint_actuator.default_rotor_inertia(), float)
        self.assertIsInstance(joint_actuator.default_gear_ratio(), float)
        joint_actuator.set_default_rotor_inertia(1.5)
        joint_actuator.set_default_gear_ratio(1.5)
        self.assertIsInstance(joint_actuator.default_reflected_inertia(), float)
        self.assertGreaterEqual(joint_actuator.input_start(), 0)
        self.assertEqual(joint_actuator.num_inputs(), 1)

        plant = MultibodyPlant_[T](0.0)
        plant.RemoveAllJointActuatorEffortLimits()

    def _test_rotational_inertia_or_unit_inertia_api(self, T, Class):
        """
        Tests the given Class (either RotationInertia or UnitInertia) for the
        API subset that they both share.
        """
        RotationMatrix = RotationMatrix_[T]
        Class()
        Class(Ixx=1.0, Iyy=1.0, Izz=1.0, Ixy=0.1, Ixz=0.1, Iyz=0.1)
        Class(Ixx=1.0, Iyy=1.0, Izz=1.0)
        dut = Class.TriaxiallySymmetric(I_triaxial=1.0)
        self.assertIsInstance(dut, Class)
        dut.rows()
        dut.cols()
        self.assertIsInstance(dut.get_moments(), np.ndarray)
        self.assertIsInstance(dut.get_products(), np.ndarray)
        self.assertIsInstance(dut.Trace(), T)
        dut.CalcMaximumPossibleMomentOfInertia()
        self.assertIsInstance(dut[0, 0], T)
        with self.assertRaisesRegex(
            Exception, r"incompatible function arguments"
        ):
            dut[0]
        with self.assertRaisesRegex(Exception, r"Unable to cast"):
            dut[0.0, 0.0]
        with self.assertRaisesRegex(Exception, r"Expected \[i,j\]"):
            dut[0, 0, 0]
        self.assertIsInstance(dut.CopyToFullMatrix3(), np.ndarray)
        dut.IsNearlyEqualTo(other=dut, precision=0.0)
        dut.SetToNaN()
        dut.IsNaN()
        dut.SetZero()
        self.assertIsInstance(dut.CalcPrincipalMomentsOfInertia(), np.ndarray)
        self.assertIsInstance(dut.CalcPrincipalMomentsAndAxesOfInertia(), tuple)
        self.assertEqual(len(dut.CalcPrincipalMomentsAndAxesOfInertia()), 2)
        self.assertIsInstance(
            dut.CalcPrincipalMomentsAndAxesOfInertia()[0], np.ndarray
        )
        self.assertIsInstance(
            dut.CalcPrincipalMomentsAndAxesOfInertia()[1],
            RotationMatrix_[float],
        )
        dut.CouldBePhysicallyValid()
        self.assertIsInstance(dut.ReExpress(R_AE=RotationMatrix()), Class)
        self.assertIsInstance(copy.copy(dut), Class)

    @numpy_compare.check_all_types
    def test_rotational_inertia_api(self, T):
        """Tests rotational inertia construction and API."""
        Class = RotationalInertia_[T]
        self._test_rotational_inertia_or_unit_inertia_api(T, Class)
        # Test methods present only on RotationalInernia, not UnitInertia.
        p = [0.1, 0.2, 0.3]
        dut = Class(mass=1.0, p_PQ_E=p)
        self.assertIsInstance(
            dut.ShiftFromCenterOfMass(mass=1.0, p_BcmQ_E=p), Class
        )
        self.assertIsInstance(
            dut.ShiftToCenterOfMass(mass=1.0, p_QBcm_E=p), Class
        )
        self.assertIsInstance(
            dut.ShiftToThenAwayFromCenterOfMass(
                mass=1.0, p_PBcm_E=p, p_QBcm_E=p
            ),
            Class,
        )
        # Test operators.
        zero = Class(Ixx=0.0, Iyy=0.0, Izz=0.0)
        if T != Expression:
            self.assertTrue(zero.IsZero())
        self.assertIsInstance(dut + zero, Class)
        self.assertIsInstance(dut - zero, Class)
        self.assertIsInstance(dut * T(1.0), Class)
        self.assertIsInstance(T(1.0) * dut, Class)
        self.assertIsInstance(dut * p, np.ndarray)
        self.assertIsInstance(dut / T(1.0), Class)
        dut += zero
        self.assertIsInstance(dut, Class)
        dut -= zero
        self.assertIsInstance(dut, Class)
        dut *= T(1.0)
        self.assertIsInstance(dut, Class)
        dut /= T(1.0)
        self.assertIsInstance(dut, Class)
        assert_pickle(self, dut, RotationalInertia_[T].CopyToFullMatrix3, T=T)

    def test_legacy_unpickle_tree_module(self):
        """Checks that data pickled as RotationalInertia_[float] in Drake
        v1.12.0 can be unpickled as RotationalInertia_𝓣float𝓤 in newer
        versions of Drake.

        Since the unpickling shim lives at the module level, testing one class
        is sufficient even though our module has several pickle-able classes.
        """
        legacy_data = b"\x80\x04\x95\r\x01\x00\x00\x00\x00\x00\x00\x8c\x16pydrake.multibody.tree\x94\x8c\x19RotationalInertia_[float]\x94\x93\x94)\x81\x94\x8c\x15numpy.core.multiarray\x94\x8c\x0c_reconstruct\x94\x93\x94\x8c\x05numpy\x94\x8c\x07ndarray\x94\x93\x94K\x00\x85\x94C\x01b\x94\x87\x94R\x94(K\x01K\x03K\x03\x86\x94h\x07\x8c\x05dtype\x94\x93\x94\x8c\x02f8\x94\x89\x88\x87\x94R\x94(K\x03\x8c\x01<\x94NNNJ\xff\xff\xff\xffJ\xff\xff\xff\xffK\x00t\x94b\x88CH\xa4p=\n\xd7\xa3\xc0?|\x14\xaeG\xe1z\x94\xbf\xb8\x1e\x85\xebQ\xb8\x9e\xbf|\x14\xaeG\xe1z\x94\xbf\x9a\x99\x99\x99\x99\x99\xb9?\xb8\x1e\x85\xebQ\xb8\xae\xbf\xb8\x1e\x85\xebQ\xb8\x9e\xbf\xb8\x1e\x85\xebQ\xb8\xae\xbf\x9b\x99\x99\x99\x99\x99\xa9?\x94t\x94bb."  # noqa
        obj = pickle.loads(legacy_data)
        self.assertIsInstance(obj, RotationalInertia_[float])
        expected_diag = np.array([0.13, 0.1, 0.05000000000000001])
        expected_off = np.array([-0.020000000000000004, -0.03, -0.06])
        numpy_compare.assert_float_equal(obj.get_moments(), expected_diag)
        numpy_compare.assert_float_equal(obj.get_products(), expected_off)

    @numpy_compare.check_all_types
    def test_unit_inertia_api(self, T):
        """Tests unit inertia construction and API."""
        UnitInertia = UnitInertia_[T]
        self._test_rotational_inertia_or_unit_inertia_api(T, UnitInertia)
        # Test methods present only on UnitInertia, not RotationalInertia.
        p = [0.1, 0.2, 0.3]
        dut = UnitInertia(I=RotationalInertia_[T](mass=1.0, p_PQ_E=p))
        self.assertIsInstance(
            dut.ShiftFromCenterOfMass(p_BcmQ_E=p), UnitInertia
        )
        self.assertIsInstance(dut.ShiftToCenterOfMass(p_QBcm_E=p), UnitInertia)
        assert_pickle(self, dut, UnitInertia.CopyToFullMatrix3, T=T)
        # N.B. There are NO valid operators on UnitInertia.  They are inherited
        # through implementation reuse, but they are broken (#6109).

        # The static constructors from "geometry".
        self.assertIsInstance(
            UnitInertia.PointMass(p_FQ=[1, 2, 3]), UnitInertia
        )
        self.assertIsInstance(
            UnitInertia.SolidEllipsoid(a=1, b=2, c=3), UnitInertia
        )
        self.assertIsInstance(UnitInertia.SolidSphere(r=1.5), UnitInertia)
        self.assertIsInstance(UnitInertia.HollowSphere(r=1.5), UnitInertia)
        self.assertIsInstance(
            UnitInertia.SolidBox(Lx=1, Ly=2, Lz=3), UnitInertia
        )
        self.assertIsInstance(UnitInertia.SolidCube(L=2), UnitInertia)
        self.assertIsInstance(
            UnitInertia.SolidCylinder(
                radius=1.5, length=2, unit_vector=[0, 0, 1]
            ),
            UnitInertia,
        )
        self.assertIsInstance(
            UnitInertia.SolidCapsule(radius=1, length=2, unit_vector=[0, 0, 1]),
            UnitInertia,
        )
        self.assertIsInstance(
            UnitInertia.SolidCylinderAboutEnd(
                radius=0.1, length=0.4, unit_vector=[0, 0, 1]
            ),
            UnitInertia,
        )
        self.assertIsInstance(
            UnitInertia.AxiallySymmetric(
                moment_parallel=1, moment_perpendicular=2, unit_vector=[0, 0, 1]
            ),
            UnitInertia,
        )
        self.assertIsInstance(
            UnitInertia.StraightLine(
                moment_perpendicular=1.5, unit_vector=[0, 0, 1]
            ),
            UnitInertia,
        )
        self.assertIsInstance(
            UnitInertia.ThinRod(length=1.5, unit_vector=[0, 0, 1]), UnitInertia
        )

    @numpy_compare.check_all_types
    def test_spatial_inertia_api(self, T):
        RotationalInertia = RotationalInertia_[T]
        UnitInertia = UnitInertia_[T]
        SpatialInertia = SpatialInertia_[T]
        RotationMatrix = RotationMatrix_[T]
        SpatialAcceleration = SpatialAcceleration_[T]
        SpatialForce = SpatialForce_[T]
        SpatialVelocity = SpatialVelocity_[T]
        SpatialMomentum = SpatialMomentum_[T]
        zero = SpatialInertia.Zero()
        if T != Expression:
            self.assertTrue(zero.IsZero())
        SpatialInertia.NaN()
        SpatialInertia.MakeFromCentralInertia(
            mass=1.3,
            p_PScm_E=[0.1, -0.2, 0.3],
            I_SScm_E=RotationalInertia(Ixx=2.0, Iyy=2.3, Izz=2.4),
        )
        SpatialInertia.SolidBoxWithDensity(density=1000, lx=0.1, ly=0.2, lz=0.3)
        SpatialInertia.SolidBoxWithMass(mass=0.123, lx=0.1, ly=0.2, lz=0.3)
        SpatialInertia.SolidCubeWithDensity(density=1000, length=0.4)
        SpatialInertia.SolidCapsuleWithDensity(
            density=1000, radius=0.1, length=0.4, unit_vector=[0, 0, 1]
        )
        SpatialInertia.SolidCapsuleWithMass(
            mass=0.123, radius=0.1, length=0.4, unit_vector=[0, 0, 1]
        )
        SpatialInertia.SolidCylinderWithDensity(
            density=1000, radius=0.1, length=0.4, unit_vector=[0, 0, 1]
        )
        SpatialInertia.SolidCylinderWithMass(
            mass=0.123, radius=0.1, length=0.4, unit_vector=[0, 0, 1]
        )
        SpatialInertia.SolidCylinderWithDensityAboutEnd(
            density=1000, radius=0.1, length=0.4, unit_vector=[0, 0, 1]
        )
        SpatialInertia.SolidCylinderWithMassAboutEnd(
            mass=0.123, radius=0.1, length=0.4, unit_vector=[0, 0, 1]
        )
        SpatialInertia.ThinRodWithMass(
            mass=2, length=0.3, unit_vector=[0, 0, 1]
        )
        SpatialInertia.ThinRodWithMassAboutEnd(
            mass=2, length=0.3, unit_vector=[0, 0, 1]
        )
        SpatialInertia.SolidEllipsoidWithDensity(
            density=1000, a=0.2, b=0.3, c=0.4
        )
        SpatialInertia.SolidEllipsoidWithMass(mass=0.123, a=0.2, b=0.3, c=0.4)
        SpatialInertia.SolidSphereWithDensity(density=1000, radius=0.5)
        SpatialInertia.SolidSphereWithMass(mass=2, radius=0.5)
        SpatialInertia.HollowSphereWithDensity(area_density=2, radius=0.5)
        SpatialInertia.HollowSphereWithMass(mass=2, radius=0.5)
        spatial_inertia = SpatialInertia(
            mass=2.5,
            p_PScm_E=[0.1, -0.2, 0.3],
            G_SP_E=UnitInertia(Ixx=2.0, Iyy=2.3, Izz=2.4),
        )
        numpy_compare.assert_float_equal(spatial_inertia.get_mass(), 2.5)
        copy.deepcopy(spatial_inertia)
        self.assertIsInstance(spatial_inertia.get_com(), np.ndarray)
        self.assertIsInstance(spatial_inertia.CalcComMoment(), np.ndarray)
        self.assertIsInstance(spatial_inertia.get_unit_inertia(), UnitInertia)
        self.assertIsInstance(
            spatial_inertia.CalcRotationalInertia(), RotationalInertia
        )
        # N.B. `numpy_compare.assert_equal(IsPhysicallyValid(), True)` does not
        # work.
        if T != Expression:
            self.assertTrue(spatial_inertia.IsPhysicallyValid())
        (semi_diameters, transform) = (
            spatial_inertia.CalcPrincipalSemiDiametersAndPoseForSolidEllipsoid()
        )
        self.assertIsInstance(semi_diameters, np.ndarray)
        self.assertIsInstance(transform, RigidTransform)
        self.assertIsInstance(spatial_inertia.CopyToFullMatrix6(), np.ndarray)
        self.assertIsInstance(
            spatial_inertia.ReExpress(R_AE=RotationMatrix()), SpatialInertia
        )
        self.assertIsInstance(spatial_inertia.Shift([1, 2, 3]), SpatialInertia)
        spatial_inertia += spatial_inertia
        self.assertIsInstance(
            spatial_inertia * SpatialAcceleration(), SpatialForce
        )
        self.assertIsInstance(
            spatial_inertia * SpatialVelocity(), SpatialMomentum
        )
        self.assertIn(
            "<pydrake.multibody.tree.SpatialInertia", repr(spatial_inertia)
        )
        if T is float:
            # This one is used as a default argument, so it's important that we
            # print it out in full.
            self.assertEqual(
                repr(SpatialInertia.Zero()), "SpatialInertia.Zero()"
            )
        assert_pickle(
            self, spatial_inertia, SpatialInertia.CopyToFullMatrix6, T=T
        )
        spatial_inertia.SetNaN()
        # N.B. `numpy_compare.assert_equal(IsNaN(), True)` does not work.
        if T != Expression:
            self.assertTrue(spatial_inertia.IsNaN())

    def test_geometry_spatial_inertia_apis(self):
        box = Box(1, 2, 3)
        M_BBo_B = CalcSpatialInertia(shape=box, density=2.5)
        self.assertIsInstance(M_BBo_B, SpatialInertia_[float])

        t_a = SurfaceTriangle(v0=0, v1=1, v2=2)
        v0 = (0, 0, 1)
        v1 = (1, 0, 0)
        v2 = (0, 1, 0)
        mesh = TriangleSurfaceMesh(triangles=(t_a,), vertices=(v0, v1, v2))
        M_MMo_M = CalcSpatialInertia(mesh=mesh, density=2.5)
        self.assertIsInstance(M_MMo_M, SpatialInertia_[float])

    @numpy_compare.check_all_types
    def test_friction_api(self, T):
        CoulombFriction = CoulombFriction_[T]
        CoulombFriction()
        CoulombFriction(static_friction=0.7, dynamic_friction=0.6)
        copy.copy(CoulombFriction())
        assert_pickle(
            self,
            CoulombFriction,
            lambda x: [x.static_friction, x.dynamic_friction],
            T=T,
        )

    def test_legacy_unpickle_plant_module(self):
        """Checks that data pickled as CoulombFriction_[float] in Drake
        v1.12.0 can be unpickled as CoulombFriction_𝓣float𝓤 in newer
        versions of Drake.
        """
        legacy_data = b"\x80\x04\x95O\x00\x00\x00\x00\x00\x00\x00\x8c\x17pydrake.multibody.plant\x94\x8c\x17CoulombFriction_[float]\x94\x93\x94)\x81\x94G?\xe0\x00\x00\x00\x00\x00\x00G?\xd0\x00\x00\x00\x00\x00\x00\x86\x94b."  # noqa
        obj = pickle.loads(legacy_data)
        self.assertIsInstance(obj, CoulombFriction_[float])
        self.assertEqual(obj.static_friction(), 0.5)
        self.assertEqual(obj.dynamic_friction(), 0.25)

    @numpy_compare.check_all_types
    def test_rigid_body_api(self, T):
        TemplatedRigidBody = RigidBody_[T]
        M = SpatialInertia_[float].NaN()
        i = ModelInstanceIndex(0)
        TemplatedRigidBody(body_name="body_name")
        TemplatedRigidBody(body_name="body_name", model_instance=i)
        TemplatedRigidBody(body_name="body_name", M_BBo_B=M)
        TemplatedRigidBody(body_name="body_name", model_instance=i, M_BBo_B=M)
        # Make sure the default (float) version also works.
        RigidBody(body_name="body_name", M_BBo_B=M)

    @numpy_compare.check_all_types
    def test_multibody_force_element(self, T):
        MultibodyPlant = MultibodyPlant_[T]
        LinearSpringDamper = LinearSpringDamper_[T]
        PrismaticSpring = PrismaticSpring_[T]
        RevoluteSpring = RevoluteSpring_[T]
        DoorHinge = DoorHinge_[T]
        LinearBushingRollPitchYaw = LinearBushingRollPitchYaw_[T]
        SpatialInertia = SpatialInertia_[float]
        UnitInertia = UnitInertia_[float]

        plant = MultibodyPlant(0.0)
        spatial_inertia = SpatialInertia(
            mass=1,
            p_PScm_E=[0, 0, 0],
            G_SP_E=UnitInertia(1, 1, 1),
        )
        body_a = plant.AddRigidBody(name="body_a", M_BBo_B=spatial_inertia)
        body_b = plant.AddRigidBody(name="body_b", M_BBo_B=spatial_inertia)
        body_c = plant.AddRigidBody(name="body_c", M_BBo_B=spatial_inertia)
        linear_spring_index = ForceElementIndex(plant.num_force_elements())
        linear_spring = plant.AddForceElement(
            LinearSpringDamper(
                bodyA=body_a,
                p_AP=[0.0, 0.0, 0.0],
                bodyB=body_b,
                p_BQ=[0.0, 0.0, 0.0],
                free_length=1.0,
                stiffness=2.0,
                damping=3.0,
            )
        )
        if T is float:
            self.assertEqual(
                repr(linear_spring),
                "<LinearSpringDamper index=1 model_instance=1>",
            )
        revolute_joint = plant.AddJoint(
            RevoluteJoint_[T](
                name="revolve_joint",
                frame_on_parent=body_a.body_frame(),
                frame_on_child=body_b.body_frame(),
                axis=[0, 0, 1],
                damping=0.0,
            )
        )
        revolute_spring = plant.AddForceElement(
            RevoluteSpring(
                joint=revolute_joint, nominal_angle=0.1, stiffness=100.0
            )
        )
        door_hinge_config = DoorHingeConfig()
        door_hinge = plant.AddForceElement(
            DoorHinge(joint=revolute_joint, config=door_hinge_config)
        )
        prismatic_joint = plant.AddJoint(
            PrismaticJoint_[T](
                name="prismatic_joint",
                frame_on_parent=body_b.body_frame(),
                frame_on_child=body_c.body_frame(),
                axis=[0, 0, 1],
                damping=0.0,
            )
        )
        prismatic_spring = plant.AddForceElement(
            PrismaticSpring(
                joint=prismatic_joint, nominal_position=0.1, stiffness=100.0
            )
        )

        torque_stiffness = np.array([10.0, 11.0, 12.0])
        torque_damping = np.array([1.0, 1.1, 1.2])
        force_stiffness = np.array([20.0, 21.0, 22.0])
        force_damping = np.array([2.0, 2.1, 2.2])
        bushing = plant.AddForceElement(
            LinearBushingRollPitchYaw(
                frameA=body_a.body_frame(),
                frameC=body_b.body_frame(),
                torque_stiffness_constants=torque_stiffness,
                torque_damping_constants=torque_damping,
                force_stiffness_constants=force_stiffness,
                force_damping_constants=force_damping,
            )
        )
        plant.Finalize()
        context = plant.CreateDefaultContext()

        # Test LinearSpringDamper accessors
        self.assertEqual(linear_spring.bodyA(), body_a)
        self.assertEqual(linear_spring.bodyB(), body_b)
        np.testing.assert_array_equal(linear_spring.p_AP(), [0, 0, 0])
        np.testing.assert_array_equal(linear_spring.p_BQ(), [0, 0, 0])
        self.assertEqual(linear_spring.free_length(), 1.0)
        self.assertEqual(linear_spring.stiffness(), 2.0)
        self.assertEqual(linear_spring.damping(), 3.0)

        # Test RevoluteSpring accessors
        self.assertEqual(revolute_spring.joint(), revolute_joint)
        self.assertEqual(revolute_spring.default_nominal_angle(), 0.1)
        self.assertEqual(revolute_spring.default_stiffness(), 100.0)

        # Test DoorHinge accessors
        self.assertEqual(door_hinge.joint(), revolute_joint)
        door_hinge_config_test = door_hinge.config()
        # Only test two members since the rest would be the same
        self.assertEqual(
            door_hinge_config_test.spring_zero_angle_rad,
            door_hinge_config.spring_zero_angle_rad,
        )
        self.assertEqual(
            door_hinge_config_test.static_friction_torque,
            door_hinge_config.static_friction_torque,
        )
        self.assertIn("spring_zero_angle_rad", repr(door_hinge_config))

        # Test DoorHinge torque calculation. Set the angle to be the half of
        # the catch width so that there is only torsional spring torque which
        # is easy to compute by hand.
        if T is float:
            self.assertEqual(
                door_hinge.CalcHingeFrictionalTorque(angular_rate=0.0), 0.0
            )
            self.assertEqual(
                door_hinge.CalcHingeSpringTorque(angle=0.01), -2.265
            )
            self.assertEqual(
                door_hinge.CalcHingeTorque(angle=0.01, angular_rate=0.0), -2.265
            )

        # Test PrismaticSpring accessors
        self.assertEqual(prismatic_spring.joint(), prismatic_joint)
        self.assertEqual(prismatic_spring.nominal_position(), 0.1)
        self.assertEqual(prismatic_spring.stiffness(), 100.0)

        # Test LinearBushingRollPitchYaw accessors.
        self.assertIs(bushing.link0(), body_a)
        self.assertIs(bushing.link1(), body_b)
        self.assertIs(bushing.frameA(), body_a.body_frame())
        self.assertIs(bushing.frameC(), body_b.body_frame())
        numpy_compare.assert_float_equal(
            bushing.torque_stiffness_constants(), torque_stiffness
        )
        numpy_compare.assert_float_equal(
            bushing.torque_damping_constants(), torque_damping
        )
        numpy_compare.assert_float_equal(
            bushing.force_stiffness_constants(), force_stiffness
        )
        numpy_compare.assert_float_equal(
            bushing.force_damping_constants(), force_damping
        )
        # Set to 2x the original value and ensure it was updated in the given
        # context.
        bushing.SetTorqueStiffnessConstants(
            context=context, torque_stiffness=2 * torque_stiffness
        )
        numpy_compare.assert_float_equal(
            bushing.GetTorqueStiffnessConstants(context=context),
            2 * torque_stiffness,
        )
        bushing.SetTorqueDampingConstants(
            context=context, torque_damping=2 * torque_damping
        )
        numpy_compare.assert_float_equal(
            bushing.GetTorqueDampingConstants(context=context),
            2 * torque_damping,
        )
        bushing.SetForceStiffnessConstants(
            context=context, force_stiffness=2 * force_stiffness
        )
        numpy_compare.assert_float_equal(
            bushing.GetForceStiffnessConstants(context=context),
            2 * force_stiffness,
        )
        bushing.SetForceDampingConstants(
            context=context, force_damping=2 * force_damping
        )
        numpy_compare.assert_float_equal(
            bushing.GetForceDampingConstants(context=context), 2 * force_damping
        )

        # Test get_force_element().
        self.assertIs(
            plant.get_force_element(force_element_index=linear_spring_index),
            linear_spring,
        )

    @numpy_compare.check_all_types
    def test_multibody_gravity_default(self, T):
        MultibodyPlant = MultibodyPlant_[T]
        UniformGravityFieldElement = UniformGravityFieldElement_[T]
        plant = MultibodyPlant(0.0)
        with self.assertRaises(RuntimeError):
            plant.AddForceElement(UniformGravityFieldElement())
        self.assertIsInstance(
            UniformGravityFieldElement.kDefaultStrength, float
        )

    @numpy_compare.check_all_types
    def test_multibody_tree_kinematics_continuous(self, T):
        self.do_test_multibody_tree_kinematics(T, 0.0)

    @numpy_compare.check_all_types
    def test_multibody_tree_kinematics_discrete(self, T):
        self.do_test_multibody_tree_kinematics(T, 0.001)

    def do_test_multibody_tree_kinematics(self, T, time_step):
        RigidTransform = RigidTransform_[T]
        SpatialVelocity = SpatialVelocity_[T]
        plant_f = MultibodyPlant_[float](time_step)

        file_name = FindResourceOrThrow(
            "drake/bindings/pydrake/multibody/test/double_pendulum.sdf"
        )
        # N.B. `Parser` only supports `MultibodyPlant_[float]`.
        (instance,) = Parser(plant_f).AddModels(file_name)
        plant_f.Finalize()
        plant = to_type(plant_f, T)
        context = plant.CreateDefaultContext()
        world_frame = plant.world_frame()
        base = plant.GetBodyByName("base")
        base_frame = plant.GetFrameByName("base")
        X_WL = plant.CalcRelativeTransform(
            context=context, frame_A=world_frame, frame_B=base_frame
        )
        self.assertIsInstance(X_WL, RigidTransform)
        R_WL = plant.CalcRelativeRotationMatrix(
            context=context, frame_A=world_frame, frame_B=base_frame
        )
        self.assertIsInstance(R_WL, RotationMatrix_[T])
        free_bodies = plant.GetFloatingBaseBodies()
        self.assertEqual(len(free_bodies), 1)
        self.assertTrue(base.index() in free_bodies)

        self.assertFalse(base.is_locked(context))
        if time_step:
            base.Lock(context)
            self.assertTrue(base.is_locked(context))
            base.Unlock(context)
            self.assertFalse(base.is_locked(context))

        p_AQi = plant.CalcPointsPositions(
            context=context,
            frame_B=base_frame,
            p_BQi=np.array([[0, 1, 2], [10, 11, 12]]).T,
            frame_A=world_frame,
        ).T
        self.assertTupleEqual(p_AQi.shape, (2, 3))

        v_AQi_E = plant.CalcPointsVelocities(
            context=context,
            frame_B=base_frame,
            p_BQi=np.array([[0, 1, 2], [10, 11, 12]]).T,
            frame_A=world_frame,
            frame_E=world_frame,
        ).T
        self.assertTupleEqual(v_AQi_E.shape, (2, 3))

        # Verify CalcTotalMass() calculates a non-zero mass.
        p_mass = plant.CalcTotalMass(context=context)
        numpy_compare.assert_float_not_equal(p_mass, 0.0)
        p_mass = plant.CalcTotalMass(
            context=context, model_instances=[instance]
        )
        numpy_compare.assert_float_not_equal(p_mass, 0.0)

        p_com = plant.CalcCenterOfMassPositionInWorld(context=context)
        self.assertTupleEqual(p_com.shape, (3,))
        p_com = plant.CalcCenterOfMassPositionInWorld(
            context=context, model_instances=[instance]
        )
        self.assertTupleEqual(p_com.shape, (3,))

        v_com = plant.CalcCenterOfMassTranslationalVelocityInWorld(
            context=context
        )
        self.assertTupleEqual(v_com.shape, (3,))
        v_com = plant.CalcCenterOfMassTranslationalVelocityInWorld(
            context=context, model_instances=[instance]
        )
        self.assertTupleEqual(v_com.shape, (3,))

        # Test center of mass translational acceleration for entire plant.
        if T == Expression and plant.time_step() != 0:
            # Discrete time dynamics are not supported for symbolic scalars.
            with self.assertRaises(Exception) as cm:
                a_com = plant.CalcCenterOfMassTranslationalAccelerationInWorld(
                    context=context
                )
            self.assertIn("Expression", str(cm.exception))
        else:
            a_com = plant.CalcCenterOfMassTranslationalAccelerationInWorld(
                context=context
            )
            self.assertTupleEqual(a_com.shape, (3,))

        # Test center of mass translational acceleration for model_instances.
        if T == Expression and plant.time_step() != 0:
            # Discrete time dynamics are not supported for symbolic scalars.
            with self.assertRaises(Exception) as cm:
                a_com = plant.CalcCenterOfMassTranslationalAccelerationInWorld(
                    context=context, model_instances=[instance]
                )
            self.assertIn("Expression", str(cm.exception))
        else:
            a_com = plant.CalcCenterOfMassTranslationalAccelerationInWorld(
                context=context, model_instances=[instance]
            )
            self.assertTupleEqual(a_com.shape, (3,))

        M_WWo_W = plant.CalcSpatialInertia(
            context=context, frame_F=world_frame, body_indexes=[BodyIndex(0)]
        )
        self.assertIsInstance(M_WWo_W, SpatialInertia_[T])

        nq = plant.num_positions()
        nv = plant.num_velocities()
        wrt_list = [
            (JacobianWrtVariable.kQDot, nq),
            (JacobianWrtVariable.kV, nv),
        ]
        for wrt, nw in wrt_list:
            Js_v_ACcm_E = plant.CalcJacobianCenterOfMassTranslationalVelocity(
                context=context,
                with_respect_to=wrt,
                frame_A=base_frame,
                frame_E=world_frame,
            )
            self.assert_sane(Js_v_ACcm_E)
            self.assertEqual(Js_v_ACcm_E.shape, (3, nw))

            Js_v_ACcm_E = plant.CalcJacobianCenterOfMassTranslationalVelocity(
                context=context,
                model_instances=[instance],
                with_respect_to=wrt,
                frame_A=base_frame,
                frame_E=world_frame,
            )
            self.assert_sane(Js_v_ACcm_E)
            self.assertEqual(Js_v_ACcm_E.shape, (3, nw))

            Js_V_ABp_E = plant.CalcJacobianSpatialVelocity(
                context=context,
                with_respect_to=wrt,
                frame_B=base_frame,
                p_BoBp_B=np.zeros(3),
                frame_A=world_frame,
                frame_E=world_frame,
            )
            self.assert_sane(Js_V_ABp_E)

            self.assertEqual(Js_V_ABp_E.shape, (6, nw))
            Js_w_AB_E = plant.CalcJacobianAngularVelocity(
                context=context,
                with_respect_to=wrt,
                frame_B=base_frame,
                frame_A=world_frame,
                frame_E=world_frame,
            )
            self.assert_sane(Js_w_AB_E)
            self.assertEqual(Js_w_AB_E.shape, (3, nw))

            Js_v_AB_E = plant.CalcJacobianTranslationalVelocity(
                context=context,
                with_respect_to=wrt,
                frame_B=base_frame,
                p_BoBi_B=np.zeros(3),
                frame_A=world_frame,
                frame_E=world_frame,
            )
            self.assert_sane(Js_v_AB_E)
            self.assertEqual(Js_v_AB_E.shape, (3, nw))
            Js_v_AB_E = plant.CalcJacobianTranslationalVelocity(
                context=context,
                with_respect_to=wrt,
                frame_B=base_frame,
                p_BoBi_B=np.zeros((3, 3)),
                frame_A=world_frame,
                frame_E=world_frame,
            )
            self.assert_sane(Js_v_AB_E)
            self.assertEqual(Js_v_AB_E.shape, (9, nw))

            Jq_p_AoBi_E = plant.CalcJacobianPositionVector(
                context=context,
                frame_B=base_frame,
                p_BoBi_B=np.zeros(3),
                frame_A=world_frame,
                frame_E=world_frame,
            )
            self.assert_sane(Jq_p_AoBi_E)
            self.assertEqual(Jq_p_AoBi_E.shape, (3, nq))
            Jq_p_AoBi_E = plant.CalcJacobianPositionVector(
                context=context,
                frame_B=base_frame,
                p_BoBi_B=np.zeros((3, 3)),
                frame_A=world_frame,
                frame_E=world_frame,
            )
            self.assert_sane(Jq_p_AoBi_E)
            self.assertEqual(Jq_p_AoBi_E.shape, (9, nq))

        L_WSP_W = plant.CalcSpatialMomentumInWorldAboutPoint(
            context=context, p_WoP_W=np.zeros(3)
        )
        self.assert_sane(L_WSP_W.translational(), nonzero=False)
        self.assert_sane(L_WSP_W.rotational(), nonzero=False)
        self.assertIsInstance(L_WSP_W, SpatialMomentum_[T])
        L_WSP_W = plant.CalcSpatialMomentumInWorldAboutPoint(
            context=context,
            model_instances=[default_model_instance()],
            p_WoP_W=np.zeros(3),
        )
        self.assert_sane(L_WSP_W.translational(), nonzero=False)
        self.assert_sane(L_WSP_W.rotational(), nonzero=False)
        self.assertIsInstance(L_WSP_W, SpatialMomentum_[T])

        aBias_ACcm_E = plant.CalcBiasCenterOfMassTranslationalAcceleration(
            context=context,
            with_respect_to=JacobianWrtVariable.kV,
            frame_A=world_frame,
            frame_E=world_frame,
        )
        self.assert_sane(aBias_ACcm_E, nonzero=False)
        self.assertEqual(aBias_ACcm_E.shape, (3,))
        aBias_ACcm_E = plant.CalcBiasCenterOfMassTranslationalAcceleration(
            context=context,
            model_instances=[instance],
            with_respect_to=wrt,
            frame_A=world_frame,
            frame_E=world_frame,
        )
        self.assert_sane(aBias_ACcm_E, nonzero=False)
        self.assertEqual(aBias_ACcm_E.shape, (3,))

        AsBias_ABp_E = plant.CalcBiasSpatialAcceleration(
            context=context,
            with_respect_to=JacobianWrtVariable.kV,
            frame_B=base_frame,
            p_BoBp_B=np.zeros(3),
            frame_A=world_frame,
            frame_E=world_frame,
        )
        self.assert_sane(AsBias_ABp_E.rotational(), nonzero=False)
        self.assert_sane(AsBias_ABp_E.translational(), nonzero=False)
        asBias_ABi_E = plant.CalcBiasTranslationalAcceleration(
            context=context,
            with_respect_to=JacobianWrtVariable.kV,
            frame_B=base_frame,
            p_BoBi_B=np.zeros(3),
            frame_A=world_frame,
            frame_E=world_frame,
        )
        self.assert_sane(asBias_ABi_E, nonzero=False)
        self.assertEqual(asBias_ABi_E.shape, (3, 1))

        # Compute body pose.
        X_WBase = plant.EvalBodyPoseInWorld(context, base)
        self.assertIsInstance(X_WBase, RigidTransform)

        # Set pose for the base.
        X_WB_desired = RigidTransform.Identity()
        X_WB = plant.CalcRelativeTransform(context, world_frame, base_frame)
        plant.SetFreeBodyPose(context=context, body=base, X_JpJc=X_WB_desired)
        numpy_compare.assert_float_equal(
            X_WB.GetAsMatrix4(),
            numpy_compare.to_float(X_WB_desired.GetAsMatrix4()),
        )
        with catch_drake_warnings(expected_count=1) as w:
            plant.SetFreeBodyPose(context=context, body=base, X_PB=X_WB_desired)
            self.assertIn("Use X_JpJc instead", str(w[0].message))

        # Compute spatial accelerations for base.
        if T == Expression and plant.time_step() != 0:
            # N.B. Discrete time dynamics are not supported for symbolic
            # scalars.
            with self.assertRaises(Exception) as cm:
                A_base = plant.EvalBodySpatialAccelerationInWorld(context, base)
            self.assertIn("Expression", str(cm.exception))
        else:
            A_base = plant.EvalBodySpatialAccelerationInWorld(context, base)
            self.assert_sane(A_base.rotational(), nonzero=False)
            self.assert_sane(A_base.translational(), nonzero=False)

        # Set a spatial velocity for the base.
        V_WB = SpatialVelocity(w=[1, 2, 3], v=[4, 5, 6])
        plant.SetFreeBodySpatialVelocity(
            context=context, body=base, V_JpJc=V_WB
        )
        V_base = plant.EvalBodySpatialVelocityInWorld(context, base)
        numpy_compare.assert_float_equal(
            V_base.rotational(), numpy_compare.to_float(V_WB.rotational())
        )
        numpy_compare.assert_float_equal(
            V_base.translational(), numpy_compare.to_float(V_WB.translational())
        )
        with catch_drake_warnings(expected_count=1) as w:
            plant.SetFreeBodySpatialVelocity(base, V_PB=V_WB, context=context)
            self.assertIn(
                "Use context, body, V_JpJc instead", str(w[0].message)
            )

        # Compute accelerations.
        vdot = np.zeros(nv)
        A_WB_array = plant.CalcSpatialAccelerationsFromVdot(
            context=context, known_vdot=vdot
        )
        self.assertEqual(len(A_WB_array), plant.num_bodies())

        # Make sure we can call these methods.
        base.SetMass(context=context, mass=1.0)
        base.SetCenterOfMassInBodyFrame(context=context, com=[0.0, 0.0, 0.0])
        M = SpatialInertia_[T](1, [0, 0, 0], UnitInertia_[T](1, 1, 1))
        base.SetSpatialInertiaInBodyFrame(context=context, M_Bo_B=M)

    @numpy_compare.check_all_types
    def test_multibody_state_access(self, T):
        plant_f = MultibodyPlant_[float](0.0)
        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf"
        )
        # N.B. `Parser` only supports `MultibodyPlant_[float]`.
        (instance,) = Parser(plant_f).AddModels(file_name)
        plant_f.Finalize()
        plant = to_type(plant_f, T)
        context = plant.CreateDefaultContext()

        nq = 2
        nv = 2
        nu = 1
        self.assertEqual(plant.num_positions(), nq)
        self.assertEqual(plant.num_velocities(), nv)
        self.assertEqual(plant.num_actuators(), nu)

        q0 = np.array([3.14, 2.0])
        v0 = np.array([-0.5, 1.0])
        x0 = np.concatenate([q0, v0])

        # The default state is all values set to zero.
        x = plant.GetPositionsAndVelocities(context)
        numpy_compare.assert_float_equal(x, np.zeros(4))

        # A convenience function to reset back to zero.
        def set_zero():
            plant.SetPositionsAndVelocities(context, np.zeros(nq + nv))

        # Change x to non-zero values.
        plant.SetPositionsAndVelocities(context, x0)

        # Verify that positions and velocities were set correctly.
        numpy_compare.assert_float_equal(plant.GetPositions(context), q0)
        numpy_compare.assert_float_equal(plant.GetVelocities(context), v0)

        # Verify we did modify the state stored in context.
        x = plant.GetPositionsAndVelocities(context)
        numpy_compare.assert_float_equal(x, x0)

        # Now set positions and velocities independently and check them.
        zeros_2 = np.zeros(
            [
                2,
            ]
        )
        set_zero()
        plant.SetPositions(context, q0)
        numpy_compare.assert_float_equal(plant.GetPositions(context), q0)
        numpy_compare.assert_float_equal(plant.GetVelocities(context), zeros_2)
        set_zero()
        plant.SetVelocities(context, v0)
        numpy_compare.assert_float_allclose(
            plant.GetPositions(context), zeros_2
        )
        numpy_compare.assert_float_allclose(plant.GetVelocities(context), v0)

        # Now test SetPositionsAndVelocities().
        set_zero()
        plant.SetPositionsAndVelocities(context, x0)
        numpy_compare.assert_float_allclose(
            plant.GetPositionsAndVelocities(context), x0
        )

        # Test Get/SetDefaultPositions
        plant.SetDefaultPositions(q=q0)
        plant.GetDefaultPositions()
        plant.SetDefaultPositions(model_instance=instance, q_instance=q0)
        plant.GetDefaultPositions(model_instance=instance)

        # Test existence of context resetting methods.
        plant.SetDefaultState(context, state=context.get_mutable_state())

        self.assertEqual(
            plant.GetPositionNames(
                add_model_instance_prefix=False, always_add_suffix=False
            ),
            ["ShoulderJoint", "ElbowJoint"],
        )
        self.assertEqual(
            plant.GetPositionNames(
                model_instance=instance,
                add_model_instance_prefix=False,
                always_add_suffix=False,
            ),
            ["ShoulderJoint", "ElbowJoint"],
        )
        self.assertEqual(
            plant.GetVelocityNames(
                model_instance=instance,
                add_model_instance_prefix=False,
                always_add_suffix=False,
            ),
            ["ShoulderJoint", "ElbowJoint"],
        )
        self.assertEqual(
            plant.GetVelocityNames(
                add_model_instance_prefix=False, always_add_suffix=False
            ),
            ["ShoulderJoint", "ElbowJoint"],
        )
        self.assertEqual(
            plant.GetStateNames(add_model_instance_prefix=False),
            [
                "ShoulderJoint_q",
                "ElbowJoint_q",
                "ShoulderJoint_w",
                "ElbowJoint_w",
            ],
        )
        self.assertEqual(
            plant.GetStateNames(
                model_instance=instance, add_model_instance_prefix=False
            ),
            [
                "ShoulderJoint_q",
                "ElbowJoint_q",
                "ShoulderJoint_w",
                "ElbowJoint_w",
            ],
        )
        self.assertEqual(
            plant.GetActuatorNames(add_model_instance_prefix=False),
            ["ElbowJoint"],
        )
        self.assertEqual(
            plant.GetActuatorNames(
                model_instance=instance, add_model_instance_prefix=False
            ),
            ["ElbowJoint"],
        )

        # Test existence of limits.
        self.assertEqual(plant.GetPositionLowerLimits().shape, (nq,))
        self.assertEqual(plant.GetPositionUpperLimits().shape, (nq,))
        self.assertEqual(plant.GetVelocityLowerLimits().shape, (nv,))
        self.assertEqual(plant.GetVelocityUpperLimits().shape, (nv,))
        self.assertEqual(plant.GetAccelerationLowerLimits().shape, (nv,))
        self.assertEqual(plant.GetAccelerationUpperLimits().shape, (nv,))
        self.assertEqual(plant.GetEffortLowerLimits().shape, (nu,))
        self.assertEqual(plant.GetEffortUpperLimits().shape, (nu,))

    @numpy_compare.check_all_types
    def test_default_free_body_pose(self, T):
        plant = MultibodyPlant_[T](0.0)
        body = plant.AddRigidBody("body")
        plant.Finalize()
        X_WB_default = RigidTransform_[float]([1, 2, 3])
        plant.SetDefaultFloatingBaseBodyPose(body=body, X_WB=X_WB_default)
        with catch_drake_warnings(expected_count=1) as w:
            plant.SetDefaultFreeBodyPose(body=body, X_PB=X_WB_default)
            self.assertIn(
                "Use SetDefaultFloatingBaseBodyPose", str(w[0].message)
            )
        numpy_compare.assert_float_equal(
            plant.GetDefaultFloatingBaseBodyPose(body=body).GetAsMatrix4(),
            X_WB_default.GetAsMatrix4(),
        )
        with catch_drake_warnings(expected_count=1) as w:
            numpy_compare.assert_float_equal(
                plant.GetDefaultFreeBodyPose(body=body).GetAsMatrix4(),
                X_WB_default.GetAsMatrix4(),
            )
            self.assertIn(
                "Use GetDefaultFloatingBaseBodyPose", str(w[0].message)
            )

    @numpy_compare.check_all_types
    def test_port_access(self, T):
        # N.B. We actually test the values because some of the value bindings
        # are somewhat special snowflakes.
        OutputPort = OutputPort_[T]
        plant_f = MultibodyPlant_[float](0.0)
        # Create a MultibodyPlant with a kuka arm and a schunk gripper.
        # the arm is welded to the world, the gripper is welded to the
        # arm's end effector.
        wsg50_sdf_url = (
            "package://drake_models/wsg_50_description/sdf/schunk_wsg_50.sdf"
        )
        iiwa_sdf_url = (
            "package://drake_models/"
            "iiwa_description/sdf/iiwa14_no_collision.sdf"
        )

        # N.B. `Parser` only supports `MultibodyPlant_[float]`.
        plant_f = MultibodyPlant_[float](time_step=2e-3)
        parser = Parser(plant_f)
        (iiwa_model,) = parser.AddModels(url=iiwa_sdf_url)
        (gripper_model,) = parser.AddModels(url=wsg50_sdf_url)
        plant_f.Finalize()
        plant = to_type(plant_f, T)
        models = [iiwa_model, gripper_model]

        # Fix inputs.
        context = plant.CreateDefaultContext()
        for model in models:
            nu = plant.num_actuated_dofs(model)
            plant.get_actuation_input_port(model_instance=model).FixValue(
                context, np.zeros(nu)
            )

        # Evaluate outputs.
        for model in models:
            self.assertIsInstance(
                plant.get_state_output_port(model_instance=model).Eval(context),
                np.ndarray,
            )
            if T == Expression:
                continue
            self.assertIsInstance(
                plant.get_generalized_acceleration_output_port(
                    model_instance=model
                ).Eval(context),
                np.ndarray,
            )
            self.assertIsInstance(
                plant.get_generalized_contact_forces_output_port(
                    model_instance=model
                ).Eval(context),
                np.ndarray,
            )

        def extract_list_value(port):
            self.assertIsInstance(port, OutputPort)
            value = port.Eval(context)
            self.assertIsInstance(value, list)
            self.assertGreater(len(value), 0)
            return value[0]

        self.assertIsInstance(
            extract_list_value(plant.get_body_poses_output_port()),
            RigidTransform_[T],
        )
        self.assertIsInstance(
            extract_list_value(plant.get_body_spatial_velocities_output_port()),
            SpatialVelocity_[T],
        )
        if T != Expression:
            self.assertIsInstance(
                extract_list_value(
                    plant.get_body_spatial_accelerations_output_port()
                ),
                SpatialAcceleration_[T],
            )
        # TODO(eric.cousineau): Merge `check_applied_force_input_ports` into
        # this test.

    @numpy_compare.check_all_types
    def test_externally_applied_spatial_force(self, T):
        ExternallyAppliedSpatialForce = ExternallyAppliedSpatialForce_[T]
        dut = ExternallyAppliedSpatialForce()
        copy.copy(dut)

    @TemplateSystem.define("AppliedForceTestSystem_")
    def AppliedForceTestSystem_(T):
        class Impl(LeafSystem_[T]):
            def _construct(self, nv, target_body_index, converter=None):
                LeafSystem_[T].__init__(self, converter=converter)
                self.set_name("applied_force_test_system")
                self.nv = nv
                self.target_body_index = target_body_index
                forces_cls = Value[List[ExternallyAppliedSpatialForce_[T]]]
                self.DeclareAbstractOutputPort(
                    "spatial_forces_vector",
                    lambda: forces_cls(),
                    self.DoCalcAbstractOutput,
                )
                self.DeclareVectorOutputPort(
                    "generalized_forces", self.nv, self.DoCalcVectorOutput
                )

            def _construct_copy(self, other, converter=None):
                Impl._construct(
                    self, other.nv, other.target_body_index, converter=converter
                )

            def DoCalcAbstractOutput(self, context, spatial_forces_vector):
                test_force = ExternallyAppliedSpatialForce_[T]()
                test_force.body_index = self.target_body_index
                test_force.p_BoBq_B = np.zeros(3)
                test_force.F_Bq_W = SpatialForce_[T](
                    tau=[0.0, 0.0, 0.0], f=[0.0, 0.0, 1.0]
                )
                spatial_forces_vector.set_value([test_force])
                numpy_compare.assert_float_equal(
                    test_force.p_BoBq_B, np.zeros(3)
                )

            def DoCalcVectorOutput(self, context, generalized_forces):
                generalized_forces.SetFromVector(np.zeros(self.nv))

        return Impl

    def test_applied_force_input_ports(self):
        self.check_applied_force_input_ports(float)
        self.check_applied_force_input_ports(AutoDiffXd)

    def check_applied_force_input_ports(self, T):
        # Create a MultibodyPlant, and ensure that a secondary system can
        # be connected to feed it vectors of ExternallyAppliedSpatialForce
        # and applied generalized force vectors.
        InputPort = InputPort_[T]
        Simulator = Simulator_[T]

        builder_f = DiagramBuilder_[float]()
        # N.B. `Parser` only supports `MultibodyPlant_[float]`.
        plant_f = builder_f.AddSystem(MultibodyPlant_[float](0.0))
        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/free_body/uniform_solid_cylinder.urdf"
        )
        Parser(plant_f).AddModels(file_name)
        plant_f.Finalize()

        # These connections will fail if the port output types
        # are not legible.
        test_system_f = builder_f.AddSystem(
            self.AppliedForceTestSystem_[float](
                plant_f.num_velocities(),
                plant_f.GetBodyByName("uniformSolidCylinder").index(),
            )
        )
        builder_f.Connect(
            test_system_f.get_output_port(0),
            plant_f.get_applied_spatial_force_input_port(),
        )
        builder_f.Connect(
            test_system_f.get_output_port(1),
            plant_f.get_applied_generalized_force_input_port(),
        )
        diagram_f = builder_f.Build()
        diagram = to_type(diagram_f, T)

        plant = diagram.GetSubsystemByName(plant_f.get_name())
        # Test that we can get those ports.
        self.assertIsInstance(
            plant.get_applied_generalized_force_input_port(), InputPort
        )
        self.assertIsInstance(
            plant.get_applied_spatial_force_input_port(), InputPort
        )

        # Ensure we can tick this system. If so, all type conversions
        # are working properly.
        simulator = Simulator(diagram)
        simulator.AdvanceTo(0.01)

    @numpy_compare.check_all_types
    def test_model_instance_state_access(self, T):
        # N.B. Please check warning above in `check_multibody_state_access`.
        # Create a MultibodyPlant with a kuka arm and a schunk gripper.
        # the arm is welded to the world, the gripper is welded to the
        # arm's end effector.

        wsg50_sdf_url = (
            "package://drake_models/wsg_50_description/sdf/"
            + "schunk_wsg_50.sdf"
        )
        iiwa_sdf_url = (
            "package://drake_models/iiwa_description/sdf/"
            + "iiwa14_no_collision.sdf"
        )

        # N.B. `Parser` only supports `MultibodyPlant_[float]`.
        plant_f = MultibodyPlant_[float](0.0)
        parser = Parser(plant_f)

        (iiwa_model,) = parser.AddModels(url=iiwa_sdf_url)
        (gripper_model,) = parser.AddModels(url=wsg50_sdf_url)

        # Weld the base of arm and gripper to reduce the number of states.
        X_EeGripper = RigidTransform_[float](
            RollPitchYaw_[float](np.pi / 2, 0, np.pi / 2), [0, 0, 0.081]
        )
        plant_f.WeldFrames(
            frame_on_parent_F=plant_f.world_frame(),
            frame_on_child_M=plant_f.GetFrameByName("iiwa_link_0", iiwa_model),
            X_FM=RigidTransform_[float](),
        )
        # Perform the second weld without named arguments to ensure that the
        # proper binding gets invoked.
        plant_f.WeldFrames(
            plant_f.GetFrameByName("iiwa_link_7", iiwa_model),
            plant_f.GetFrameByName("body", gripper_model),
            X_EeGripper,
        )
        plant_f.Finalize()
        plant = to_type(plant_f, T)

        # Create a context of the MBP and set the state of the context
        # to desired values.
        context = plant.CreateDefaultContext()

        nq = plant.num_positions()
        nv = plant.num_velocities()

        nq_iiwa = 7
        nv_iiwa = 7
        nq_gripper = 2
        nv_gripper = 2
        q_iiwa_desired = np.zeros(nq_iiwa)
        v_iiwa_desired = np.zeros(nv_iiwa)
        q_gripper_desired = np.zeros(nq_gripper)
        v_gripper_desired = np.zeros(nv_gripper)

        q_iiwa_desired[2] = np.pi / 3
        q_gripper_desired[0] = 0.1
        v_iiwa_desired[1] = 5.0
        q_gripper_desired[0] = -0.3

        x_iiwa_desired = np.zeros(nq_iiwa + nv_iiwa)
        x_iiwa_desired[0:nq_iiwa] = q_iiwa_desired
        x_iiwa_desired[nq_iiwa : nq_iiwa + nv_iiwa] = v_iiwa_desired

        x_gripper_desired = np.zeros(nq_gripper + nv_gripper)
        x_gripper_desired[0:nq_gripper] = q_gripper_desired
        x_gripper_desired[nq_gripper : nq_gripper + nv_gripper] = (
            v_gripper_desired
        )

        x_desired = np.zeros(nq + nv)
        x_desired[0:7] = q_iiwa_desired
        x_desired[7:9] = q_gripper_desired
        x_desired[nq : nq + 7] = v_iiwa_desired
        x_desired[nq + 7 : nq + nv] = v_gripper_desired

        # Check SetPositionsAndVelocities() for each model instance.
        # Do the iiwa model first.
        plant.SetPositionsAndVelocities(context, np.zeros(nq + nv))
        numpy_compare.assert_float_equal(
            plant.GetPositionsAndVelocities(context), np.zeros(nq + nv)
        )
        plant.SetPositionsAndVelocities(context, iiwa_model, x_iiwa_desired)
        numpy_compare.assert_float_equal(
            plant.GetPositionsAndVelocities(context, iiwa_model), x_iiwa_desired
        )
        numpy_compare.assert_float_equal(
            plant.GetPositionsAndVelocities(context, gripper_model),
            np.zeros(nq_gripper + nv_gripper),
        )
        # Do the gripper model.
        plant.SetPositionsAndVelocities(context, np.zeros(nq + nv))
        numpy_compare.assert_float_equal(
            plant.GetPositionsAndVelocities(context), np.zeros(nq + nv)
        )
        plant.SetPositionsAndVelocities(
            context, gripper_model, x_gripper_desired
        )
        numpy_compare.assert_float_equal(
            plant.GetPositionsAndVelocities(context, gripper_model),
            x_gripper_desired,
        )
        numpy_compare.assert_float_equal(
            plant.GetPositionsAndVelocities(context, iiwa_model),
            np.zeros(nq_iiwa + nv_iiwa),
        )

        # Check SetPositions() for each model instance.
        # Do the iiwa model first.
        plant.SetPositionsAndVelocities(context, np.zeros(nq + nv))
        numpy_compare.assert_float_equal(
            plant.GetPositionsAndVelocities(context), np.zeros(nq + nv)
        )
        plant.SetPositions(context, iiwa_model, q_iiwa_desired)
        numpy_compare.assert_float_equal(
            plant.GetPositions(context, iiwa_model), q_iiwa_desired
        )
        numpy_compare.assert_float_equal(
            plant.GetVelocities(context, iiwa_model), np.zeros(nv_iiwa)
        )
        numpy_compare.assert_float_equal(
            plant.GetPositionsAndVelocities(context, gripper_model),
            np.zeros(nq_gripper + nv_gripper),
        )
        # Do the gripper model.
        plant.SetPositionsAndVelocities(context, np.zeros(nq + nv))
        numpy_compare.assert_float_equal(
            plant.GetPositionsAndVelocities(context), np.zeros(nq + nv)
        )
        plant.SetPositions(context, gripper_model, q_gripper_desired)
        numpy_compare.assert_float_equal(
            plant.GetPositions(context, gripper_model), q_gripper_desired
        )
        numpy_compare.assert_float_equal(
            plant.GetVelocities(context, gripper_model), np.zeros(nq_gripper)
        )
        numpy_compare.assert_float_equal(
            plant.GetPositionsAndVelocities(context, iiwa_model),
            np.zeros(nq_iiwa + nv_iiwa),
        )

        # Check SetVelocities() for each model instance.
        # Do the iiwa model first.
        plant.SetPositionsAndVelocities(context, np.zeros(nq + nv))
        numpy_compare.assert_float_equal(
            plant.GetPositionsAndVelocities(context), np.zeros(nq + nv)
        )
        plant.SetVelocities(context, iiwa_model, v_iiwa_desired)
        numpy_compare.assert_float_equal(
            plant.GetVelocities(context, iiwa_model), v_iiwa_desired
        )
        numpy_compare.assert_float_equal(
            plant.GetPositions(context, iiwa_model), np.zeros(nq_iiwa)
        )
        numpy_compare.assert_float_equal(
            plant.GetPositionsAndVelocities(context, gripper_model),
            np.zeros(nq_gripper + nv_gripper),
        )
        # Do the gripper model.
        plant.SetPositionsAndVelocities(context, np.zeros(nq + nv))
        numpy_compare.assert_float_equal(
            plant.GetPositionsAndVelocities(context), np.zeros(nq + nv)
        )
        plant.SetVelocities(context, gripper_model, v_gripper_desired)
        numpy_compare.assert_float_equal(
            plant.GetVelocities(context, gripper_model), v_gripper_desired
        )
        numpy_compare.assert_float_equal(
            plant.GetPositions(context, gripper_model), np.zeros(nv_gripper)
        )
        numpy_compare.assert_float_equal(
            plant.GetPositionsAndVelocities(context, iiwa_model),
            np.zeros(nq_iiwa + nv_iiwa),
        )

    @numpy_compare.check_all_types
    def test_model_instance_state_access_by_array(self, T):
        # N.B. Please check warning above in `check_multibody_state_access`.
        # Create a MultibodyPlant with a kuka arm and a schunk gripper.
        # the arm is welded to the world, the gripper is welded to the
        # arm's end effector.
        wsg50_sdf_url = (
            "package://drake_models/wsg_50_description/sdf/schunk_wsg_50.sdf"
        )
        iiwa_sdf_url = (
            "package://drake_models/"
            "iiwa_description/sdf/iiwa14_no_collision.sdf"
        )

        time_step = 0.0002
        # N.B. `Parser` only supports `MultibodyPlant_[float]`.
        plant_f = MultibodyPlant_[float](time_step)
        parser = Parser(plant_f)

        (iiwa_model,) = parser.AddModels(url=iiwa_sdf_url)
        (gripper_model,) = parser.AddModels(url=wsg50_sdf_url)

        # Weld the base of arm and gripper to reduce the number of states.
        X_EeGripper = RigidTransform_[float](
            RollPitchYaw_[float](np.pi / 2, 0, np.pi / 2), [0, 0, 0.081]
        )
        plant_f.WeldFrames(
            frame_on_parent_F=plant_f.world_frame(),
            frame_on_child_M=plant_f.GetFrameByName("iiwa_link_0", iiwa_model),
        )
        plant_f.WeldFrames(
            frame_on_parent_F=plant_f.GetFrameByName("iiwa_link_7", iiwa_model),
            frame_on_child_M=plant_f.GetFrameByName("body", gripper_model),
            X_FM=X_EeGripper,
        )
        plant_f.Finalize()
        plant = to_type(plant_f, T)

        # Create a context of the MBP and set the state of the context
        # to desired values.
        context = plant.CreateDefaultContext()

        nq = plant.num_positions()
        nq_iiwa = plant.num_positions(iiwa_model)
        nv = plant.num_velocities()
        nv_iiwa = plant.num_velocities(iiwa_model)

        q_iiwa_desired = np.linspace(0, 0.3, 7)
        v_iiwa_desired = q_iiwa_desired + 0.4
        q_gripper_desired = [0.4, 0.5]
        v_gripper_desired = [-1.0, -2.0]

        x_desired = np.zeros(nq + nv)
        x_desired[0:7] = q_iiwa_desired
        x_desired[7:9] = q_gripper_desired
        x_desired[nq : nq + 7] = v_iiwa_desired
        x_desired[nq + 7 : nq + nv] = v_gripper_desired

        plant.SetPositionsAndVelocities(context, x_desired)

        q = plant.GetPositions(context=context)
        v = plant.GetVelocities(context=context)

        # Get state from context.
        plant.GetPositionsAndVelocities(context=context)

        # Get positions and velocities of specific model instances
        # from the position/velocity vector of the plant.
        q_iiwa = plant.GetPositions(context=context, model_instance=iiwa_model)
        q_iiwa_array = plant.GetPositionsFromArray(
            model_instance=iiwa_model, q=q
        )
        numpy_compare.assert_equal(q_iiwa, q_iiwa_array)
        q_gripper = plant.GetPositions(
            context=context, model_instance=gripper_model
        )
        v_iiwa = plant.GetVelocities(context=context, model_instance=iiwa_model)
        v_iiwa_array = plant.GetVelocitiesFromArray(
            model_instance=iiwa_model, v=v
        )
        numpy_compare.assert_equal(v_iiwa, v_iiwa_array)
        v_gripper = plant.GetVelocities(
            context=context, model_instance=gripper_model
        )

        # Assert that the `GetPositions` and `GetVelocities` return
        # the desired values set earlier.
        numpy_compare.assert_float_equal(q_iiwa, q_iiwa_desired)
        numpy_compare.assert_float_equal(v_iiwa, v_iiwa_desired)
        numpy_compare.assert_float_equal(q_gripper, q_gripper_desired)
        numpy_compare.assert_float_equal(v_gripper, v_gripper_desired)

        if T is float:
            # Verify that SetPositionsInArray() and SetVelocitiesInArray()
            # works.
            plant.SetPositionsInArray(
                model_instance=iiwa_model, q_instance=np.zeros(nq_iiwa), q=q
            )
            numpy_compare.assert_float_equal(
                plant.GetPositionsFromArray(model_instance=iiwa_model, q=q),
                np.zeros(nq_iiwa),
            )
            plant.SetVelocitiesInArray(
                model_instance=iiwa_model, v_instance=np.zeros(nv_iiwa), v=v
            )
            numpy_compare.assert_float_equal(
                plant.GetVelocitiesFromArray(model_instance=iiwa_model, v=v),
                np.zeros(nv_iiwa),
            )

            # Check actuation.
            nu = plant.num_actuated_dofs()
            u = np.zeros(nu)
            u_iiwa = np.arange(nv_iiwa)
            plant.SetActuationInArray(
                model_instance=iiwa_model, u_instance=u_iiwa, u=u
            )
            numpy_compare.assert_float_equal(u_iiwa, u[:7])
            numpy_compare.assert_float_equal(
                u_iiwa,
                plant.GetActuationFromArray(model_instance=iiwa_model, u=u),
            )

    @numpy_compare.check_all_types
    def test_map_qdot_to_v_and_back(self, T):
        RigidTransform = RigidTransform_[T]
        RollPitchYaw = RollPitchYaw_[T]
        # N.B. `Parser` only supports `MultibodyPlant_[float]`.
        plant_f = MultibodyPlant_[float](0.0)
        iiwa_sdf_url = (
            "package://drake_models/iiwa_description/sdf/"
            + "iiwa14_no_collision.sdf"
        )
        # Use floating base to effectively add a quaternion in the generalized
        # quaternion.
        (iiwa_model,) = Parser(plant_f).AddModels(url=iiwa_sdf_url)
        plant_f.Finalize()
        plant = to_type(plant_f, T)
        context = plant.CreateDefaultContext()
        # Try mapping velocity to qdot and back.
        nq = plant.num_positions()
        nv = plant.num_velocities()
        q_init = np.linspace(start=1.0, stop=nq, num=nq)
        plant.SetPositions(context, q_init)
        # Overwrite the (invalid) base coordinates, wherever in `q` they are.
        link0 = plant.GetBodyByName("iiwa_link_0")
        plant.SetFreeBodyPose(
            context=context,
            body=link0,
            X_JpJc=RigidTransform(
                RollPitchYaw([0.1, 0.2, 0.3]), p=[0.4, 0.5, 0.6]
            ),
        )
        numpy_compare.assert_float_allclose(
            plant.GetFreeBodyPose(context=context, body=link0).translation(),
            [0.4, 0.5, 0.6],
        )
        plant.SetFloatingBaseBodyPoseInWorldFrame(
            context=context,
            body=link0,
            X_WB=RigidTransform(
                RollPitchYaw([0.4, 0.5, 0.6]), p=[0.7, 0.8, 0.9]
            ),
        )
        numpy_compare.assert_float_allclose(
            plant.GetFreeBodyPose(context=context, body=link0).translation(),
            [0.7, 0.8, 0.9],
        )
        plant.SetFloatingBaseBodyPoseInAnchoredFrame(
            context=context,
            frame_F=plant.world_frame(),
            body=link0,
            X_FB=RigidTransform(
                RollPitchYaw([0.45, 0.55, 0.65]), p=[0.75, 0.85, 0.95]
            ),
        )
        numpy_compare.assert_float_allclose(
            plant.GetFreeBodyPose(context=context, body=link0).translation(),
            [0.75, 0.85, 0.95],
        )
        self.assertNotEqual(link0.floating_positions_start(), -1)
        self.assertNotEqual(link0.floating_velocities_start_in_v(), -1)
        self.assertFalse(plant.IsVelocityEqualToQDot())
        v_expected = np.linspace(start=-1.0, stop=-nv, num=nv)
        qdot = plant.MapVelocityToQDot(context, v_expected)
        v_remap = plant.MapQDotToVelocity(context, qdot)
        numpy_compare.assert_float_allclose(v_remap, v_expected)
        # Bindings for Eigen::SparseMatrix only support T=float for now.
        if T is float:
            N = plant.MakeVelocityToQDotMap(context)
            numpy_compare.assert_float_allclose(qdot, N.todense() @ v_expected)
            Nplus = plant.MakeQDotToVelocityMap(context)
            numpy_compare.assert_float_allclose(
                v_expected, Nplus.todense() @ qdot
            )

    @numpy_compare.check_all_types
    def test_multibody_add_joint(self, T):
        """
        Tests joint constructors, `AddJoint`, `AddJointActuator` and
        `HasJointActuatorNamed`.
        """
        array_T = np.vectorize(T)
        damping = 2.0
        different_damping = 3.4
        x_axis = [1.0, 0.0, 0.0]
        X_PC = RigidTransform_[float](p=[1.0, 2.0, 3.0])

        def make_ball_rpy_joint(plant, P, C):
            return BallRpyJoint_[T](
                name="ball_rpy",
                frame_on_parent=P,
                frame_on_child=C,
                damping=damping,
            )

        def make_planar_joint(plant, P, C):
            return PlanarJoint_[T](
                name="planar",
                frame_on_parent=P,
                frame_on_child=C,
                damping=[damping, damping, damping],
            )

        def make_prismatic_joint(plant, P, C):
            return PrismaticJoint_[T](
                name="prismatic",
                frame_on_parent=P,
                frame_on_child=C,
                axis=x_axis,
                damping=damping,
            )

        def make_quaternion_floating_joint(plant, P, C):
            return QuaternionFloatingJoint_[T](
                name="quaternion_floating",
                frame_on_parent=P,
                frame_on_child=C,
                angular_damping=damping,
                translational_damping=damping,
            )

        def make_revolute_joint(plant, P, C):
            # First, check that the sans-limits overload works.
            RevoluteJoint_[T](
                name="revolute",
                frame_on_parent=P,
                frame_on_child=C,
                axis=x_axis,
                damping=damping,
            )
            # Then, create one using limits.
            return RevoluteJoint_[T](
                name="revolute",
                frame_on_parent=P,
                frame_on_child=C,
                axis=x_axis,
                pos_lower_limit=-1.5,
                pos_upper_limit=1.5,
                damping=damping,
            )

        def make_rpy_floating_joint(plant, P, C):
            return RpyFloatingJoint_[T](
                name="rpy_floating",
                frame_on_parent=P,
                frame_on_child=C,
                angular_damping=damping,
                translational_damping=damping,
            )

        def make_screw_joint(plant, P, C):
            # First, check that the no-axis overload works.
            ScrewJoint_[T](
                name="screw",
                frame_on_parent=P,
                frame_on_child=C,
                screw_pitch=0.005,
                damping=damping,
            )
            # Then, create one with an explicit axis.
            return ScrewJoint_[T](
                name="screw",
                frame_on_parent=P,
                frame_on_child=C,
                axis=x_axis,
                screw_pitch=0.005,
                damping=damping,
            )

        def make_universal_joint(plant, P, C):
            return UniversalJoint_[T](
                name="universal",
                frame_on_parent=P,
                frame_on_child=C,
                damping=damping,
            )

        def make_weld_joint(plant, P, C):
            return WeldJoint_[T](
                name="weld",
                frame_on_parent_F=P,
                frame_on_child_M=C,
                X_FM=X_PC,
            )

        make_joint_list = [
            make_ball_rpy_joint,
            make_planar_joint,
            make_prismatic_joint,
            make_quaternion_floating_joint,
            make_revolute_joint,
            make_rpy_floating_joint,
            make_screw_joint,
            make_universal_joint,
            make_weld_joint,
        ]

        def loop_body(make_joint, time_step):
            plant = MultibodyPlant_[T](time_step)
            child = plant.AddRigidBody("Child")
            joint = plant.AddJoint(
                joint=make_joint(
                    plant=plant, P=plant.world_frame(), C=child.body_frame()
                )
            )
            if joint.num_velocities() == 1:
                self.assertFalse(plant.HasJointActuatorNamed("tau"))
                self.assertFalse(
                    plant.HasJointActuatorNamed("tau", default_model_instance())
                )
                actuator = plant.AddJointActuator(
                    name="tau", joint=joint, effort_limit=1.0
                )
                self.assertTrue(plant.HasJointActuatorNamed("tau"))
                self.assertTrue(
                    plant.HasJointActuatorNamed("tau", default_model_instance())
                )
                self.assertIsInstance(actuator, JointActuator_[T])

            damping_vector = []
            different_damping_vector = []

            if joint.name() != "weld":
                damping_vector = joint.num_velocities() * [damping]
                different_damping_vector = joint.num_velocities() * [
                    different_damping
                ]
                numpy_compare.assert_equal(
                    joint.default_damping_vector(), damping_vector
                )
                joint.set_default_damping_vector(
                    damping=different_damping_vector
                )
                numpy_compare.assert_equal(
                    joint.default_damping_vector(), different_damping_vector
                )
                joint.set_default_damping_vector(damping=damping_vector)

            if joint.name() in ["prismatic", "revolute"]:
                # This must be called pre-Finalize().
                joint.set_default_damping(damping=damping)
                self.assertEqual(joint.default_damping(), damping)

            plant.Finalize()
            context = plant.CreateDefaultContext()
            self._test_joint_api(T, joint)
            if joint.num_velocities() == 1 and T is float:
                u = np.array([0.1])
                numpy_compare.assert_float_equal(
                    actuator.get_actuation_vector(u=u), [0.1]
                )
                actuator.set_actuation_vector(u_actuator=np.array([0.2]), u=u)
                numpy_compare.assert_float_equal(u, [0.2])
                self.assertIsInstance(actuator.rotor_inertia(context), float)
                self.assertIsInstance(actuator.gear_ratio(context), float)
                actuator.SetRotorInertia(context=context, rotor_inertia=1.5)
                actuator.SetGearRatio(context=context, gear_ratio=1.5)

            for p in range(joint.num_positions()):
                self.assertIsNotNone(joint.position_suffix(p))
            for v in range(joint.num_velocities()):
                self.assertIsNotNone(joint.velocity_suffix(v))

            uniform_random = Variable(
                name="uniform_random", type=Variable.Type.RANDOM_UNIFORM
            )

            self.assertFalse(joint.is_locked(context))
            if time_step:
                joint.Lock(context)
                self.assertTrue(joint.is_locked(context))
                joint.Unlock(context)
                self.assertFalse(joint.is_locked(context))

            if joint.name() != "weld":
                numpy_compare.assert_equal(
                    joint.GetDampingVector(context), array_T(damping_vector)
                )
                joint.SetDampingVector(
                    context, array_T(different_damping_vector)
                )
                numpy_compare.assert_equal(
                    joint.GetDampingVector(context),
                    array_T(different_damping_vector),
                )
                joint.SetDampingVector(context, array_T(damping_vector))

            if joint.name() == "ball_rpy":
                self.assertEqual(joint.default_damping(), damping)
                set_point = array_T([1.0, 2.0, 3.0])
                joint.set_angles(context=context, angles=set_point)
                numpy_compare.assert_equal(
                    joint.get_angles(context=context), set_point
                )
                joint.set_angular_velocity(context=context, w_FM=set_point)
                numpy_compare.assert_equal(
                    joint.get_angular_velocity(context=context), set_point
                )
                joint.set_random_angles_distribution(
                    uniform_random * np.array([1.0, 1.0, 1.0])
                )
                joint.get_default_angles()
                joint.set_default_angles(angles=[0.0, 0.0, 0.0])
            elif joint.name() == "planar":
                self.assertEqual(len(joint.default_damping()), 3)
                set_translation = array_T([1.0, 2.0])
                set_angle = T(3.0)
                joint.set_translation(context=context, p_FoMo_F=set_translation)
                numpy_compare.assert_equal(
                    joint.get_translation(context=context), set_translation
                )
                joint.set_rotation(context=context, theta=set_angle)
                numpy_compare.assert_equal(
                    joint.get_rotation(context=context), set_angle
                )
                joint.set_pose(
                    context=context, p_FoMo_F=set_translation, theta=set_angle
                )
                joint.set_translational_velocity(
                    context=context, v_FoMo_F=set_translation
                )
                numpy_compare.assert_equal(
                    joint.get_translational_velocity(context=context),
                    set_translation,
                )
                joint.set_angular_velocity(context=context, theta_dot=set_angle)
                numpy_compare.assert_equal(
                    joint.get_angular_velocity(context=context), set_angle
                )
                joint.set_random_pose_distribution(
                    p_FoMo_F=uniform_random * np.array([1.0, 1.0]),
                    theta=uniform_random,
                )
                joint.get_default_translation()
                joint.set_default_translation(p_FoMo_F=[0.0, 0.0])
                joint.get_default_rotation()
                joint.set_default_rotation(theta=0.0)
                joint.set_default_pose(p_FoMo_F=[0.0, 0.0], theta=0.0)
            elif joint.name() == "prismatic":
                self.assertEqual(joint.default_damping(), damping)
                numpy_compare.assert_equal(joint.translation_axis(), x_axis)
                set_point = T(1.0)
                joint.set_translation(context=context, translation=set_point)
                numpy_compare.assert_equal(
                    joint.get_translation(context=context), set_point
                )
                joint.set_translation_rate(
                    context=context, translation_dot=set_point
                )
                numpy_compare.assert_equal(
                    joint.get_translation_rate(context=context), set_point
                )
                joint.set_random_translation_distribution(uniform_random)
                joint.position_lower_limit()
                joint.position_upper_limit()
                joint.velocity_lower_limit()
                joint.velocity_upper_limit()
                joint.acceleration_lower_limit()
                joint.acceleration_upper_limit()
                joint.get_default_translation()
                joint.set_default_translation(translation=0.0)
                numpy_compare.assert_equal(
                    joint.GetDamping(context), T(damping)
                )
                joint.SetDamping(context, T(different_damping))
                numpy_compare.assert_equal(
                    joint.GetDamping(context), T(different_damping)
                )
            elif joint.name() == "quaternion_floating":
                self.assertEqual(joint.default_angular_damping(), damping)
                self.assertEqual(joint.default_translational_damping(), damping)
                joint.get_quaternion(context=context)
                joint.get_translation(context=context)
                joint.GetPose(context=context)
                joint.get_angular_velocity(context=context)
                joint.get_translational_velocity(context=context)
                joint.SetQuaternion(context=context, q_FM=Quaternion_[T]())
                joint.SetOrientation(context=context, R=RotationMatrix_[T]())
                joint.SetTranslation(context=context, p_FM=[0, 0, 0])
                joint.SetPose(context=context, X_FM=RigidTransform_[T]())
                joint.set_angular_velocity(context=context, w_FM=[0, 0, 0])
                joint.set_translational_velocity(
                    context=context, v_FM=[0, 0, 0]
                )
                joint.set_random_translation_distribution(translation=[0, 0, 0])
                joint.set_random_quaternion_distribution(
                    q_FM=Quaternion_[Expression]()
                )
                joint.set_random_quaternion_distribution_to_uniform()
                joint.get_default_quaternion()
                joint.get_default_translation()
                joint.set_default_quaternion(q_FM=Quaternion_[float]())
                joint.set_default_translation(translation=[0, 0, 0])
                # Check that the base class supports these for this joint.
                joint.SetDefaultPose(X_FM=RigidTransform_[float]())
                joint.SetDefaultPosePair(
                    q_FM=Quaternion_[float](), p_FM=[0, 0, 0]
                )
                joint.GetDefaultPose()
                joint.GetDefaultPosePair()
            elif joint.name() == "revolute":
                numpy_compare.assert_equal(joint.revolute_axis(), x_axis)
                self.assertEqual(joint.default_damping(), damping)
                set_point = T(1.0)
                joint.set_angle(context=context, angle=set_point)
                numpy_compare.assert_equal(
                    joint.get_angle(context=context), set_point
                )
                joint.set_random_angle_distribution(uniform_random)
                joint.position_lower_limit()
                joint.position_upper_limit()
                joint.velocity_lower_limit()
                joint.velocity_upper_limit()
                joint.acceleration_lower_limit()
                joint.acceleration_upper_limit()
                joint.get_angular_rate(context=context)
                joint.set_angular_rate(context=context, angle=0.0)
                joint.get_default_angle()
                joint.set_default_angle(angle=0.0)
                # Check the Joint base class sugar for 1dof.
                joint.GetOnePosition(context=context)
                joint.GetOneVelocity(context=context)
                # Check the Joint base class AddIn... methods.
                forces = MultibodyForces_[T](plant=plant)
                joint.AddInOneForce(
                    context=context, joint_dof=0, joint_tau=0.0, forces=forces
                )
                joint.AddInDamping(context=context, forces=forces)
                numpy_compare.assert_equal(
                    joint.GetDamping(context), T(damping)
                )
                joint.SetDamping(context, T(different_damping))
                numpy_compare.assert_equal(
                    joint.GetDamping(context), T(different_damping)
                )
            elif joint.name() == "rpy_floating":
                self.assertEqual(joint.type_name(), "rpy_floating")
                self.assertEqual(joint.default_angular_damping(), damping)
                self.assertEqual(joint.default_translational_damping(), damping)
                joint.get_angles(context=context)
                joint.set_angles(context=context, angles=[0, 0, 0])
                joint.SetOrientation(context=context, R_FM=RotationMatrix_[T]())
                joint.get_translation(context=context)
                joint.SetTranslation(context=context, p_FM=[0, 0, 0])
                joint.GetPose(context=context)
                joint.SetPose(context=context, X_FM=RigidTransform_[T]())
                joint.get_angular_velocity(context=context)
                joint.set_angular_velocity(context=context, w_FM=[0, 0, 0])
                joint.get_translational_velocity(context=context)
                joint.set_translational_velocity(
                    context=context, v_FM=[0, 0, 0]
                )
                joint.set_random_angles_distribution(angles=[0, 0, 0])
                joint.set_random_translation_distribution(p_FM=[0, 0, 0])
                joint.get_default_angles()
                joint.set_default_angles(angles=[0, 0, 0])
                joint.get_default_translation()
                joint.set_default_translation(p_FM=[0, 0, 0])
                # Check that the base class supports these for this joint.
                joint.SetDefaultPose(X_FM=RigidTransform_[float]())
                joint.SetDefaultPosePair(
                    q_FM=Quaternion_[float](), p_FM=[0, 0, 0]
                )
                joint.GetDefaultPose()
                joint.GetDefaultPosePair()
            elif joint.name() == "screw":
                self.assertEqual(joint.default_damping(), damping)
                joint.screw_pitch()
                joint.get_default_rotation()
                joint.set_default_rotation(0.0)
                joint.get_default_translation()
                joint.set_default_translation(0.0)
                joint.get_translational_velocity(context)
                joint.set_translational_velocity(context, 0.0)
                joint.get_rotation(context=context)
                joint.set_angular_velocity(context, 0.0)
                joint.get_angular_velocity(context=context)
                joint.set_random_pose_distribution(theta=np.array([[1]]))
                # Check the Joint base class sugar for 1dof.
                joint.GetOnePosition(context=context)
                joint.GetOneVelocity(context=context)
                numpy_compare.assert_equal(
                    joint.GetDamping(context), T(damping)
                )
                joint.SetDamping(context, T(different_damping))
                numpy_compare.assert_equal(
                    joint.GetDamping(context), T(different_damping)
                )
            elif joint.name() == "universal":
                self.assertEqual(joint.default_damping(), damping)
                set_point = array_T([1.0, 2.0])
                joint.set_angles(context=context, angles=set_point)
                numpy_compare.assert_equal(
                    joint.get_angles(context=context), set_point
                )
                joint.set_angular_rates(context=context, theta_dot=set_point)
                numpy_compare.assert_equal(
                    joint.get_angular_rates(context=context), set_point
                )
                joint.set_random_angles_distribution(
                    uniform_random * np.array([1.0, 1.0])
                )
                joint.get_default_angles()
                joint.set_default_angles(angles=[1.0, 2.0])
            elif joint.name() == "weld":
                numpy_compare.assert_float_equal(
                    joint.X_FM().GetAsMatrix4(), X_PC.GetAsMatrix4()
                )
            else:
                raise TypeError(
                    "Joint type " + joint.name() + " not recognized."
                )

        for make_joint in make_joint_list:
            with self.subTest(make_joint=make_joint):
                loop_body(make_joint, 0.0)
                loop_body(make_joint, 0.001)

    def test_actuation_matrix(self):
        iiwa_sdf_url = (
            "package://drake_models/iiwa_description/sdf/"
            + "iiwa14_no_collision.sdf"
        )

        plant = MultibodyPlant_[float](0.0)
        parser = Parser(plant)
        (iiwa_model,) = parser.AddModels(url=iiwa_sdf_url)
        plant.WeldFrames(
            frame_on_parent_F=plant.world_frame(),
            frame_on_child_M=plant.GetFrameByName("iiwa_link_0", iiwa_model),
        )
        plant.Finalize()

        B = plant.MakeActuationMatrix()
        np.testing.assert_array_equal(B, np.eye(7))
        B_inv = plant.MakeActuationMatrixPseudoinverse()
        np.testing.assert_array_equal(B_inv.todense(), np.eye(7))

    def test_deprecated_weld_joint_api(self):
        plant = MultibodyPlant_[float](0.01)
        plant.AddRigidBody(name="body1")
        body2 = plant.AddRigidBody(name="body2")

        # No keywords defaults to the first constructor defined in the binding.
        # No warning.
        WeldJoint_[float](
            "world_body2",
            plant.world_frame(),
            body2.body_frame(),
            RigidTransform_[float].Identity(),
        )

    def test_deprecated_weld_frames_api(self):
        plant = MultibodyPlant_[float](0.01)
        plant.AddRigidBody(name="body1")
        body2 = plant.AddRigidBody(name="body2")

        # No keywords defaults to the first function named `WeldFrames` defined
        # in the binding. No warning.
        plant.WeldFrames(
            plant.world_frame(),
            body2.body_frame(),
            RigidTransform_[float].Identity(),
        )

    @numpy_compare.check_all_types
    def test_multibody_add_frame(self, T):
        MultibodyPlant = MultibodyPlant_[T]
        FixedOffsetFrame = FixedOffsetFrame_[T]

        Frame = Frame_[T]

        plant = MultibodyPlant(0.0)
        frame = plant.AddFrame(
            frame=FixedOffsetFrame(
                name="frame",
                P=plant.world_frame(),
                X_PF=RigidTransform_[float].Identity(),
                model_instance=None,
            )
        )
        self.assertIsInstance(frame, Frame)
        numpy_compare.assert_float_equal(
            frame.GetFixedPoseInBodyFrame().GetAsMatrix4(), np.eye(4)
        )

        plant.Finalize()
        context = plant.CreateDefaultContext()

        X_PF = RigidTransform_[T](p=[1.0, 2.0, 3.0])
        frame.SetPoseInParentFrame(context=context, X_PF=X_PF)
        numpy_compare.assert_float_equal(
            frame.GetPoseInParentFrame(context).GetAsMatrix34(),
            numpy_compare.to_float(X_PF.GetAsMatrix34()),
        )

    @numpy_compare.check_all_types
    def test_frame_context_methods(self, T):
        plant = MultibodyPlant_[T](0.0)
        dut = plant.AddFrame(
            frame=FixedOffsetFrame_[T](
                name="name",
                P=plant.world_frame(),
                X_PF=RigidTransform_[float](),
            )
        )
        plant.Finalize()
        context = plant.CreateDefaultContext()
        X = RigidTransform_[T]()
        self.assertIsInstance(
            dut.CalcRotationMatrixInBodyFrame(context=context),
            RotationMatrix_[T],
        )
        self.assertIsInstance(
            dut.CalcOffsetPoseInBody(context=context, X_FQ=X),
            RigidTransform_[T],
        )
        self.assertIsInstance(
            dut.CalcOffsetRotationMatrixInBody(
                context=context, R_FQ=X.rotation()
            ),
            RotationMatrix_[T],
        )
        self.assertIsInstance(
            dut.GetFixedOffsetPoseInBody(X_FQ=X), RigidTransform_[T]
        )
        self.assertIsInstance(
            dut.GetFixedRotationMatrixInBody(R_FQ=X.rotation()),
            RotationMatrix_[T],
        )
        self.assertIsInstance(
            dut.CalcPoseInWorld(context=context), RigidTransform_[T]
        )
        self.assertIsInstance(
            dut.CalcPose(context=context, frame_M=dut), RigidTransform_[T]
        )
        self.assertIsInstance(
            dut.CalcRotationMatrix(context=context, frame_M=dut),
            RotationMatrix_[T],
        )
        self.assertIsInstance(
            dut.CalcRotationMatrixInWorld(context=context), RotationMatrix_[T]
        )
        self.assertIsInstance(
            dut.EvalAngularVelocityInWorld(context=context), np.ndarray
        )
        self.assertIsInstance(
            dut.CalcAngularVelocity(
                context=context, measured_in_frame=dut, expressed_in_frame=dut
            ),
            np.ndarray,
        )
        self.assertIsInstance(
            dut.CalcSpatialVelocityInWorld(context=context), SpatialVelocity_[T]
        )
        self.assertIsInstance(
            dut.CalcSpatialVelocity(context=context, frame_M=dut, frame_E=dut),
            SpatialVelocity_[T],
        )
        self.assertIsInstance(
            dut.CalcRelativeSpatialVelocityInWorld(
                context=context, other_frame=dut
            ),
            SpatialVelocity_[T],
        )
        self.assertIsInstance(
            dut.CalcRelativeSpatialVelocity(
                context=context,
                other_frame=dut,
                measured_in_frame=dut,
                expressed_in_frame=dut,
            ),
            SpatialVelocity_[T],
        )
        self.assertIsInstance(
            dut.CalcSpatialAccelerationInWorld(context=context),
            SpatialAcceleration_[T],
        )
        self.assertIsInstance(
            dut.CalcSpatialAcceleration(
                context=context, measured_in_frame=dut, expressed_in_frame=dut
            ),
            SpatialAcceleration_[T],
        )
        self.assertIsInstance(
            dut.CalcRelativeSpatialAccelerationInWorld(
                context=context, other_frame=dut
            ),
            SpatialAcceleration_[T],
        )
        self.assertIsInstance(
            dut.CalcRelativeSpatialAcceleration(
                context=context,
                other_frame=dut,
                measured_in_frame=dut,
                expressed_in_frame=dut,
            ),
            SpatialAcceleration_[T],
        )

    @numpy_compare.check_all_types
    def test_fixed_offset_frame_api(self, T):
        FixedOffsetFrame = FixedOffsetFrame_[T]
        P = MultibodyPlant_[T](0.0).world_frame()
        B = RigidBody_[T]("body")
        X = RigidTransform_[float].Identity()
        FixedOffsetFrame(name="name", P=P, X_PF=X, model_instance=None)
        FixedOffsetFrame(name="name", bodyB=B, X_BF=X)

    def test_coupler_constraint_api(self):
        # Create a MultibodyPlant with only a WSG gripper.
        plant = MultibodyPlant_[float](0.01)
        plant.set_discrete_contact_approximation(
            DiscreteContactApproximation.kSap
        )
        Parser(plant).AddModelsFromUrl(
            "package://drake_models/wsg_50_description/sdf/schunk_wsg_50.sdf"
        )

        # Add coupler constraint.
        left_slider = plant.GetJointByName("left_finger_sliding_joint")
        right_slider = plant.GetJointByName("right_finger_sliding_joint")
        plant.AddCouplerConstraint(
            joint0=left_slider, joint1=right_slider, gear_ratio=1.2, offset=3.4
        )

        # We are done creating the model.
        plant.Finalize()

        # Verify the constraint was added.
        self.assertEqual(plant.num_constraints(), 1)

    def test_distance_constraint_params_api(self):
        bodyA = BodyIndex(1)
        bodyB = BodyIndex(2)
        p_AP = [1.0, 2.0, 3.0]
        p_BQ = [4.0, 5.0, 6.0]
        distance = 0.1
        stiffness = 1.0e4
        damping = 100.0
        dut = DistanceConstraintParams(
            bodyA, p_AP, bodyB, p_BQ, distance, stiffness, damping
        )
        self.assertEqual(dut.bodyA(), bodyA)
        self.assertEqual(dut.bodyB(), bodyB)
        np.testing.assert_array_equal(dut.p_AP(), p_AP)
        np.testing.assert_array_equal(dut.p_BQ(), p_BQ)
        self.assertEqual(dut.distance(), distance)
        self.assertEqual(dut.stiffness(), stiffness)
        self.assertEqual(dut.damping(), damping)

    @numpy_compare.check_all_types
    def test_distance_constraint_api(self, T):
        plant = MultibodyPlant_[T](0.01)
        plant.set_discrete_contact_approximation(
            DiscreteContactApproximation.kSap
        )

        # Add a distance constraint.
        body_A = plant.AddRigidBody(name="A")
        body_B = plant.AddRigidBody(name="B")
        p_AP = [0.0, 0.0, 0.0]
        p_BQ = [0.0, 0.0, 0.0]
        id = plant.AddDistanceConstraint(
            body_A=body_A, p_AP=p_AP, body_B=body_B, p_BQ=p_BQ, distance=0.01
        )

        # We are done creating the model.
        plant.Finalize()
        context = plant.CreateDefaultContext()

        # Verify the constraint was added.
        self.assertEqual(plant.num_constraints(), 1)

        all_default_params = plant.GetDefaultDistanceConstraintParams()
        all_params = plant.GetDistanceConstraintParams(context)
        distance_params = plant.GetDistanceConstraintParams(context, id)

        # Testing equality function for DistanceConstraintParams.
        def params_are_equal(p1, p2):
            if p1.bodyA() != p2.bodyA():
                return False
            if p1.bodyB() != p2.bodyB():
                return False
            if (p1.p_AP() != p2.p_AP()).all():
                return False
            if (p1.p_BQ() != p2.p_BQ()).all():
                return False
            if p1.distance() != p2.distance():
                return False
            if p1.stiffness() != p2.stiffness():
                return False
            if p1.damping() != p2.damping():
                return False
            return True

        DistanceConstraintParams.__eq__ = params_are_equal

        self.assertEqual(len(all_default_params), 1)
        self.assertEqual(len(all_params), 1)
        self.assertIsInstance(distance_params, DistanceConstraintParams)
        self.assertEqual(distance_params, all_default_params[id])
        self.assertEqual(all_params, all_default_params)

        new_params = DistanceConstraintParams(
            body_A.index(),
            p_AP,
            body_B.index(),
            p_BQ,
            distance=0.05,
            stiffness=1e6,
            damping=0.1,
        )
        self.assertNotEqual(distance_params, new_params)
        plant.SetDistanceConstraintParams(context, id, new_params)
        updated_params = plant.GetDistanceConstraintParams(context, id)
        self.assertEqual(updated_params, new_params)

    @numpy_compare.check_all_types
    def test_ball_constraint_api(self, T):
        plant = MultibodyPlant_[T](0.01)
        plant.set_discrete_contact_approximation(
            DiscreteContactApproximation.kSap
        )

        # Add ball constraint.
        body_A = plant.AddRigidBody(name="A")
        body_B = plant.AddRigidBody(name="B")
        p_AP = [0.0, 0.0, 0.0]
        p_BQ = [0.0, 0.0, 0.0]
        plant.AddBallConstraint(
            body_A=body_A, p_AP=p_AP, body_B=body_B, p_BQ=p_BQ
        )

        # We are done creating the model.
        plant.Finalize()

        # Verify the constraint was added.
        self.assertEqual(plant.num_constraints(), 1)

    def test_constraint_active_status_api(self):
        plant = MultibodyPlant_[float](0.01)
        plant.set_discrete_contact_approximation(
            DiscreteContactApproximation.kSap
        )

        body_A = plant.AddRigidBody(name="A")
        body_B = plant.AddRigidBody(name="B")

        # Add ball and distance constraints.
        p_AP = [0.0, 0.0, 0.0]
        p_BQ = [0.0, 0.0, 0.0]
        X_AP = RigidTransform_[float](p_AP)
        X_BQ = RigidTransform_[float](p_BQ)
        distance_id = plant.AddDistanceConstraint(
            body_A=body_A, p_AP=p_AP, body_B=body_B, p_BQ=p_BQ, distance=0.01
        )
        ball_id = plant.AddBallConstraint(
            body_A=body_A, p_AP=p_AP, body_B=body_B, p_BQ=p_BQ
        )
        # Add a second ball constraint using the default (unspecified) p_BQ.
        ball_id2 = plant.AddBallConstraint(
            body_A=body_A, p_AP=p_AP, body_B=body_B
        )
        weld_id = plant.AddWeldConstraint(
            body_A=body_A, X_AP=X_AP, body_B=body_B, X_BQ=X_BQ
        )

        Parser(plant).AddModelsFromUrl(
            "package://drake_models/wsg_50_description/sdf/schunk_wsg_50.sdf"
        )

        # Add coupler constraint.
        left_slider = plant.GetJointByName("left_finger_sliding_joint")
        right_slider = plant.GetJointByName("right_finger_sliding_joint")
        coupler_id = plant.AddCouplerConstraint(
            joint0=left_slider, joint1=right_slider, gear_ratio=1.2, offset=3.4
        )

        # We are done creating the model.
        plant.Finalize()

        # Test GetConstraintIds()
        ids = plant.GetConstraintIds()
        # Confirm that indices and [distance_id, ball_id, weld_id, coupler_id]
        # are the same up to a permutation.
        self.assertTrue(
            collections.Counter(ids)
            == collections.Counter(
                [distance_id, ball_id, ball_id2, weld_id, coupler_id]
            )
        )

        # Default context.
        context = plant.CreateDefaultContext()

        # Verify all constraints are active in a default context.
        self.assertTrue(
            plant.GetConstraintActiveStatus(context=context, id=coupler_id)
        )
        self.assertTrue(
            plant.GetConstraintActiveStatus(context=context, id=distance_id)
        )
        self.assertTrue(
            plant.GetConstraintActiveStatus(context=context, id=ball_id)
        )
        self.assertTrue(
            plant.GetConstraintActiveStatus(context=context, id=ball_id2)
        )
        self.assertTrue(
            plant.GetConstraintActiveStatus(context=context, id=weld_id)
        )

        # Set all constraints to inactive.
        plant.SetConstraintActiveStatus(
            context=context, id=coupler_id, status=False
        )
        plant.SetConstraintActiveStatus(
            context=context, id=distance_id, status=False
        )
        plant.SetConstraintActiveStatus(
            context=context, id=ball_id, status=False
        )
        plant.SetConstraintActiveStatus(
            context=context, id=ball_id2, status=False
        )
        plant.SetConstraintActiveStatus(
            context=context, id=weld_id, status=False
        )

        # Verify all constraints are inactive in the context.
        self.assertFalse(
            plant.GetConstraintActiveStatus(context=context, id=coupler_id)
        )
        self.assertFalse(
            plant.GetConstraintActiveStatus(context=context, id=distance_id)
        )
        self.assertFalse(
            plant.GetConstraintActiveStatus(context=context, id=ball_id)
        )
        self.assertFalse(
            plant.GetConstraintActiveStatus(context=context, id=ball_id2)
        )
        self.assertFalse(
            plant.GetConstraintActiveStatus(context=context, id=weld_id)
        )

        # Set all constraints to back to active.
        plant.SetConstraintActiveStatus(
            context=context, id=coupler_id, status=True
        )
        plant.SetConstraintActiveStatus(
            context=context, id=distance_id, status=True
        )
        plant.SetConstraintActiveStatus(
            context=context, id=ball_id, status=True
        )
        plant.SetConstraintActiveStatus(
            context=context, id=ball_id2, status=True
        )
        plant.SetConstraintActiveStatus(
            context=context, id=weld_id, status=True
        )

        # Verify all constraints are active in the context.
        self.assertTrue(
            plant.GetConstraintActiveStatus(context=context, id=coupler_id)
        )
        self.assertTrue(
            plant.GetConstraintActiveStatus(context=context, id=distance_id)
        )
        self.assertTrue(
            plant.GetConstraintActiveStatus(context=context, id=ball_id)
        )
        self.assertTrue(
            plant.GetConstraintActiveStatus(context=context, id=ball_id2)
        )
        self.assertTrue(
            plant.GetConstraintActiveStatus(context=context, id=weld_id)
        )

    @numpy_compare.check_all_types
    def test_weld_constraint_api(self, T):
        plant = MultibodyPlant_[T](0.01)
        plant.set_discrete_contact_approximation(
            DiscreteContactApproximation.kSap
        )

        # Add weld constraint.
        body_A = plant.AddRigidBody(name="A")
        body_B = plant.AddRigidBody(name="B")
        X_AP = RigidTransform_[float]()
        X_BQ = RigidTransform_[float]()
        plant.AddWeldConstraint(
            body_A=body_A, X_AP=X_AP, body_B=body_B, X_BQ=X_BQ
        )

        # We are done creating the model.
        plant.Finalize()

        # Verify the constraint was added.
        self.assertEqual(plant.num_constraints(), 1)

    @numpy_compare.check_all_types
    def test_remove_constraint(self, T):
        plant = MultibodyPlant_[T](0.01)
        plant.set_discrete_contact_approximation(
            DiscreteContactApproximation.kSap
        )

        # Add weld constraint.
        body_A = plant.AddRigidBody(name="A")
        body_B = plant.AddRigidBody(name="B")
        X_AP = RigidTransform_[float]()
        X_BQ = RigidTransform_[float]()
        id = plant.AddWeldConstraint(
            body_A=body_A, X_AP=X_AP, body_B=body_B, X_BQ=X_BQ
        )

        plant.RemoveConstraint(id=id)

        # We are done creating the model.
        plant.Finalize()

        # Verify no constraint was added.
        self.assertEqual(plant.num_constraints(), 0)

    @numpy_compare.check_all_types
    def test_multibody_dynamics(self, T):
        MultibodyForces = MultibodyForces_[T]
        SpatialForce = SpatialForce_[T]

        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf"
        )
        # N.B. `Parser` only supports `MultibodyPlant_[float]`.
        # N.B. PD controllers below are only supported by discrete models.
        plant_f = MultibodyPlant_[float](0.01)
        (model_instance,) = Parser(plant_f).AddModels(file_name)
        # Getting ready for when we set foot on Mars :-).
        gravity_vector = np.array([0.0, 0.0, -3.71])
        plant_f.mutable_gravity_field().set_gravity_vector(gravity_vector)

        # MultibodyPlant APIs to enable/disable gravity.
        self.assertTrue(plant_f.is_gravity_enabled(model_instance))
        plant_f.set_gravity_enabled(model_instance, is_enabled=False)
        self.assertFalse(plant_f.is_gravity_enabled(model_instance))
        plant_f.set_gravity_enabled(model_instance, is_enabled=True)

        # UniformGravityField APIs to enable/disable gravity.
        gravity = plant_f.gravity_field()
        self.assertTrue(gravity.is_enabled(model_instance))
        gravity.set_enabled(model_instance, is_enabled=False)
        self.assertFalse(gravity.is_enabled(model_instance))
        gravity.set_enabled(model_instance, is_enabled=True)

        # Smoke test PD controllers APIs.
        elbow = plant_f.GetJointActuatorByName("ElbowJoint")
        mutable_elbow = plant_f.get_mutable_joint_actuator(elbow.index())
        gains = PdControllerGains(p=2000.0, d=100.0)
        mutable_elbow.set_controller_gains(gains)
        self.assertTrue(mutable_elbow.has_controller())
        mutable_elbow.get_controller_gains()

        plant_f.Finalize()
        plant = to_type(plant_f, T)
        context = plant.CreateDefaultContext()

        numpy_compare.assert_float_equal(
            plant.gravity_field().gravity_vector(), gravity_vector
        )

        # Set an arbitrary configuration away from the model's fixed point.
        plant.SetPositions(context, [0.1, 0.2])

        M = plant.CalcMassMatrixViaInverseDynamics(context)
        M = plant.CalcMassMatrix(context)
        Cv = plant.CalcBiasTerm(context)

        self.assertTrue(M.shape == (2, 2))
        self.assert_sane(M)
        self.assertTrue(Cv.shape == (2,))
        self.assert_sane(Cv, nonzero=False)
        nv = plant.num_velocities()
        vd_d = np.zeros(nv)
        tau = plant.CalcInverseDynamics(context, vd_d, MultibodyForces(plant))
        self.assertEqual(tau.shape, (2,))
        self.assert_sane(tau, nonzero=False)
        # - Existence checks.
        # Gravity leads to non-zero potential energy.
        potential_energy = plant.CalcPotentialEnergy(context)
        numpy_compare.assert_float_not_equal(potential_energy, 0.0)
        plant.CalcConservativePower(context)
        tau_g = plant.CalcGravityGeneralizedForces(context)
        self.assertEqual(tau_g.shape, (nv,))
        self.assert_sane(tau_g, nonzero=True)
        plant.gravity_field().CalcGravityGeneralizedForces(context=context)

        # Gravity is the only force element
        self.assertEqual(plant.num_force_elements(), 1)

        B = plant.MakeActuationMatrix()
        numpy_compare.assert_float_equal(B, np.array([[0.0], [1.0]]))

        sample_actuator = plant.GetJointActuatorByName("ElbowJoint")
        Ba = plant.MakeActuatorSelectorMatrix(
            user_to_actuator_index_map=[sample_actuator.index()]
        )
        numpy_compare.assert_float_equal(Ba, np.array([[1.0]]))

        sample_joint = plant.GetJointByName("ElbowJoint")
        Bj = plant.MakeActuatorSelectorMatrix(
            user_to_joint_index_map=[sample_joint.index()]
        )
        numpy_compare.assert_float_equal(Bj, np.array([[1.0]]))

        S = plant.MakeStateSelectorMatrix(
            user_to_joint_index_map=[sample_joint.index()]
        )
        numpy_compare.assert_float_equal(
            S, np.array([[0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        )

        forces = MultibodyForces(plant=plant)
        plant.CalcForceElementsContribution(context=context, forces=forces)
        tau = plant.CalcGeneralizedForces(context=context, forces=forces)
        self.assertEqual(tau.shape, (2,))
        copy.copy(forces)

        # Test generalized forces.
        # N.B. Cannot use `ndarray[object]` to reference existing C arrays
        # (#8116).
        if T is float:
            forces.mutable_generalized_forces()[:] = 1
            np.testing.assert_equal(forces.generalized_forces(), 1)
            forces.SetZero()
            np.testing.assert_equal(forces.generalized_forces(), 0)
            tau = plant.CalcGeneralizedForces(context=context, forces=forces)
            np.testing.assert_equal(tau, [0, 0])

        # Test standalone construction.
        standalone_forces = MultibodyForces(nb=1, nv=2)
        self.assertEqual(standalone_forces.num_bodies(), 1)
        self.assertEqual(standalone_forces.num_velocities(), 2)

        # Test body force accessors and mutators.
        link2 = plant.GetBodyByName("Link2")
        self.assertIsInstance(
            link2.GetForceInWorld(context, forces), SpatialForce
        )
        self.assertFalse(link2.is_floating_base_body())
        with catch_drake_warnings(expected_count=1) as w:
            self.assertFalse(link2.is_floating())
            self.assertIn("Use is_floating_base_body", str(w[0].message))

        forces.SetZero()
        F_expected = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
        link2.AddInForceInWorld(
            context, F_Bo_W=SpatialForce(F=F_expected), forces=forces
        )
        coeff = numpy_compare.to_float(
            link2.GetForceInWorld(context, forces).get_coeffs()
        )
        numpy_compare.assert_float_equal(coeff, F_expected)
        link2.AddInForce(
            context,
            p_BP_E=[0, 0, 0],
            F_Bp_E=SpatialForce(F=F_expected),
            frame_E=plant.world_frame(),
            forces=forces,
        )
        # Also check accumulation.
        np.testing.assert_equal(
            numpy_compare.to_float(
                link2.GetForceInWorld(context, forces).get_coeffs()
            ),
            2 * F_expected,
        )

    @numpy_compare.check_nonsymbolic_types
    def test_contact(self, T):
        # PenetrationAsPointPair has been bound for non-symbolic types only.
        PenetrationAsPointPair = PenetrationAsPointPair_[T]
        PointPairContactInfo = PointPairContactInfo_[T]
        ContactResults = ContactResults_[T]

        # PenetrationAsContactPair
        point_pair = PenetrationAsPointPair()
        self.assertTrue(isinstance(point_pair.id_A, GeometryId))
        self.assertTrue(isinstance(point_pair.id_B, GeometryId))
        self.assertTrue(point_pair.p_WCa.shape == (3,))
        self.assertTrue(point_pair.p_WCb.shape == (3,))
        self.assertTrue(isinstance(point_pair.depth, T))

        # PointPairContactInfo
        id_A = BodyIndex(0)
        id_B = BodyIndex(1)
        contact_info = PointPairContactInfo(
            bodyA_index=id_A,
            bodyB_index=id_B,
            f_Bc_W=np.array([0, 0, 1]),
            p_WC=np.array([0, 0, 0]),
            separation_speed=0,
            slip_speed=0,
            point_pair=point_pair,
        )
        self.assertTrue(isinstance(contact_info.bodyA_index(), BodyIndex))
        self.assertTrue(isinstance(contact_info.bodyB_index(), BodyIndex))
        self.assertTrue(contact_info.contact_force().shape == (3,))
        self.assertTrue(contact_info.contact_point().shape == (3,))
        self.assertTrue(isinstance(contact_info.slip_speed(), T))
        self.assertIsInstance(contact_info.point_pair(), PenetrationAsPointPair)
        copy.copy(contact_info)

        # ContactResults
        contact_results = ContactResults()
        self.assertTrue(contact_results.num_point_pair_contacts() == 0)
        self.assertIsNone(contact_results.plant())
        copy.copy(contact_results)
        contact_results.SelectHydroelastic(selector=lambda _: True)

    def test_contact_model(self):
        plant = MultibodyPlant_[float](0.1)
        models = [
            ContactModel.kHydroelastic,
            ContactModel.kPoint,
            ContactModel.kHydroelasticWithFallback,
            # Also test the legacy aliases.
            ContactModel.kHydroelasticsOnly,
            ContactModel.kPointContactOnly,
        ]
        for model in models:
            plant.set_contact_model(model)
            self.assertEqual(plant.get_contact_model(), model)

    def test_discrete_contact_approximation(self):
        plant = MultibodyPlant_[float](0.1)
        approximations = [
            DiscreteContactApproximation.kTamsi,
            DiscreteContactApproximation.kSap,
            DiscreteContactApproximation.kLagged,
            DiscreteContactApproximation.kSimilar,
        ]
        for approximation in approximations:
            plant.set_discrete_contact_approximation(approximation)
            self.assertEqual(
                plant.get_discrete_contact_approximation(), approximation
            )
        plant.get_sap_near_rigid_threshold()
        plant.set_sap_near_rigid_threshold(near_rigid_threshold=0.03)
        plant.get_discrete_contact_solver()

    def test_contact_surface_representation(self):
        for time_step in [0.0, 0.1]:
            plant = MultibodyPlant_[float](time_step)
            self.assertEqual(
                plant.get_contact_surface_representation(),
                plant.GetDefaultContactSurfaceRepresentation(time_step),
            )
            reps = [
                HydroelasticContactRepresentation.kTriangle,
                HydroelasticContactRepresentation.kPolygon,
            ]
            for rep in reps:
                plant.set_contact_surface_representation(rep)
                self.assertEqual(
                    plant.get_contact_surface_representation(), rep
                )

    def test_adjacent_bodies_collision_filters(self):
        plant = MultibodyPlant_[float](0.1)
        values = [False, True]
        for value in values:
            plant.set_adjacent_bodies_collision_filters(value=value)
            self.assertEqual(
                plant.get_adjacent_bodies_collision_filters(), value
            )

    def test_contact_results_to_lcm(self):
        # ContactResultsToLcmSystem
        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf"
        )
        plant = MultibodyPlant_[float](0.0)

        Parser(plant).AddModels(file_name)
        plant.Finalize()
        plant.set_penetration_allowance(penetration_allowance=0.0001)
        plant.set_stiction_tolerance(v_stiction=0.001)
        self.assertIsInstance(
            plant.get_contact_penalty_method_time_scale(), float
        )
        contact_results_to_lcm = ContactResultsToLcmSystem(plant=plant)
        context = contact_results_to_lcm.CreateDefaultContext()
        contact_results_to_lcm.get_input_port(0).FixValue(
            context, ContactResults_[float]()
        )
        output = contact_results_to_lcm.AllocateOutput()
        contact_results_to_lcm.CalcOutput(context, output)
        result = output.get_data(0)
        self.assertIsInstance(result, AbstractValue)

    def test_connect_contact_results(self):
        # For this test to be meaningful, the sdf file must contain collision
        # geometries. We'll do a reality check after instantiating.
        file_name = FindResourceOrThrow(
            "drake/bindings/pydrake/multibody/test/double_pendulum.sdf"
        )
        builder = DiagramBuilder_[float]()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
        Parser(plant).AddModels(file_name)
        plant.Finalize()
        self.assertGreater(
            len(
                plant.GetCollisionGeometriesForBody(plant.GetBodyByName("base"))
            ),
            0,
        )
        # Check all valid combinations of the optional arguments.
        for optional_args in itertools.product(
            [{}, {"lcm": None}, {"lcm": DrakeLcm()}],
            [{}, {"publish_period": None}, {"publish_period": 1.0 / 32}],
        ):
            kwargs = collections.ChainMap(*optional_args)
            with self.subTest(num_optional_args=len(kwargs), **kwargs):
                publisher = ConnectContactResultsToDrakeVisualizer(
                    builder=builder,
                    plant=plant,
                    scene_graph=scene_graph,
                    **kwargs,
                )
                self.assertIsInstance(publisher, LcmPublisherSystem)

    def test_collision_filter(self):
        builder_f = DiagramBuilder_[float]()
        # N.B. `Parser` only supports `MultibodyPlant_[float]`.
        plant_f, scene_graph_f = AddMultibodyPlantSceneGraph(builder_f, 0.0)
        parser = Parser(plant=plant_f, scene_graph=scene_graph_f)

        parser.AddModels(
            FindResourceOrThrow(
                "drake/bindings/pydrake/multibody/test/two_bodies.sdf"
            )
        )
        plant_f.Finalize()

        # Build a set with geometries from bodies 1 and 2.
        body_a = plant_f.GetBodyByName("body1")
        body_b = plant_f.GetBodyByName("body2")
        geometries = plant_f.CollectRegisteredGeometries([body_a, body_b])
        self.assertIsInstance(geometries, GeometrySet)

    @numpy_compare.check_nonsymbolic_types
    def test_scene_graph_queries(self, T):
        # TODO(sean.curtis@tri.global) The unit test of QueryObject/SceneGraph
        # don't belong in plant_test.py. They belong in geometry_test.py. This
        # test should be moved.
        # SceneGraph does not support `Expression` type.

        builder_f = DiagramBuilder_[float]()
        # N.B. `Parser` only supports `MultibodyPlant_[float]`.
        plant_f, scene_graph_f = AddMultibodyPlantSceneGraph(builder_f, 0.0)
        parser = Parser(plant=plant_f, scene_graph=scene_graph_f)
        # The validity of this test depends on the colliding geometries to be
        # supported across all nonsymbolic scalar types (double, autodiffxd).
        # If two_bodies.sdf were to change to unsupported geometries, this
        # test would break.
        parser.AddModels(
            FindResourceOrThrow(
                "drake/bindings/pydrake/multibody/test/two_bodies.sdf"
            )
        )
        plant_f.Finalize()
        diagram_f = builder_f.Build()

        # Do conversion.
        diagram = to_type(diagram_f, T)
        plant = diagram.GetSubsystemByName(plant_f.get_name())
        scene_graph = diagram.GetSubsystemByName(scene_graph_f.get_name())

        # The model `two_bodies` has two (implicitly) floating bodies that are
        # placed in the same position. The default state would be for these two
        # bodies to be coincident, and thus collide.
        context = diagram.CreateDefaultContext()
        sg_context = diagram.GetMutableSubsystemContext(scene_graph, context)
        plant_context = diagram.GetMutableSubsystemContext(plant, context)
        query_object = scene_graph.get_query_output_port().Eval(sg_context)
        # Implicitly require that this should be size 1.
        (point_pair,) = query_object.ComputePointPairPenetration()
        self.assertIsInstance(point_pair, PenetrationAsPointPair_[T])
        (signed_distance_pair,) = (
            query_object.ComputeSignedDistancePairwiseClosestPoints()
        )
        self.assertIsInstance(signed_distance_pair, SignedDistancePair_[T])
        signed_distance_to_point = query_object.ComputeSignedDistanceToPoint(
            p_WQ=np.ones(3)
        )
        self.assertEqual(len(signed_distance_to_point), 2)
        self.assertIsInstance(
            signed_distance_to_point[0], SignedDistanceToPoint_[T]
        )
        self.assertIsInstance(
            signed_distance_to_point[1], SignedDistanceToPoint_[T]
        )
        # Test SceneGraphInspector
        inspector = plant.EvalSceneGraphInspector(context=plant_context)

        self.assertEqual(inspector.num_geometries(), 4)
        self.assertEqual(
            inspector.num_geometries(), len(inspector.GetAllGeometryIds())
        )
        for geometry_id in inspector.GetAllGeometryIds(Role.kProximity):
            frame_id = inspector.GetFrameId(geometry_id)
            self.assertEqual(
                inspector.GetGeometryIdByName(
                    frame_id, Role.kProximity, inspector.GetName(geometry_id)
                ),
                geometry_id,
            )
            self.assertIsInstance(inspector.GetShape(geometry_id), Sphere)
            self.assertIsInstance(
                inspector.GetPoseInFrame(geometry_id), RigidTransform_[float]
            )

        def get_body_from_frame_id(frame_id):
            # Get body from frame id, and check inverse method.
            body = plant.GetBodyFromFrameId(frame_id)
            self.assertEqual(
                plant.GetBodyFrameIdIfExists(body.index()), frame_id
            )
            self.assertEqual(
                plant.GetBodyFrameIdOrThrow(body.index()), frame_id
            )
            return body

        bodies = {
            get_body_from_frame_id(inspector.GetFrameId(id_))
            for id_ in [point_pair.id_A, point_pair.id_B]
        }
        self.assertSetEqual(
            bodies, {plant.GetBodyByName("body1"), plant.GetBodyByName("body2")}
        )

        (id_,) = plant.GetVisualGeometriesForBody(
            body=plant.GetBodyByName("body1")
        )
        self.assertIsInstance(id_, GeometryId)

        (id_,) = plant.GetCollisionGeometriesForBody(
            body=plant.GetBodyByName("body1")
        )
        self.assertIsInstance(id_, GeometryId)

    @numpy_compare.check_all_types
    def test_externally_applied_spatial_force_multiplexer(self, T):
        system = ExternallyAppliedSpatialForceMultiplexer_[T](num_inputs=2)
        self.assertIsInstance(system, LeafSystem_[T])

    def test_propeller(self):
        plant = MultibodyPlant_[float](time_step=0.0)
        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf"
        )
        Parser(plant).AddModels(file_name)
        plant.Finalize()
        body = plant.GetBodyByName("Link1")

        info = PropellerInfo(
            body_index=body.index(),
            X_BP=RigidTransform_[float](),
            thrust_ratio=1.0,
            moment_ratio=0.1,
        )
        self.assertEqual(info.thrust_ratio, 1.0)
        self.assertEqual(info.moment_ratio, 0.1)
        copy.copy(info)

        prop = Propeller_[float](
            body_index=body.index(),
            X_BP=RigidTransform_[float](),
            thrust_ratio=1.0,
            moment_ratio=0.1,
        )
        self.assertEqual(prop.num_propellers(), 1)
        self.assertIsInstance(prop.get_command_input_port(), InputPort_[float])
        self.assertIsInstance(
            prop.get_body_poses_input_port(), InputPort_[float]
        )
        self.assertIsInstance(
            prop.get_spatial_forces_output_port(), OutputPort_[float]
        )

        prop2 = Propeller_[float]([info, info])
        self.assertEqual(prop2.num_propellers(), 2)

    def test_wing(self):
        builder = DiagramBuilder()
        plant = builder.AddSystem(MultibodyPlant(0.0))
        Parser(plant).AddModels(
            FindResourceOrThrow("drake/multibody/models/box.urdf")
        )
        plant.Finalize()

        body = plant.GetBodyByName("box")

        # Constructor
        Wing(
            body_index=body.index(),
            surface_area=1.0,
            X_BodyWing=RigidTransform(),
            fluid_density=1.0,
        )

        # AddToBuilder
        wing = Wing.AddToBuilder(
            builder=builder,
            plant=plant,
            body_index=body.index(),
            surface_area=1.0,
            X_BodyWing=RigidTransform(),
            fluid_density=1.0,
        )

        self.assertIsInstance(
            wing.get_body_poses_input_port(), InputPort_[float]
        )
        self.assertIsInstance(
            wing.get_body_spatial_velocities_input_port(), InputPort_[float]
        )
        self.assertIsInstance(
            wing.get_body_poses_input_port(), InputPort_[float]
        )
        self.assertIsInstance(
            wing.get_wind_velocity_input_port(), InputPort_[float]
        )
        self.assertIsInstance(
            wing.get_fluid_density_input_port(), InputPort_[float]
        )
        self.assertIsInstance(
            wing.get_spatial_force_output_port(), OutputPort_[float]
        )
        self.assertIsInstance(
            wing.get_aerodynamic_center_output_port(), OutputPort_[float]
        )

    def test_hydroelastic_contact_results(self):
        time_steps = [
            0.0,  # Continuous mode.
            0.01,  # Discrete mode.
        ]
        for time_step in time_steps:
            with self.subTest(time_step=time_step):
                self._check_hydroelastic_contact_results(time_step)

    def _check_hydroelastic_contact_results(self, time_step):
        builder = DiagramBuilder_[float]()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step)
        Parser(plant).AddModels(
            FindResourceOrThrow(
                "drake/bindings/pydrake/multibody/test/hydroelastic.sdf"
            )
        )
        plant.set_contact_model(ContactModel.kHydroelastic)
        plant.SetUseSampledOutputPorts(False)  # We're not stepping time.
        plant.Finalize()

        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        plant_context = plant.GetMyContextFromRoot(context)

        body1 = plant.GetBodyByName("body1")
        X_WB = RigidTransform(p=[0, 0, 0.25])
        plant.SetFreeBodyPose(plant_context, body1, X_WB)

        contact_results = plant.get_contact_results_output_port().Eval(
            plant_context
        )

        self.assertEqual(contact_results.num_hydroelastic_contacts(), 1)
        contact_info = contact_results.hydroelastic_contact_info(0)
        self._sanity_check_contact_surface(contact_info.contact_surface())
        contact_info.F_Ac_W().get_coeffs()

    def _sanity_check_contact_surface(self, dut):
        dut.id_M()
        dut.id_N()
        dut.num_faces()
        dut.num_vertices()
        dut.area(face_index=0)
        dut.total_area()
        dut.face_normal(face_index=0)
        dut.centroid(face_index=0)
        dut.centroid()
        dut.is_triangle()
        dut.representation()
        if dut.is_triangle():
            dut.tri_mesh_W().centroid()
        else:
            dut.poly_mesh_W().centroid()
        if dut.HasGradE_M():
            dut.EvaluateGradE_M_W(index=0)
        if dut.HasGradE_N():
            dut.EvaluateGradE_N_W(index=0)
        dut.Equal(surface=dut)
        copy.copy(dut)

    def test_floating_base_bodies(self):
        plant = MultibodyPlant_[float](time_step=0.01)
        model_instance = plant.AddModelInstance("new instance")
        added_body = plant.AddRigidBody(
            name="body", model_instance=model_instance
        )
        plant.Finalize()
        self.assertTrue(plant.HasBodyNamed("body", model_instance))
        self.assertTrue(
            plant.HasUniqueFloatingBaseBody(model_instance=model_instance)
        )
        body = plant.GetUniqueFloatingBaseBodyOrThrow(
            model_instance=model_instance
        )
        self.assertEqual(body.index(), added_body.index())
        with catch_drake_warnings(expected_count=1) as w:
            self.assertTrue(
                plant.HasUniqueFreeBaseBody(model_instance=model_instance)
            )
            self.assertIn("Use HasUniqueFloatingBaseBody", str(w[0].message))
        with catch_drake_warnings(expected_count=1) as w:
            body = plant.GetUniqueFreeBaseBodyOrThrow(
                model_instance=model_instance
            )
            self.assertEqual(body.index(), added_body.index())
            self.assertIn(
                "Use GetUniqueFloatingBaseBodyOrThrow", str(w[0].message)
            )

    @numpy_compare.check_all_types
    def test_deformable_contact_info(self, T):
        if T == Expression:
            return

        vertices = [
            np.array([0, 0, 0]),
            np.array([1, 0, 0]),
            np.array([1, 1, 0]),
            np.array([0, 1, 0]),
        ]
        face_data = [3, 0, 1, 2, 3, 2, 3, 0]
        contact_mesh = PolygonSurfaceMesh_[T](face_data, vertices)
        id_A = GeometryId.get_new_id()
        id_B = GeometryId.get_new_id()
        F_Ac_W = SpatialForce_[T](np.array([1, 2, 3]), np.array([4, 5, 6]))
        dut = DeformableContactInfo_[T](id_A, id_B, contact_mesh, F_Ac_W)

        self.assertEqual(dut.id_A(), id_A)
        self.assertEqual(dut.id_B(), id_B)
        self.assertTrue(dut.contact_mesh().Equal(contact_mesh))
        numpy_compare.assert_equal(
            dut.F_Ac_W().translational(), F_Ac_W.translational()
        )
        numpy_compare.assert_equal(
            dut.F_Ac_W().rotational(), F_Ac_W.rotational()
        )

    def test_deformable_model_empty_model(self):
        builder = DiagramBuilder_[float]()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.01)
        dut = plant.deformable_model()
        self.assertEqual(dut.num_bodies(), 0)
        self.assertTrue(dut.is_empty())

    def test_deformable_model_registration_and_query(self):
        builder = DiagramBuilder_[float]()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.01)
        dut = plant.mutable_deformable_model()

        config = DeformableBodyConfig_[float]()
        sphere = GeometryInstance(RigidTransform(), Sphere(1.0), "sphere")
        props = ProximityProperties()
        props.AddProperty(
            "material", "coulomb_friction", CoulombFriction_[float](1.0, 1.0)
        )
        sphere.set_proximity_properties(props)
        id1 = dut.RegisterDeformableBody(
            geometry_instance=sphere, config=config, resolution_hint=1.0
        )

        model_instance = plant.AddModelInstance("deformable_instance")
        sphere2 = GeometryInstance(RigidTransform(), Sphere(2.0), "sphere2")
        sphere2.set_proximity_properties(props)
        id2 = dut.RegisterDeformableBody(
            geometry_instance=sphere2,
            model_instance=model_instance,
            config=config,
            resolution_hint=1.0,
        )

        # Two bodies registered
        self.assertEqual(dut.num_bodies(), 2)

        # Round-trip geometry <-> body id
        geom_id = dut.GetGeometryId(id1)
        self.assertEqual(dut.GetBodyId(geometry_id=geom_id), id1)

        # GetBody by id
        body1 = dut.GetBody(id=id1)
        index1 = body1.index()

        # GetBody by index
        body1_again = dut.GetBody(index=index1)
        self.assertEqual(body1.body_id(), body1_again.body_id())

        # index <-> id
        self.assertEqual(dut.GetBodyId(index=index1), id1)
        self.assertEqual(dut.GetBodyIndex(id=id1), index1)

        # Mutable body
        self.assertEqual(dut.GetMutableBody(id=id1).body_id(), id1)

        # Per-instance listing
        ids = dut.GetBodyIds(model_instance=model_instance)
        self.assertEqual(ids, [id2])

        # Name-based lookup
        self.assertTrue(dut.HasBodyNamed(name="sphere"))
        self.assertTrue(
            dut.HasBodyNamed(name="sphere2", model_instance=model_instance)
        )
        body1 = plant.deformable_model().GetBodyByName(name="sphere")
        self.assertEqual(body1.body_id(), id1)
        body2 = plant.deformable_model().GetBodyByName(
            name="sphere2", model_instance=model_instance
        )
        self.assertEqual(body2.body_id(), id2)

    def test_deformable_model_constraints_and_lookup(self):
        builder = DiagramBuilder_[float]()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.01)
        dut = plant.mutable_deformable_model()

        # Register one body
        config = DeformableBodyConfig_[float]()
        geometry = GeometryInstance(RigidTransform(), Sphere(1.0), "sphere")
        body_id = dut.RegisterDeformableBody(
            geometry_instance=geometry, config=config, resolution_hint=1.0
        )

        # Wall boundary condition
        dut.SetWallBoundaryCondition(id=body_id, p_WQ=[0, 0, 0], n_W=[0, 0, 1])

        # Add a rigid body and fixed constraint
        model_instance = plant.AddModelInstance("deformable_instance")
        inertia = SpatialInertia_[float].SolidCubeWithDensity(1, 1)
        rigid_body = plant.AddRigidBody("rigid_body", model_instance, inertia)
        dut.AddFixedConstraint(
            body_A_id=body_id,
            body_B=rigid_body,
            X_BA=RigidTransform(),
            shape_G=Box(1, 1, 1),
            X_BG=RigidTransform(),
        )
        self.assertTrue(dut.HasConstraint(id=body_id))

    def test_deformable_model_parallelism(self):
        builder = DiagramBuilder_[float]()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.01)
        dut = plant.mutable_deformable_model()

        # Switch to 2 threads
        dut._set_parallelism(parallelism=Parallelism(2))
        self.assertEqual(dut._parallelism().num_threads(), 2)
        # Back to single thread
        dut._set_parallelism(parallelism=Parallelism(False))
        self.assertEqual(dut._parallelism().num_threads(), 1)

    def test_deformable_model_simulation_and_positions(self):
        builder = DiagramBuilder_[float]()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1.0e-3)
        dut = plant.mutable_deformable_model()

        # Register one body
        config = DeformableBodyConfig_[float]()
        geometry = GeometryInstance(RigidTransform(), Sphere(1.0), "sphere")
        props = ProximityProperties()
        props.AddProperty(
            "material", "coulomb_friction", CoulombFriction_[float](1.0, 1.0)
        )
        geometry.set_proximity_properties(props)
        body_id = dut.RegisterDeformableBody(
            geometry_instance=geometry, config=config, resolution_hint=1.0
        )

        plant.Finalize()
        diagram = builder.Build()
        sim = Simulator_[float](diagram)
        sim.AdvanceTo(0.01)
        plant_context = plant.GetMyContextFromRoot(sim.get_context())

        # Reference vs. simulated positions
        q_ref = dut.GetReferencePositions(id=body_id).reshape((3, -1))
        q_sim = dut.GetPositions(context=plant_context, id=body_id)
        numpy_compare.assert_float_not_equal(q_ref, q_sim)

        # Positions from state
        num_dofs = q_sim.size
        state_index = dut.GetDiscreteStateIndex(id=body_id)
        discrete_state = plant_context.get_discrete_state(state_index)
        state_value = discrete_state.get_value()
        q_state = state_value[:num_dofs]
        numpy_compare.assert_float_equal(
            q_state.reshape((3, -1), order="F"), q_sim
        )

        # Round-trip set/get under context
        dut.SetPositions(context=plant_context, id=body_id, q=q_ref)
        q2 = dut.GetPositions(context=plant_context, id=body_id)
        numpy_compare.assert_float_equal(q_ref, q2)

        # Set velocities and get them back.
        num_nodes = num_dofs // 3
        v = np.arange(num_dofs, dtype=float).reshape(3, num_nodes)
        dut.SetVelocities(context=plant_context, id=body_id, v=v)
        v_back = dut.GetVelocities(context=plant_context, id=body_id)
        numpy_compare.assert_float_equal(v, v_back)

        # Set positions and velocities and get them back.
        dut.SetPositionsAndVelocities(
            context=plant_context, id=body_id, q=2 * q_ref, v=2 * v
        )
        q2 = dut.GetPositions(context=plant_context, id=body_id)
        v2 = dut.GetVelocities(context=plant_context, id=body_id)
        numpy_compare.assert_float_equal(2 * q_ref, q2)
        numpy_compare.assert_float_equal(2 * v, v2)

        contact_results = plant.get_contact_results_output_port().Eval(
            plant_context
        )
        # There is no deformable contact, but we can still try the API.
        self.assertEqual(contact_results.num_deformable_contacts(), 0)
        # Complains about index out of range.
        with self.assertRaisesRegex(
            SystemExit, ".*i < num_deformable_contacts().*"
        ):
            contact_results.deformable_contact_info(0)

    def test_deformable_model_external_force(self):
        builder = DiagramBuilder_[float]()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.01)
        dut = plant.mutable_deformable_model()

        # Register one body
        config = DeformableBodyConfig_[float]()
        geometry = GeometryInstance(RigidTransform(), Sphere(1.0), "sphere")
        body_id = dut.RegisterDeformableBody(
            geometry_instance=geometry, config=config, resolution_hint=1.0
        )

        dut.AddExternalForce(
            GravityForceField_[float](
                gravity_vector=[0, 0, 1], mass_density=1.23
            )
        )
        plant.Finalize()
        context = plant.CreateDefaultContext()

        external_forces = dut.GetExternalForces(body_id)
        numpy_compare.assert_float_equal(
            external_forces[-1].EvaluateAt(context, [0, 0, 0]), [0, 0, 1.23]
        )

    def test_deformable_model_disable_enable(self):
        builder = DiagramBuilder_[float]()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1.0e-3)
        dut = plant.deformable_model()

        # Register one body
        config = DeformableBodyConfig_[float]()
        geometry = GeometryInstance(RigidTransform(), Sphere(1.0), "sphere")
        props = ProximityProperties()
        props.AddProperty(
            "material", "coulomb_friction", CoulombFriction_[float](1.0, 1.0)
        )
        geometry.set_proximity_properties(props)
        body_id = dut.RegisterDeformableBody(
            geometry_instance=geometry, config=config, resolution_hint=1.0
        )

        plant.Finalize()
        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        plant_context = plant.GetMyContextFromRoot(context)

        # Disable and re-enable the deformable body
        self.assertTrue(dut.is_enabled(id=body_id, context=plant_context))
        dut.Disable(id=body_id, context=plant_context)
        self.assertFalse(dut.is_enabled(id=body_id, context=plant_context))
        dut.Enable(id=body_id, context=plant_context)
        self.assertTrue(dut.is_enabled(id=body_id, context=plant_context))

    def test_deformable_body_creation_and_metadata(self):
        builder = DiagramBuilder_[float]()
        plant, scene_graph = AddMultibodyPlantSceneGraph(
            builder=builder, time_step=0.01
        )
        deformable_model = plant.mutable_deformable_model()

        config = DeformableBodyConfig_[float]()
        props = ProximityProperties()
        props.AddProperty(
            "material",
            "coulomb_friction",
            CoulombFriction_[float](static_friction=1.0, dynamic_friction=1.0),
        )
        geometry = GeometryInstance(
            X_PG=RigidTransform(), shape=Sphere(radius=1.0), name="sphere"
        )
        geometry.set_proximity_properties(properties=props)

        body_id = deformable_model.RegisterDeformableBody(
            geometry_instance=geometry, config=config, resolution_hint=1.0
        )
        plant.Finalize()

        body = deformable_model.GetBody(id=body_id)

        # body_id, name, geometry_id, config
        self.assertEqual(body.body_id(), body_id)
        self.assertEqual(body.name(), "sphere")
        self.assertIsInstance(body.scoped_name(), ScopedName)
        self.assertEqual(
            body.geometry_id(), deformable_model.GetGeometryId(id=body_id)
        )
        self.assertIsInstance(body.config(), DeformableBodyConfig_[float])

        # num_dofs() and reference_positions()
        num_dofs = body.num_dofs()
        reference_positions = body.reference_positions()
        self.assertIsInstance(reference_positions, np.ndarray)
        self.assertEqual(num_dofs, reference_positions.size)

        # external_forces
        external_forces = body.external_forces()
        for external_force in external_forces:
            self.assertIsInstance(external_force, ForceDensityField_[float])

        # discrete_state_index()
        state_index = body.discrete_state_index()
        self.assertIsInstance(state_index, DiscreteStateIndex)

        # is_enabled_parameter_index()
        parameter_index = body.is_enabled_parameter_index()
        self.assertIsInstance(parameter_index, AbstractParameterIndex)

    def test_deformable_body_boundary_and_constraints(self):
        builder = DiagramBuilder_[float]()
        plant, scene_graph = AddMultibodyPlantSceneGraph(
            builder=builder, time_step=0.01
        )
        deformable_model = plant.mutable_deformable_model()

        config = DeformableBodyConfig_[float]()
        props = ProximityProperties()
        props.AddProperty(
            "material",
            "coulomb_friction",
            CoulombFriction_[float](static_friction=1.0, dynamic_friction=1.0),
        )
        geometry = GeometryInstance(
            X_PG=RigidTransform(), shape=Sphere(radius=1.0), name="sphere"
        )
        geometry.set_proximity_properties(properties=props)
        id = deformable_model.RegisterDeformableBody(
            geometry_instance=geometry, config=config, resolution_hint=1.0
        )
        body = deformable_model.GetBody(id=id)

        # No fixed constraint yet
        self.assertFalse(body.has_fixed_constraint())

        # Wall boundary condition
        body.SetWallBoundaryCondition(p_WQ=[0, 0, 1], n_W=[0, 0, 1])

        # Add a fixed constraint.
        inertia = SpatialInertia_[float].SolidCubeWithDensity(1, 1)
        rigid_body = plant.AddRigidBody("rigid_body", inertia)
        body.AddFixedConstraint(
            body_B=rigid_body,
            X_BA=RigidTransform(),
            shape_G=Box(width=0.5, depth=0.5, height=0.5),
            X_BG=RigidTransform(),
        )
        self.assertTrue(body.has_fixed_constraint())

    def test_deformable_body_state_methods(self):
        builder = DiagramBuilder_[float]()
        plant, scene_graph = AddMultibodyPlantSceneGraph(
            builder=builder, time_step=0.01
        )
        deformable_model = plant.mutable_deformable_model()

        config = DeformableBodyConfig_[float]()
        props = ProximityProperties()
        props.AddProperty(
            "material",
            "coulomb_friction",
            CoulombFriction_[float](static_friction=1.0, dynamic_friction=1.0),
        )
        geometry = GeometryInstance(
            X_PG=RigidTransform(), shape=Sphere(radius=1.0), name="sphere"
        )
        geometry.set_proximity_properties(properties=props)
        body_id = deformable_model.RegisterDeformableBody(
            geometry_instance=geometry, config=config, resolution_hint=1.0
        )

        # Build and finalize the diagram
        plant.Finalize()
        diagram = builder.Build()
        diagram_context = diagram.CreateDefaultContext()
        plant_context = plant.GetMyContextFromRoot(root_context=diagram_context)

        body = deformable_model.GetBody(id=body_id)

        # Enabled by default
        self.assertTrue(body.is_enabled(context=plant_context))

        # Disable then re-enable
        body.Disable(context=plant_context)
        self.assertFalse(body.is_enabled(context=plant_context))
        body.Enable(context=plant_context)
        self.assertTrue(body.is_enabled(context=plant_context))

        # Round-trip SetPositions / GetPositions
        reference_positions = body.reference_positions()
        reference_positions_reshaped = reference_positions.reshape(
            (3, -1), order="F"
        )
        body.SetPositions(context=plant_context, q=reference_positions_reshaped)
        positions_after = body.GetPositions(context=plant_context)
        numpy_compare.assert_float_equal(
            reference_positions_reshaped, positions_after
        )

        # Round-trip SetVelocities / GetVelocities
        num_dofs = body.num_dofs()
        num_nodes = num_dofs // 3
        v = np.arange(num_dofs, dtype=float).reshape(3, num_nodes)
        body.SetVelocities(context=plant_context, v=v)
        v_back = body.GetVelocities(context=plant_context)
        numpy_compare.assert_float_equal(v, v_back)

        # Round-trip SetPositionsAndVelocities / GetPositionsAndVelocities
        q = 2 * reference_positions_reshaped
        v = 2 * v
        body.SetPositionsAndVelocities(context=plant_context, q=q, v=v)
        q2 = body.GetPositions(context=plant_context)
        v2 = body.GetVelocities(context=plant_context)
        numpy_compare.assert_float_equal(q, q2)
        numpy_compare.assert_float_equal(v, v2)

        # set_default_pose and get_default_pose
        X_WD = RigidTransform_[float](
            RollPitchYaw_[float](np.pi / 2, 0, np.pi / 2), [1, 2, 3]
        )
        body.set_default_pose(X_WD=X_WD)
        self.assertTrue(body.get_default_pose().IsExactlyEqualTo(X_WD))

        # CalcCenterOfMassPositionInWorld
        p_WScm = body.CalcCenterOfMassPositionInWorld(context=plant_context)
        self.assertIsInstance(p_WScm, np.ndarray)
        self.assertEqual(p_WScm.size, 3)

        # CalcCenterOfMassTranslationalVelocityInWorld
        v_WScm = body.CalcCenterOfMassTranslationalVelocityInWorld(
            context=plant_context
        )
        self.assertIsInstance(v_WScm, np.ndarray)
        self.assertEqual(v_WScm.size, 3)

        # CalcEffectiveAngularVelocity
        w_WScm = body.CalcEffectiveAngularVelocity(context=plant_context)
        self.assertIsInstance(w_WScm, np.ndarray)
        self.assertEqual(w_WScm.size, 3)

    @numpy_compare.check_all_types
    def test_gravity_force_field(self, T):
        plant = MultibodyPlant_[T](time_step=0.0)
        plant.Finalize()
        context = plant.CreateDefaultContext()

        dut = GravityForceField_[T](gravity_vector=[0, 0, 1], mass_density=1.23)

        self.assertEqual(
            dut.density_type(), ForceDensityType.kPerReferenceVolume
        )
        self.assertFalse(dut.has_parent_system())

        numpy_compare.assert_float_equal(
            dut.EvaluateAt(context, [0, 0, 0]), [0, 0, 1.23]
        )

        dut.Clone()
        copy.copy(dut)
        copy.deepcopy(dut)

    @numpy_compare.check_all_types
    def test_force_density_field(self, T):
        class DummyField(ForceDensityField_[T]):
            def __init__(self, scale):
                super().__init__()
                self._scale = scale

            def DoEvaluateAt(self, context, p_WQ):
                return p_WQ * self._scale

            def DoClone(self):
                return DummyField(self._scale)

        plant = MultibodyPlant_[T](time_step=0.0)
        plant.Finalize()
        context = plant.CreateDefaultContext()

        dut = DummyField(2.0)
        self.assertEqual(dut.density_type(), ForceDensityType.kPerCurrentVolume)
        self.assertFalse(dut.has_parent_system())

        p_WQ = [1.0, 2.0, 3.0]
        value = [2.0, 4.0, 6.0]
        numpy_compare.assert_float_equal(dut.EvaluateAt(context, p_WQ), value)
        numpy_compare.assert_float_equal(
            dut.Clone().EvaluateAt(context, p_WQ), value
        )
        numpy_compare.assert_float_equal(
            copy.copy(dut).EvaluateAt(context, p_WQ), value
        )
        numpy_compare.assert_float_equal(
            copy.deepcopy(dut).EvaluateAt(context, p_WQ), value
        )
