# -*- coding: utf-8 -*-

import unittest

import numpy as np
import warnings

from pydrake.autodiffutils import AutoDiffXd
from pydrake.symbolic import Expression
from pydrake.multibody.tree import (
    BallRpyJoint_,
    Body_,
    BodyIndex,
    DoorHinge_,
    DoorHingeConfig,
    FixedOffsetFrame_,
    ForceElement_,
    ForceElementIndex,
    Frame_,
    FrameIndex,
    JacobianWrtVariable,
    Joint_,
    JointActuator_,
    JointActuatorIndex,
    JointIndex,
    LinearSpringDamper_,
    ModelInstanceIndex,
    MultibodyForces_,
    PrismaticJoint_,
    RevoluteJoint_,
    RevoluteSpring_,
    RotationalInertia_,
    RigidBody_,
    SpatialInertia_,
    UniformGravityFieldElement_,
    UnitInertia_,
    UniversalJoint_,
    WeldJoint_,
    world_index,
    world_model_instance,
    default_model_instance
)
from pydrake.multibody.math import (
    SpatialForce_,
    SpatialMomentum_,
    SpatialVelocity_,
    SpatialAcceleration_,
)
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph,
    CalcContactFrictionFromSurfaceProperties,
    ConnectContactResultsToDrakeVisualizer,
    ContactResults_,
    ContactResultsToLcmSystem,
    CoulombFriction_,
    ExternallyAppliedSpatialForce_,
    MultibodyPlant_,
    PointPairContactInfo_,
    PropellerInfo,
    Propeller_,
    VectorExternallyAppliedSpatialForced,
    VectorExternallyAppliedSpatialForced_,
)
from pydrake.multibody.parsing import Parser
from pydrake.multibody.benchmarks.acrobot import (
    AcrobotParameters,
    MakeAcrobotPlant,
)
from pydrake.common.cpp_param import List
from pydrake.common import FindResourceOrThrow
from pydrake.common.deprecation import install_numpy_warning_filters
from pydrake.common.test_utilities import numpy_compare
from pydrake.common.test_utilities.deprecation import catch_drake_warnings
from pydrake.common.value import AbstractValue, Value
from pydrake.geometry import (
    Box,
    GeometryId,
    GeometrySet,
    Role,
    PenetrationAsPointPair_,
    ProximityProperties,
    SceneGraph_,
    SignedDistancePair_,
    SignedDistanceToPoint_,
    Sphere,
)
from pydrake.math import (
    RigidTransform_,
    RollPitchYaw_,
    RotationMatrix_,
)
from pydrake.systems.analysis import Simulator_
from pydrake.systems.framework import (
    BasicVector_,
    DiagramBuilder_,
    System_,
    LeafSystem_,
    InputPort_,
    OutputPort_,
)
from pydrake.systems.scalar_conversion import TemplateSystem
from pydrake.systems.lcm import LcmPublisherSystem


def get_index_class(cls, T):
    # Maps a class to its corresponding index class, accommdating inheritance.
    class_to_index_class_map = {
        Body_[T]: BodyIndex,
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
# TODO(eric.cousineau): Consider hoisting this to `test_utilities` or adding
# `System.ToType[T]`.
def to_type(system, T):
    assert isinstance(system, System_[float])
    if T == float:
        return system
    elif T == AutoDiffXd:
        return system.ToAutoDiffXd()
    elif T == Expression:
        return system.ToSymbolic()
    else:
        assert False, "Invalid type, {}".format(T)


class TestPlant(unittest.TestCase):
    def setUp(self):
        unittest.TestCase.setUp(self)
        # For some reason, something in how `unittest` tries to scope warnings
        # causes the previous filters to be lost. Re-install here.
        # TODO(eric.cousineau): This used to be necessary for PY3-only, but
        # with NumPy 1.16, it became PY2 too. Figure out why.
        install_numpy_warning_filters(force=True)

    def test_type_safe_indices(self):
        self.assertEqual(world_index(), BodyIndex(0))
        self.assertEqual(repr(world_index()), "BodyIndex(0)")
        self.assertEqual(world_model_instance(), ModelInstanceIndex(0))
        self.assertEqual(repr(world_model_instance()), "ModelInstanceIndex(0)")
        self.assertEqual(default_model_instance(), ModelInstanceIndex(1))
        self.assertTrue(ModelInstanceIndex(0) < ModelInstanceIndex(1))

    def assert_sane(self, x, nonzero=True):
        self.assertTrue(np.all(np.isfinite(numpy_compare.to_float(x))))
        if nonzero:
            numpy_compare.assert_float_not_equal(x, 0.)

    @numpy_compare.check_nonsymbolic_types
    def test_multibody_plant_construction_api(self, T):
        # SceneGraph does not support `Expression` type.
        DiagramBuilder = DiagramBuilder_[T]
        SpatialInertia = SpatialInertia_[float]
        RigidTransform = RigidTransform_[T]
        CoulombFriction = CoulombFriction_[T]

        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
        self.assertEqual(plant.time_step(), 0.0)
        spatial_inertia = SpatialInertia()
        body = plant.AddRigidBody(name="new_body",
                                  M_BBo_B=spatial_inertia)
        body_mass = body.default_mass()
        body_com = body.default_com()
        body_default_unit_inertia = body.default_unit_inertia()
        body_default_spatial_inertial = body.default_spatial_inertia()
        new_model_instance = plant.AddModelInstance("new_model_instance")
        body = plant.AddRigidBody(name="new_body_2",
                                  M_BBo_B=spatial_inertia,
                                  model_instance=new_model_instance)
        box = Box(width=0.5, depth=1.0, height=2.0)
        body_X_BG = RigidTransform()
        body_friction = CoulombFriction(static_friction=0.6,
                                        dynamic_friction=0.5)
        self.assertEqual(body_friction.static_friction(), 0.6)
        self.assertEqual(body_friction.dynamic_friction(), 0.5)
        body_friction2 = CoulombFriction(static_friction=0.2,
                                         dynamic_friction=0.1)
        combined_friction = CalcContactFrictionFromSurfaceProperties(
            body_friction, body_friction2)

        if T == float:
            plant.RegisterVisualGeometry(
                body=body, X_BG=body_X_BG, shape=box, name="new_body_visual",
                diffuse_color=[1., 0.64, 0.0, 0.5])
            plant.RegisterCollisionGeometry(
                body=body, X_BG=body_X_BG, shape=box,
                name="new_body_collision", coulomb_friction=body_friction)
            self.assertGreater(plant.num_collision_geometries(), 0)
            body0_props = scene_graph.model_inspector().GetProximityProperties(
                plant.GetCollisionGeometriesForBody(body)[0])
            body0_friction = body0_props.GetProperty(
                "material", "coulomb_friction")
            self.assertEqual(body0_friction.static_friction(), 0.6)
            self.assertEqual(body0_friction.dynamic_friction(), 0.5)
            explicit_props = ProximityProperties()
            explicit_props.AddProperty("material", "coulomb_friction",
                                       CoulombFriction(1.1, 0.8))
            plant.RegisterCollisionGeometry(
                body=body, X_BG=body_X_BG, shape=box,
                name="new_body_collision2", properties=explicit_props)
            body1_props = scene_graph.model_inspector().GetProximityProperties(
                plant.GetCollisionGeometriesForBody(body)[1])
            body1_friction = body1_props.GetProperty(
                "material", "coulomb_friction")
            self.assertGreater(plant.num_collision_geometries(), 1)
            self.assertEqual(body1_friction.static_friction(), 1.1)
            self.assertEqual(body1_friction.dynamic_friction(), 0.8)

    @numpy_compare.check_all_types
    def test_multibody_plant_api_via_parsing(self, T):
        MultibodyPlant = MultibodyPlant_[T]
        Joint = Joint_[T]
        Body = Body_[T]
        Frame = Frame_[T]
        JointActuator = JointActuator_[T]
        InputPort = InputPort_[T]
        OutputPort = OutputPort_[T]

        def check_repr(element, expected):
            if T == float:
                self.assertEqual(repr(element), expected)

        # TODO(eric.cousineau): Decouple this when construction can be done
        # without parsing.
        # This a subset of `multibody_plant_sdf_parser_test.cc`.
        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf")
        # N.B. `Parser` only supports `MultibodyPlant_[float]`.
        plant_f = MultibodyPlant_[float](time_step=0.01)
        model_instance = Parser(plant_f).AddModelFromFile(file_name)
        self.assertIsInstance(model_instance, ModelInstanceIndex)
        check_repr(model_instance, "ModelInstanceIndex(2)")
        plant_f.Finalize()
        plant = to_type(plant_f, T)

        benchmark = MakeAcrobotPlant(AcrobotParameters(), True)
        self.assertEqual(plant.num_bodies(), benchmark.num_bodies())
        self.assertEqual(plant.num_joints(), benchmark.num_joints())
        self.assertEqual(plant.num_actuators(), benchmark.num_actuators())
        self.assertEqual(
            plant.num_model_instances(), benchmark.num_model_instances() + 1)
        self.assertEqual(plant.num_positions(), benchmark.num_positions())
        self.assertEqual(
            plant.num_positions(model_instance=model_instance),
            benchmark.num_positions())
        self.assertEqual(
            plant.num_velocities(), benchmark.num_velocities())
        self.assertEqual(
            plant.num_velocities(model_instance=model_instance),
            benchmark.num_velocities())
        self.assertEqual(
            plant.num_multibody_states(), benchmark.num_multibody_states())
        self.assertEqual(
            plant.num_multibody_states(model_instance=model_instance),
            benchmark.num_multibody_states())
        self.assertEqual(
            plant.num_actuated_dofs(), benchmark.num_actuated_dofs())
        self.assertEqual(
            plant.num_actuated_dofs(model_instance=model_instance),
            benchmark.num_actuated_dofs())
        self.assertTrue(plant.is_finalized())
        self.assertTrue(plant.HasBodyNamed(name="Link1"))
        self.assertTrue(plant.HasBodyNamed(
            name="Link1", model_instance=model_instance))
        self.assertTrue(plant.HasJointNamed(name="ShoulderJoint"))
        self.assertTrue(plant.HasJointNamed(
            name="ShoulderJoint", model_instance=model_instance))
        shoulder = plant.GetJointByName(name="ShoulderJoint")
        self._test_joint_api(T, shoulder)
        check_repr(
            shoulder,
            "<RevoluteJoint_[float] name='ShoulderJoint' index=0 "
            "model_instance=2>")
        np.testing.assert_array_equal(
            shoulder.position_lower_limits(), [-np.inf])
        np.testing.assert_array_equal(
            shoulder.position_upper_limits(), [np.inf])
        self.assertIs(shoulder, plant.GetJointByName(
            name="ShoulderJoint", model_instance=model_instance))
        self._test_joint_actuator_api(
            T, plant.GetJointActuatorByName(name="ElbowJoint"))
        link1 = plant.GetBodyByName(name="Link1")
        self._test_body_api(T, link1)
        self.assertIs(
            link1,
            plant.GetBodyByName(name="Link1", model_instance=model_instance))
        self.assertEqual(len(plant.GetBodyIndices(model_instance)), 2)
        check_repr(
            link1,
            "<RigidBody_[float] name='Link1' index=1 model_instance=2>")
        self._test_frame_api(T, plant.GetFrameByName(name="Link1"))
        link1_frame = plant.GetFrameByName(name="Link1")
        check_repr(
            link1_frame,
            "<BodyFrame_[float] name='Link1' index=1 model_instance=2>")
        self.assertIs(
            link1_frame,
            plant.GetFrameByName(name="Link1", model_instance=model_instance))
        self.assertTrue(plant.HasModelInstanceNamed(name="acrobot"))
        self.assertEqual(
            model_instance, plant.GetModelInstanceByName(name="acrobot"))
        self.assertIsInstance(
            plant.get_actuation_input_port(), InputPort)
        self.assertIsInstance(
            plant.get_state_output_port(), OutputPort)
        self.assertIsInstance(
            plant.get_generalized_acceleration_output_port(), OutputPort)
        self.assertIsInstance(
            plant.get_reaction_forces_output_port(), OutputPort)
        self.assertIsInstance(
            plant.get_contact_results_output_port(), OutputPort)
        self.assertIsInstance(plant.num_frames(), int)
        self.assertIsInstance(plant.get_body(body_index=BodyIndex(0)), Body)
        self.assertIs(shoulder, plant.get_joint(joint_index=JointIndex(0)))
        joint_actuator = plant.get_joint_actuator(
            actuator_index=JointActuatorIndex(0))
        self.assertIsInstance(joint_actuator, JointActuator)
        check_repr(
            joint_actuator,
            "<JointActuator_[float] name='ElbowJoint' index=0 "
            "model_instance=2>")
        self.assertIsInstance(
            plant.get_frame(frame_index=FrameIndex(0)), Frame)
        self.assertEqual("acrobot", plant.GetModelInstanceName(
            model_instance=model_instance))
        self.assertIn("acrobot", plant.GetTopologyGraphvizString())

    def _test_multibody_tree_element_mixin(self, T, element):
        cls = type(element)
        self.assertIsInstance(element.index(), get_index_class(cls, T))
        self.assertIsInstance(element.model_instance(), ModelInstanceIndex)

    def _test_frame_api(self, T, frame):
        Frame = Frame_[T]

        self.assertIsInstance(frame, Frame)
        self._test_multibody_tree_element_mixin(T, frame)
        self.assertIsInstance(frame.name(), str)

    def _test_body_api(self, T, body):
        Body = Body_[T]

        self.assertIsInstance(body, Body)
        self._test_multibody_tree_element_mixin(T, body)
        self.assertIsInstance(body.name(), str)

    def _test_joint_api(self, T, joint):
        Joint = Joint_[T]
        Body = Body_[T]
        Frame = Frame_[T]

        self.assertIsInstance(joint, Joint)
        self._test_multibody_tree_element_mixin(T, joint)
        self.assertIsInstance(joint.name(), str)
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
            lower_limits=a_lower, upper_limits=a_upper)

    def _test_joint_actuator_api(self, T, joint_actuator):
        JointActuator = JointActuator_[T]
        Joint = Joint_[T]
        self.assertIsInstance(joint_actuator, JointActuator)
        self._test_multibody_tree_element_mixin(T, joint_actuator)
        self.assertIsInstance(joint_actuator.name(), str)
        self.assertIsInstance(joint_actuator.joint(), Joint)
        self.assertIsInstance(joint_actuator.effort_limit(), float)

    def check_old_spelling_exists(self, value):
        # Just to make it obvious when this is being tested.
        self.assertIsNot(value, None)

    @numpy_compare.check_all_types
    def test_inertia_api(self, T):
        RotationalInertia = RotationalInertia_[T]
        UnitInertia = UnitInertia_[T]
        SpatialInertia = SpatialInertia_[T]
        RotationMatrix = RotationMatrix_[T]
        SpatialAcceleration = SpatialAcceleration_[T]
        SpatialForce = SpatialForce_[T]
        SpatialVelocity = SpatialVelocity_[T]
        SpatialMomentum = SpatialMomentum_[T]
        # Test unit inertia construction.
        UnitInertia()
        unit_inertia = UnitInertia(Ixx=2.0, Iyy=2.3, Izz=2.4)
        self.assertIsInstance(unit_inertia, RotationalInertia)
        self.assertIsInstance(unit_inertia.CopyToFullMatrix3(), np.ndarray)
        # Test spatial inertia construction.
        SpatialInertia()
        spatial_inertia = SpatialInertia(
            mass=2.5, p_PScm_E=[0.1, -0.2, 0.3], G_SP_E=unit_inertia)
        numpy_compare.assert_float_equal(spatial_inertia.get_mass(), 2.5)
        self.assertIsInstance(spatial_inertia.get_com(), np.ndarray)
        self.assertIsInstance(spatial_inertia.CalcComMoment(), np.ndarray)
        self.assertIsInstance(spatial_inertia.get_unit_inertia(), UnitInertia)
        self.assertIsInstance(
            spatial_inertia.CalcRotationalInertia(), RotationalInertia)
        # N.B. `numpy_compare.assert_equal(IsPhysicallyValid(), True)` does not
        # work.
        if T != Expression:
            self.assertTrue(spatial_inertia.IsPhysicallyValid())
        self.assertIsInstance(spatial_inertia.CopyToFullMatrix6(), np.ndarray)
        self.assertIsInstance(
            spatial_inertia.ReExpress(RotationMatrix()), SpatialInertia)
        self.assertIsInstance(
            spatial_inertia.Shift([1, 2, 3]), SpatialInertia)
        spatial_inertia += spatial_inertia
        self.assertIsInstance(
            spatial_inertia * SpatialAcceleration(), SpatialForce)
        self.assertIsInstance(
            spatial_inertia * SpatialVelocity(), SpatialMomentum)
        spatial_inertia.SetNaN()
        # N.B. `numpy_compare.assert_equal(IsNaN(), True)` does not work.
        if T != Expression:
            self.assertTrue(spatial_inertia.IsNaN())

    @numpy_compare.check_all_types
    def test_friction_api(self, T):
        CoulombFriction = CoulombFriction_[T]
        CoulombFriction(static_friction=0.7, dynamic_friction=0.6)

    @numpy_compare.check_all_types
    def test_multibody_force_element(self, T):
        MultibodyPlant = MultibodyPlant_[T]
        LinearSpringDamper = LinearSpringDamper_[T]
        RevoluteSpring = RevoluteSpring_[T]
        DoorHinge = DoorHinge_[T]
        SpatialInertia = SpatialInertia_[float]

        plant = MultibodyPlant(0.0)
        spatial_inertia = SpatialInertia()
        body_a = plant.AddRigidBody(name="body_a",
                                    M_BBo_B=spatial_inertia)
        body_b = plant.AddRigidBody(name="body_b",
                                    M_BBo_B=spatial_inertia)
        linear_spring = plant.AddForceElement(LinearSpringDamper(
            bodyA=body_a, p_AP=[0., 0., 0.],
            bodyB=body_b, p_BQ=[0., 0., 0.],
            free_length=1., stiffness=2., damping=3.))
        if T == float:
            self.assertEqual(
                repr(linear_spring),
                "<LinearSpringDamper_[float] index=1 model_instance=1>")
        revolute_joint = plant.AddJoint(RevoluteJoint_[T](
                name="revolve_joint", frame_on_parent=body_a.body_frame(),
                frame_on_child=body_b.body_frame(), axis=[0, 0, 1],
                damping=0.))
        revolute_spring = plant.AddForceElement(RevoluteSpring(
            joint=revolute_joint, nominal_angle=0.1, stiffness=100.))
        door_hinge_config = DoorHingeConfig()
        door_hinge = plant.AddForceElement(DoorHinge(
            joint=revolute_joint, config=door_hinge_config))
        plant.Finalize()

        # Test LinearSpringDamper accessors
        self.assertEqual(linear_spring.bodyA(), body_a)
        self.assertEqual(linear_spring.bodyB(), body_b)
        np.testing.assert_array_equal(linear_spring.p_AP(), [0, 0, 0])
        np.testing.assert_array_equal(linear_spring.p_BQ(), [0, 0, 0])
        self.assertEqual(linear_spring.free_length(), 1.)
        self.assertEqual(linear_spring.stiffness(), 2.)
        self.assertEqual(linear_spring.damping(), 3.)

        # Test RevoluteSpring accessors
        self.assertEqual(revolute_spring.joint(), revolute_joint)
        self.assertEqual(revolute_spring.nominal_angle(), 0.1)
        self.assertEqual(revolute_spring.stiffness(), 100.)

        # Test DoorHinge accessors
        self.assertEqual(door_hinge.joint(), revolute_joint)
        door_hinge_config_test = door_hinge.config()
        # Only test two members since the rest would be the same
        self.assertEqual(door_hinge_config_test.spring_zero_angle_rad,
                         door_hinge_config.spring_zero_angle_rad)
        self.assertEqual(door_hinge_config_test.static_friction_torque,
                         door_hinge_config.static_friction_torque)

        # Test DoorHinge torque calculation. Set the angle to be the half of
        # the catch width so that there is only torsional spring torque which
        # is easy to compute by hand.
        if T == float:
            self.assertEqual(door_hinge.CalcHingeFrictionalTorque(
                angular_rate=0.0), 0.0)
            self.assertEqual(door_hinge.CalcHingeSpringTorque(
                angle=0.01), -2.265)
            self.assertEqual(door_hinge.CalcHingeTorque(
                angle=0.01, angular_rate=0.0), -2.265)

    @numpy_compare.check_all_types
    def test_multibody_gravity_default(self, T):
        MultibodyPlant = MultibodyPlant_[T]
        UniformGravityFieldElement = UniformGravityFieldElement_[T]
        plant = MultibodyPlant(0.0)
        with self.assertRaises(RuntimeError) as cm:
            plant.AddForceElement(UniformGravityFieldElement())

    @numpy_compare.check_all_types
    def test_multibody_tree_kinematics(self, T):
        RigidTransform = RigidTransform_[T]
        SpatialVelocity = SpatialVelocity_[T]
        plant_f = MultibodyPlant_[float](0.0)

        file_name = FindResourceOrThrow(
            "drake/bindings/pydrake/multibody/test/double_pendulum.sdf")
        # N.B. `Parser` only supports `MultibodyPlant_[float]`.
        Parser(plant_f).AddModelFromFile(file_name)
        plant_f.Finalize()
        plant = to_type(plant_f, T)
        context = plant.CreateDefaultContext()
        world_frame = plant.world_frame()
        base = plant.GetBodyByName("base")
        base_frame = plant.GetFrameByName("base")
        X_WL = plant.CalcRelativeTransform(
            context, frame_A=world_frame, frame_B=base_frame)
        self.assertIsInstance(X_WL, RigidTransform)

        p_AQi = plant.CalcPointsPositions(
            context=context, frame_B=base_frame,
            p_BQi=np.array([[0, 1, 2], [10, 11, 12]]).T,
            frame_A=world_frame).T
        self.assertTupleEqual(p_AQi.shape, (2, 3))

        p_com = plant.CalcCenterOfMassPosition(context=context)
        self.assertTupleEqual(p_com.shape, (3, ))

        nq = plant.num_positions()
        nv = plant.num_velocities()
        wrt_list = [
            (JacobianWrtVariable.kQDot, nq),
            (JacobianWrtVariable.kV, nv),
        ]
        for wrt, nw in wrt_list:
            Js_V_ABp_E = plant.CalcJacobianSpatialVelocity(
                context=context, with_respect_to=wrt, frame_B=base_frame,
                p_BP=np.zeros(3), frame_A=world_frame,
                frame_E=world_frame)
            self.assert_sane(Js_V_ABp_E)
            self.assertEqual(Js_V_ABp_E.shape, (6, nw))
            Js_w_AB_E = plant.CalcJacobianAngularVelocity(
                context=context, with_respect_to=wrt, frame_B=base_frame,
                frame_A=world_frame, frame_E=world_frame)
            self.assert_sane(Js_w_AB_E)
            self.assertEqual(Js_w_AB_E.shape, (3, nw))
            Js_v_AB_E = plant.CalcJacobianTranslationalVelocity(
                context=context, with_respect_to=wrt, frame_B=base_frame,
                p_BoBi_B=np.zeros(3), frame_A=world_frame, frame_E=world_frame)
            self.assert_sane(Js_v_AB_E)
            self.assertEqual(Js_v_AB_E.shape, (3, nw))

        AsBias_ABp_E = plant.CalcBiasSpatialAcceleration(
            context=context, with_respect_to=JacobianWrtVariable.kV,
            frame_B=base_frame, p_BoBp_B=np.zeros(3), frame_A=world_frame,
            frame_E=world_frame)
        self.assert_sane(AsBias_ABp_E.rotational(), nonzero=False)
        self.assert_sane(AsBias_ABp_E.translational(), nonzero=False)
        asBias_ABi_E = plant.CalcBiasTranslationalAcceleration(
            context=context, with_respect_to=JacobianWrtVariable.kV,
            frame_B=base_frame, p_BoBi_B=np.zeros(3), frame_A=world_frame,
            frame_E=world_frame)
        self.assert_sane(asBias_ABi_E, nonzero=False)
        self.assertEqual(asBias_ABi_E.shape, (3, 1))

            Js_v_AB_E = plant.CalcJacobianTranslationalVelocity(
                context=context, with_respect_to=wrt, frame_B=base_frame,
                p_BoBi_B=np.zeros((3, 3)), frame_A=world_frame,
                frame_E=world_frame)
            self.assert_sane(Js_v_AB_E)
            self.assertEqual(Js_v_AB_E.shape, (9, nw))

        # Compute body pose.
        X_WBase = plant.EvalBodyPoseInWorld(context, base)
        self.assertIsInstance(X_WBase, RigidTransform)

        # Set pose for the base.
        X_WB_desired = RigidTransform.Identity()
        X_WB = plant.CalcRelativeTransform(context, world_frame, base_frame)
        plant.SetFreeBodyPose(
            context=context, body=base, X_WB=X_WB_desired)
        numpy_compare.assert_float_equal(
            X_WB.GetAsMatrix4(),
            numpy_compare.to_float(X_WB_desired.GetAsMatrix4()))

        # Set a spatial velocity for the base.
        v_WB = SpatialVelocity(w=[1, 2, 3], v=[4, 5, 6])
        plant.SetFreeBodySpatialVelocity(
            context=context, body=base, V_WB=v_WB)
        v_base = plant.EvalBodySpatialVelocityInWorld(context, base)
        numpy_compare.assert_float_equal(
                v_base.rotational(), numpy_compare.to_float(v_WB.rotational()))
        numpy_compare.assert_float_equal(
                v_base.translational(),
                numpy_compare.to_float(v_WB.translational()))

        # Compute accelerations.
        vdot = np.zeros(nv)
        A_WB_array = plant.CalcSpatialAccelerationsFromVdot(
            context=context, known_vdot=vdot)
        self.assertEqual(len(A_WB_array), plant.num_bodies())

    @numpy_compare.check_all_types
    def test_multibody_state_access(self, T):
        MultibodyPlant = MultibodyPlant_[T]

        plant_f = MultibodyPlant_[float](0.0)
        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf")
        # N.B. `Parser` only supports `MultibodyPlant_[float]`.
        Parser(plant_f).AddModelFromFile(file_name)
        plant_f.Finalize()
        plant = to_type(plant_f, T)
        context = plant.CreateDefaultContext()

        nq = 2
        nv = 2
        self.assertEqual(plant.num_positions(), nq)
        self.assertEqual(plant.num_velocities(), nv)

        q0 = np.array([3.14, 2.])
        v0 = np.array([-0.5, 1.])
        x0 = np.concatenate([q0, v0])

        # The default state is all values set to zero.
        x = plant.GetPositionsAndVelocities(context)
        numpy_compare.assert_float_equal(x, np.zeros(4))

        # WARNING: The following oddities occur from the fact that
        # `ndarray[object]` cannot be referenced (#8116). Be careful when
        # writing scalar-generic code.
        if T == float:
            # Can reference matrices. Use `x_ref`.
            # Write into a mutable reference to the state vector.
            x_ref = plant.GetMutablePositionsAndVelocities(context)
            x_ref[:] = x0

            def set_zero():
                x_ref.fill(0)

        else:
            # Cannot reference matrices. Use setters.
            plant.SetPositionsAndVelocities(context, x0)

            def set_zero():
                plant.SetPositionsAndVelocities(
                    context, np.zeros(nq + nv))

        # Verify that positions and velocities were set correctly.
        numpy_compare.assert_float_equal(plant.GetPositions(context), q0)
        numpy_compare.assert_float_equal(plant.GetVelocities(context), v0)

        # Verify we did modify the state stored in context.
        x = plant.GetPositionsAndVelocities(context)
        numpy_compare.assert_float_equal(x, x0)

        # Now set positions and velocities independently and check them.
        zeros_2 = np.zeros([2, ])
        set_zero()
        plant.SetPositions(context, q0)
        numpy_compare.assert_float_equal(plant.GetPositions(context), q0)
        numpy_compare.assert_float_equal(plant.GetVelocities(context), zeros_2)
        set_zero()
        plant.SetVelocities(context, v0)
        numpy_compare.assert_float_allclose(
            plant.GetPositions(context), zeros_2)
        numpy_compare.assert_float_allclose(plant.GetVelocities(context), v0)

        # Now test SetPositionsAndVelocities().
        set_zero()
        plant.SetPositionsAndVelocities(context, x0)
        numpy_compare.assert_float_allclose(
            plant.GetPositionsAndVelocities(context), x0)

        # Test existence of context resetting methods.
        plant.SetDefaultState(context, state=context.get_mutable_state())

        # Test existence of default free body pose setting.
        body = plant.GetBodyByName("Link1")
        X_WB_default = RigidTransform_[float]()
        plant.SetDefaultFreeBodyPose(body=body, X_WB=X_WB_default)
        numpy_compare.assert_float_equal(
            plant.GetDefaultFreeBodyPose(body=body).GetAsMatrix4(),
            X_WB_default.GetAsMatrix4())

        # Test existence of limits.
        self.assertEqual(plant.GetPositionLowerLimits().shape, (nq,))
        self.assertEqual(plant.GetPositionUpperLimits().shape, (nq,))
        self.assertEqual(plant.GetVelocityLowerLimits().shape, (nv,))
        self.assertEqual(plant.GetVelocityUpperLimits().shape, (nv,))
        self.assertEqual(plant.GetAccelerationLowerLimits().shape, (nv,))
        self.assertEqual(plant.GetAccelerationUpperLimits().shape, (nv,))

    @numpy_compare.check_all_types
    def test_deprecated_vector_externally_applied_spatial_forced(self, T):
        # Deprecated custom template instantiation.
        with catch_drake_warnings(expected_count=1) as w:
            cls = VectorExternallyAppliedSpatialForced_[T]
        self.assertIn(
            "Value[List[ExternallyAppliedSpatialForce]]",
            str(w[0].message))
        message_expected = str(w[0].message)

        if T == float:
            # Check alias.
            self.assertIs(cls, VectorExternallyAppliedSpatialForced)

        # Deprecated Value[] instantiation which aliases to correct
        # instantiation.
        with catch_drake_warnings(expected_count=1) as w:
            value_cls = Value[cls]
        self.assertEqual(str(w[0].message), message_expected)
        self.assertIs(
            value_cls, Value[List[ExternallyAppliedSpatialForce_[T]]])

        # Deprecated constructor which simply creates a list (not a derived
        # class).
        with catch_drake_warnings(expected_count=1) as w:
            x = cls()
        self.assertEqual(type(x), list)
        self.assertEqual(str(w[0].message), message_expected)

    @numpy_compare.check_all_types
    def test_port_access(self, T):
        # N.B. We actually test the values because some of the value bindings
        # are somewhat special snowflakes.
        MultibodyPlant = MultibodyPlant_[T]
        InputPort = InputPort_[T]
        OutputPort = OutputPort_[T]
        plant_f = MultibodyPlant_[float](0.0)
        # Create a MultibodyPlant with a kuka arm and a schunk gripper.
        # the arm is welded to the world, the gripper is welded to the
        # arm's end effector.
        wsg50_sdf_path = FindResourceOrThrow(
            "drake/manipulation/models/"
            "wsg_50_description/sdf/schunk_wsg_50.sdf")
        iiwa_sdf_path = FindResourceOrThrow(
            "drake/manipulation/models/"
            "iiwa_description/sdf/iiwa14_no_collision.sdf")

        # N.B. `Parser` only supports `MultibodyPlant_[float]`.
        plant_f = MultibodyPlant_[float](time_step=2e-3)
        parser = Parser(plant_f)
        iiwa_model = parser.AddModelFromFile(
            file_name=iiwa_sdf_path, model_name='robot')
        gripper_model = parser.AddModelFromFile(
            file_name=wsg50_sdf_path, model_name='gripper')
        plant_f.Finalize()
        plant = to_type(plant_f, T)
        models = [iiwa_model, gripper_model]

        # Fix inputs.
        context = plant.CreateDefaultContext()
        for model in models:
            nu = plant.num_actuated_dofs(model)
            plant.get_actuation_input_port(model_instance=model).FixValue(
                context, np.zeros(nu))

        # Evaluate outputs.
        for model in models:
            self.assertIsInstance(
                plant.get_state_output_port(
                    model_instance=model).Eval(context),
                np.ndarray)
            if T == Expression:
                continue
            self.assertIsInstance(
                plant.get_generalized_acceleration_output_port(
                    model_instance=model).Eval(context),
                np.ndarray)
            self.assertIsInstance(
                plant.get_generalized_contact_forces_output_port(
                    model_instance=model).Eval(context),
                np.ndarray)

        def extract_list_value(port):
            self.assertIsInstance(port, OutputPort)
            value = port.Eval(context)
            self.assertIsInstance(value, list)
            self.assertGreater(len(value), 0)
            return value[0]

        self.assertIsInstance(
            extract_list_value(plant.get_body_poses_output_port()),
            RigidTransform_[T])
        self.assertIsInstance(
            extract_list_value(
                plant.get_body_spatial_velocities_output_port()),
            SpatialVelocity_[T])
        if T != Expression:
            self.assertIsInstance(
                extract_list_value(
                    plant.get_body_spatial_accelerations_output_port()),
                SpatialAcceleration_[T])
        # TODO(eric.cousineau): Merge `check_applied_force_input_ports` into
        # this test.

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
                    self.DoCalcAbstractOutput)
                self.DeclareVectorOutputPort(
                    "generalized_forces",
                    BasicVector_[T](self.nv),
                    self.DoCalcVectorOutput)

            def _construct_copy(self, other, converter=None):
                Impl._construct(
                    self, other.nv, other.target_body_index,
                    converter=converter)

            def DoCalcAbstractOutput(self, context, spatial_forces_vector):
                test_force = ExternallyAppliedSpatialForce_[T]()
                test_force.body_index = self.target_body_index
                test_force.p_BoBq_B = np.zeros(3)
                test_force.F_Bq_W = SpatialForce_[T](
                    tau=[0., 0., 0.], f=[0., 0., 1.])
                spatial_forces_vector.set_value([test_force])

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
            "drake/multibody/benchmarks/free_body/uniform_solid_cylinder.urdf")
        Parser(plant_f).AddModelFromFile(file_name)
        plant_f.Finalize()

        # These connections will fail if the port output types
        # are not legible.
        test_system_f = builder_f.AddSystem(
            self.AppliedForceTestSystem_[float](
                plant_f.num_velocities(),
                plant_f.GetBodyByName("uniformSolidCylinder").index()))
        builder_f.Connect(
            test_system_f.get_output_port(0),
            plant_f.get_applied_spatial_force_input_port())
        builder_f.Connect(
            test_system_f.get_output_port(1),
            plant_f.get_applied_generalized_force_input_port())
        diagram_f = builder_f.Build()
        diagram = to_type(diagram_f, T)

        plant = diagram.GetSubsystemByName(plant_f.get_name())
        # Test that we can get those ports.
        self.assertIsInstance(
            plant.get_applied_generalized_force_input_port(), InputPort)
        self.assertIsInstance(
            plant.get_applied_spatial_force_input_port(), InputPort)

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
        RigidTransform = RigidTransform_[T]
        RollPitchYaw = RollPitchYaw_[T]

        wsg50_sdf_path = FindResourceOrThrow(
            "drake/manipulation/models/"
            "wsg_50_description/sdf/schunk_wsg_50.sdf")
        iiwa_sdf_path = FindResourceOrThrow(
            "drake/manipulation/models/"
            "iiwa_description/sdf/iiwa14_no_collision.sdf")

        # N.B. `Parser` only supports `MultibodyPlant_[float]`.
        plant_f = MultibodyPlant_[float](0.0)
        parser = Parser(plant_f)

        iiwa_model = parser.AddModelFromFile(
            file_name=iiwa_sdf_path, model_name='robot')
        gripper_model = parser.AddModelFromFile(
            file_name=wsg50_sdf_path, model_name='gripper')

        # Weld the base of arm and gripper to reduce the number of states.
        X_EeGripper = RigidTransform_[float](
            RollPitchYaw_[float](np.pi / 2, 0, np.pi / 2), [0, 0, 0.081])
        plant_f.WeldFrames(
                A=plant_f.world_frame(),
                B=plant_f.GetFrameByName("iiwa_link_0", iiwa_model))
        plant_f.WeldFrames(
            A=plant_f.GetFrameByName("iiwa_link_7", iiwa_model),
            B=plant_f.GetFrameByName("body", gripper_model),
            X_AB=X_EeGripper)
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

        q_iiwa_desired[2] = np.pi/3
        q_gripper_desired[0] = 0.1
        v_iiwa_desired[1] = 5.0
        q_gripper_desired[0] = -0.3

        x_iiwa_desired = np.zeros(nq_iiwa + nv_iiwa)
        x_iiwa_desired[0:nq_iiwa] = q_iiwa_desired
        x_iiwa_desired[nq_iiwa:nq_iiwa+nv_iiwa] = v_iiwa_desired

        x_gripper_desired = np.zeros(nq_gripper + nv_gripper)
        x_gripper_desired[0:nq_gripper] = q_gripper_desired
        x_gripper_desired[nq_gripper:nq_gripper+nv_gripper] = v_gripper_desired

        x_desired = np.zeros(nq + nv)
        x_desired[0:7] = q_iiwa_desired
        x_desired[7:9] = q_gripper_desired
        x_desired[nq:nq+7] = v_iiwa_desired
        x_desired[nq+7:nq+nv] = v_gripper_desired

        # Check SetPositionsAndVelocities() for each model instance.
        # Do the iiwa model first.
        plant.SetPositionsAndVelocities(context, np.zeros(nq + nv))
        numpy_compare.assert_float_equal(
                plant.GetPositionsAndVelocities(context), np.zeros(nq + nv))
        plant.SetPositionsAndVelocities(context, iiwa_model, x_iiwa_desired)
        numpy_compare.assert_float_equal(
            plant.GetPositionsAndVelocities(context, iiwa_model),
            x_iiwa_desired)
        numpy_compare.assert_float_equal(plant.GetPositionsAndVelocities(
            context, gripper_model), np.zeros(nq_gripper + nv_gripper))
        # Do the gripper model.
        plant.SetPositionsAndVelocities(context, np.zeros(nq + nv))
        numpy_compare.assert_float_equal(
                plant.GetPositionsAndVelocities(context), np.zeros(nq + nv))
        plant.SetPositionsAndVelocities(
            context, gripper_model, x_gripper_desired)
        numpy_compare.assert_float_equal(
            plant.GetPositionsAndVelocities(context, gripper_model),
            x_gripper_desired)
        numpy_compare.assert_float_equal(
                plant.GetPositionsAndVelocities(context, iiwa_model),
                np.zeros(nq_iiwa + nv_iiwa))

        # Check SetPositions() for each model instance.
        # Do the iiwa model first.
        plant.SetPositionsAndVelocities(context, np.zeros(nq + nv))
        numpy_compare.assert_float_equal(
                plant.GetPositionsAndVelocities(context), np.zeros(nq + nv))
        plant.SetPositions(context, iiwa_model, q_iiwa_desired)
        numpy_compare.assert_float_equal(
            plant.GetPositions(context, iiwa_model), q_iiwa_desired)
        numpy_compare.assert_float_equal(plant.GetVelocities(
            context, iiwa_model), np.zeros(nv_iiwa))
        numpy_compare.assert_float_equal(plant.GetPositionsAndVelocities(
            context, gripper_model), np.zeros(nq_gripper + nv_gripper))
        # Do the gripper model.
        plant.SetPositionsAndVelocities(context, np.zeros(nq + nv))
        numpy_compare.assert_float_equal(
                plant.GetPositionsAndVelocities(context), np.zeros(nq + nv))
        plant.SetPositions(context, gripper_model, q_gripper_desired)
        numpy_compare.assert_float_equal(
            plant.GetPositions(context, gripper_model),
            q_gripper_desired)
        numpy_compare.assert_float_equal(plant.GetVelocities(
            context, gripper_model), np.zeros(nq_gripper))
        numpy_compare.assert_float_equal(plant.GetPositionsAndVelocities(
            context, iiwa_model), np.zeros(nq_iiwa + nv_iiwa))

        # Check SetVelocities() for each model instance.
        # Do the iiwa model first.
        plant.SetPositionsAndVelocities(context, np.zeros(nq + nv))
        numpy_compare.assert_float_equal(
                plant.GetPositionsAndVelocities(context), np.zeros(nq + nv))
        plant.SetVelocities(context, iiwa_model, v_iiwa_desired)
        numpy_compare.assert_float_equal(
            plant.GetVelocities(context, iiwa_model), v_iiwa_desired)
        numpy_compare.assert_float_equal(plant.GetPositions(
            context, iiwa_model), np.zeros(nq_iiwa))
        numpy_compare.assert_float_equal(plant.GetPositionsAndVelocities(
            context, gripper_model), np.zeros(nq_gripper + nv_gripper))
        # Do the gripper model.
        plant.SetPositionsAndVelocities(context, np.zeros(nq + nv))
        numpy_compare.assert_float_equal(
                plant.GetPositionsAndVelocities(context), np.zeros(nq + nv))
        plant.SetVelocities(context, gripper_model, v_gripper_desired)
        numpy_compare.assert_float_equal(
            plant.GetVelocities(context, gripper_model),
            v_gripper_desired)
        numpy_compare.assert_float_equal(plant.GetPositions(
            context, gripper_model), np.zeros(nv_gripper))
        numpy_compare.assert_float_equal(plant.GetPositionsAndVelocities(
            context, iiwa_model), np.zeros(nq_iiwa + nv_iiwa))

    @numpy_compare.check_all_types
    def test_model_instance_state_access_by_array(self, T):
        # N.B. Please check warning above in `check_multibody_state_access`.
        MultibodyPlant = MultibodyPlant_[T]
        # Create a MultibodyPlant with a kuka arm and a schunk gripper.
        # the arm is welded to the world, the gripper is welded to the
        # arm's end effector.
        wsg50_sdf_path = FindResourceOrThrow(
            "drake/manipulation/models/"
            "wsg_50_description/sdf/schunk_wsg_50.sdf")
        iiwa_sdf_path = FindResourceOrThrow(
            "drake/manipulation/models/"
            "iiwa_description/sdf/iiwa14_no_collision.sdf")

        timestep = 0.0002
        # N.B. `Parser` only supports `MultibodyPlant_[float]`.
        plant_f = MultibodyPlant_[float](timestep)
        parser = Parser(plant_f)

        iiwa_model = parser.AddModelFromFile(
            file_name=iiwa_sdf_path, model_name='robot')
        gripper_model = parser.AddModelFromFile(
            file_name=wsg50_sdf_path, model_name='gripper')

        # Weld the base of arm and gripper to reduce the number of states.
        X_EeGripper = RigidTransform_[float](
            RollPitchYaw_[float](np.pi / 2, 0, np.pi / 2), [0, 0, 0.081])
        plant_f.WeldFrames(
            A=plant_f.world_frame(),
            B=plant_f.GetFrameByName("iiwa_link_0", iiwa_model))
        plant_f.WeldFrames(
            A=plant_f.GetFrameByName("iiwa_link_7", iiwa_model),
            B=plant_f.GetFrameByName("body", gripper_model),
            X_AB=X_EeGripper)
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
        v_gripper_desired = [-1., -2.]

        x_desired = np.zeros(nq + nv)
        x_desired[0:7] = q_iiwa_desired
        x_desired[7:9] = q_gripper_desired
        x_desired[nq:nq+7] = v_iiwa_desired
        x_desired[nq+7:nq+nv] = v_gripper_desired

        if T == float:
            x = plant.GetMutablePositionsAndVelocities(context=context)
            x[:] = x_desired
        else:
            plant.SetPositionsAndVelocities(context, x_desired)

        q = plant.GetPositions(context=context)
        v = plant.GetVelocities(context=context)

        # Get state from context.
        x = plant.GetPositionsAndVelocities(context=context)
        if T == float:
            x_tmp = plant.GetMutablePositionsAndVelocities(context=context)
            self.assertTrue(np.allclose(x_desired, x_tmp))

        # Get positions and velocities of specific model instances
        # from the position/velocity vector of the plant.
        q_iiwa = plant.GetPositions(context=context, model_instance=iiwa_model)
        q_iiwa_array = plant.GetPositionsFromArray(
            model_instance=iiwa_model, q=q)
        numpy_compare.assert_equal(q_iiwa, q_iiwa_array)
        q_gripper = plant.GetPositions(
            context=context, model_instance=gripper_model)
        v_iiwa = plant.GetVelocities(
            context=context, model_instance=iiwa_model)
        v_iiwa_array = plant.GetVelocitiesFromArray(
            model_instance=iiwa_model, v=v)
        numpy_compare.assert_equal(v_iiwa, v_iiwa_array)
        v_gripper = plant.GetVelocities(
            context=context, model_instance=gripper_model)

        # Assert that the `GetPositions` and `GetVelocities` return
        # the desired values set earlier.
        numpy_compare.assert_float_equal(q_iiwa, q_iiwa_desired)
        numpy_compare.assert_float_equal(v_iiwa, v_iiwa_desired)
        numpy_compare.assert_float_equal(q_gripper, q_gripper_desired)
        numpy_compare.assert_float_equal(v_gripper, v_gripper_desired)

        if T == float:
            # Verify that SetPositionsInArray() and SetVelocitiesInArray()
            # works.
            plant.SetPositionsInArray(
                model_instance=iiwa_model, q_instance=np.zeros(nq_iiwa), q=q)
            numpy_compare.assert_float_equal(
                plant.GetPositionsFromArray(model_instance=iiwa_model, q=q),
                np.zeros(nq_iiwa))
            plant.SetVelocitiesInArray(
                model_instance=iiwa_model, v_instance=np.zeros(nv_iiwa), v=v)
            numpy_compare.assert_float_equal(
                plant.GetVelocitiesFromArray(model_instance=iiwa_model, v=v),
                np.zeros(nv_iiwa))

            # Check actuation.
            nu = plant.num_actuated_dofs()
            u = np.zeros(nu)
            u_iiwa = np.arange(nv_iiwa)
            plant.SetActuationInArray(
                model_instance=iiwa_model, u_instance=u_iiwa, u=u)
            numpy_compare.assert_float_equal(u_iiwa, u[:7])

    @numpy_compare.check_all_types
    def test_map_qdot_to_v_and_back(self, T):
        MultibodyPlant = MultibodyPlant_[T]
        RigidTransform = RigidTransform_[T]
        RollPitchYaw = RollPitchYaw_[T]
        # N.B. `Parser` only supports `MultibodyPlant_[float]`.
        plant_f = MultibodyPlant_[float](0.0)
        iiwa_sdf_path = FindResourceOrThrow(
            "drake/manipulation/models/"
            "iiwa_description/sdf/iiwa14_no_collision.sdf")
        # Use floating base to effectively add a quatnerion in the generalized
        # quaternion.
        iiwa_model = Parser(plant=plant_f).AddModelFromFile(
            file_name=iiwa_sdf_path, model_name='robot')
        plant_f.Finalize()
        plant = to_type(plant_f, T)
        context = plant.CreateDefaultContext()
        # Try mapping velocity to qdot and back.
        nq = plant.num_positions()
        nv = plant.num_velocities()
        q_init = np.linspace(start=1.0, stop=nq, num=nq)
        plant.SetPositions(context, q_init)
        # Overwrite the (invalid) base coordinates, wherever in `q` they are.
        plant.SetFreeBodyPose(
            context, plant.GetBodyByName("iiwa_link_0"),
            RigidTransform(RollPitchYaw([0.1, 0.2, 0.3]),
                           p=[0.4, 0.5, 0.6]))
        v_expected = np.linspace(start=-1.0, stop=-nv, num=nv)
        qdot = plant.MapVelocityToQDot(context, v_expected)
        v_remap = plant.MapQDotToVelocity(context, qdot)
        numpy_compare.assert_float_allclose(v_remap, v_expected)

    @numpy_compare.check_all_types
    def test_multibody_add_joint(self, T):
        """
        Tests joint constructors, `AddJoint`, `AddJointActuator` and
        `HasJointActuatorNamed`.
        """
        damping = 2.
        x_axis = [1., 0., 0.]
        X_PC = RigidTransform_[float](p=[1., 2., 3.])

        def make_ball_rpy_joint(plant, P, C):
            return BallRpyJoint_[T](
                name="ball_rpy",
                frame_on_parent=P,
                frame_on_child=C,
                damping=damping,
            )

        def make_prismatic_joint(plant, P, C):
            return PrismaticJoint_[T](
                name="prismatic",
                frame_on_parent=P,
                frame_on_child=C,
                axis=x_axis,
                damping=damping,
            )

        def make_revolute_joint(plant, P, C):
            return RevoluteJoint_[T](
                name="revolute",
                frame_on_parent=P,
                frame_on_child=C,
                axis=x_axis,
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
            # TODO(eric.cousineau): Update WeldJoint arg names to be consistent
            # with other joints.
            return WeldJoint_[T](
                name="weld",
                parent_frame_P=P,
                child_frame_C=C,
                X_PC=X_PC,
            )

        make_joint_list = [
            make_ball_rpy_joint,
            make_prismatic_joint,
            make_revolute_joint,
            make_universal_joint,
            make_weld_joint,
        ]

        for make_joint in make_joint_list:
            plant = MultibodyPlant_[T](0.0)
            child = plant.AddRigidBody("Child", SpatialInertia_[float]())
            joint = make_joint(
                plant=plant, P=plant.world_frame(), C=child.body_frame())
            joint_out = plant.AddJoint(joint)
            self.assertIs(joint, joint_out)
            if joint.num_velocities() == 1:
                self.assertFalse(plant.HasJointActuatorNamed("tau"))
                self.assertFalse(plant.HasJointActuatorNamed(
                    "tau", default_model_instance()))
                actuator = plant.AddJointActuator(
                    name="tau", joint=joint, effort_limit=1.0)
                self.assertTrue(plant.HasJointActuatorNamed("tau"))
                self.assertTrue(plant.HasJointActuatorNamed(
                    "tau", default_model_instance()))
                self.assertIsInstance(actuator, JointActuator_[T])
            plant.Finalize()
            self._test_joint_api(T, joint)

            context = plant.CreateDefaultContext()
            if joint.name() == "ball_rpy":
                set_point = np.array([1., 2., 3.])
                joint.set_angles(context=context, angles=set_point)
                self.assertEqual(len(joint.get_angles(context=context)), 3)
                joint.set_angular_velocity(context=context, w_FM=set_point)
                self.assertEqual(
                    len(joint.get_angular_velocity(context=context)), 3)
            elif joint.name() == "prismatic":
                self.assertEqual(joint.damping(), damping)
                numpy_compare.assert_equal(joint.translation_axis(), x_axis)
                set_point = 1.
                joint.set_translation(context=context, translation=set_point)
                self.assertIsInstance(
                    joint.get_translation(context=context), T)
                joint.set_translation_rate(context=context,
                                           translation_dot=set_point)
                self.assertIsInstance(
                    joint.get_translation_rate(context=context), T)
            elif joint.name() == "revolute":
                numpy_compare.assert_equal(joint.revolute_axis(), x_axis)
                self.assertEqual(joint.damping(), damping)
                set_point = 1.
                joint.set_angle(context=context, angle=set_point)
                self.assertIsInstance(joint.get_angle(context=context), T)
            elif joint.name() == "universal":
                self.assertEqual(joint.damping(), damping)
                set_point = np.array([1., 2.])
                joint.set_angles(context=context, angles=set_point)
                self.assertEqual(len(joint.get_angles(context=context)), 2)
                joint.set_angular_rates(context=context, theta_dot=set_point)
                self.assertEqual(
                    len(joint.get_angular_rates(context=context)), 2)
            elif joint.name() == "weld":
                numpy_compare.assert_float_equal(
                    joint.X_PC().GetAsMatrix4(),
                    X_PC.GetAsMatrix4())
            else:
                raise TypeError(
                    "Joint type " + joint.name() + " not recognized.")

    @numpy_compare.check_all_types
    def test_multibody_add_frame(self, T):
        MultibodyPlant = MultibodyPlant_[T]
        FixedOffsetFrame = FixedOffsetFrame_[T]

        Frame = Frame_[T]

        plant = MultibodyPlant(0.0)
        frame = plant.AddFrame(frame=FixedOffsetFrame(
            name="frame", P=plant.world_frame(),
            X_PF=RigidTransform_[float].Identity(), model_instance=None))
        self.assertIsInstance(frame, Frame)
        numpy_compare.assert_float_equal(
            frame.GetFixedPoseInBodyFrame().GetAsMatrix4(),
            np.eye(4))

    @numpy_compare.check_all_types
    def test_multibody_dynamics(self, T):
        MultibodyPlant = MultibodyPlant_[T]
        MultibodyForces = MultibodyForces_[T]
        SpatialForce = SpatialForce_[T]

        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf")
        # N.B. `Parser` only supports `MultibodyPlant_[float]`.
        plant_f = MultibodyPlant_[float](0.0)
        Parser(plant_f).AddModelFromFile(file_name)
        # Getting ready for when we set foot on Mars :-).
        gravity_vector = np.array([0.0, 0.0, -3.71])
        plant_f.mutable_gravity_field().set_gravity_vector(gravity_vector)
        plant_f.Finalize()
        plant = to_type(plant_f, T)
        context = plant.CreateDefaultContext()

        numpy_compare.assert_float_equal(
            plant.gravity_field().gravity_vector(), gravity_vector)

        # Set an arbitrary configuration away from the model's fixed point.
        plant.SetPositions(context, [0.1, 0.2])

        M = plant.CalcMassMatrixViaInverseDynamics(context)
        Cv = plant.CalcBiasTerm(context)

        self.assertTrue(M.shape == (2, 2))
        self.assert_sane(M)
        self.assertTrue(Cv.shape == (2, ))
        self.assert_sane(Cv, nonzero=False)
        nv = plant.num_velocities()
        vd_d = np.zeros(nv)
        tau = plant.CalcInverseDynamics(
            context, vd_d, MultibodyForces(plant))
        self.assertEqual(tau.shape, (2,))
        self.assert_sane(tau, nonzero=False)
        # - Existence checks.
        # Gravity leads to non-zero potential energy.
        potential_energy = plant.CalcPotentialEnergy(context)
        numpy_compare.assert_float_not_equal(potential_energy, 0.)
        plant.CalcConservativePower(context)
        tau_g = plant.CalcGravityGeneralizedForces(context)
        self.assertEqual(tau_g.shape, (nv,))
        self.assert_sane(tau_g, nonzero=True)

        # Gravity is the only force element
        self.assertEqual(plant.num_force_elements(), 1)

        B = plant.MakeActuationMatrix()
        numpy_compare.assert_float_equal(B, np.array([[0.], [1.]]))

        forces = MultibodyForces(plant=plant)
        plant.CalcForceElementsContribution(context=context, forces=forces)

        # Test generalized forces.
        # N.B. Cannot use `ndarray[object]` to reference existing C arrays
        # (#8116).
        if T == float:
            forces.mutable_generalized_forces()[:] = 1
            np.testing.assert_equal(forces.generalized_forces(), 1)
            forces.SetZero()
            np.testing.assert_equal(forces.generalized_forces(), 0)

        # Test body force accessors and mutators.
        link2 = plant.GetBodyByName("Link2")
        self.assertIsInstance(
            link2.GetForceInWorld(context, forces), SpatialForce)
        self.assertFalse(link2.is_floating())
        forces.SetZero()
        F_expected = np.array([1., 2., 3., 4., 5., 6.])
        link2.AddInForceInWorld(
            context, F_Bo_W=SpatialForce(F=F_expected), forces=forces)
        coeff = numpy_compare.to_float(
                link2.GetForceInWorld(context, forces).get_coeffs())
        numpy_compare.assert_float_equal(coeff, F_expected)
        link2.AddInForce(
            context, p_BP_E=[0, 0, 0], F_Bp_E=SpatialForce(F=F_expected),
            frame_E=plant.world_frame(), forces=forces)
        # Also check accumulation.
        np.testing.assert_equal(numpy_compare.to_float(
            link2.GetForceInWorld(context, forces).get_coeffs()),
            2 * F_expected)

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
            bodyA_index=id_A, bodyB_index=id_B,
            f_Bc_W=np.array([0, 0, 1]), p_WC=np.array([0, 0, 0]),
            separation_speed=0, slip_speed=0, point_pair=point_pair)
        self.assertTrue(
            isinstance(contact_info.bodyA_index(), BodyIndex))
        self.assertTrue(
            isinstance(contact_info.bodyB_index(), BodyIndex))
        self.assertTrue(contact_info.contact_force().shape == (3,))
        self.assertTrue(contact_info.contact_point().shape == (3,))
        self.assertTrue(isinstance(contact_info.slip_speed(), T))
        self.assertIsInstance(
            contact_info.point_pair(), PenetrationAsPointPair)

        # ContactResults
        contact_results = ContactResults()
        self.assertTrue(contact_results.num_point_pair_contacts() == 0)

    def test_contact_results_to_lcm(self):
        # ContactResultsToLcmSystem
        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf")
        plant = MultibodyPlant_[float](0.0)
        Parser(plant).AddModelFromFile(file_name)
        plant.Finalize()
        plant.set_penetration_allowance(penetration_allowance=0.0001)
        plant.set_stiction_tolerance(v_stiction=0.001)
        self.assertIsInstance(
            plant.get_contact_penalty_method_time_scale(), float)
        contact_results_to_lcm = ContactResultsToLcmSystem(plant)
        context = contact_results_to_lcm.CreateDefaultContext()
        contact_results_to_lcm.get_input_port(0).FixValue(
            context, ContactResults_[float]())
        output = contact_results_to_lcm.AllocateOutput()
        contact_results_to_lcm.CalcOutput(context, output)
        result = output.get_data(0)
        self.assertIsInstance(result, AbstractValue)

    def test_connect_contact_results(self):
        DiagramBuilder = DiagramBuilder_[float]
        MultibodyPlant = MultibodyPlant_[float]

        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf")
        builder = DiagramBuilder()
        plant = builder.AddSystem(MultibodyPlant(0.001))
        Parser(plant).AddModelFromFile(file_name)
        plant.Finalize()

        publisher = ConnectContactResultsToDrakeVisualizer(builder, plant)
        self.assertIsInstance(publisher, LcmPublisherSystem)

    def test_collision_filter(self):
        builder_f = DiagramBuilder_[float]()
        # N.B. `Parser` only supports `MultibodyPlant_[float]`.
        plant_f, scene_graph_f = AddMultibodyPlantSceneGraph(builder_f, 0.0)
        parser = Parser(plant=plant_f, scene_graph=scene_graph_f)

        parser.AddModelFromFile(
            FindResourceOrThrow(
                "drake/bindings/pydrake/multibody/test/two_bodies.sdf"))
        plant_f.Finalize()

        # Build a set with geometries from bodies 1 and 2.
        body_a = plant_f.GetBodyByName("body1")
        body_b = plant_f.GetBodyByName("body2")
        geometries = plant_f.CollectRegisteredGeometries([body_a, body_b])
        self.assertIsInstance(geometries, GeometrySet)

    @numpy_compare.check_nonsymbolic_types
    def test_scene_graph_queries(self, T):
        # SceneGraph does not support `Expression` type.
        PenetrationAsPointPair = PenetrationAsPointPair_[T]

        builder_f = DiagramBuilder_[float]()
        # N.B. `Parser` only supports `MultibodyPlant_[float]`.
        plant_f, scene_graph_f = AddMultibodyPlantSceneGraph(builder_f, 0.0)
        parser = Parser(plant=plant_f, scene_graph=scene_graph_f)
        parser.AddModelFromFile(
            FindResourceOrThrow(
                "drake/bindings/pydrake/multibody/test/two_bodies.sdf"))
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
        query_object = scene_graph.get_query_output_port().Eval(sg_context)
        # Implicitly require that this should be size 1.
        point_pair, = query_object.ComputePointPairPenetration()
        self.assertIsInstance(point_pair, PenetrationAsPointPair_[float])
        signed_distance_pair, = query_object.\
            ComputeSignedDistancePairwiseClosestPoints()
        self.assertIsInstance(signed_distance_pair, SignedDistancePair_[T])
        signed_distance_to_point = query_object.\
            ComputeSignedDistanceToPoint(p_WQ=np.ones(3))
        self.assertEqual(len(signed_distance_to_point), 2)
        self.assertIsInstance(signed_distance_to_point[0],
                              SignedDistanceToPoint_[T])
        self.assertIsInstance(signed_distance_to_point[1],
                              SignedDistanceToPoint_[T])
        # Test SceneGraphInspector
        inspector = query_object.inspector()

        self.assertEqual(inspector.num_geometries(), 2)
        self.assertEqual(inspector.num_geometries(),
                         len(inspector.GetAllGeometryIds()))
        for geometry_id in inspector.GetAllGeometryIds():
            frame_id = inspector.GetFrameId(geometry_id)
            self.assertEqual(
                inspector.GetGeometryIdByName(
                    frame_id, Role.kProximity,
                    inspector.GetName(geometry_id)), geometry_id)
            self.assertIsInstance(inspector.GetShape(geometry_id), Sphere)
            self.assertIsInstance(inspector.GetPoseInFrame(geometry_id),
                                  RigidTransform_[float])

        def get_body_from_frame_id(frame_id):
            # Get body from frame id, and check inverse method.
            body = plant.GetBodyFromFrameId(frame_id)
            self.assertEqual(
                plant.GetBodyFrameIdIfExists(body.index()), frame_id)
            self.assertEqual(plant.GetBodyFrameIdOrThrow(body.index()),
                             frame_id)
            return body

        bodies = {get_body_from_frame_id(inspector.GetFrameId(id_))
                  for id_ in [point_pair.id_A, point_pair.id_B]}
        self.assertSetEqual(
            bodies,
            {plant.GetBodyByName("body1"), plant.GetBodyByName("body2")})

        id_, = plant.GetCollisionGeometriesForBody(
            body=plant.GetBodyByName("body1"))
        self.assertIsInstance(id_, GeometryId)

    def test_propeller(self):
        plant = MultibodyPlant_[float](time_step=0.0)
        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf")
        Parser(plant).AddModelFromFile(file_name)
        plant.Finalize()

        info = PropellerInfo(body_index=BodyIndex(1),
                             X_BP=RigidTransform_[float](),
                             thrust_ratio=1.0,
                             moment_ratio=0.1)
        self.assertEqual(info.thrust_ratio, 1.0)
        self.assertEqual(info.moment_ratio, 0.1)

        prop = Propeller_[float](body_index=BodyIndex(1),
                                 X_BP=RigidTransform_[float](),
                                 thrust_ratio=1.0,
                                 moment_ratio=0.1)
        self.assertEqual(prop.num_propellers(), 1)
        self.assertIsInstance(prop.get_command_input_port(), InputPort_[float])
        self.assertIsInstance(prop.get_body_poses_input_port(),
                              InputPort_[float])
        self.assertIsInstance(prop.get_spatial_forces_output_port(),
                              OutputPort_[float])

        prop2 = Propeller_[float]([info, info])
        self.assertEqual(prop2.num_propellers(), 2)
