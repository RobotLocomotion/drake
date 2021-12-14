# -*- coding: utf-8 -*-

import collections
import copy
import itertools
import unittest

import numpy as np

from pydrake.autodiffutils import AutoDiffXd
from pydrake.symbolic import Expression, Variable
from pydrake.common.eigen_geometry import Isometry3
from pydrake.lcm import DrakeLcm
from pydrake.math import RigidTransform
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
    LinearBushingRollPitchYaw_,
    LinearSpringDamper_,
    ModelInstanceIndex,
    MultibodyForces_,
    PlanarJoint_,
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
    ContactModel,
    ContactResults_,
    ContactResultsToLcmSystem,
    ContactResultsToMeshcatParams,
    ContactResultsToMeshcat_,
    CoulombFriction_,
    ExternallyAppliedSpatialForce_,
    MultibodyPlant_,
    PointPairContactInfo_,
    PropellerInfo,
    Propeller_,
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
from pydrake.common.test_utilities.pickle_compare import assert_pickle
from pydrake.common.value import AbstractValue, Value
from pydrake.geometry import (
    Box,
    GeometryId,
    GeometrySet,
    Meshcat,
    Rgba,
    Role,
    PenetrationAsPointPair_,
    ProximityProperties,
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
def to_type(system, T):
    assert isinstance(system, System_[float])
    if T == float:
        return system
    else:
        return system.ToScalarType[T]()


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
        body_default_rotational_inertial = body.default_rotational_inertia()
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

        # Confirm that passing the diagram context into the plant (a common
        # user error) throws a RuntimeError instead of crashing.
        plant.Finalize()
        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        with self.assertRaises(RuntimeError):
            plant.EvalBodyPoseInWorld(context, body)

    def test_deprecated_register_visual_geometry(self):
        builder = DiagramBuilder_[float]()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
        body = plant.AddRigidBody(name="B", M_BBo_B=SpatialInertia_[float]())
        with catch_drake_warnings(expected_count=1):
            plant.RegisterVisualGeometry(
                body=body, X_BG=Isometry3(), shape=Box(1.0, 1.0, 1.0),
                name="G", diffuse_color=[1., 0., 0., 0.])

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
        self.assertIs(link1.GetParentPlant(), plant)
        self.assertTrue(plant.HasModelInstanceNamed(name="acrobot"))
        self.assertEqual(
            model_instance, plant.GetModelInstanceByName(name="acrobot"))
        self.assertEqual(
            plant.GetBodiesWeldedTo(plant.world_body()), [plant.world_body()])
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
        self.assertIs(shoulder, plant.get_mutable_joint(
            joint_index=JointIndex(0)))
        self.assertEqual([JointIndex(0), JointIndex(1)],
                         plant.GetJointIndices(model_instance=model_instance))
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
        element.GetParentPlant()

    def _test_frame_api(self, T, frame):
        Frame = Frame_[T]

        self.assertIsInstance(frame, Frame)
        self._test_multibody_tree_element_mixin(T, frame)

        self.assertIsInstance(frame.body(), Body_[T])
        self.assertIsInstance(frame.is_world_frame(), bool)
        self.assertIsInstance(frame.is_body_frame(), bool)
        self.assertIsInstance(frame.name(), str)

        self.assertIsInstance(
            frame.GetFixedPoseInBodyFrame(),
            RigidTransform_[T])
        self.assertIsInstance(
            frame.GetFixedRotationMatrixInBodyFrame(),
            RotationMatrix_[T])

    def _test_body_api(self, T, body):
        Body = Body_[T]

        self.assertIsInstance(body, Body)
        self._test_multibody_tree_element_mixin(T, body)
        self.assertIsInstance(body.name(), str)
        self.assertIsInstance(body.get_num_flexible_positions(), int)
        self.assertIsInstance(body.get_num_flexible_velocities(), int)
        self.assertIsInstance(body.is_floating(), bool)
        self.assertIsInstance(body.has_quaternion_dofs(), bool)
        self.assertIsInstance(body.floating_positions_start(), int)
        self.assertIsInstance(body.floating_velocities_start(), int)
        self.assertIsInstance(body.get_default_mass(), float)

    @numpy_compare.check_all_types
    def test_body_context_methods(self, T):
        plant = MultibodyPlant_[T](0.0)
        dut = plant.AddRigidBody(
            name="name", M_BBo_B=SpatialInertia_[float](
                mass=1.0, p_PScm_E=[0.0, 0.0, 0.0],
                G_SP_E=UnitInertia_[float](Ixx=1.0, Iyy=1.0, Izz=1.0)))
        plant.Finalize()
        context = plant.CreateDefaultContext()
        self.assertIsInstance(
            dut.get_mass(context=context),
            T)
        self.assertIsInstance(
            dut.CalcCenterOfMassInBodyFrame(context=context),
            np.ndarray)
        self.assertIsInstance(
            dut.CalcSpatialInertiaInBodyFrame(context=context),
            SpatialInertia_[T])
        self.assertIsInstance(
            dut.EvalPoseInWorld(context=context),
            RigidTransform_[T])
        self.assertIsInstance(
            dut.EvalSpatialVelocityInWorld(context=context),
            SpatialVelocity_[T])
        self.assertIsInstance(
            dut.EvalSpatialAccelerationInWorld(context=context),
            SpatialAcceleration_[T])
        # The test_multibody_dynamics already covers GetForceInWorld,
        # AddInForce, AddInForceInWorld.

        # Also test these body api methods which require a finalized plant.
        for i in range(7):
            self.assertIsNotNone(dut.floating_position_suffix(i))
        for i in range(6):
            self.assertIsNotNone(dut.floating_velocity_suffix(i))

    def _test_joint_api(self, T, joint):
        Joint = Joint_[T]
        Body = Body_[T]
        Frame = Frame_[T]

        self.assertIsInstance(joint, Joint)
        self._test_multibody_tree_element_mixin(T, joint)
        self.assertIsInstance(joint.name(), str)
        self.assertIsInstance(joint.type_name(), str)
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
                Exception, r"incompatible function arguments"):
            dut[0]
        with self.assertRaisesRegex(
                Exception, r"Unable to cast"):
            dut[0.0, 0.0]
        with self.assertRaisesRegex(
                Exception, r"Expected \[i,j\]"):
            dut[0, 0, 0]
        self.assertIsInstance(dut.CopyToFullMatrix3(), np.ndarray)
        dut.IsNearlyEqualTo(other=dut, precision=0.0)
        dut.SetToNaN()
        dut.IsNaN()
        dut.SetZero()
        self.assertIsInstance(dut.CalcPrincipalMomentsOfInertia(), np.ndarray)
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
            dut.ShiftFromCenterOfMass(mass=1.0, p_BcmQ_E=p), Class)
        self.assertIsInstance(
            dut.ShiftToCenterOfMass(mass=1.0, p_QBcm_E=p), Class)
        self.assertIsInstance(
            dut.ShiftToThenAwayFromCenterOfMass(
                mass=1.0, p_PBcm_E=p, p_QBcm_E=p),
            Class)
        # Test operators.
        zero = Class(Ixx=0.0, Iyy=0.0, Izz=0.0)
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

    @numpy_compare.check_all_types
    def test_unit_inertia_api(self, T):
        """Tests unit inertia construction and API."""
        UnitInertia = UnitInertia_[T]
        self._test_rotational_inertia_or_unit_inertia_api(T, UnitInertia)
        # Test methods present only on UnitInertia, not RotationalInernia.
        p = [0.1, 0.2, 0.3]
        dut = UnitInertia(I=RotationalInertia_[T](mass=1.0, p_PQ_E=p))
        self.assertIsInstance(
            dut.ShiftFromCenterOfMass(p_BcmQ_E=p), UnitInertia)
        self.assertIsInstance(
            dut.ShiftToCenterOfMass(p_QBcm_E=p), UnitInertia)
        assert_pickle(self, dut, UnitInertia.CopyToFullMatrix3, T=T)
        # N.B. There are NO valid operators on UnitInertia.  They are inherited
        # through implementation reuse, but they are broken (#6109).

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
        SpatialInertia()
        SpatialInertia.MakeFromCentralInertia(
            mass=1.3, p_PScm_E=[0.1, -0.2, 0.3],
            I_SScm_E=RotationalInertia(Ixx=2.0, Iyy=2.3, Izz=2.4))
        spatial_inertia = SpatialInertia(
            mass=2.5, p_PScm_E=[0.1, -0.2, 0.3],
            G_SP_E=UnitInertia(Ixx=2.0, Iyy=2.3, Izz=2.4))
        numpy_compare.assert_float_equal(spatial_inertia.get_mass(), 2.5)
        copied_inertia = copy.deepcopy(spatial_inertia)
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
            spatial_inertia.ReExpress(R_AE=RotationMatrix()), SpatialInertia)
        self.assertIsInstance(
            spatial_inertia.Shift([1, 2, 3]), SpatialInertia)
        spatial_inertia += spatial_inertia
        self.assertIsInstance(
            spatial_inertia * SpatialAcceleration(), SpatialForce)
        self.assertIsInstance(
            spatial_inertia * SpatialVelocity(), SpatialMomentum)
        assert_pickle(
            self, spatial_inertia, SpatialInertia.CopyToFullMatrix6, T=T)
        spatial_inertia.SetNaN()
        # N.B. `numpy_compare.assert_equal(IsNaN(), True)` does not work.
        if T != Expression:
            self.assertTrue(spatial_inertia.IsNaN())

    @numpy_compare.check_all_types
    def test_friction_api(self, T):
        CoulombFriction = CoulombFriction_[T]
        CoulombFriction()
        CoulombFriction(static_friction=0.7, dynamic_friction=0.6)
        copy.copy(CoulombFriction())
        assert_pickle(
            self, CoulombFriction,
            lambda x: [x.static_friction, x.dynamic_friction], T=T
        )

    @numpy_compare.check_all_types
    def test_rigid_body_api(self, T):
        RigidBody = RigidBody_[T]
        M = SpatialInertia_[float]()
        i = ModelInstanceIndex(0)
        RigidBody(M_BBo_B=M)
        RigidBody(body_name="body_name", M_BBo_B=M)
        RigidBody(body_name="body_name", model_instance=i, M_BBo_B=M)

    @numpy_compare.check_all_types
    def test_multibody_force_element(self, T):
        MultibodyPlant = MultibodyPlant_[T]
        LinearSpringDamper = LinearSpringDamper_[T]
        RevoluteSpring = RevoluteSpring_[T]
        DoorHinge = DoorHinge_[T]
        LinearBushingRollPitchYaw = LinearBushingRollPitchYaw_[T]
        SpatialInertia = SpatialInertia_[float]
        UnitInertia = UnitInertia_[float]
        SpatialForce = SpatialForce_[T]

        plant = MultibodyPlant(0.0)
        spatial_inertia = SpatialInertia(
            mass=1,
            p_PScm_E=[0, 0, 0],
            G_SP_E=UnitInertia(1, 1, 1),
        )
        body_a = plant.AddRigidBody(name="body_a",
                                    M_BBo_B=spatial_inertia)
        body_b = plant.AddRigidBody(name="body_b",
                                    M_BBo_B=spatial_inertia)
        linear_spring_index = ForceElementIndex(plant.num_force_elements())
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

        torque_stiffness = np.array([10.0, 11.0, 12.0])
        torque_damping = np.array([1.0, 1.1, 1.2])
        force_stiffness = np.array([20.0, 21.0, 22.0])
        force_damping = np.array([2.0, 2.1, 2.2])
        bushing = plant.AddForceElement(LinearBushingRollPitchYaw(
            frameA=body_a.body_frame(),
            frameC=body_b.body_frame(),
            torque_stiffness_constants=torque_stiffness,
            torque_damping_constants=torque_damping,
            force_stiffness_constants=force_stiffness,
            force_damping_constants=force_damping,
        ))
        plant.Finalize()
        context = plant.CreateDefaultContext()

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

        # Test LinearBushingRollPitchYaw accessors.
        self.assertIs(bushing.link0(), body_a)
        self.assertIs(bushing.link1(), body_b)
        self.assertIs(bushing.frameA(), body_a.body_frame())
        self.assertIs(bushing.frameC(), body_b.body_frame())
        numpy_compare.assert_float_equal(
            bushing.torque_stiffness_constants(), torque_stiffness)
        numpy_compare.assert_float_equal(
            bushing.torque_damping_constants(), torque_damping)
        numpy_compare.assert_float_equal(
            bushing.force_stiffness_constants(), force_stiffness)
        numpy_compare.assert_float_equal(
            bushing.force_damping_constants(), force_damping)
        # Set to 2x the original value and ensure it was updated in the given
        # context.
        bushing.SetTorqueStiffnessConstants(
            context=context, torque_stiffness=2 * torque_stiffness)
        numpy_compare.assert_float_equal(
            bushing.GetTorqueStiffnessConstants(context=context),
            2 * torque_stiffness)
        bushing.SetTorqueDampingConstants(
            context=context, torque_damping=2 * torque_damping)
        numpy_compare.assert_float_equal(
            bushing.GetTorqueDampingConstants(context=context),
            2 * torque_damping)
        bushing.SetForceStiffnessConstants(
            context=context, force_stiffness=2 * force_stiffness)
        numpy_compare.assert_float_equal(
            bushing.GetForceStiffnessConstants(context=context),
            2 * force_stiffness)
        bushing.SetForceDampingConstants(
            context=context, force_damping=2 * force_damping)
        numpy_compare.assert_float_equal(
            bushing.GetForceDampingConstants(context=context),
            2 * force_damping)

        # Test get_force_element().
        self.assertIs(
            plant.get_force_element(force_element_index=linear_spring_index),
            linear_spring)

    @numpy_compare.check_all_types
    def test_multibody_gravity_default(self, T):
        MultibodyPlant = MultibodyPlant_[T]
        UniformGravityFieldElement = UniformGravityFieldElement_[T]
        plant = MultibodyPlant(0.0)
        with self.assertRaises(RuntimeError) as cm:
            plant.AddForceElement(UniformGravityFieldElement())
        self.assertIsInstance(
            UniformGravityFieldElement.kDefaultStrength,
            float)

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
            "drake/bindings/pydrake/multibody/test/double_pendulum.sdf")
        # N.B. `Parser` only supports `MultibodyPlant_[float]`.
        instance = Parser(plant_f).AddModelFromFile(file_name)
        plant_f.Finalize()
        plant = to_type(plant_f, T)
        context = plant.CreateDefaultContext()
        world_frame = plant.world_frame()
        base = plant.GetBodyByName("base")
        base_frame = plant.GetFrameByName("base")
        X_WL = plant.CalcRelativeTransform(
            context, frame_A=world_frame, frame_B=base_frame)
        self.assertIsInstance(X_WL, RigidTransform)
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
            context=context, frame_B=base_frame,
            p_BQi=np.array([[0, 1, 2], [10, 11, 12]]).T,
            frame_A=world_frame).T
        self.assertTupleEqual(p_AQi.shape, (2, 3))

        # Verify CalcTotalMass() calculates a non-zero mass.
        p_mass = plant.CalcTotalMass(context=context)
        numpy_compare.assert_float_not_equal(p_mass, 0.)
        p_mass = plant.CalcTotalMass(
            context=context, model_instances=[instance])
        numpy_compare.assert_float_not_equal(p_mass, 0.)

        p_com = plant.CalcCenterOfMassPositionInWorld(context=context)
        self.assertTupleEqual(p_com.shape, (3, ))
        p_com = plant.CalcCenterOfMassPositionInWorld(
            context=context, model_instances=[instance])
        self.assertTupleEqual(p_com.shape, (3, ))

        nq = plant.num_positions()
        nv = plant.num_velocities()
        wrt_list = [
            (JacobianWrtVariable.kQDot, nq),
            (JacobianWrtVariable.kV, nv),
        ]
        for wrt, nw in wrt_list:
            Js_v_ACcm_E = plant.CalcJacobianCenterOfMassTranslationalVelocity(
                context=context, with_respect_to=wrt, frame_A=base_frame,
                frame_E=world_frame)
            self.assert_sane(Js_v_ACcm_E)
            self.assertEqual(Js_v_ACcm_E.shape, (3, nw))

            Js_v_ACcm_E = plant.CalcJacobianCenterOfMassTranslationalVelocity(
                context=context, model_instances=[instance],
                with_respect_to=wrt, frame_A=base_frame, frame_E=world_frame)
            self.assert_sane(Js_v_ACcm_E)
            self.assertEqual(Js_v_ACcm_E.shape, (3, nw))

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

            Js_v_AB_E = plant.CalcJacobianTranslationalVelocity(
                context=context, with_respect_to=wrt, frame_B=base_frame,
                p_BoBi_B=np.zeros((3, 3)), frame_A=world_frame,
                frame_E=world_frame)
            self.assert_sane(Js_v_AB_E)
            self.assertEqual(Js_v_AB_E.shape, (9, nw))

        L_WSP_W = plant.CalcSpatialMomentumInWorldAboutPoint(
            context=context, p_WoP_W=np.zeros(3))
        self.assert_sane(L_WSP_W.translational(), nonzero=False)
        self.assert_sane(L_WSP_W.rotational(), nonzero=False)
        self.assertIsInstance(L_WSP_W, SpatialMomentum_[T])
        L_WSP_W = plant.CalcSpatialMomentumInWorldAboutPoint(
            context=context, model_instances=[default_model_instance()],
            p_WoP_W=np.zeros(3))
        self.assert_sane(L_WSP_W.translational(), nonzero=False)
        self.assert_sane(L_WSP_W.rotational(), nonzero=False)
        self.assertIsInstance(L_WSP_W, SpatialMomentum_[T])

        abias_ACcm_E = plant.CalcBiasCenterOfMassTranslationalAcceleration(
            context=context, with_respect_to=JacobianWrtVariable.kV,
            frame_A=world_frame, frame_E=world_frame)
        self.assert_sane(abias_ACcm_E, nonzero=False)
        self.assertEqual(abias_ACcm_E.shape, (3,))

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

        # Make sure we can call these methods.
        base.SetMass(context=context, mass=1.0)
        base.SetCenterOfMassInBodyFrame(context=context, com=[0.0, 0.0, 0.0])
        M = SpatialInertia_[T](1, [0, 0, 0], UnitInertia_[T](1, 1, 1))
        base.SetSpatialInertiaInBodyFrame(context=context, M_Bo_B=M)

    def test_deprecated_set_free_body_pose(self):
        plant = MultibodyPlant_[float](0.0)
        Parser(plant).AddModelFromFile(FindResourceOrThrow(
            "drake/bindings/pydrake/multibody/test/double_pendulum.sdf"))
        plant.Finalize()
        with catch_drake_warnings(expected_count=1):
            plant.SetFreeBodyPose(
                context=plant.CreateDefaultContext(),
                body=plant.GetBodyByName("base"),
                X_WB=Isometry3())

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
                    self.DoCalcAbstractOutput)
                self.DeclareVectorOutputPort("generalized_forces", self.nv,
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
            frame_on_parent_P=plant_f.world_frame(),
            frame_on_child_C=plant_f.GetFrameByName("iiwa_link_0", iiwa_model),
            X_PC=RigidTransform_[float]())
        # Perform the second weld without named arguments to ensure that the
        # proper binding gets invoked.
        plant_f.WeldFrames(
            plant_f.GetFrameByName("iiwa_link_7", iiwa_model),
            plant_f.GetFrameByName("body", gripper_model),
            X_EeGripper)
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
            frame_on_parent_P=plant_f.world_frame(),
            frame_on_child_C=plant_f.GetFrameByName("iiwa_link_0", iiwa_model))
        plant_f.WeldFrames(
            frame_on_parent_P=plant_f.GetFrameByName(
                "iiwa_link_7", iiwa_model),
            frame_on_child_C=plant_f.GetFrameByName("body", gripper_model),
            X_PC=X_EeGripper)
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
        # Use floating base to effectively add a quaternion in the generalized
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
        link0 = plant.GetBodyByName("iiwa_link_0")
        plant.SetFreeBodyPose(
            context, link0,
            RigidTransform(RollPitchYaw([0.1, 0.2, 0.3]),
                           p=[0.4, 0.5, 0.6]))
        numpy_compare.assert_float_allclose(
            plant.GetFreeBodyPose(context, link0).translation(),
            [0.4, 0.5, 0.6])
        self.assertNotEqual(link0.floating_positions_start(), -1)
        self.assertNotEqual(link0.floating_velocities_start(), -1)
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
        array_T = np.vectorize(T)
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

        def make_planar_joint(plant, P, C):
            return PlanarJoint_[T](
                name="planar",
                frame_on_parent=P,
                frame_on_child=C,
                damping=[1., 2., 3.],
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
                frame_on_parent_P=P,
                frame_on_child_C=C,
                X_PC=X_PC,
            )

        make_joint_list = [
            make_ball_rpy_joint,
            make_planar_joint,
            make_prismatic_joint,
            make_revolute_joint,
            make_universal_joint,
            make_weld_joint,
        ]

        def loop_body(make_joint, time_step):
            plant = MultibodyPlant_[T](time_step)
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
            if joint.num_velocities() == 1 and T == float:
                u = np.array([0.1])
                numpy_compare.assert_float_equal(
                    actuator.get_actuation_vector(u=u), [0.1])
                actuator.set_actuation_vector(
                    u_instance=np.array([0.2]), u=u)
                numpy_compare.assert_float_equal(u, [0.2])

            for p in range(joint.num_positions()):
                self.assertIsNotNone(joint.position_suffix(p))
            for v in range(joint.num_velocities()):
                self.assertIsNotNone(joint.velocity_suffix(v))

            uniform_random = Variable(
                name="uniform_random",
                type=Variable.Type.RANDOM_UNIFORM)

            context = plant.CreateDefaultContext()
            self.assertFalse(joint.is_locked(context))
            if time_step:
                joint.Lock(context)
                self.assertTrue(joint.is_locked(context))
                joint.Unlock(context)
                self.assertFalse(joint.is_locked(context))

            if joint.name() == "ball_rpy":
                joint.damping()
                set_point = array_T([1., 2., 3.])
                joint.set_angles(context=context, angles=set_point)
                numpy_compare.assert_equal(
                    joint.get_angles(context=context),
                    set_point)
                joint.set_angular_velocity(context=context, w_FM=set_point)
                numpy_compare.assert_equal(
                    joint.get_angular_velocity(context=context),
                    set_point)
                joint.set_random_angles_distribution(
                    uniform_random * np.array([1., 1., 1.]))
                joint.get_default_angles()
                joint.set_default_angles(angles=[0.0, 0.0, 0.0])
            elif joint.name() == "planar":
                self.assertEqual(len(joint.damping()), 3)
                set_translation = array_T([1., 2.])
                set_angle = T(3.)
                joint.set_translation(context=context,
                                      p_FoMo_F=set_translation)
                numpy_compare.assert_equal(
                    joint.get_translation(context=context), set_translation)
                joint.set_rotation(context=context, theta=set_angle)
                numpy_compare.assert_equal(joint.get_rotation(context=context),
                                           set_angle)
                joint.set_pose(context=context, p_FoMo_F=set_translation,
                               theta=set_angle)
                joint.set_translational_velocity(context=context,
                                                 v_FoMo_F=set_translation)
                numpy_compare.assert_equal(
                    joint.get_translational_velocity(context=context),
                    set_translation)
                joint.set_angular_velocity(context=context,
                                           theta_dot=set_angle)
                numpy_compare.assert_equal(
                    joint.get_angular_velocity(context=context), set_angle)
                joint.set_random_pose_distribution(
                    p_FoMo_F=uniform_random * np.array([1., 1.]),
                    theta=uniform_random)
                joint.get_default_translation()
                joint.set_default_translation(p_FoMo_F=[0.0, 0.0])
                joint.get_default_rotation()
                joint.set_default_rotation(theta=0.0)
                joint.set_default_pose(p_FoMo_F=[0.0, 0.0], theta=0.0)
            elif joint.name() == "prismatic":
                self.assertEqual(joint.damping(), damping)
                numpy_compare.assert_equal(joint.translation_axis(), x_axis)
                set_point = T(1.)
                joint.set_translation(context=context, translation=set_point)
                numpy_compare.assert_equal(
                    joint.get_translation(context=context),
                    set_point)
                joint.set_translation_rate(
                    context=context, translation_dot=set_point)
                numpy_compare.assert_equal(
                    joint.get_translation_rate(context=context),
                    set_point)
                joint.set_random_translation_distribution(uniform_random)
                joint.position_lower_limit()
                joint.position_upper_limit()
                joint.velocity_lower_limit()
                joint.velocity_upper_limit()
                joint.acceleration_lower_limit()
                joint.acceleration_upper_limit()
                joint.get_default_translation()
                joint.set_default_translation(translation=0.0)
            elif joint.name() == "revolute":
                numpy_compare.assert_equal(joint.revolute_axis(), x_axis)
                self.assertEqual(joint.damping(), damping)
                set_point = T(1.)
                joint.set_angle(context=context, angle=set_point)
                numpy_compare.assert_equal(
                    joint.get_angle(context=context),
                    set_point)
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
                    context=context, joint_dof=0, joint_tau=0.0, forces=forces)
                joint.AddInDamping(context=context, forces=forces)
            elif joint.name() == "universal":
                self.assertEqual(joint.damping(), damping)
                set_point = array_T([1., 2.])
                joint.set_angles(context=context, angles=set_point)
                numpy_compare.assert_equal(
                    joint.get_angles(context=context),
                    set_point)
                joint.set_angular_rates(context=context, theta_dot=set_point)
                numpy_compare.assert_equal(
                    joint.get_angular_rates(context=context),
                    set_point)
                joint.set_random_angles_distribution(
                    uniform_random * np.array([1., 1.]))
                joint.get_default_angles()
                joint.set_default_angles(angles=[1.0, 2.0])
            elif joint.name() == "weld":
                numpy_compare.assert_float_equal(
                    joint.X_PC().GetAsMatrix4(),
                    X_PC.GetAsMatrix4())
            else:
                raise TypeError(
                    "Joint type " + joint.name() + " not recognized.")

        for make_joint in make_joint_list:
            with self.subTest(make_joint=make_joint):
                loop_body(make_joint, 0.0)
                loop_body(make_joint, 0.001)

    @numpy_compare.check_all_types
    def test_deprecated_weld_joint(self, T):
        plant = MultibodyPlant_[T](0.0)
        child = plant.AddRigidBody("Child", SpatialInertia_[float]())
        with catch_drake_warnings(expected_count=1):
            joint = WeldJoint_[T](
                name="weld",
                parent_frame_P=plant.world_frame(),
                child_frame_C=child.body_frame(),
                X_PC=Isometry3())
        self.assertIsInstance(joint, Joint_[T])

    def test_deprecated_weld_frames(self):
        plant = MultibodyPlant_[float](0.0)
        parser = Parser(plant)
        iiwa_sdf_path = FindResourceOrThrow(
            "drake/manipulation/models/"
            "iiwa_description/sdf/iiwa14_no_collision.sdf")
        iiwa_model = parser.AddModelFromFile(
            file_name=iiwa_sdf_path, model_name="robot")
        with catch_drake_warnings(expected_count=1):
            weld = plant.WeldFrames(
                A=plant.world_frame(),
                B=plant.GetFrameByName("iiwa_link_0", iiwa_model),
                X_AB=Isometry3())
        self.assertIsInstance(weld, Joint_[float])

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

        plant.Finalize()
        context = plant.CreateDefaultContext()

        X_PF = RigidTransform_[T](p=[1., 2., 3.])
        frame.SetPoseInBodyFrame(context=context, X_PF=X_PF)

        numpy_compare.assert_float_equal(
            frame.CalcPoseInBodyFrame(context).GetAsMatrix34(),
            numpy_compare.to_float(X_PF.GetAsMatrix34()))

    @numpy_compare.check_all_types
    def test_deprecated_fixed_offset_frame(self, T):
        plant = MultibodyPlant_[T](0.0)
        with catch_drake_warnings(expected_count=1):
            frame = plant.AddFrame(frame=FixedOffsetFrame_[T](
                name="frame", P=plant.world_frame(),
                X_PF=Isometry3(), model_instance=None))
        self.assertIsInstance(frame, Frame_[T])

    @numpy_compare.check_all_types
    def test_frame_context_methods(self, T):
        plant = MultibodyPlant_[T](0.0)
        dut = plant.AddFrame(frame=FixedOffsetFrame_[T](
            name="name", P=plant.world_frame(), X_PF=RigidTransform_[float]()))
        plant.Finalize()
        context = plant.CreateDefaultContext()
        X = RigidTransform_[T]()
        self.assertIsInstance(
            dut.CalcRotationMatrixInBodyFrame(context=context),
            RotationMatrix_[T])
        self.assertIsInstance(
            dut.CalcOffsetPoseInBody(context=context, X_FQ=X),
            RigidTransform_[T])
        self.assertIsInstance(
            dut.CalcOffsetRotationMatrixInBody(
                context=context, R_FQ=X.rotation()),
            RotationMatrix_[T])
        self.assertIsInstance(
            dut.GetFixedOffsetPoseInBody(X_FQ=X),
            RigidTransform_[T])
        self.assertIsInstance(
            dut.GetFixedRotationMatrixInBody(R_FQ=X.rotation()),
            RotationMatrix_[T])
        self.assertIsInstance(
            dut.CalcPoseInWorld(context=context),
            RigidTransform_[T])
        self.assertIsInstance(
            dut.CalcPose(context=context, frame_M=dut),
            RigidTransform_[T])
        self.assertIsInstance(
            dut.CalcRotationMatrix(context=context, frame_M=dut),
            RotationMatrix_[T])
        self.assertIsInstance(
            dut.CalcRotationMatrixInWorld(context=context),
            RotationMatrix_[T])
        self.assertIsInstance(
            dut.CalcSpatialVelocityInWorld(context=context),
            SpatialVelocity_[T])
        self.assertIsInstance(
            dut.CalcSpatialVelocity(context=context, frame_M=dut, frame_E=dut),
            SpatialVelocity_[T])
        self.assertIsInstance(
            dut.CalcSpatialAccelerationInWorld(context=context),
            SpatialAcceleration_[T])

    @numpy_compare.check_all_types
    def test_fixed_offset_frame_api(self, T):
        FixedOffsetFrame = FixedOffsetFrame_[T]
        P = MultibodyPlant_[T](0.0).world_frame()
        B = RigidBody_[T](SpatialInertia_[float]())
        X = RigidTransform_[float].Identity()
        FixedOffsetFrame(name="name", P=P, X_PF=X, model_instance=None)
        FixedOffsetFrame(P=P, X_PF=X)
        FixedOffsetFrame(name="name", bodyB=B, X_BF=X)
        FixedOffsetFrame(bodyB=B, X_BF=X)

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
        M = plant.CalcMassMatrix(context)
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
        plant.gravity_field().CalcGravityGeneralizedForces(context=context)

        # Gravity is the only force element
        self.assertEqual(plant.num_force_elements(), 1)

        B = plant.MakeActuationMatrix()
        numpy_compare.assert_float_equal(B, np.array([[0.], [1.]]))

        forces = MultibodyForces(plant=plant)
        plant.CalcForceElementsContribution(context=context, forces=forces)
        copy.copy(forces)

        # Test generalized forces.
        # N.B. Cannot use `ndarray[object]` to reference existing C arrays
        # (#8116).
        if T == float:
            forces.mutable_generalized_forces()[:] = 1
            np.testing.assert_equal(forces.generalized_forces(), 1)
            forces.SetZero()
            np.testing.assert_equal(forces.generalized_forces(), 0)

        # Test standalone construction.
        standalone_forces = MultibodyForces(nb=1, nv=2)
        self.assertEqual(standalone_forces.num_bodies(), 1)
        self.assertEqual(standalone_forces.num_velocities(), 2)

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
        copy.copy(contact_info)

        # ContactResults
        contact_results = ContactResults()
        self.assertTrue(contact_results.num_point_pair_contacts() == 0)
        copy.copy(contact_results)

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
        contact_results_to_lcm = ContactResultsToLcmSystem(plant=plant)
        context = contact_results_to_lcm.CreateDefaultContext()
        contact_results_to_lcm.get_input_port(0).FixValue(
            context, ContactResults_[float]())
        output = contact_results_to_lcm.AllocateOutput()
        contact_results_to_lcm.CalcOutput(context, output)
        result = output.get_data(0)
        self.assertIsInstance(result, AbstractValue)

    def test_connect_contact_results(self):
        # For this test to be meaningful, the sdf file must contain collision
        # geometries. We'll do a reality check after instantiating.
        file_name = FindResourceOrThrow(
            "drake/bindings/pydrake/multibody/test/double_pendulum.sdf")
        builder = DiagramBuilder_[float]()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
        Parser(plant).AddModelFromFile(file_name)
        plant.Finalize()
        self.assertGreater(
            len(plant.GetCollisionGeometriesForBody(
                plant.GetBodyByName("base"))),
            0)
        # Check all valid combinations of the optional arguments.
        for optional_args in itertools.product(
                [{}, {"scene_graph": scene_graph}],
                [{}, {"lcm": None}, {"lcm": DrakeLcm()}],
                [{}, {"publish_period": None}, {"publish_period": 1.0/32}]):
            kwargs = collections.ChainMap(*optional_args)
            with self.subTest(num_optional_args=len(kwargs), **kwargs):
                is_deprecated = 0 if "scene_graph" in kwargs else 1
                with catch_drake_warnings(expected_count=is_deprecated) as w:
                    publisher = ConnectContactResultsToDrakeVisualizer(
                        builder=builder, plant=plant, **kwargs)
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
        # TODO(sean.curtis@tri.global) The unit test of QueryObject/SceneGraph
        # don't belong in plant_test.py. They belong in geometry_test.py. This
        # test should be moved.
        # SceneGraph does not support `Expression` type.
        PenetrationAsPointPair = PenetrationAsPointPair_[T]

        builder_f = DiagramBuilder_[float]()
        # N.B. `Parser` only supports `MultibodyPlant_[float]`.
        plant_f, scene_graph_f = AddMultibodyPlantSceneGraph(builder_f, 0.0)
        parser = Parser(plant=plant_f, scene_graph=scene_graph_f)
        # The validity of this test depends on the colliding geometries to be
        # supported across all nonsymbolic scalar types (double, autodiffxd).
        # If two_bodies.sdf were to change to unsupported geometries, this
        # test would break.
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
        self.assertIsInstance(point_pair, PenetrationAsPointPair_[T])
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
        copy.copy(info)

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
        Parser(plant).AddModelFromFile(
            FindResourceOrThrow(
                "drake/bindings/pydrake/multibody/test/hydroelastic.sdf"))
        plant.set_contact_model(ContactModel.kHydroelastic)
        plant.Finalize()

        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        plant_context = plant.GetMyContextFromRoot(context)

        body1 = plant.GetBodyByName("body1")
        X_WB = RigidTransform(p=[0, 0, 0.25])
        plant.SetFreeBodyPose(plant_context, body1, X_WB)

        contact_results = (
            plant.get_contact_results_output_port().Eval(plant_context))

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

    @numpy_compare.check_nonsymbolic_types
    def test_contact_results_to_meshcat(self, T):
        meshcat = Meshcat()
        params = ContactResultsToMeshcatParams()
        params.publish_period = 0.123
        params.default_color = Rgba(0.5, 0.5, 0.5)
        params.prefix = "py_visualizer"
        params.delete_on_initialization_event = False
        params.force_threshold = 0.2
        params.newtons_per_meter = 5
        params.radius = 0.1
        self.assertNotIn("object at 0x", repr(params))
        params2 = ContactResultsToMeshcatParams(publish_period=0.4)
        self.assertEqual(params2.publish_period, 0.4)
        vis = ContactResultsToMeshcat_[T](meshcat=meshcat, params=params)
        vis.Delete()
        self.assertIsInstance(vis.contact_results_input_port(), InputPort_[T])

        builder = DiagramBuilder_[T]()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.01)
        plant.Finalize()
        ContactResultsToMeshcat_[T].AddToBuilder(
            builder=builder, plant=plant, meshcat=meshcat, params=params)
        ContactResultsToMeshcat_[T].AddToBuilder(
            builder=builder,
            contact_results_port=plant.get_contact_results_output_port(),
            meshcat=meshcat,
            params=params)
