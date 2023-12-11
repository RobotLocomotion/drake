import pydrake.math as mut
from pydrake.math import (BarycentricMesh, wrap_to)
from pydrake.common import RandomGenerator
from pydrake.common.cpp_param import List
from pydrake.common.eigen_geometry import Isometry3_, Quaternion_, AngleAxis_
from pydrake.common.value import Value
from pydrake.autodiffutils import AutoDiffXd
from pydrake.symbolic import Expression
from pydrake.common.test_utilities.deprecation import catch_drake_warnings
import pydrake.common.test_utilities.numpy_compare as numpy_compare
from pydrake.common.test_utilities.pickle_compare import assert_pickle

import copy
import math
import pickle
import textwrap
import unittest

import numpy as np


class TestBarycentricMesh(unittest.TestCase):
    def test_spelling(self):
        mesh = BarycentricMesh([{0, 1}, {0, 1}])
        values = np.array([[0, 1, 2, 3]])
        grid = mesh.get_input_grid()
        self.assertIsInstance(grid, list)
        self.assertEqual(len(grid), 2)
        self.assertIsInstance(grid[0], set)
        self.assertEqual(len(grid[0]), 2)
        self.assertEqual(mesh.get_input_size(), 2)
        self.assertEqual(mesh.get_num_mesh_points(), 4)
        self.assertEqual(mesh.get_num_interpolants(), 3)
        self.assertTrue((mesh.get_mesh_point(0) == [0., 0.]).all())
        points = mesh.get_all_mesh_points()
        self.assertEqual(points.shape, (2, 4))
        self.assertTrue((points[:, 3] == [1., 1.]).all())
        self.assertEqual(mesh.Eval(values, (0, 0))[0], 0)
        self.assertEqual(mesh.Eval(values, (1, 0))[0], 1)
        self.assertEqual(mesh.Eval(values, (0, 1))[0], 2)
        self.assertEqual(mesh.Eval(values, (1, 1))[0], 3)

    def test_weight(self):
        mesh = BarycentricMesh([{0, 1}, {0, 1}])

        (Ti, T) = mesh.EvalBarycentricWeights((0., 1.))
        np.testing.assert_equal(Ti, [2, 2, 0])
        np.testing.assert_almost_equal(T, (1., 0., 0.))

    def test_mesh_values_from(self):
        mesh = BarycentricMesh([{0, 1}, {0, 1}])

        def mynorm(x):
            return [x.dot(x)]

        values = mesh.MeshValuesFrom(mynorm)
        self.assertEqual(values.size, 4)


# TODO(eric.cousineau): Test wrappings against non-identity transforms.


class TestMath(unittest.TestCase):
    def test_math(self):
        # Compare against `math` functions.
        # TODO(eric.cousineau): Consider removing this and only rely on
        # `math_overloads_test`, which already tests this.
        unary = [
            (mut.log, math.log),
            (mut.abs, math.fabs),
            (mut.exp, math.exp),
            (mut.sqrt, math.sqrt),
            (mut.sin, math.sin),
            (mut.cos, math.cos),
            (mut.tan, math.tan),
            (mut.asin, math.asin),
            (mut.acos, math.acos),
            (mut.atan, math.atan),
            (mut.sinh, math.sinh),
            (mut.cosh, math.cosh),
            (mut.tanh, math.tanh),
            (mut.ceil, math.ceil),
            (mut.floor, math.floor),
            (mut.isnan, math.isnan),
        ]
        binary = [
            (mut.min, min),
            (mut.max, max),
            (mut.pow, math.pow),
            (mut.atan2, math.atan2),
        ]

        a = 0.1
        b = 0.2
        for f_core, f_cpp in unary:
            self.assertEqual(f_core(a), f_cpp(a), (f_core, f_cpp))
        for f_core, f_cpp in binary:
            self.assertEqual(f_core(a, b), f_cpp(a, b))

    def check_cast(self, template, T):
        value = template[T]()
        # Refer to docstrings for `CastUPack` in `default_scalars_pybind.h`.
        if T == float:
            U_list = [float, AutoDiffXd, Expression]
        else:
            U_list = [T]
        for U in U_list:
            self.assertIsInstance(value.cast[U](), template[U], U)

    @numpy_compare.check_all_types
    def test_rigid_transform(self, T):
        RigidTransform = mut.RigidTransform_[T]
        RotationMatrix = mut.RotationMatrix_[T]
        RollPitchYaw = mut.RollPitchYaw_[T]
        Isometry3 = Isometry3_[T]
        Quaternion = Quaternion_[T]
        AngleAxis = AngleAxis_[T]

        def check_equality(X_actual, X_expected_matrix):
            self.assertIsInstance(X_actual, RigidTransform)
            numpy_compare.assert_float_equal(
                    X_actual.GetAsMatrix4(), X_expected_matrix)

        # - Constructors.
        X_I_np = np.eye(4)
        check_equality(RigidTransform(), X_I_np)
        check_equality(RigidTransform(other=RigidTransform()), X_I_np)
        check_equality(copy.copy(RigidTransform()), X_I_np)
        R_I = RotationMatrix()
        p_I = np.zeros(3)
        rpy_I = RollPitchYaw(0, 0, 0)
        quaternion_I = Quaternion.Identity()
        angle = np.pi * 0
        axis = [0, 0, 1]
        angle_axis = AngleAxis(angle=angle, axis=axis)
        check_equality(RigidTransform(R=R_I, p=p_I), X_I_np)
        check_equality(RigidTransform(rpy=rpy_I, p=p_I), X_I_np)
        check_equality(RigidTransform(quaternion=quaternion_I, p=p_I), X_I_np)
        check_equality(RigidTransform(theta_lambda=angle_axis, p=p_I), X_I_np)
        check_equality(RigidTransform(R=R_I), X_I_np)
        check_equality(RigidTransform(p=p_I), X_I_np)
        check_equality(RigidTransform(pose=p_I), X_I_np)
        check_equality(RigidTransform(pose=X_I_np), X_I_np)
        check_equality(RigidTransform(pose=X_I_np[:3]), X_I_np)
        # - Cast.
        self.check_cast(mut.RigidTransform_, T)
        # - Accessors, mutators, and general methods.
        X = RigidTransform()
        X.set(R=R_I, p=p_I)
        X.SetFromIsometry3(pose=Isometry3.Identity())
        check_equality(RigidTransform.Identity(), X_I_np)
        self.assertIsInstance(X.rotation(), RotationMatrix)
        X.set_rotation(R=R_I)
        X.set_rotation(rpy=rpy_I)
        X.set_rotation(quaternion=quaternion_I)
        X.set_rotation(theta_lambda=angle_axis)
        self.assertIsInstance(X.translation(), np.ndarray)
        X.set_translation(p=np.zeros(3))
        numpy_compare.assert_float_equal(X.GetAsMatrix4(), X_I_np)
        numpy_compare.assert_float_equal(X.GetAsMatrix34(), X_I_np[:3])
        self.assertIsInstance(X.GetAsIsometry3(), Isometry3)
        check_equality(X.inverse(), X_I_np)
        self.assertIsInstance(
            X.multiply(other=RigidTransform()), RigidTransform)
        self.assertIsInstance(
            X.InvertAndCompose(other=RigidTransform()), RigidTransform)
        self.assertIsInstance(
            X.GetMaximumAbsoluteDifference(other=RigidTransform()), T)
        self.assertIsInstance(
            X.GetMaximumAbsoluteTranslationDifference(
                other=RigidTransform()), T)
        self.assertIsInstance(X @ RigidTransform(), RigidTransform)
        self.assertIsInstance(X @ [0, 0, 0], np.ndarray)
        if T != Expression:
            self.assertTrue(X.IsExactlyIdentity())
            self.assertTrue(X.IsNearlyIdentity(translation_tolerance=0))
            self.assertTrue(X.IsNearlyEqualTo(other=X, tolerance=0))
            self.assertTrue(X.IsExactlyEqualTo(other=X))
        # - Test shaping (#13885).
        v = np.array([0., 0., 0.])
        vs = np.array([[1., 2., 3.], [4., 5., 6.]]).T
        self.assertEqual((X @ v).shape, (3,))
        self.assertEqual((X @ v.reshape((3, 1))).shape, (3, 1))
        self.assertEqual((X @ vs).shape, (3, 2))
        # - Test 3-element vector multiplication.
        R_AB = RotationMatrix([
            [0., 1, 0],
            [-1, 0, 0],
            [0, 0, 1]])
        p_AB = np.array([1., 2, 3])
        X_AB = RigidTransform(R=R_AB, p=p_AB)
        p_BQ = [10, 20, 30]
        p_AQ = [21., -8, 33]
        numpy_compare.assert_float_equal(X_AB.multiply(p_BoQ_B=p_BQ), p_AQ)
        # - Test 4-element vector multiplication.
        p_BQ_vec4 = np.array([10, 20, 30, 1])
        p_AQ_vec4 = np.array([21., -8, 33, 1])
        numpy_compare.assert_float_equal(
            X_AB.multiply(vec_B=p_BQ_vec4), p_AQ_vec4)
        # N.B. Remember that this takes ndarray[3, n], NOT ndarray[n, 3]!
        p_BQlist = np.array([p_BQ, p_BQ]).T
        p_AQlist = np.array([p_AQ, p_AQ]).T
        numpy_compare.assert_float_equal(
            X_AB.multiply(p_BoQ_B=p_BQlist), p_AQlist)
        # - Repr.
        z = repr(T(0.0))
        i = repr(T(1.0))
        type_suffix = {
            float: "",
            AutoDiffXd: "_[AutoDiffXd]",
            Expression: "_[Expression]",
        }[T]
        self.assertEqual(repr(RigidTransform()), textwrap.dedent(f"""\
        RigidTransform{type_suffix}(
          R=RotationMatrix{type_suffix}([
            [{i}, {z}, {z}],
            [{z}, {i}, {z}],
            [{z}, {z}, {i}],
          ]),
          p=[{z}, {z}, {z}],
        )"""))
        if T == float:
            # TODO(jwnimmer-tri) Once AutoDiffXd and Expression implement an
            # eval-able repr, then we can test more than just T=float here.
            roundtrip = eval(repr(RigidTransform()))
            # TODO(jwnimmer-tri) Once IsExactlyEqualTo is bound, we can easily
            # check the contents of the roundtrip object here.
            self.assertIsInstance(roundtrip, RigidTransform)
        # Test pickling.
        assert_pickle(self, X_AB, RigidTransform.GetAsMatrix4, T=T)

    def test_legacy_unpickle(self):
        """Checks that data pickled as RotationMatrix_[float] in Drake v1.12.0
        can be unpickled as RotationMatrix_𝓣float𝓤 in newer versions of Drake.

        Since the unpickling shim lives at the module level, testing one class
        is sufficient even though our module has several pickle-able classes.
        """
        legacy_data = b"\x80\x04\x95\x18\x01\x00\x00\x00\x00\x00\x00\x8c\x0cpydrake.math\x94\x8c\x16RigidTransform_[float]\x94\x93\x94)\x81\x94\x8c\x15numpy.core.multiarray\x94\x8c\x0c_reconstruct\x94\x93\x94\x8c\x05numpy\x94\x8c\x07ndarray\x94\x93\x94K\x00\x85\x94C\x01b\x94\x87\x94R\x94(K\x01K\x03K\x04\x86\x94h\x07\x8c\x05dtype\x94\x93\x94\x8c\x02f8\x94\x89\x88\x87\x94R\x94(K\x03\x8c\x01<\x94NNNJ\xff\xff\xff\xffJ\xff\xff\xff\xffK\x00t\x94b\x88C`\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xf0\xbf\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xf0?\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xf0?\x00\x00\x00\x00\x00\x00\xf0?\x00\x00\x00\x00\x00\x00\x00@\x00\x00\x00\x00\x00\x00\x08@\x94t\x94bb."  # noqa
        obj = pickle.loads(legacy_data)
        self.assertIsInstance(obj, mut.RigidTransform_[float])
        expected = np.array([
            [0.0,  1.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0, 2.0],
            [0.0,  0.0, 1.0, 3.0],
        ])
        numpy_compare.assert_float_equal(obj.GetAsMatrix34(), expected)

    @numpy_compare.check_all_types
    def test_rotation_matrix(self, T):
        # - Constructors.
        RotationMatrix = mut.RotationMatrix_[T]
        AngleAxis = AngleAxis_[T]
        Quaternion = Quaternion_[T]
        RollPitchYaw = mut.RollPitchYaw_[T]

        R = RotationMatrix()
        numpy_compare.assert_float_equal(
                RotationMatrix(other=R).matrix(), np.eye(3))
        numpy_compare.assert_float_equal(R.matrix(), np.eye(3))
        numpy_compare.assert_float_equal(copy.copy(R).matrix(), np.eye(3))
        numpy_compare.assert_float_equal(
                RotationMatrix.Identity().matrix(), np.eye(3))
        R = RotationMatrix(R=np.eye(3))
        numpy_compare.assert_float_equal(R.matrix(), np.eye(3))
        R = RotationMatrix(quaternion=Quaternion.Identity())
        numpy_compare.assert_float_equal(R.matrix(), np.eye(3))
        R = RotationMatrix(theta_lambda=AngleAxis(angle=0, axis=[0, 0, 1]))
        numpy_compare.assert_float_equal(R.matrix(), np.eye(3))
        R = RotationMatrix(rpy=RollPitchYaw(rpy=[0, 0, 0]))
        numpy_compare.assert_float_equal(R.matrix(), np.eye(3))
        # One axis RotationMatrices
        R = RotationMatrix.MakeXRotation(theta=0)
        numpy_compare.assert_float_equal(R.matrix(), np.eye(3))
        R = RotationMatrix.MakeYRotation(theta=0)
        numpy_compare.assert_float_equal(R.matrix(), np.eye(3))
        R = RotationMatrix.MakeZRotation(theta=0)
        numpy_compare.assert_float_equal(R.matrix(), np.eye(3))
        # TODO(eric.cousineau): #11575, remove the conditional.
        if T == float:
            numpy_compare.assert_float_equal(R.row(index=0), [1., 0., 0.])
            numpy_compare.assert_float_equal(R.col(index=0), [1., 0., 0.])
            R = RotationMatrix.MakeFromOneVector(b_A=[1, 0, 0], axis_index=0)
            numpy_compare.assert_equal(R.IsValid(), True)
        R.set(R=np.eye(3))
        numpy_compare.assert_float_equal(R.matrix(), np.eye(3))
        # - Cast.
        self.check_cast(mut.RotationMatrix_, T)
        # - Conversion to RollPitchYaw
        roll_pitch_yaw = R.ToRollPitchYaw()
        self.assertIsInstance(roll_pitch_yaw, RollPitchYaw)
        # - Nontrivial quaternion.
        q = Quaternion(wxyz=[0.5, 0.5, 0.5, 0.5])
        R = RotationMatrix(quaternion=q)
        q_R = R.ToQuaternion()
        numpy_compare.assert_float_equal(
            q.wxyz(), numpy_compare.to_float(q_R.wxyz()))
        # - Conversion to AngleAxis
        angle_axis = R.ToAngleAxis()
        self.assertIsInstance(angle_axis, AngleAxis)
        R_AngleAxis = RotationMatrix(angle_axis)
        R_I = R.inverse().multiply(R_AngleAxis)
        numpy_compare.assert_equal(R_I.IsNearlyIdentity(), True)
        numpy_compare.assert_equal(R_I.IsNearlyIdentity(2E-15), True)
        R_I = R.InvertAndCompose(other=R_AngleAxis)
        numpy_compare.assert_equal(R_I.IsNearlyIdentity(2E-15), True)
        # - Inverse, transpose, projection
        R_I = R.inverse().multiply(R)
        numpy_compare.assert_float_equal(R_I.matrix(), np.eye(3))
        numpy_compare.assert_float_equal((R.inverse() @ R).matrix(), np.eye(3))
        R_T = R.transpose().multiply(R)
        numpy_compare.assert_float_equal(R_T.matrix(), np.eye(3))
        R_P = RotationMatrix.ProjectToRotationMatrix(M=2*np.eye(3))
        numpy_compare.assert_float_equal(R_P.matrix(), np.eye(3))
        # - Multiplication.
        R_AB = RotationMatrix([
            [0., 1, 0],
            [-1, 0, 0],
            [0, 0, 1]])
        v_B = [10, 20, 30]
        v_A = [20., -10., 30]
        numpy_compare.assert_float_equal(R_AB.multiply(v_B=v_B), v_A)
        # N.B. Remember that this takes ndarray[3, n], NOT ndarray[n, 3]!
        vlist_B = np.array([v_B, v_B]).T
        vlist_A = np.array([v_A, v_A]).T
        numpy_compare.assert_float_equal(R_AB.multiply(v_B=vlist_B), vlist_A)
        # - Test shaping (#13885).
        v = np.array([0., 0., 0.])
        vs = np.array([[1., 2., 3.], [4., 5., 6.]]).T
        self.assertEqual((R_AB @ v).shape, (3,))
        self.assertEqual((R_AB @ v.reshape((3, 1))).shape, (3, 1))
        self.assertEqual((R_AB @ vs).shape, (3, 2))
        # Matrix checks
        numpy_compare.assert_equal(R.IsValid(), True)
        R = RotationMatrix()
        numpy_compare.assert_equal(R.IsExactlyIdentity(), True)
        numpy_compare.assert_equal(R.IsNearlyIdentity(0.0), True)
        numpy_compare.assert_equal(R.IsNearlyIdentity(tolerance=1E-15), True)
        # - Repr.
        z = repr(T(0.0))  # "z" for zero
        i = repr(T(1.0))  # "i" for identity (one)
        t = repr(T(2.0))  # "t" for two
        type_suffix = {
            float: "",
            AutoDiffXd: "_[AutoDiffXd]",
            Expression: "_[Expression]",
        }[T]
        self.assertEqual(repr(RotationMatrix()), textwrap.dedent(f"""\
        RotationMatrix{type_suffix}([
          [{i}, {z}, {z}],
          [{z}, {i}, {z}],
          [{z}, {z}, {i}],
        ])"""))
        self.assertEqual(repr(RollPitchYaw(rpy=[2, 1, 0])),
                         f"RollPitchYaw{type_suffix}("
                         f"roll={t}, pitch={i}, yaw={z})")
        if T == float:
            # TODO(jwnimmer-tri) Once AutoDiffXd and Expression implement an
            # eval-able repr, then we can test more than just T=float here.
            roundtrip = eval(repr(RotationMatrix()))
            self.assertTrue(roundtrip.IsExactlyIdentity())
            roundtrip = eval(repr(RollPitchYaw(rpy=[2, 1, 0])))
            self.assertAlmostEqual(roundtrip.roll_angle(), 2)
            self.assertAlmostEqual(roundtrip.pitch_angle(), 1)
            self.assertAlmostEqual(roundtrip.yaw_angle(), 0)
        # Test pickling.
        assert_pickle(self, R_AB, RotationMatrix.matrix, T=T)

    @numpy_compare.check_all_types
    def test_roll_pitch_yaw(self, T):
        # - Constructors.
        RollPitchYaw = mut.RollPitchYaw_[T]
        RotationMatrix = mut.RotationMatrix_[T]
        Quaternion = Quaternion_[T]

        rpy = RollPitchYaw(rpy=[0, 0, 0])
        numpy_compare.assert_float_equal(
                RollPitchYaw(other=rpy).vector(), [0., 0., 0.])
        numpy_compare.assert_float_equal(rpy.vector(), [0., 0., 0.])
        rpy = RollPitchYaw(roll=0, pitch=0, yaw=0)
        numpy_compare.assert_float_equal([
            rpy.roll_angle(), rpy.pitch_angle(), rpy.yaw_angle()],
            [0., 0., 0.])
        rpy = RollPitchYaw(R=RotationMatrix())
        numpy_compare.assert_float_equal(rpy.vector(), [0., 0., 0.])
        rpy = RollPitchYaw(matrix=np.eye(3))
        numpy_compare.assert_float_equal(rpy.vector(), [0., 0., 0.])
        q_I = Quaternion()
        rpy_q_I = RollPitchYaw(quaternion=q_I)
        numpy_compare.assert_float_equal(rpy_q_I.vector(), [0., 0., 0.])
        # - Additional properties.
        numpy_compare.assert_float_equal(
                rpy.ToQuaternion().wxyz(), numpy_compare.to_float(q_I.wxyz()))
        R = rpy.ToRotationMatrix().matrix()
        numpy_compare.assert_float_equal(R, np.eye(3))
        # - Converting changes in orientation
        numpy_compare.assert_float_equal(rpy.CalcRotationMatrixDt(
            rpyDt=[0, 0, 0]), np.zeros((3, 3)))
        numpy_compare.assert_float_equal(
            rpy.CalcAngularVelocityInParentFromRpyDt(rpyDt=[0, 0, 0]),
            [0., 0., 0.])
        numpy_compare.assert_float_equal(
            rpy.CalcAngularVelocityInChildFromRpyDt(rpyDt=[0, 0, 0]),
            [0., 0., 0.])
        numpy_compare.assert_float_equal(
            rpy.CalcRpyDtFromAngularVelocityInParent(w_AD_A=[0, 0, 0]),
            [0., 0., 0.])
        numpy_compare.assert_float_equal(
            rpy.CalcRpyDtFromAngularVelocityInChild(w_AD_D=[0, 0, 0]),
            [0., 0., 0.])
        numpy_compare.assert_float_equal(
            rpy.CalcRpyDDtFromRpyDtAndAngularAccelInParent(
                rpyDt=[0, 0, 0], alpha_AD_A=[0, 0, 0]), [0., 0., 0.])
        numpy_compare.assert_float_equal(rpy.CalcRpyDDtFromAngularAccelInChild(
            rpyDt=[0, 0, 0], alpha_AD_D=[0, 0, 0]), [0., 0., 0.])
        # Test pickling.
        assert_pickle(self, rpy, RollPitchYaw.vector, T=T)

    @numpy_compare.check_all_types
    def test_bspline_basis(self, T):
        BsplineBasis = mut.BsplineBasis_[T]

        bspline = BsplineBasis()
        self.assertEqual(bspline.order(), 0)
        self.assertEqual(BsplineBasis(other=bspline).order(), 0)
        bspline = BsplineBasis(order=2, knots=[0, 1, 3, 5])
        self.assertEqual(bspline.order(), 2)
        bspline = BsplineBasis(order=2, num_basis_functions=3,
                               type=mut.KnotVectorType.kUniform,
                               initial_parameter_value=5.,
                               final_parameter_value=6.)
        self.assertEqual(bspline.order(), 2)
        self.assertEqual(bspline.degree(), 1)
        self.assertEqual(bspline.num_basis_functions(), 3)
        numpy_compare.assert_float_equal(bspline.knots(),
                                         [4.5, 5.0, 5.5, 6.0, 6.5])
        numpy_compare.assert_float_equal(bspline.initial_parameter_value(), 5.)
        numpy_compare.assert_float_equal(bspline.final_parameter_value(), 6.)
        self.assertEqual(
            bspline.FindContainingInterval(parameter_value=5.2), 1)
        self.assertEqual(
            bspline.ComputeActiveBasisFunctionIndices(
                parameter_interval=[5.2, 5.7]),
            [0, 1, 2])
        self.assertEqual(
            bspline.ComputeActiveBasisFunctionIndices(parameter_value=5.4),
            [0, 1])
        val = bspline.EvaluateCurve(control_points=[[1, 2], [2, 3], [3, 4]],
                                    parameter_value=5.7)
        self.assertEqual(val.shape, (2,))
        numpy_compare.assert_float_equal(
            bspline.EvaluateBasisFunctionI(i=0, parameter_value=5.7), 0.)
        assert_pickle(self, bspline, BsplineBasis.knots, T=T)

    @numpy_compare.check_all_types
    def test_wrap_to(self, T):
        value = wrap_to(T(1.5), T(0.), T(1.))
        if T != Expression:
            self.assertEqual(value, T(.5))

    @numpy_compare.check_nonsymbolic_types
    def test_soft_min_max(self, T):
        x = [T(1), T(2), T(3)]
        self.assertLess(mut.SoftUnderMax(x=x, alpha=1), T(3))
        self.assertGreater(mut.SoftOverMax(x=x, alpha=1), T(3))
        self.assertLess(mut.SoftUnderMin(x=x, alpha=1), T(1))
        self.assertGreater(mut.SoftOverMin(x=x, alpha=1), T(1))

    @numpy_compare.check_all_types
    def test_cross_product(self, T):
        p = np.array([T(1), T(2), T(3)])
        p_cross = mut.VectorToSkewSymmetric(p)
        self.assertEqual(p_cross.shape, (3, 3))

    @numpy_compare.check_all_types
    def test_quaternion(self, T):
        q1 = Quaternion_[T]()
        q2 = Quaternion_[T]()
        w = np.zeros(3)
        tolerance = 1e-4
        quat = mut.ClosestQuaternion(quat1=q1, quat2=q2)
        self.assertIsInstance(quat, Quaternion_[T])
        b = mut.is_quaternion_in_canonical_form(quat=q1)
        numpy_compare.assert_equal(b, True)
        quat = mut.QuaternionToCanonicalForm(quat=q1)
        self.assertIsInstance(quat, Quaternion_[T])
        b = mut.AreQuaternionsEqualForOrientation(quat1=q1,
                                                  quat2=q2,
                                                  tolerance=tolerance)
        numpy_compare.assert_equal(b, True)
        quatDt = mut.CalculateQuaternionDtFromAngularVelocityExpressedInB(
            quat_AB=q1, w_AB_B=w)
        numpy_compare.assert_float_equal(quatDt, np.zeros(4))
        w2 = mut.CalculateAngularVelocityExpressedInBFromQuaternionDt(
            quat_AB=q1, quatDt=quatDt)
        self.assertEqual(len(w2), 3)
        v = mut.CalculateQuaternionDtConstraintViolation(quat=q1,
                                                         quatDt=quatDt)
        self.assertIsInstance(v, T)
        b = mut.IsQuaternionValid(quat=q1, tolerance=tolerance)
        numpy_compare.assert_equal(b, True)
        b = mut.IsBothQuaternionAndQuaternionDtOK(quat=q1,
                                                  quatDt=quatDt,
                                                  tolerance=tolerance)
        numpy_compare.assert_equal(b, True)

    def test_random_rotations(self):
        g = RandomGenerator()
        quat = mut.UniformlyRandomQuaternion(g)
        self.assertIsInstance(quat, Quaternion_[float])
        angle_axis = mut.UniformlyRandomAngleAxis(g)
        self.assertIsInstance(angle_axis, AngleAxis_[float])
        rot_mat = mut.UniformlyRandomRotationMatrix(g)
        self.assertIsInstance(rot_mat, mut.RotationMatrix)
        rpy = mut.UniformlyRandomRPY(g)
        self.assertIsInstance(rpy, np.ndarray)
        self.assertEqual(len(rpy), 3)

    def test_matrix_util(self):
        A = np.array([[1, 2], [3, 4]])

        self.assertFalse(mut.IsSymmetric(matrix=A))
        self.assertFalse(mut.IsSymmetric(matrix=A, precision=0))
        self.assertTrue(mut.IsSymmetric(np.eye(3), 0.))

        self.assertFalse(mut.IsPositiveDefinite(matrix=A, tolerance=0))
        self.assertTrue(mut.IsPositiveDefinite(A.dot(A.T)))

        lower_triangular = np.array([1, 2, 3, 4, 5, 6.])
        symmetric_mat = mut.ToSymmetricMatrixFromLowerTriangularColumns(
            lower_triangular_columns=lower_triangular)
        np.testing.assert_array_equal(
            symmetric_mat, np.array([[1, 2, 3], [2, 4, 5], [3, 5, 6]]))

        lower_triangular2 = mut.ToLowerTriangularColumnsFromMatrix(
            matrix=symmetric_mat)
        np.testing.assert_array_equal(lower_triangular, lower_triangular2)

        minor_indices = {0, 2}
        minor = mut.ExtractPrincipalSubmatrix(matrix=symmetric_mat,
                                              indices=minor_indices)
        np.testing.assert_array_equal(
            minor,
            symmetric_mat[np.ix_(list(minor_indices), list(minor_indices))])

    def test_quadratic_form(self):
        Q = np.diag([1., 2., 3.])
        X = mut.DecomposePSDmatrixIntoXtransposeTimesX(Q, 1e-8)
        np.testing.assert_array_almost_equal(X, np.sqrt(Q))
        b = np.zeros(3)
        c = 4.
        R, d = mut.DecomposePositiveQuadraticForm(Q, b, c)
        self.assertEqual(np.size(R, 0), 4)
        self.assertEqual(np.size(R, 1), 3)
        self.assertEqual(len(d), 4)
        T = mut.BalanceQuadraticForms(S=np.eye(3), P=np.eye(3))
        np.testing.assert_array_almost_equal(T, np.eye(3))

    def test_riccati_lyapunov(self):
        A = 0.1*np.eye(2)
        B = np.eye(2)
        Q = np.eye(2)
        R = np.eye(2)

        mut.ContinuousAlgebraicRiccatiEquation(A=A, B=B, Q=Q, R=R)
        mut.RealContinuousLyapunovEquation(A=A, Q=Q)
        mut.RealDiscreteLyapunovEquation(A=A, Q=Q)

        A = np.array([[1, 1], [0, 1]])
        B = np.array([[0], [1]])
        Q = np.array([[1, 0], [0, 0]])
        R = [0.3]

        mut.DiscreteAlgebraicRiccatiEquation(A=A, B=B, Q=Q, R=R)

    def test_compute_numerical_gradient(self):
        option = mut.NumericalGradientOption(
            method=mut.NumericalGradientMethod.kCentral,
            function_accuracy=1E-15)

        self.assertIn("kCentral", repr(option))

        def foo(x):
            return np.array([x[0] ** 2, x[0] * x[1]])

        grad = mut.ComputeNumericalGradient(
            calc_func=foo, x=np.array([1., 2.]), option=option)
        np.testing.assert_allclose(
            grad, np.array([[2., 0.], [2., 1.]]), atol=1E-5)

    @numpy_compare.check_all_types
    def test_value_instantiations(self, T):
        # Existence checks.
        Value[mut.RigidTransform_[T]]
        Value[List[mut.RigidTransform_[T]]]
        Value[mut.RotationMatrix_[T]]
        Value[List[mut.RotationMatrix_[T]]]
