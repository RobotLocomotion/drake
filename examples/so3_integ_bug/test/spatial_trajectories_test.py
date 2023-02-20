"""
Shows possible bug in MultibodyPlant dynamics by integrating a
SE(3) trajectory that has a non-constant axis of rotation.
"""

import unittest
import warnings

import numpy as np

from pydrake.all import (
    ApplySimulatorConfig,
    ConstantVectorSource_,
    DiagramBuilder,
    DiagramBuilder_,
    Expression,
    Simulator,
    SimulatorConfig,
    Variable,
)

from spatial_trajectories import (
    MbpForceToAccel,
    MbpPlant,
    MbpPlant_,
    MujocoForceToAccel,
    NaiveFeedforward,
    NaiveForceToAccel,
    NaiveForceToAccel_,
    NaivePlant,
    NaivePlant_,
    SecondOrderIntegrator,
    SimpleRotationReference,
    SimpleSpatialReference,
    angle_axis_deg_to_quat,
    cat,
    drake_sym_replace,
    eval_port,
    make_rot_info_quat_drake_jacobian,
    make_rot_info_quat_sym,
    make_rot_info_rpy_sym,
    make_rotation_2nd_order_integrator,
    maxabs,
    mujoco,
    num_spatial,
    se3_diff,
    to_float,
)


def make_sample_rotation_reference(use_rpy=True):
    """
    Sample SO(3) reference where the axis of rotation should be time-varying.
    """
    if use_rpy:
        rot_info = make_rot_info_rpy_sym()
        As = np.array([0.6, 0.4, 0.4])
        Ts = np.array([6.0, 6.0, 12.0])
        T0_ratios = np.array([0, 0.25, 0])
    else:
        rot_info = make_rot_info_quat_sym()
        As = np.array([0.1, 0.5, 1.0, 2.0])
        Ts = np.array([3.0, 1.0, 6.0, 9.0])
        T0_ratios = np.array([0.0, 0.25, 0.0, 0.0])
    reference = SimpleRotationReference(
        rot_info,
        As=As,
        Ts=Ts,
        T0_ratios=T0_ratios,
    )
    return reference, rot_info


def make_identity_inertia_matrix():
    return np.eye(6)


def make_sample_spatial_reference(use_rpy=True):
    """
    Sample spatial reference where all subspaces of SE(3) should have non-zero
    acceleration at some point.
    """
    # N.B. The translation portions of this don't really matter.
    As_p = [0.5, 0.6, 0.7]
    Ts_p = [1.0, 2.0, 3.0]
    T0_ratios_p = [0.0, 0.0, 0.0]
    # N.B. If you change any of the rotation amplitiudes (As_r) to only change
    # along one axis, then it should be fine for the MbP case.
    if use_rpy:
        rot_info = make_rot_info_rpy_sym()
        As_r = [0.1, 0.2, 0.3]
        # As_r = [0.0, 0.2, 0.0]  # This can make MbP integration work.
        Ts_r = [3.0, 6.0, 9.0]
        T0_ratios_r = [0.0, 0.0, 0.0]
    else:
        rot_info = make_rot_info_quat_sym()
        As_r = [0.1, 0.5, 1.0, 2.0]
        # As_r = [0.0, 0.0, 0, 0.5]  # This can make MbP integration work.
        Ts_r = [3.0, 1.0, 6.0, 9.0]
        T0_ratios_r = [0.0, 0.25, 0.0, 0.0]

    reference = SimpleSpatialReference(
        rot_info,
        As=cat(As_r, As_p),
        Ts=cat(Ts_r, Ts_p),
        T0_ratios=cat(T0_ratios_r, T0_ratios_p),
    )
    return reference, rot_info


def assert_equal(a, b):
    np.testing.assert_equal(a, b)


def assert_allclose(a, b, *, tol):
    np.testing.assert_allclose(a, b, rtol=0, atol=tol)


class Test(unittest.TestCase):
    def setUp(self):
        warnings.simplefilter("ignore", RuntimeWarning)

    def test_rotation_integration(self):
        self.check_rotation_integration(use_rpy=True)
        self.check_rotation_integration(use_rpy=False)

    def test_rotation_integration_drake(self):
        # Fails, even with large tolerance.
        with self.assertRaises(AssertionError):
            self.check_rotation_integration(
                use_rpy=False, use_drake=True, tol=0.1
            )

    def check_rotation_integration(
        self, *, use_rpy, use_drake=False, tol=1e-5, accuracy=1e-6,
    ):
        """
        With a given SO(3) trajectory through time, take the angular
        acceleration, integrate it forward, and ensure we recover the original
        trajectory with our given coordinate representation.
        """
        reference, rot_info = make_sample_rotation_reference(use_rpy)
        if use_drake:
            assert not use_rpy
            rot_info = make_rot_info_quat_drake_jacobian()

        num_r = rot_info.num_rot

        builder = DiagramBuilder()
        builder.AddSystem(reference)

        integ = builder.AddSystem(
            SecondOrderIntegrator(num_q=num_r, q0=rot_info.r0)
        )
        builder.Connect(reference.rdd, integ.get_input_port())

        integ_map = builder.AddSystem(
            make_rotation_2nd_order_integrator(rot_info)
        )
        builder.Connect(reference.wd, integ_map.get_input_port())
        diagram = builder.Build()

        simulator = Simulator(diagram)
        config = SimulatorConfig(accuracy=accuracy)
        ApplySimulatorConfig(config, simulator)

        def monitor(diagram_context):
            # Original.
            r = eval_port(reference.r, diagram_context)
            rd = eval_port(reference.rd, diagram_context)
            w = eval_port(reference.w, diagram_context)
            # Integrated w/ no mapping (Euclidean).
            ri = eval_port(integ.q, diagram_context)
            rdi = eval_port(integ.qd, diagram_context)
            # Integrated w/ mapping (maybe non-Euclidean)
            rm = eval_port(integ_map.q, diagram_context)
            wm = eval_port(integ_map.v, diagram_context)
            # Check integrated.
            self.assertLess(maxabs(r - ri), tol)
            self.assertLess(maxabs(rd - rdi), tol)
            # Check integration-via-map.
            self.assertLess(maxabs(r - rm), tol)
            self.assertLess(maxabs(w - wm), tol)

        # Initialize positions.
        simulator.Initialize()
        simulator.set_monitor(monitor)

        print(f"Check rotation integration w/ use_rpy={use_rpy}...")
        try:
            simulator.AdvanceTo(3.0)
        except AssertionError:
            t = simulator.get_context().get_time()
            print(f"Error at time t={t}s")
            raise
        print("  Done")

    def test_floating_tracking_naive(self):
        """
        Check integration of spatial acceleration into an SE(3) trajectory
        by using a unit-inertia body and providing spatial forces which are
        equal to the desired accelerations.
        """
        self.check_floating_tracking(use_rpy=True, mode="naive")
        self.check_floating_tracking(use_rpy=False, mode="naive")

    def test_floating_tracking_mbp(self):
        """
        Same as above, but use MultibodyPlant.
        """
        # WARNING: This currently fails :(
        with self.assertRaises(AssertionError):
            self.check_floating_tracking(use_rpy=False, mode="full_mbp")

    def test_floating_tracking_naive_mbp(self):
        self.check_floating_tracking(use_rpy=False, mode="naive_mbp")

    @unittest.skipIf(mujoco is None, "no mujoco")
    def test_floating_tracking_naive_mujoco(self):
        self.check_floating_tracking(use_rpy=False, mode="naive_mujoco")

    def check_floating_tracking(
        self,
        *,
        use_rpy,
        mode,
        do_print=False,
    ):
        print(f"Check use_rpy={use_rpy}, mode={mode}...")
        max_tol = 1e-4

        M = make_identity_inertia_matrix()
        reference, rot_info = make_sample_spatial_reference(use_rpy=use_rpy)

        builder = DiagramBuilder()

        if mode == "full_mbp":
            plant = MbpPlant(M)
        elif mode == "naive":
            plant = NaivePlant(rot_info, NaiveForceToAccel(M))
        elif mode == "naive_mbp":
            plant = NaivePlant(rot_info, MbpForceToAccel(M))
        elif mode == "naive_mujoco":
            plant = NaivePlant(rot_info, MujocoForceToAccel(M))
        else:
            assert False
        builder.AddSystem(plant)

        controller = builder.AddSystem(NaiveFeedforward(M))
        builder.Connect(
            controller.generalized_forces_output,
            plant.generalized_forces_input,
        )

        builder.AddSystem(reference)

        builder.Connect(
            reference.A,
            controller.A_des,
        )

        diagram = builder.Build()

        def monitor(diagram_context):
            X_actual = eval_port(plant.X, diagram_context)
            V_actual = eval_port(plant.V, diagram_context)
            X_des = eval_port(reference.X, diagram_context)
            V_des = eval_port(reference.V, diagram_context)

            ax3_err, p_err = se3_diff(X_actual, X_des)
            V_err = V_actual - V_des
            w_err = V_err.rotational()
            v_err = V_err.translational()

            t = diagram_context.get_time()
            if do_print:
                print(t)
                print(ax3_err, p_err)
                print(w_err, v_err)
                print()
            self.assertLess(maxabs(ax3_err), max_tol)
            self.assertLess(maxabs(p_err), max_tol)
            self.assertLess(maxabs(w_err), max_tol)
            self.assertLess(maxabs(v_err), max_tol)

        simulator = Simulator(diagram)
        simulator.set_monitor(monitor)
        simulator.Initialize()
        simulator.AdvanceTo(3.0)
        print("  Done")

    def test_symbolic_difference(self):
        """
        Show the difference in time derivatives for custom "naive" floating
        body integrator vs. MultibodyPlant.
        """
        M = make_identity_inertia_matrix()
        # u = np.array([Variable(f"u{i}") for i in range(num_spatial)])
        u = np.zeros(num_spatial)
        rot_info = make_rot_info_quat_sym()

        T = Expression
        # T = float
        builder = DiagramBuilder_[T]()
        u_sys = builder.AddSystem(ConstantVectorSource_[T](u))
        u_port = u_sys.get_output_port()

        # MultibodyPlant based floating body plant.
        mbp = builder.AddSystem(MbpPlant_[T](M))
        builder.Connect(u_port, mbp.generalized_forces_input)

        # Naive floating body plant.
        force_to_accel = NaiveForceToAccel_[T](M)
        naive = builder.AddSystem(NaivePlant_[T](rot_info, force_to_accel))
        builder.Connect(u_port, naive.generalized_forces_input)

        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        mbp_context = mbp.GetMyContextFromRoot(context)
        naive_context = naive.GetMyContextFromRoot(context)

        def calc_derivs(q0, v0):
            mbp.set_state(context, q0, v0)
            mbp_dx = mbp.EvalTimeDerivatives(mbp_context).CopyToVector()
            naive.set_state(context, q0, v0)
            naive_dx = naive.EvalTimeDerivatives(naive_context).CopyToVector()
            return mbp_dx, naive_dx

        def calc_quat_dot_diff(q0, v0):
            mbp_dx, naive_dx = calc_derivs(q0, v0)
            diff_dx = to_float(mbp_dx - naive_dx)
            # Diff should only be in Jacobian mapping for quaternion rate.
            assert (diff_dx[4:] == 0).all()
            quat_dot_diff = diff_dx[:4]
            print(quat_dot_diff)
            return quat_dot_diff

        q0 = np.zeros(7, dtype=T)
        q0[0] = 1.0
        v0 = np.zeros(6, dtype=T)

        tol = 1e-15

        # Nominal should be matched.
        quat_dot_diff = calc_quat_dot_diff(q0, v0)
        assert_equal(quat_dot_diff, 0.0)

        # Any "crazy" angular velocity at identity rotation should be fine.
        q0[:4] = angle_axis_deg_to_quat(0, [0, 0, 1])
        v0[:3] = [0.3, 0.4, 0.5]
        quat_dot_diff = calc_quat_dot_diff(q0, v0)
        assert_equal(quat_dot_diff, 0.0)

        # Angular velocity on same axis as rotation (I believe indicating
        # constant axis of rotation) should be fine.
        q0[:4] = angle_axis_deg_to_quat(90, [0, 0, 1])
        v0[:3] = [0, 0, 1]
        quat_dot_diff = calc_quat_dot_diff(q0, v0)
        assert_allclose(quat_dot_diff, 0.0, tol=tol)

        q0[:4] = angle_axis_deg_to_quat(65, [0, 1, 1])
        v0[:3] = [0, 1, 1]
        quat_dot_diff = calc_quat_dot_diff(q0, v0)
        assert_allclose(quat_dot_diff, 0.0, tol=tol)

        # However, axis of rotation with a misalined axis of angular velocity
        # (perhaps relating non-constant axis) shows a diff.
        q0[:4] = angle_axis_deg_to_quat(90, [0, 0, 1])
        v0[:3] = [0, 0.1, 1]
        quat_dot_diff = calc_quat_dot_diff(q0, v0)
        s2i = 1 / np.sqrt(2)
        assert_allclose(quat_dot_diff, [0, v0[1] * s2i, 0, 0], tol=tol)
        print(f"Difference!")

        # Look at dem symbolics.
        v0[:3] = [0, Variable("a"), 1]
        mbp_dx, naive_dx = calc_derivs(q0, v0)
        diff_dx = mbp_dx - naive_dx
        print(diff_dx[:4])
        print(mbp_dx[1])
        print(naive_dx[1])

        # Moar symbolics.
        q0[:4] = [Variable(n) for n in "wxyz"]
        quat = q0[:4]
        mbp_dx, naive_dx = calc_derivs(q0, v0)
        diff_dx = mbp_dx - naive_dx
        # # Not legible :(
        # bad = diff_dx[1]
        # bad = drake_sym_replace(bad, np.sum(quat**2), 1.0)
        # print(bad)
