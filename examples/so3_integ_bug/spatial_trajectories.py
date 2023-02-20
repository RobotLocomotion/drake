import dataclasses as dc
import functools
import textwrap

import numpy as np

from pydrake.all import (
    AngleAxis,
    BasicVector_,
    BodyIndex,
    DiagramBuilder_,
    Diagram_,
    Evaluate,
    Expression,
    Jacobian,
    LeafSystem,
    LeafSystem_,
    MultibodyPlant_,
    Quaternion,
    Quaternion_,
    RigidTransform,
    RollPitchYaw_,
    RotationMatrix,
    RotationMatrix_,
    SpatialAcceleration,
    SpatialInertia,
    SpatialVelocity,
    TemplateSystem,
    UnitInertia,
    Value,
    Variable,
)
import pydrake.math as drake_math

from cc import QuaternionRate, hack_quaternion

try:
    import mujoco
except ImportError:
    mujoco = None

try:
    import pinocchio as pin
except ImportError:
    pin = None


# Vector and matrix manipulation.


def cat(*args):
    return np.concatenate(args)


def skew(r):
    r1, r2, r3 = r
    return np.array(
        [
            [0, -r3, r2],
            [r3, 0, -r1],
            [-r2, r1, 0],
        ]
    )


def unskew(R, *, tol=1e-10):
    if tol is not None:
        dR = R + R.T
        assert np.all(np.max(np.abs(dR)) < tol)
    r1 = R[2, 1]
    r2 = R[0, 2]
    r3 = R[1, 0]
    return np.array([r1, r2, r3])


def maxabs(x):
    return np.max(np.abs(x))


def make_spatial_inertia(M):
    assert M.shape == (6, 6)
    mass = M[3, 3]
    G_raw = M[:3, :3] / mass
    G_SP = UnitInertia(
        Ixx=G_raw[0, 0],
        Iyy=G_raw[1, 1],
        Izz=G_raw[2, 2],
        Ixy=G_raw[0, 1],
        Ixz=G_raw[0, 2],
        Iyz=G_raw[1, 2],
    )
    p_PScm = unskew(M[0:3, 3:] / mass)
    inertia = SpatialInertia(mass, p_PScm, G_SP)
    assert (inertia.CopyToFullMatrix6() == M).all()
    return inertia


def se3_diff(X_actual, X_des):
    dp = X_actual.translation() - X_des.translation()
    dR = X_des.rotation() @ X_actual.rotation().inverse()
    daxang = AngleAxis(dR.matrix())
    dax3 = daxang.axis() * daxang.angle()
    return dax3, dp


def angle_axis_deg_to_quat(angle_deg, ax):
    angle = np.deg2rad(angle_deg)
    ax = ax / np.linalg.norm(ax)
    return AngleAxis(angle, ax).quaternion().wxyz()


# System stuff.


def eval_port(port, parent_context, *, resolve=True):
    if resolve:
        context = port.get_system().GetMyContextFromRoot(parent_context)
    else:
        context = parent_context
    return port.Eval(context)


# Symbolics.


@np.vectorize
def to_float(x):
    if isinstance(x, float):
        return x
    elif isinstance(x, Expression):
        _, (c,) = x.Unapply()
        assert isinstance(c, float), type(c)
        return c
    else:
        assert False


def drake_sym_replace(expr, old, new):
    def recurse(sub):
        return drake_sym_replace(sub, old, new)

    if isinstance(expr, np.ndarray):
        recurse = np.vectorize(recurse)
        return recurse(expr)

    if not isinstance(expr, Expression):
        return expr
    if expr.EqualTo(old):
        return new
    ctor, old_args = expr.Unapply()
    new_args = [recurse(x) for x in old_args]
    return ctor(*new_args)


def differentiate(A, x):
    dx = np.vectorize(lambda ai: ai.Differentiate(x))
    return dx(A)


def derivatives_1st(A, xs, xds):
    Ad = np.zeros_like(A)
    for x, xd in zip(xs, xds):
        J = differentiate(A, x)
        Ad += J * xd
    return Ad


def derivatives_1st_and_2nd(A, xs, xds, xdds):
    Ad = derivatives_1st(A, xs, xds)
    vs = cat(xs, xds)
    vds = cat(xds, xdds)
    Add = derivatives_1st(Ad, vs, vds)
    return Ad, Add


def sym_values_and_1st_and_2nd_derivatives(names):
    x = [Variable(name) for name in names]
    xd = [Variable(f"{name}_dot") for name in names]
    xdd = [Variable(f"{name}_ddot") for name in names]
    return [np.array(s) for s in (x, xd, xdd)]


def aliased_scalar(x, i):
    return x[i : i + 1].reshape(())


def make_env(syms, xs=None):
    return_xs = False
    if xs is None:
        alias = True
        xs = tuple(np.zeros(len(sym_i)) for sym_i in syms)
        return_xs = True
    else:
        alias = False
    env = {}
    for sym, x in zip(syms, xs):
        for i, sym_i in enumerate(sym):
            if alias:
                xi = aliased_scalar(x, i)
            else:
                xi = x[i]
            env[sym_i] = xi
    if return_xs:
        return env, xs
    else:
        return env


# Rotation coordinates.


@dc.dataclass
class RotationInfo:
    num_rot: int
    # Unity representation (for offsetting, integrator, etc.).
    r0: np.ndarray
    # R(r), w(r, rd), a(r, rd, rdd)
    calc_values: object
    # J+ = dr/dw
    calc_rate_jacobian: object
    # Projects "raw" inputs (r_i, rd_i, rdd_i) onto proper subspace
    # (r, rd, rdd)
    project_values: object


def evaluate_sym(expr, env):
    for old, new in env.items():
        expr = drake_sym_replace(expr, old, new)
    return expr


def infer_sym_dtype_stuff(x):
    if x.dtype == object:
        T = Expression
        evaluate = evaluate_sym
        tol = None
    else:
        T = float
        evaluate = Evaluate
        tol = 1e-10
    return T, evaluate, tol


@functools.lru_cache
def make_rot_info_rpy_sym():
    """Makes RollPitchYaw rotation coordinate info."""
    syms = sym_values_and_1st_and_2nd_derivatives("rpy")
    r_s, rd_s, rdd_s = syms

    # Compute expressions.
    R_s = RollPitchYaw_[Expression](r_s).ToRotationMatrix().matrix()
    Rd_s, Rdd_s = derivatives_1st_and_2nd(R_s, r_s, rd_s, rdd_s)
    wh_s = R_s.T @ Rd_s
    ah_s = wh_s.T @ wh_s + R_s.T @ Rdd_s
    # ah = derivatives_1st(wh, cat(r, rd), cat(rd, rdd))
    w_s = unskew(wh_s, tol=None)
    J_s = Jacobian(w_s, rd_s)

    def calc_values(r_e, rd_e, rdd_e):
        T, evaluate, tol = infer_sym_dtype_stuff(r_e)
        env = make_env(syms, (r_e, rd_e, rdd_e))
        R_e = evaluate(R_s, env)
        wh_e = evaluate(wh_s, env)
        w_e = unskew(wh_e, tol=tol)
        ah_e = evaluate(ah_s, env)
        a_e = unskew(ah_e, tol=tol)
        return RotationMatrix_[T](R_e), w_e, a_e

    def project_values(r_e, rd_e, rdd_e):
        # For now, we're ignoring non-uniqueness of rpy.
        return r_e, rd_e, rdd_e

    def calc_angular_velocity_jacobian(r_e):
        T, evaluate, tol = infer_sym_dtype_stuff(r_e)
        env = make_env((r_s,), (r_e,))
        J_e = evaluate(J_s, env)
        return J_e

    return RotationInfo(
        num_rot=len(r_s),
        r0=np.zeros(3),
        calc_values=calc_values,
        calc_rate_jacobian=make_pinv(calc_angular_velocity_jacobian),
        project_values=project_values,
    )


@functools.lru_cache
def make_rot_info_quat_sym():
    """Makes Quaternion rotation coordinate info."""
    syms = sym_values_and_1st_and_2nd_derivatives(["w", "x", "y", "z"])
    q_s, qd_s, qdd_s = syms

    q_qd_s = cat(q_s, qd_s)
    qd_qdd_s = cat(qd_s, qdd_s)

    q_norm_squared_s = (q_s**2).sum()
    q_norm_s = np.sqrt(q_norm_squared_s)
    q_full_s = Quaternion_[Expression](wxyz=q_s)

    def remove_q_norm_unit(expr):
        # Assume norm(q)^2 == 1 for expression.
        return drake_sym_replace(expr, q_norm_squared_s, 1.0)

    # Derivatives along normalization (SO(3) projection).
    q_normed_s = q_s / q_norm_s
    qd_normed_s, qdd_normed_s = derivatives_1st_and_2nd(
        q_normed_s, q_s, qd_s, qdd_s
    )
    J_normed_s = Jacobian(q_normed_s, q_s)

    # Nominal SO(3) mapping.
    R_s = RotationMatrix_[Expression](q_full_s).matrix()
    R_s = remove_q_norm_unit(R_s)

    Rd_s, Rdd_s = derivatives_1st_and_2nd(R_s, q_s, qd_s, qdd_s)
    wh_s = R_s.T @ Rd_s
    w_s = unskew(wh_s, tol=None)
    J_s = Jacobian(w_s, qd_s)
    ah_s = derivatives_1st(wh_s, q_qd_s, qd_qdd_s)

    def calc_values(q_e, qd_e, qdd_e):
        T, evaluate, tol = infer_sym_dtype_stuff(q_e)
        env = make_env(syms, (q_e, qd_e, qdd_e))
        R_e = evaluate(R_s, env)
        wh_e = evaluate(wh_s, env)
        w_e = unskew(wh_e, tol=tol)
        ah_e = evaluate(ah_s, env)
        a_e = unskew(ah_e, tol=tol)
        return RotationMatrix_[T](R_e), w_e, a_e

    def project_values(q_e, qd_e, qdd_e):
        T, evaluate, tol = infer_sym_dtype_stuff(q_e)
        env = make_env(syms, (q_e, qd_e, qdd_e))
        # Normalize input values.
        # TODO(eric.cousineau): What about upper half-sphere?
        q_p = evaluate(q_normed_s, env).reshape((-1,))
        qd_p = evaluate(qd_normed_s, env).reshape((-1,))
        qdd_p = evaluate(qdd_normed_s, env).reshape((-1,))
        return q_p, qd_p, qdd_p

    def calc_angular_velocity_jacobian(q_e):
        T, evaluate, tol = infer_sym_dtype_stuff(q_e)
        env = make_env((q_s,), (q_e,))
        q_normed_e = evaluate(q_normed_s, env).reshape((-1,))
        J_normed_e = evaluate(J_normed_s, env)
        env = make_env((q_s,), (q_normed_e,))
        J_e = evaluate(J_s, env)
        return J_e @ J_normed_e

    return RotationInfo(
        num_rot=len(q_s),
        r0=np.array([1.0, 0.0, 0.0, 0.0]),
        calc_values=calc_values,
        project_values=project_values,
        calc_rate_jacobian=make_pinv(calc_angular_velocity_jacobian),
    )


def make_rot_info_quat_drake_jacobian():
    quat_info = make_rot_info_quat_sym()

    def calc_rate_jacobian(q):
        q = hack_quaternion(q)
        Jqd = QuaternionRate.AngularVelocityToQuaternionRateMatrix(q)
        return Jqd

    return RotationInfo(
        num_rot=quat_info.num_rot,
        r0=quat_info.r0,
        calc_values=quat_info.calc_values,
        project_values=quat_info.project_values,
        calc_rate_jacobian=calc_rate_jacobian,
    )



def calc_rotational_values(rot_info, r, rd, rdd):
    r, rd, rdd = rot_info.project_values(r, rd, rdd)
    R, w, wd = rot_info.calc_values(r, rd, rdd)
    return (R, w, wd), (r, rd, rdd)


num_ang_vel = 3
num_pos = 3
num_spatial = num_ang_vel + num_pos


def split_spatial(dX, num_rot=3):
    r = dX[:num_rot]
    p = dX[num_rot:]
    return r, p


def calc_spatial_values(rot_info, dX, dXd, dXdd):
    num_rot = rot_info.num_rot
    r, p = split_spatial(dX, num_rot)
    rd, pd = split_spatial(dXd, num_rot)
    rdd, pdd = split_spatial(dXdd, num_rot)
    r, rd, rdd = rot_info.project_values(r, rd, rdd)
    R, w, wd = rot_info.calc_values(r, rd, rdd)
    X = RigidTransform(R, p)
    V = cat(w, pd)
    A = cat(wd, pdd)
    dX = cat(r, p)
    dXd = cat(rd, pd)
    dXdd = cat(rdd, pdd)
    return (X, V, A), (dX, dXd, dXdd)


def make_rotation_2nd_order_integrator(rot_info):
    return SecondOrderIntegratorWithMapping(
        num_q=rot_info.num_rot,
        num_v=num_ang_vel,
        calc_dqd_dv=rot_info.calc_rate_jacobian,
        q0=rot_info.r0,
    )


def calc_rate_jacobian_spatial(dX, rot_info):
    num_rot = rot_info.num_rot
    r, p = split_spatial(dX, num_rot)
    Jr = rot_info.calc_rate_jacobian(r)
    num_q = num_rot + num_pos
    JdX = np.zeros_like(Jr, shape=(num_q, num_spatial))
    JdX[:num_rot, :num_ang_vel] = Jr
    JdX[num_rot:, num_ang_vel:] = np.eye(num_pos)
    return JdX


def make_spatial_2nd_order_integrator(rot_info, *, T=float):
    calc_dqd_dv = functools.partial(
        calc_rate_jacobian_spatial,
        rot_info=rot_info,
    )
    p0 = np.zeros(num_pos)
    dX0 = cat(rot_info.r0, p0)
    return SecondOrderIntegratorWithMapping_[T](
        num_q=rot_info.num_rot + num_pos,
        num_v=num_spatial,
        calc_dqd_dv=calc_dqd_dv,
        q0=dX0,
    )


# Reference trajectories.


def min_jerk(s):
    """
    Simple polynomial f(t) for minimum-jerk (zero velocity and acceleration at
    the start and end):

        s=0: f=0, f'=0, f''=0
        s>=1: f=1, f'=0, f''=0
    """
    # TODO(eric.cousineau): Use Drake's polynomial and/or symbolic stuff.
    s = np.clip(s, 0, 1)
    c = [0, 0, 0, 10, -15, 6]
    p = [1, s, s**2, s**3, s**4, s**5]
    pd = [0, 1, 2 * s, 3 * s**2, 4 * s**3, 5 * s**4]
    pdd = [0, 0, 2, 6 * s, 12 * s**2, 20 * s**3]
    f = np.dot(c, p)
    fd = np.dot(c, pd)
    fdd = np.dot(c, pdd)
    return f, fd, fdd


class SimpleRotationReference(LeafSystem):
    """
    rot_info - RotationInfo
    As - amplitudes
    Ts - periods (sec) for harmonics (sin, cos)
    T0_ratios - offsets (multiplied by 2 pi) for harmonics (sin, cos)
    """

    def __init__(
        self,
        rot_info,
        *,
        As,
        Ts,
        T0_ratios,
    ):
        super().__init__()

        num_r = rot_info.num_rot

        assert len(As) == num_r
        r0 = rot_info.r0

        def calc_instant(context):
            t = context.get_time()

            # Blending.
            s, sd, sdd = min_jerk(t)

            # Frequencies.
            ws = 2 * np.pi / Ts
            # Inputs.
            x = ws * t + 2 * np.pi * T0_ratios
            dx = ws
            # Harmonics.
            y = np.sin(x)
            yd = dx * np.cos(x)
            ydd = -dx * dx * np.sin(x)

            # Generate kinda arbitrary R^n trajectory, but then project to
            # proper subspace relating to SO(3) and related tangent spaces.
            r = s * As * y + r0
            rd = s * As * yd + sd * As * y
            rdd = s * As * ydd + 2 * (sd * As * yd) + sdd * As * y

            (R, w, wd), (r, rd, rdd) = calc_rotational_values(
                rot_info, r, rd, rdd
            )

            if t == 0:
                assert R.IsNearlyIdentity(1e-8)
                assert np.all(w == 0.0)
                assert np.all(wd == 0.0)

            return (R, w, wd), (r, rd, rdd)

        def calc_R(context, output):
            (R, w, wd), (r, rd, rdd) = calc_instant(context)
            output.set_value(R)

        def calc_w(context, output):
            (R, w, wd), (r, rd, rdd) = calc_instant(context)
            output.set_value(w)

        def calc_wd(context, output):
            (R, w, wd), (r, rd, rdd) = calc_instant(context)
            output.set_value(wd)

        self.R = self.DeclareAbstractOutputPort(
            "R", Value[RotationMatrix], calc_R
        )
        self.w = self.DeclareVectorOutputPort("w", num_ang_vel, calc_w)
        self.wd = self.DeclareVectorOutputPort("wd", num_ang_vel, calc_wd)

        def calc_r(context, output):
            (R, w, wd), (r, rd, rdd) = calc_instant(context)
            output.set_value(r)

        def calc_rd(context, output):
            (R, w, wd), (r, rd, rdd) = calc_instant(context)
            output.set_value(rd)

        def calc_rdd(context, output):
            (R, w, wd), (r, rd, rdd) = calc_instant(context)
            output.set_value(rdd)

        self.r = self.DeclareVectorOutputPort("r", num_r, calc_r)
        self.rd = self.DeclareVectorOutputPort("rd", num_r, calc_rd)
        self.rdd = self.DeclareVectorOutputPort("rdd", num_r, calc_rdd)


class SimpleSpatialReference(LeafSystem):
    """Similar setup as SimpleRotationReference."""

    def __init__(
        self,
        rot_info,
        *,
        As,
        Ts,
        T0_ratios,
    ):
        super().__init__()

        num_x = num_pos + rot_info.num_rot
        assert len(As) == num_x
        assert len(Ts) == num_x
        assert len(T0_ratios) == num_x
        r0 = rot_info.r0
        p0 = np.zeros(num_pos)
        dX0 = cat(r0, p0)

        def calc_instant(context):
            t = context.get_time()

            # Blending.
            s, sd, sdd = min_jerk(t)

            # Frequencies.
            ws = 2 * np.pi / Ts
            # Inputs.
            x = ws * t + 2 * np.pi * T0_ratios
            dx = ws

            # Harmonics.
            y = np.sin(x)
            yd = dx * np.cos(x)
            ydd = -dx * dx * np.sin(x)

            # Generate kinda arbitrary R^n trajectory, but then project to
            # proper subspace relating to SE(3) and related tangent spaces.
            dX = s * As * y + dX0
            dXd = s * As * yd + sd * As * y
            dXdd = s * As * ydd + 2 * (sd * As * yd) + sdd * As * y

            (X, V, A), (dX, dXd, dXdd) = calc_spatial_values(
                rot_info, dX, dXd, dXdd
            )
            if t == 0:
                assert X.IsNearlyIdentity(1e-8)
                assert np.all(V == 0.0)
                assert np.all(A == 0.0)
            V = SpatialVelocity(V)
            A = SpatialAcceleration(A)
            return (X, V, A), (dX, dXd, dXdd)

        def calc_X(context, output):
            (X, _, _), _ = calc_instant(context)
            output.set_value(X)

        def calc_V(context, output):
            (_, V, _), _ = calc_instant(context)
            output.set_value(V)

        def calc_A(context, output):
            (_, _, A), _ = calc_instant(context)
            output.set_value(A)

        self.X = self.DeclareAbstractOutputPort(
            "X", Value[RigidTransform], calc_X
        )
        self.V = self.DeclareAbstractOutputPort(
            "V", Value[SpatialVelocity], calc_V
        )
        self.A = self.DeclareAbstractOutputPort(
            "A", Value[SpatialAcceleration], calc_A
        )

        def calc_dX(context, output):
            _, (dX, _, _) = calc_instant(context)
            output.set_value(dX)

        def calc_dXd(context, output):
            _, (_, dXd, _) = calc_instant(context)
            output.set_value(dXd)

        def calc_dXdd(context, output):
            _, (_, _, dXdd) = calc_instant(context)
            output.set_value(dXdd)

        self.dX = self.DeclareVectorOutputPort("dX", num_x, calc_dX)
        self.dXd = self.DeclareVectorOutputPort("dXd", num_x, calc_dXd)
        self.dXdd = self.DeclareVectorOutputPort("dXdd", num_x, calc_dXdd)


# Integrators.


T_list = [float, Expression]


@TemplateSystem.define("SecondOrderIntegrator_", T_list=T_list)
def SecondOrderIntegrator_(T):
    class Impl(LeafSystem_[T]):
        """
        Given qdd, integrates to provide z = [q, xd]
        """

        def _construct(self, num_q, *, q0=None, qd0=None, converter=None):
            LeafSystem_[T].__init__(self, converter=converter)

            if q0 is None:
                q0 = np.zeros(num_q)
            if qd0 is None:
                qd0 = np.zeros(num_q)

            x0 = cat(q0, qd0)
            self.DeclareContinuousState(BasicVector_[T](x0))
            self.DeclareVectorInputPort("qdd", num_q)

            def calc_q(context, output):
                x = context.get_continuous_state_vector().CopyToVector()
                q = x[:num_q]
                output.set_value(q)

            def calc_qd(context, output):
                x = context.get_continuous_state_vector().CopyToVector()
                qd = x[num_q:]
                output.set_value(qd)

            def calc_x(context, output):
                x = context.get_continuous_state_vector().CopyToVector()
                output.set_value(x)

            prereqs = {self.all_state_ticket()}
            self.q = self.DeclareVectorOutputPort("q", num_q, calc_q, prereqs)
            self.qd = self.DeclareVectorOutputPort(
                "qd", num_q, calc_qd, prereqs
            )
            self.x = self.DeclareVectorOutputPort(
                "x", 2 * num_q, calc_x, prereqs
            )

            def calc_xd(context, derivatives):
                vector = derivatives.get_mutable_vector()
                x = context.get_continuous_state_vector().CopyToVector()
                qd = x[num_q:]
                qdd = self.get_input_port().Eval(context)
                xd = cat(qd, qdd)
                vector.set_value(xd)

            self.DoCalcTimeDerivatives = calc_xd

            def set_state(context, q, qd):
                x = cat(q, qd)
                context.SetContinuousState(x)

            self.set_state = set_state

        def _construct_copy(self, other, converter=None):
            raise NotImplementedError()

    return Impl


SecondOrderIntegrator = SecondOrderIntegrator_[None]


def pinv_raw(A):
    return A.T @ drake_math.inv(A @ A.T)


def pinv(A):
    if False:  # A.dtype == float:
        return np.linalg.pinv(A)
    else:
        return pinv_raw(A)


def make_pinv(calc):

    def calc_pinv(q):
        M = calc(q)
        Mpinv = pinv(M)
        return Mpinv

    return calc_pinv


@TemplateSystem.define("SecondOrderIntegratorWithMapping_", T_list=T_list)
def SecondOrderIntegratorWithMapping_(T):
    class Impl(LeafSystem_[T]):
        """
        Given vd, integrates to provide x = [q, v].

        Note that qd = N+(q) * v, where N+ = dqd/dv
        """

        def _construct(
            self, num_q, num_v, calc_dqd_dv, *, q0=None, v0=None, converter=None
        ):
            LeafSystem_[T].__init__(self, converter=converter)
            num_x = num_q + num_v

            if q0 is None:
                q0 = np.zeros(num_q)
            if v0 is None:
                v0 = np.zeros(num_v)
            x0 = cat(q0, v0)

            self.DeclareContinuousState(BasicVector_[T](x0))
            self.DeclareVectorInputPort("vd", num_v)

            def split(z):
                x = z[:num_q]
                y = z[num_q:]
                assert len(y) == num_v
                return x, y

            def calc_q(context, output):
                x = context.get_continuous_state_vector().CopyToVector()
                q, _ = split(x)
                output.set_value(q)

            def calc_v(context, output):
                x = context.get_continuous_state_vector().CopyToVector()
                _, v = split(x)
                output.set_value(v)

            def calc_x(context, output):
                x = context.get_continuous_state_vector().CopyToVector()
                output.set_value(x)

            prereqs = {self.all_state_ticket()}
            self.q = self.DeclareVectorOutputPort("q", num_q, calc_q, prereqs)
            self.v = self.DeclareVectorOutputPort("v", num_v, calc_v, prereqs)
            self.x = self.DeclareVectorOutputPort("x", num_x, calc_x, prereqs)

            def calc_xd(context, derivatives):
                vector = derivatives.get_mutable_vector()
                x = context.get_continuous_state_vector().CopyToVector()
                q, v = split(x)
                Npinv = calc_dqd_dv(q)
                qd = Npinv @ v
                vd = self.get_input_port().Eval(context)
                xd = cat(qd, vd)
                vector.set_value(xd)

            self.DoCalcTimeDerivatives = calc_xd

            def set_state(context, q, v):
                x = cat(q, v)
                context.SetContinuousState(x)

            self.set_state = set_state

        def _construct_copy(self, other, converter=None):
            raise NotImplementedError()

    return Impl


SecondOrderIntegratorWithMapping = SecondOrderIntegratorWithMapping_[None]


# Misc stuff.


@TemplateSystem.define("NaivePlant_", T_list=T_list)
def MatrixMultiply_(T):
    class Impl(LeafSystem_[T]):
        def _construct(self, A, *, converter=None):
            super().__init__(converter=converter)
            self._kwargs = dict(A=A)

            num_y, num_u = A.shape
            self.DeclareVectorInputPort("u", num_u)

            def calc_y(context, output):
                u = self.get_input_port().Eval(context)
                y = A @ u
                output.set_value(y)

            self.DeclareVectorOutputPort("y", num_y, calc_y)

        def _construct_copy(self, other, *, converter=None):
            self._construct(**other._kwargs, converter=converter)

    return Impl


class CalcSpatialValues(LeafSystem):
    def __init__(self, rot_info):
        super().__init__()
        num_x = rot_info.num_rot + num_pos

        def calc_instant(context):
            dX = self.dX.Eval(context)
            # A bit wasteful, but oh well.
            dXd = np.zeros(num_x)
            dXdd = np.zeros(num_x)
            (X, _, _), _ = calc_spatial_values(rot_info, dX, dXd, dXdd)
            V = SpatialVelocity(self.V_raw.Eval(context))
            return X, V

        self.dX = self.DeclareVectorInputPort("dX", num_x)
        self.V_raw = self.DeclareVectorInputPort("V_raw", num_spatial)

        def calc_X(context, output):
            X, _ = calc_instant(context)
            output.set_value(X)

        def calc_V(context, output):
            _, V = calc_instant(context)
            output.set_value(V)

        self.X = self.DeclareAbstractOutputPort(
            "X", Value[RigidTransform], calc_X
        )
        self.V = self.DeclareAbstractOutputPort(
            "V", Value[SpatialVelocity], calc_V
        )


class CalcPlantSpatialValues(LeafSystem):
    def __init__(self, plant):
        super().__init__()
        context = plant.CreateDefaultContext()
        num_x = plant.num_positions() + plant.num_velocities()
        assert num_x == 13
        self.plant_state_input = self.DeclareVectorInputPort("x", num_x)
        assert plant.num_bodies() == 2
        body = plant.get_body(BodyIndex(1))

        def calc_instant(sys_context):
            x = self.plant_state_input.Eval(sys_context)
            plant.SetPositionsAndVelocities(context, x)
            X = plant.EvalBodyPoseInWorld(context, body)
            V = plant.EvalBodySpatialVelocityInWorld(context, body)
            return X, V

        def calc_X(sys_context, output):
            X, _ = calc_instant(sys_context)
            output.set_value(X)

        def calc_V(sys_context, output):
            _, V = calc_instant(sys_context)
            output.set_value(V)

        self.X = self.DeclareAbstractOutputPort(
            "X", Value[RigidTransform], calc_X
        )
        self.V = self.DeclareAbstractOutputPort(
            "V", Value[SpatialVelocity], calc_V
        )


def is_axisymmetric(M):
    assert M.shape == (6, 6)
    M_r = M[:3, :3]
    rot = np.diag(M_r)
    return (np.diag(rot) == M_r).all() and (rot == rot[0]).all()


# Simple plant.


@TemplateSystem.define("NaiveForceToAccel_", T_list=T_list)
def NaiveForceToAccel_(T):
    class Impl(Diagram_[T]):
        def _construct(self, M, *, converter=None):
            super().__init__()
            # M should be axisymmetric s.t. it is invariant of rotation.
            # TODO(eric.cousineau): Pipe in rotation matrix?
            assert is_axisymmetric(M)
            Minv = np.linalg.inv(M)
            builder = DiagramBuilder_[T]()
            force_to_accel = builder.AddSystem(MatrixMultiply_[T](Minv))
            # Junk so that we can export an input port.
            num_x = 13
            x_empty = np.zeros((0, num_x))
            x_sink = builder.AddSystem(MatrixMultiply_[T](x_empty))
            x_index = builder.ExportInput(x_sink.get_input_port())
            u_index = builder.ExportInput(force_to_accel.get_input_port())
            vd_index = builder.ExportOutput(force_to_accel.get_output_port())
            builder.BuildInto(self)
            self.x = self.get_input_port(x_index)
            self.u = self.get_input_port(u_index)
            self.vd = self.get_output_port(vd_index)

        def _construct_copy(self, other, *, converter=None):
            raise NotImplementedError()

    return Impl


NaiveForceToAccel = NaiveForceToAccel_[None]


def make_plant(M, *, T=float):
    plant = MultibodyPlant_[T](time_step=0.0)
    plant.mutable_gravity_field().set_gravity_vector(np.zeros(3))
    plant.AddRigidBody("body", make_spatial_inertia(M))
    plant.Finalize()
    return plant


class MbpForceToAccel(LeafSystem):
    def __init__(self, M):
        super().__init__()
        plant = make_plant(M)
        plant_context = plant.CreateDefaultContext()

        self.x = self.DeclareVectorInputPort("x", 13)
        self.u = self.DeclareVectorInputPort("u", 6)

        def calc_vd(context, output):
            x = self.x.Eval(context)
            u = self.u.Eval(context)
            plant.SetPositionsAndVelocities(plant_context, x)
            plant.get_applied_generalized_force_input_port().FixValue(
                plant_context, u
            )
            xd = plant.EvalTimeDerivatives(plant_context).CopyToVector()
            vd = xd[7:]
            output.set_value(vd)

        self.vd = self.DeclareVectorOutputPort("vd", 6, calc_vd)


def spatial_drake_to_mujoco(x):
    # (rot, pos) to (pos, rot)
    num_rot = len(x) - 3
    rot = x[:num_rot]
    pos = x[num_rot:]
    return cat(pos, rot)


def spatial_mujoco_to_drake(xm):
    # (pos, rot) to (rot, pos)
    pos = xm[:3]
    rot = xm[3:]
    return cat(rot, pos)


class MujocoForceToAccel(LeafSystem):
    def __init__(self, M):
        super().__init__()
        assert (M == np.eye(6)).all()
        xml = textwrap.dedent(r"""
        <mujoco>
          <option gravity="0 0 0"/>
          <worldbody>
            <body>
              <inertial mass="1" diaginertia="1 1 1" pos="0 0 0"/>
              <freejoint/>
            </body>
          </worldbody>
        </mujoco>
        """.lstrip())
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        mujoco.mj_resetData(model, data)

        self.x = self.DeclareVectorInputPort("x", 13)
        self.u = self.DeclareVectorInputPort("u", 6)

        def calc_vd(context, output):
            x = self.x.Eval(context)
            u = self.u.Eval(context)
            q = x[:7]
            v = x[7:]
            # N.B. Drake does (rot, pos); MuJoCo does (pos, rot).
            data.qpos = spatial_drake_to_mujoco(q)
            data.qvel = spatial_drake_to_mujoco(v)
            data.qfrc_applied = spatial_drake_to_mujoco(u)
            mujoco.mj_forward(model, data)
            vd = spatial_mujoco_to_drake(data.qacc)
            output.set_value(vd)

        self.vd = self.DeclareVectorOutputPort("vd", 6, calc_vd)


class PinnochioForceToAccel(LeafSystem):
    def __init__(self, M):
        assert False


@TemplateSystem.define("NaivePlant_", T_list=T_list)
def NaivePlant_(T):
    class Impl(Diagram_[T]):
        """
        Represents a plant that is only a single floating body, axisymmetric
        rotational inertia, with "identity" initial state.

        No gravity or Coriolis forces are present. Only inertia should play a
        role.

        Spatial coordinates are composed of rotation and positional
        coordinates. Rotation coordinates are as specified by `rot_info`.
        """

        def _construct(self, rot_info, force_to_accel, *, converter=None):
            # assert converter is None
            super().__init__()

            builder = DiagramBuilder_[T]()
            integ = builder.AddSystem(
                make_spatial_2nd_order_integrator(rot_info, T=T)
            )
            builder.AddSystem(force_to_accel)
            if rot_info.num_rot == 4:
                builder.Connect(integ.x, force_to_accel.x)
            builder.Connect(force_to_accel.vd, integ.get_input_port())

            if T == float:
                calc_spatial = builder.AddSystem(CalcSpatialValues(rot_info))
                builder.Connect(integ.q, calc_spatial.dX)
                builder.Connect(integ.v, calc_spatial.V_raw)
                X_index = builder.ExportOutput(calc_spatial.X, "X")
                V_index = builder.ExportOutput(calc_spatial.V, "V")

            u_index = builder.ExportInput(force_to_accel.u, "u")
            state_index = builder.ExportOutput(integ.x, "state")
            builder.BuildInto(self)
            self.generalized_forces_input = self.get_input_port(u_index)
            self.state = self.get_output_port(state_index)

            if T == float:
                self.X = self.get_output_port(X_index)
                self.V = self.get_output_port(V_index)

            def set_state(context, q, v):
                integ_context = integ.GetMyContextFromRoot(context)
                integ.set_state(integ_context, q, v)

            self.set_state = set_state

        def _construct_copy(self, other, *, converter=None):
            raise NotImplementedError()

    return Impl


NaivePlant = NaivePlant_[None]


@TemplateSystem.define("MbpPlant_", T_list=T_list)
def MbpPlant_(T):
    class Impl(Diagram_[T]):
        def _construct(self, M, *, converter=None):
            # assert converter is None
            super().__init__()

            builder = DiagramBuilder_[T]()
            plant = builder.AddSystem(make_plant(M, T=T))

            if T == float:
                calc_spatial = builder.AddSystem(CalcPlantSpatialValues(plant))
                builder.Connect(
                    plant.get_state_output_port(),
                    calc_spatial.plant_state_input,
                )
                X_index = builder.ExportOutput(calc_spatial.X, "X")
                V_index = builder.ExportOutput(calc_spatial.V, "V")

            u_index = builder.ExportInput(
                plant.get_applied_generalized_force_input_port(), "u"
            )
            state_index = builder.ExportOutput(
                plant.get_state_output_port(), "state"
            )
            builder.BuildInto(self)

            self.generalized_forces_input = self.get_input_port(u_index)
            self.state = self.get_output_port(state_index)
            if T == float:
                self.X = self.get_output_port(X_index)
                self.V = self.get_output_port(V_index)

            def set_state(context, q, v):
                plant_context = plant.GetMyContextFromRoot(context)
                plant.SetPositions(plant_context, q)
                plant.SetVelocities(plant_context, v)

            self.set_state = set_state

        def _construct_copy(self, other, *, converter=None):
            raise NotImplementedError()

    return Impl


MbpPlant = MbpPlant_[None]


class NaiveFeedforward(LeafSystem):
    def __init__(self, M):
        super().__init__()
        assert is_axisymmetric(M)

        self.A_des = self.DeclareAbstractInputPort(
            "A_des", Value[SpatialAcceleration]()
        )

        def calc_u(context, output):
            A_des = self.A_des.Eval(context)
            vd_des = A_des.get_coeffs()
            u = M @ vd_des
            output.set_value(u)

        self.generalized_forces_output = self.DeclareVectorOutputPort(
            "u",
            size=num_spatial,
            calc=calc_u,
        )
