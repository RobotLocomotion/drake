#pragma once

// GENERATED FILE DO NOT EDIT
// This file contains docstrings for the Python bindings that were
// automatically extracted by mkdoc.py.

#include <array>
#include <utility>

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

// #include "drake/multibody/benchmarks/free_body/free_body.h"

// Symbol: pydrake_doc_multibody_benchmarks_free_body
constexpr struct /* pydrake_doc_multibody_benchmarks_free_body */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::multibody
    struct /* multibody */ {
      // Symbol: drake::multibody::benchmarks
      struct /* benchmarks */ {
        // Symbol: drake::multibody::benchmarks::free_body
        struct /* free_body */ {
          // Symbol: drake::multibody::benchmarks::free_body::FreeBody
          struct /* FreeBody */ {
            // Source: drake/multibody/benchmarks/free_body/free_body.h
            const char* doc =
R"""(The purpose of the FreeBody class is to provide the data (initial
values and gravity) and methods for calculating the exact analytical
solution for the translational and rotational motion of a torque-free
rigid body B with axially symmetric inertia, in a Newtonian frame
(World) N. Examples of bodies with axially symmetric inertia include
cylinders, rods or bars with a circular or square cross section and
spinning tops. Since the only external forces on B are uniform
gravitational forces, there exists an exact closed-form analytical
solution for B's motion. The closed- form rotational solution is
available since B is "torque-free", i.e., the moment of all forces
about B's mass center is zero. This class calculates the body B's
quaternion, angular velocity and angular acceleration expressed in B
(body-frame) as well as the position, velocity, acceleration of Bcm
(B's center of mass) in N (World). Algorithm from [Kane, 1983]
Sections 1.13 and 3.1, Pages 60-62 and 159-169.

- [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983
  (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
  https://ecommons.cornell.edu/handle/1813/637)""";
            // Symbol: drake::multibody::benchmarks::free_body::FreeBody::CalcAngularRates_s_p
            struct /* CalcAngularRates_s_p */ {
              // Source: drake/multibody/benchmarks/free_body/free_body.h
              const char* doc =
R"""(Returns angular rates associated with spin ``s`` and precession ``p``
from the analytical solution [Kane, 1983] for rotational motion
(angular velocity and quaternion) for torque-free motion of an
axis-symmetric rigid body B in a Newtonian frame (World). Kane's
solution for B's angular velocity ``wx*Bx + wy*By + wz*Bz`` is in
terms of initial values wx0, wy0, wz0 as wx = wx0 * cos(s * t) + wy0 *
sin(s * t) wy = -wx0 * sin(s * t) + wy0 * cos(s * t) wz = wz0 For more
information, see [Kane, 1983] Pages 60-62 and 159-169.

Note:
    the return value of ``s`` may be negative, zero, or positive,
    whereas the return value of ``p`` is nonnegative. The values of
    ``s`` and ``p`` are returned in units of radian/second.

- [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York,
  1983 (with P. W. Likins and D. A. Levinson).  Available for free .pdf
  download: https://ecommons.cornell.edu/handle/1813/637)""";
            } CalcAngularRates_s_p;
            // Symbol: drake::multibody::benchmarks::free_body::FreeBody::CalcInitial_v_NBcm_N
            struct /* CalcInitial_v_NBcm_N */ {
              // Source: drake/multibody/benchmarks/free_body/free_body.h
              const char* doc = R"""()""";
            } CalcInitial_v_NBcm_N;
            // Symbol: drake::multibody::benchmarks::free_body::FreeBody::CalcInitial_w_NB_N
            struct /* CalcInitial_w_NB_N */ {
              // Source: drake/multibody/benchmarks/free_body/free_body.h
              const char* doc = R"""()""";
            } CalcInitial_w_NB_N;
            // Symbol: drake::multibody::benchmarks::free_body::FreeBody::CalculateExactRotationalSolutionNB
            struct /* CalculateExactRotationalSolutionNB */ {
              // Source: drake/multibody/benchmarks/free_body/free_body.h
              const char* doc =
R"""(Calculates exact solutions for quaternion and angular velocity
expressed in body-frame, and their time derivatives for torque-free
rotational motion of axis-symmetric rigid body B in Newtonian frame
(World) N, where torque-free means the moment of forces about B's mass
center is zero. The quaternion characterizes the orientation between
right-handed orthogonal unit vectors Nx, Ny, Nz fixed in N and
right-handed orthogonal unit vectors Bx, By, Bz fixed in B, where Bz
is parallel to B's symmetry axis.

Note:
    CalculateExactRotationalSolutionABInitiallyAligned() implements
    the algorithm from [Kane, 1983] Sections 1.13 and 3.1, Pages 60-62
    and 159-169.

Parameter ``t``:
    Current value of time.

Returns:
    Machine-precision values at time t are returned as defined below.

Note:
    This function allows for initial misalignment of Nx, Ny, Nz and
    Bx, By, Bz.

| Return values | Description
|---------------|------------------------------------------------- |
quat_NB | Quaternion [e0, e1, e2, e3] relating frame N to frame B. |
quatDt | Time-derivative of quat_NB, i.e., [ė0, ė1, ė2, ė3]. |
w_NB_B | B's angular velocity in N, expressed in B. | alpha_NB_B | B's
angular acceleration in N, expressed in B.

Note: quat_NB is analogous to the rotation matrix R_NB.

- [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York,
  1983 (with P. W. Likins and D. A. Levinson).  Available for free .pdf
  download: https://ecommons.cornell.edu/handle/1813/637)""";
            } CalculateExactRotationalSolutionNB;
            // Symbol: drake::multibody::benchmarks::free_body::FreeBody::CalculateExactTranslationalSolution
            struct /* CalculateExactTranslationalSolution */ {
              // Source: drake/multibody/benchmarks/free_body/free_body.h
              const char* doc =
R"""(Calculates exact solutions for translational motion of an arbitrary
rigid body B in a Newtonian frame (world) N. Algorithm from
high-school physics.

Parameter ``t``:
    Current value of time.

Returns:
    Machine-precision values at time t are returned as defined below.

std∷tuple | Description
-----------|-----------------------------------------------------------
xyz | Vector3d [x, y, z], Bcm's position from No, expressed in N.
xyzDt | Vector3d [ẋ, ẏ, ż] Bcm's velocity in N, expressed in N.
xyzDDt | Vector3d [ẍ ÿ z̈], Bcm's acceleration in N, expressed in N.)""";
            } CalculateExactTranslationalSolution;
            // Symbol: drake::multibody::benchmarks::free_body::FreeBody::FreeBody
            struct /* ctor */ {
              // Source: drake/multibody/benchmarks/free_body/free_body.h
              const char* doc =
R"""(Constructs a class that can be queried for exact values of
orientation, position, and motion of a torque-free rigid body at time
t.

Parameter ``initial_quat_NB``:
    Value at time t = 0 of the quaternion relating right-handed
    orthonormal vectors Nx, Ny, Nz fixed in N (world) to right-handed
    orthonormal unit vectors Bx, By, Bz fixed in B (body). Note: The
    unit vector Bz is parallel to body B's symmetry axis. Note: The
    quaternion should already be normalized before it is passed.

Parameter ``initial_w_NB_B``:
    Value at time t = 0 of the angular velocity in N of body B,
    expressed in N.

Parameter ``initial_p_NoBcm_N``:
    Value at time t = 0 of the position vector from No (origin of
    world N) to Bcm (B's center of mass), expressed in N.

Parameter ``initial_v_NBcm_B``:
    Value at time t = 0 of the velocity in N of Bcm (B's center of
    mass), expressed in N.

Parameter ``gravity_N``:
    Local gravitational acceleration, expressed in N.)""";
            } ctor;
            // Symbol: drake::multibody::benchmarks::free_body::FreeBody::SetUniformGravityExpressedInWorld
            struct /* SetUniformGravityExpressedInWorld */ {
              // Source: drake/multibody/benchmarks/free_body/free_body.h
              const char* doc = R"""()""";
            } SetUniformGravityExpressedInWorld;
            // Symbol: drake::multibody::benchmarks::free_body::FreeBody::get_I
            struct /* get_I */ {
              // Source: drake/multibody/benchmarks/free_body/free_body.h
              const char* doc =
R"""(Returns body B's moment of inertia about any axis that passes through
Bcm (B's center of mass) and is perpendicular to B's inertia symmetry
axis. For example, for a cylinder of radius r, length h and uniformly
distributed mass m with its cylindrical axis aligned along its body
frame z-axis this would be: I = Ixx = Iyy = m / 12 (3 r² + h²))""";
            } get_I;
            // Symbol: drake::multibody::benchmarks::free_body::FreeBody::get_J
            struct /* get_J */ {
              // Source: drake/multibody/benchmarks/free_body/free_body.h
              const char* doc =
R"""(Returns body's moment of inertia about the axis that passes through
Bcm (B's center of mass) and is parallel to B's inertia symmetry axis.
For example, for a cylinder of radius r, length h and uniformly
distributed mass m with its cylindrical axis aligned along its body
frame z-axis this would be: J = Izz = m r² / 2)""";
            } get_J;
            // Symbol: drake::multibody::benchmarks::free_body::FreeBody::get_initial_p_NoBcm_N
            struct /* get_initial_p_NoBcm_N */ {
              // Source: drake/multibody/benchmarks/free_body/free_body.h
              const char* doc = R"""()""";
            } get_initial_p_NoBcm_N;
            // Symbol: drake::multibody::benchmarks::free_body::FreeBody::get_initial_quat_NB
            struct /* get_initial_quat_NB */ {
              // Source: drake/multibody/benchmarks/free_body/free_body.h
              const char* doc = R"""()""";
            } get_initial_quat_NB;
            // Symbol: drake::multibody::benchmarks::free_body::FreeBody::get_initial_w_NB_B
            struct /* get_initial_w_NB_B */ {
              // Source: drake/multibody/benchmarks/free_body/free_body.h
              const char* doc = R"""()""";
            } get_initial_w_NB_B;
            // Symbol: drake::multibody::benchmarks::free_body::FreeBody::get_uniform_gravity_expressed_in_world
            struct /* get_uniform_gravity_expressed_in_world */ {
              // Source: drake/multibody/benchmarks/free_body/free_body.h
              const char* doc = R"""()""";
            } get_uniform_gravity_expressed_in_world;
            // Symbol: drake::multibody::benchmarks::free_body::FreeBody::set_initial_p_NoBcm_N
            struct /* set_initial_p_NoBcm_N */ {
              // Source: drake/multibody/benchmarks/free_body/free_body.h
              const char* doc = R"""()""";
            } set_initial_p_NoBcm_N;
            // Symbol: drake::multibody::benchmarks::free_body::FreeBody::set_initial_quat_NB
            struct /* set_initial_quat_NB */ {
              // Source: drake/multibody/benchmarks/free_body/free_body.h
              const char* doc = R"""()""";
            } set_initial_quat_NB;
            // Symbol: drake::multibody::benchmarks::free_body::FreeBody::set_initial_v_NBcm_B
            struct /* set_initial_v_NBcm_B */ {
              // Source: drake/multibody/benchmarks/free_body/free_body.h
              const char* doc = R"""()""";
            } set_initial_v_NBcm_B;
            // Symbol: drake::multibody::benchmarks::free_body::FreeBody::set_initial_w_NB_B
            struct /* set_initial_w_NB_B */ {
              // Source: drake/multibody/benchmarks/free_body/free_body.h
              const char* doc = R"""()""";
            } set_initial_w_NB_B;
          } FreeBody;
        } free_body;
      } benchmarks;
    } multibody;
  } drake;
} pydrake_doc_multibody_benchmarks_free_body;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
