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

// #include "drake/multibody/tree/acceleration_kinematics_cache.h"
// #include "drake/multibody/tree/articulated_body_force_cache.h"
// #include "drake/multibody/tree/articulated_body_inertia.h"
// #include "drake/multibody/tree/articulated_body_inertia_cache.h"
// #include "drake/multibody/tree/ball_rpy_joint.h"
// #include "drake/multibody/tree/block_system_jacobian_cache.h"
// #include "drake/multibody/tree/body_node.h"
// #include "drake/multibody/tree/body_node_impl.h"
// #include "drake/multibody/tree/body_node_world.h"
// #include "drake/multibody/tree/curvilinear_joint.h"
// #include "drake/multibody/tree/curvilinear_mobilizer.h"
// #include "drake/multibody/tree/deformable_body.h"
// #include "drake/multibody/tree/door_hinge.h"
// #include "drake/multibody/tree/element_collection.h"
// #include "drake/multibody/tree/fixed_offset_frame.h"
// #include "drake/multibody/tree/force_density_field.h"
// #include "drake/multibody/tree/force_element.h"
// #include "drake/multibody/tree/frame.h"
// #include "drake/multibody/tree/frame_body_pose_cache.h"
// #include "drake/multibody/tree/geometry_spatial_inertia.h"
// #include "drake/multibody/tree/joint.h"
// #include "drake/multibody/tree/joint_actuator.h"
// #include "drake/multibody/tree/linear_bushing_roll_pitch_yaw.h"
// #include "drake/multibody/tree/linear_spring_damper.h"
// #include "drake/multibody/tree/mobilizer.h"
// #include "drake/multibody/tree/mobilizer_impl.h"
// #include "drake/multibody/tree/model_instance.h"
// #include "drake/multibody/tree/multibody_element.h"
// #include "drake/multibody/tree/multibody_forces.h"
// #include "drake/multibody/tree/multibody_tree-inl.h"
// #include "drake/multibody/tree/multibody_tree.h"
// #include "drake/multibody/tree/multibody_tree_indexes.h"
// #include "drake/multibody/tree/multibody_tree_system.h"
// #include "drake/multibody/tree/parameter_conversion.h"
// #include "drake/multibody/tree/planar_joint.h"
// #include "drake/multibody/tree/planar_mobilizer.h"
// #include "drake/multibody/tree/position_kinematics_cache.h"
// #include "drake/multibody/tree/prismatic_joint.h"
// #include "drake/multibody/tree/prismatic_mobilizer.h"
// #include "drake/multibody/tree/prismatic_spring.h"
// #include "drake/multibody/tree/quaternion_floating_joint.h"
// #include "drake/multibody/tree/quaternion_floating_mobilizer.h"
// #include "drake/multibody/tree/revolute_joint.h"
// #include "drake/multibody/tree/revolute_mobilizer.h"
// #include "drake/multibody/tree/revolute_spring.h"
// #include "drake/multibody/tree/rigid_body.h"
// #include "drake/multibody/tree/rotational_inertia.h"
// #include "drake/multibody/tree/rpy_ball_mobilizer.h"
// #include "drake/multibody/tree/rpy_floating_joint.h"
// #include "drake/multibody/tree/rpy_floating_mobilizer.h"
// #include "drake/multibody/tree/scoped_name.h"
// #include "drake/multibody/tree/screw_joint.h"
// #include "drake/multibody/tree/screw_mobilizer.h"
// #include "drake/multibody/tree/spatial_inertia.h"
// #include "drake/multibody/tree/uniform_gravity_field_element.h"
// #include "drake/multibody/tree/unit_inertia.h"
// #include "drake/multibody/tree/universal_joint.h"
// #include "drake/multibody/tree/universal_mobilizer.h"
// #include "drake/multibody/tree/velocity_kinematics_cache.h"
// #include "drake/multibody/tree/weld_joint.h"
// #include "drake/multibody/tree/weld_mobilizer.h"

// Symbol: pydrake_doc_multibody_tree
constexpr struct /* pydrake_doc_multibody_tree */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::multibody
    struct /* multibody */ {
      // Symbol: drake::multibody::ArticulatedBodyInertia
      struct /* ArticulatedBodyInertia */ {
        // Source: drake/multibody/tree/articulated_body_inertia.h
        const char* doc =
R"""(_Articulated Body Inertia_ is the inertia that a body appears to have
when it is the base (or root) of a rigid-body system, also referred to
as *Articulated Body* in the context of articulated body algorithms.
The *Articulated Body Inertia* is a very useful multibody dynamics
concept that was introduced by Featherstone [Featherstone 1983] to
develop the remarkable ``O(n)`` Articulated Body Algorithm (ABA) for
solving forward dynamics. Recall that the Newton-Euler equations allow
us to describe the combined rotational and translational dynamics of a
rigid body:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    F_BBo_W = M_B_W * A_WB + Fb_Bo_W                                     (1)

.. raw:: html

    </details>

where the spatial inertia (see SpatialInertia) ``M_B_W`` of body B
expressed in the world frame W linearly relates the spatial
acceleration (see SpatialAcceleration) of body B in the world frame
with the total applied spatial forces (see SpatialForce) ``F_BBo`` on
body B and where ``Fb_Bo_W`` contains the velocity dependent
gyroscopic terms.

A similar relationship is found for an articulated body with a rigid
body B at the base (or root). Even though the bodies in this multibody
system are allowed to have relative motions among them, there still is
a linear relationship between the spatial force ``F_BBo_W`` applied on
this body and the resulting acceleration ``A_WB``:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    F_BBo_W = P_B_W * A_WB + Z_Bo_W                                       (2)

.. raw:: html

    </details>

where ``P_B_W`` is the articulated body inertia of body B and
``Z_Bo_W`` is a bias force that includes the gyroscopic and Coriolis
forces and becomes zero when all body velocities and all applied
generalized forces outboard from body B are zero [Jain 2010, §7.2.1].
The articulated body inertia ``P_B_W`` is related to the multibody
subsystem consisting only of bodies that are outboard of body B. We
refer to this subsystem as the *articulated body subsystem* associated
with body B. Equation (2) describes the acceleration response of body
B, but also taking into account all outboard bodies connected to B. A
special case is that of an articulated body composed of a single rigid
body. For this special case, Eq. (2) reduces to Eq. (1) for the
dynamics of rigid body B. In other words, the ABI for an articulated
body consisting of a single rigid body exactly equals the spatial
inertia of that body.

Articulated body inertias are elements of ℝ⁶ˣ⁶ that, as for spatial
inertias, are symmetric and positive semi-definite. However, ABI
objects **are not** spatial inertias. The spatial inertia of a rigid
body can be described by a reduced set of ten parameters, namely the
mass, center of mass and the six components of the rotational inertia
for that body. However, this parametrization by ten parameters is just
not possible for an ABI and the full 21 elements of the symmetric
``6x6`` matrix must be specified [Jain 2010, §6.4]. As a result ABI
objects can have different properties than spatial inertia objects. As
an example, the apparent mass of an articulated body will in general
depend on the direction of the applied force. That is, the simple
relationship ``F = m * a`` is no longer valid for an articulated
body's center of mass (refer to the excellent example 7.1 in
[Featherstone 2008]).

We adopt the notation introduced by [Jain 2010] and generally we will
use an uppercase P to represent an ABI. Thus, in typeset material we
use the symbol :math:`[P^{A/Q}]_E` to represent the spatial inertia of
an articulated body A, about a point Q, expressed in a frame E. For
this inertia, the monogram notation reads ``P_AQ_E``.

Note:
    This class does not implement any mechanism to track the frame E
    in which an articulated body inertia is expressed or about what
    point is computed. Methods and operators on this class have no
    means to determine frame consistency through operations. It is
    therefore the responsibility of users of this class to keep track
    of frames in which operations are performed. We suggest doing that
    using disciplined notation, as described above.

- [Featherstone 1983] Featherstone, R., 1983.
    The calculation of robot dynamics using articulated-body inertias. The
    International Journal of Robotics Research, 2(1), pp.13-30.
- [Featherstone 2008] Featherstone, R., 2008.
    Rigid body dynamics algorithms. Springer.
- [Jain 2010]  Jain, A., 2010.
    Robot and multibody dynamics: analysis and algorithms.
    Springer Science & Business Media.)""";
        // Symbol: drake::multibody::ArticulatedBodyInertia::ArticulatedBodyInertia<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/articulated_body_inertia.h
          const char* doc_0args =
R"""(Default ArticulatedBodyInertia constructor initializes all matrix
values to NaN for a quick detection of uninitialized values.)""";
          // Source: drake/multibody/tree/articulated_body_inertia.h
          const char* doc_1args_M_SQ_E =
R"""(Constructs an articulated body inertia for an articulated body
consisting of a single rigid body given its spatial inertia. From an
input spatial inertia ``M_SQ_E`` for a body or composite body S, about
point Q, and expressed in a frame E, this constructor creates an
articulated body inertia about the same point Q and expressed in the
same frame E.

Parameter ``M_SQ_E``:
    The spatial inertia of a body or composite body S about point Q
    and expressed in frame E.)""";
          // Source: drake/multibody/tree/articulated_body_inertia.h
          const char* doc_1args_constEigenMatrixBase =
R"""(Constructs an articulated body inertia from an input matrix.

In Debug, this constructor checks for the physical validity of the
resulting ArticulatedBodyInertia with IsPhysicallyValid() and throws a
RuntimeError exception in the event the provided input matrix leads to
a non-physically viable articulated body inertia.

Parameter ``matrix``:
    A matrix or matrix expression representing the articulated body
    inertia. Only the lower triangular region is used and the strictly
    upper part is ignored.

Raises:
    RuntimeError in Debug builds if IsPhysicallyValid() for ``this``
    inertia is ``False``.)""";
        } ctor;
        // Symbol: drake::multibody::ArticulatedBodyInertia::CopyToFullMatrix6
        struct /* CopyToFullMatrix6 */ {
          // Source: drake/multibody/tree/articulated_body_inertia.h
          const char* doc =
R"""(Copy to a full 6x6 matrix representation.)""";
        } CopyToFullMatrix6;
        // Symbol: drake::multibody::ArticulatedBodyInertia::IsPhysicallyValid
        struct /* IsPhysicallyValid */ {
          // Source: drake/multibody/tree/articulated_body_inertia.h
          const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Performs a number of checks to verify that this is a physically valid
articulated body inertia.

The checks performed are: - The matrix is positive semi-definite.)""";
        } IsPhysicallyValid;
        // Symbol: drake::multibody::ArticulatedBodyInertia::Shift
        struct /* Shift */ {
          // Source: drake/multibody/tree/articulated_body_inertia.h
          const char* doc =
R"""(Given ``this`` articulated body inertia ``P_AQ_E`` for some
articulated body A, computed about point Q, and expressed in frame E,
this method uses the rigid body shift operator to compute the same
articulated body inertia about a new point R. The result still is
expressed in frame E.

See also:
    ShiftInPlace() for more details.

Parameter ``p_QR_E``:
    Vector from the original about point Q to the new about point R,
    expressed in the same frame E ``this`` articulated body inertia is
    expressed in.

Returns ``P_AR_E``:
    This same articulated body inertia for articulated body A but now
    computed about a new point R.)""";
        } Shift;
        // Symbol: drake::multibody::ArticulatedBodyInertia::ShiftInPlace
        struct /* ShiftInPlace */ {
          // Source: drake/multibody/tree/articulated_body_inertia.h
          const char* doc =
R"""(Given ``this`` articulated body inertia ``P_AQ_E`` for some
articulated body A, computed about point Q, and expressed in frame E,
this method uses the rigid body shift operator to compute the same
articulated body inertia about a new point R. The result still is
expressed in frame E.

Mathematically, this is equivalent to:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    P_AR_E = Φ(P_RQ_E) P_AQ_E Φ(p_RQ_E)ᵀ

.. raw:: html

    </details>

where ``Φ(p_RQ_E)`` is the rigid body shift operator as defined by
[Jain 2010]. The definition of ``Φ(p_RQ_E)`` uses ``p_QR_E×``, which
is the skew-symmetric cross product matrix (defined such that ``a× b =
a.cross(b)``).


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    Φ(p_RQ_E) =
    | I₃  p_RQ_E× |
    | 0        I₃ |

.. raw:: html

    </details>

where ``p_RQ_E× = -p_QR_E×``.

This operation is performed in-place modifying the original object.

See also:
    Shift() which does not modify this object.

For details see Section 6.2.5, Page 105 of [Jain 2010].

Parameter ``p_QR_E``:
    Vector from the original about point Q to the new about point R,
    expressed in the same frame E ``this`` articulated body inertia is
    expressed in.)""";
        } ShiftInPlace;
        // Symbol: drake::multibody::ArticulatedBodyInertia::cast
        struct /* cast */ {
          // Source: drake/multibody/tree/articulated_body_inertia.h
          const char* doc =
R"""(Returns a new ArticulatedBodyInertia object templated on ``Scalar``
with casted values of ``this`` articulated body inertia.

Template parameter ``Scalar``:
    The scalar type on which the new articulated body inertia will be
    templated.

Note:
    ``ArticulatedBodyInertia<From>::cast<To>()`` creates a new
    ``ArticulatedBodyInertia<To>`` from an
    ``ArticulatedBodyInertia<From>`` but only if type ``To`` is
    constructible from type ``From``. As an example of this,
    ``ArticulatedBodyInertia<double>::cast<AutoDiffXd>()`` is valid
    since ``AutoDiffXd a(1.0)`` is valid. However,
    ``ArticulatedBodyInertia<AutoDiffXd>::cast<double>()`` is not.)""";
        } cast;
        // Symbol: drake::multibody::ArticulatedBodyInertia::operator*
        struct /* operator_mul */ {
          // Source: drake/multibody/tree/articulated_body_inertia.h
          const char* doc_1args_constEigenMatrixBase =
R"""(Multiplies ``this`` articulated body inertia on the right by a matrix
or vector.

Note:
    This method does not evaluate the product immediately. Instead, it
    returns an intermediate Eigen quantity that can be optimized
    automatically during compile time.)""";
          // Source: drake/multibody/tree/articulated_body_inertia.h
          const char* doc_1args_A_WB_E =
R"""(Multiplies ``this`` articulated body inertia on the right by a spatial
acceleration. See abi_eq_definition "Eq. (2)" for an example.)""";
        } operator_mul;
        // Symbol: drake::multibody::ArticulatedBodyInertia::operator+=
        struct /* operator_iadd */ {
          // Source: drake/multibody/tree/articulated_body_inertia.h
          const char* doc =
R"""(Adds in to this articulated body inertia ``P_AQ_E`` for an articulated
body A about a point Q and expressed in a frame E the articulated body
inertia ``P_BQ_E`` for a second articulated body B about the same
point Q and expressed in the same frame E. The result is equivalent to
the articulated body inertia ``P_CQ_E`` for the composite articulated
body C which has at its base a rigid body composed of the bases of A
and B welded together [Featherstone 2008, example 7.1]. The composite
articulated body inertia ``P_CQ_E`` is also about the same point Q and
expressed in the same frame E as the addends.

Parameter ``P_BQ_E``:
    An articulated body inertia of some articulated body B to be added
    to ``this`` articulated body inertia. It must be defined about the
    same point Q as ``this`` inertia, and expressed in the same frame
    E.

Returns:
    A reference to ``this`` articulated body inertia, which has been
    updated to include the given articulated body inertia ``P_BQ_E``.

Warning:
    This operation is only valid if both articulated body inertias are
    computed about the same point Q and expressed in the same frame E.)""";
        } operator_iadd;
        // Symbol: drake::multibody::ArticulatedBodyInertia::operator-=
        struct /* operator_isub */ {
          // Source: drake/multibody/tree/articulated_body_inertia.h
          const char* doc =
R"""(Subtracts ``P_BQ_E`` from ``this`` articulated body inertia.
``P_BQ_E`` must be for the same articulated body B as this ABI (about
the same point Q and expressed in the same frame E). The resulting
inertia will have the same properties.)""";
        } operator_isub;
      } ArticulatedBodyInertia;
      // Symbol: drake::multibody::BallRpyJoint
      struct /* BallRpyJoint */ {
        // Source: drake/multibody/tree/ball_rpy_joint.h
        const char* doc =
R"""(This Joint allows two bodies to rotate freely relative to one another.
That is, given a frame F attached to the parent body P and a frame M
attached to the child body B (see the Joint class's documentation),
this Joint allows frame M to rotate freely with respect to F, while
the origins, Mo and Fo, of frames M and F respectively remain
coincident. The orientation of M relative to F is parameterized with
space ``x-y-z`` Euler angles.)""";
        // Symbol: drake::multibody::BallRpyJoint::BallRpyJoint<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/ball_rpy_joint.h
          const char* doc =
R"""(Constructor to create a ball rpy joint between two bodies so that
frame F attached to the parent body P and frame M attached to the
child body B rotate freely relative to one another. See this class's
documentation for further details on the definition of these frames,
get_angles() for an explanation of the angles defining orientation,
and get_angular_velocity() for an explanation of the generalized
velocities. This constructor signature creates a joint with no joint
limits, i.e. the joint position, velocity and acceleration limits are
the pair ``(-∞, ∞)``. These can be set using the Joint methods
set_position_limits(), set_velocity_limits() and
set_acceleration_limits(). The first three arguments to this
constructor are those of the Joint class constructor. See the Joint
class's documentation for details. The additional parameters are:

Parameter ``damping``:
    Viscous damping coefficient, in N⋅m⋅s, used to model losses within
    the joint. See documentation of default_damping() for details on
    modelling of the damping torque.

Raises:
    RuntimeError if damping is negative.)""";
        } ctor;
        // Symbol: drake::multibody::BallRpyJoint::DoAddInDamping
        struct /* DoAddInDamping */ {
          // Source: drake/multibody/tree/ball_rpy_joint.h
          const char* doc =
R"""(Joint<T> override called through public NVI, Joint::AddInDamping().
Therefore arguments were already checked to be valid. This method adds
into ``forces`` a dissipative torque according to the viscous law ``τ
= -d⋅ω``, with d the damping coefficient (see default_damping()).)""";
        } DoAddInDamping;
        // Symbol: drake::multibody::BallRpyJoint::DoAddInOneForce
        struct /* DoAddInOneForce */ {
          // Source: drake/multibody/tree/ball_rpy_joint.h
          const char* doc =
R"""(Joint<T> override called through public NVI, Joint::AddInForce().
Adding forces per-dof makes no physical sense. Therefore, this method
throws an exception if invoked.)""";
        } DoAddInOneForce;
        // Symbol: drake::multibody::BallRpyJoint::default_damping
        struct /* default_damping */ {
          // Source: drake/multibody/tree/ball_rpy_joint.h
          const char* doc =
R"""(Returns ``this`` joint's default damping constant in N⋅m⋅s. The
damping torque (in N⋅m) is modeled as ``τ = -damping⋅ω``, i.e.
opposing motion, with ω the angular velocity of frame M in F (see
get_angular_velocity()) and τ the torque on child body B (to which M
is rigidly attached).)""";
        } default_damping;
        // Symbol: drake::multibody::BallRpyJoint::get_angles
        struct /* get_angles */ {
          // Source: drake/multibody/tree/ball_rpy_joint.h
          const char* doc =
R"""(Gets the rotation angles of ``this`` joint from ``context``.

The orientation ``R_FM`` of the child frame M in parent frame F is
parameterized with space ``x-y-z`` Euler angles (also known as
extrinsic angles). That is, the angles θr, θp, θy, correspond to a
sequence of rotations about the x̂, ŷ, ẑ axes of parent frame F,
respectively. Mathematically, rotation ``R_FM`` is given in terms of
angles θr, θp, θy by:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    R_FM(q) = Rz(θy) * Ry(θp) * Rx(θr)

.. raw:: html

    </details>

where ``Rx(θ)``, `Ry(θ)` and ``Rz(θ)`` correspond to the elemental
rotations in amount of θ about the x, y and z axes respectively. Zero
θr, θp, θy angles corresponds to frames F and M being coincident.
Angles θr, θp, θy are defined to be positive according to the
right-hand-rule with the thumb aligned in the direction of their
respective axes.

Note:
    Space ``x-y-z`` angles (extrinsic) are equivalent to Body
    ``z-y-x`` angles (intrinsic).

Note:
    This particular choice of angles θr, θp, θy for this joint are
    many times referred to as the roll, pitch and yaw angles by many
    dynamicists. They are also known as the Tait-Bryan angles or
    Cardan angles.

Parameter ``context``:
    The context of the model this joint belongs to.

Returns:
    The angle coordinates of ``this`` joint stored in the ``context``
    ordered as θr, θp, θy.)""";
        } get_angles;
        // Symbol: drake::multibody::BallRpyJoint::get_angular_velocity
        struct /* get_angular_velocity */ {
          // Source: drake/multibody/tree/ball_rpy_joint.h
          const char* doc =
R"""(Retrieves from ``context`` the angular velocity ``w_FM`` of the child
frame M in the parent frame F, expressed in F.

Parameter ``context``:
    The context of the model this joint belongs to.

Returns ``w_FM``:
    A vector in ℝ³ with the angular velocity of the child frame M in
    the parent frame F, expressed in F. Refer to this class's
    documentation for further details and definitions of these frames.)""";
        } get_angular_velocity;
        // Symbol: drake::multibody::BallRpyJoint::get_default_angles
        struct /* get_default_angles */ {
          // Source: drake/multibody/tree/ball_rpy_joint.h
          const char* doc =
R"""(Gets the default angles for ``this`` joint. Wrapper for the more
general ``Joint::default_positions()``.

Returns:
    The default angles of ``this`` stored in ``default_positions_``)""";
        } get_default_angles;
        // Symbol: drake::multibody::BallRpyJoint::set_angles
        struct /* set_angles */ {
          // Source: drake/multibody/tree/ball_rpy_joint.h
          const char* doc =
R"""(Sets the ``context`` so that the generalized coordinates corresponding
to the rotation angles of ``this`` joint equals ``angles``.

Parameter ``context``:
    The context of the model this joint belongs to.

Parameter ``angles``:
    The desired angles in radians to be stored in ``context`` ordered
    as θr, θp, θy. See get_angles() for details.

Returns:
    a constant reference to ``this`` joint.)""";
        } set_angles;
        // Symbol: drake::multibody::BallRpyJoint::set_angular_velocity
        struct /* set_angular_velocity */ {
          // Source: drake/multibody/tree/ball_rpy_joint.h
          const char* doc =
R"""(Sets in ``context`` the state for ``this`` joint so that the angular
velocity of the child frame M in the parent frame F is ``w_FM``.

Parameter ``context``:
    The context of the model this joint belongs to.

Parameter ``w_FM``:
    A vector in ℝ³ with the angular velocity of the child frame M in
    the parent frame F, expressed in F. Refer to this class's
    documentation for further details and definitions of these frames.

Returns:
    a constant reference to ``this`` joint.)""";
        } set_angular_velocity;
        // Symbol: drake::multibody::BallRpyJoint::set_default_angles
        struct /* set_default_angles */ {
          // Source: drake/multibody/tree/ball_rpy_joint.h
          const char* doc =
R"""(Sets the default angles of this joint.

Parameter ``angles``:
    The desired default angles of the joint)""";
        } set_default_angles;
        // Symbol: drake::multibody::BallRpyJoint::set_random_angles_distribution
        struct /* set_random_angles_distribution */ {
          // Source: drake/multibody/tree/ball_rpy_joint.h
          const char* doc =
R"""(Sets the random distribution that angles of this joint will be
randomly sampled from. See get_angles() for details on the angle
representation.)""";
        } set_random_angles_distribution;
        // Symbol: drake::multibody::BallRpyJoint::type_name
        struct /* type_name */ {
          // Source: drake/multibody/tree/ball_rpy_joint.h
          const char* doc = R"""()""";
        } type_name;
      } BallRpyJoint;
      // Symbol: drake::multibody::BodyIndex
      struct /* BodyIndex */ {
        // Source: drake/multibody/tree/multibody_tree_indexes.h
        const char* doc =
R"""(Type used to identify RigidBodies (a.k.a. Links) by index in a
multibody plant. Interchangeable with LinkIndex.)""";
      } BodyIndex;
      // Symbol: drake::multibody::CalcSpatialInertia
      struct /* CalcSpatialInertia */ {
        // Source: drake/multibody/tree/geometry_spatial_inertia.h
        const char* doc_shape =
R"""(Computes the SpatialInertia of a body made up of a homogeneous
material (of given ``density`` in kg/m³) uniformly distributed in the
volume of the given ``shape``.

The ``shape`` is defined in its canonical frame S and the body in
frame B. The two frames are coincident and aligned (i.e., X_SB = I).

Most shapes are defined such that their center of mass is coincident
with So (and, therefore, Bo). These are the shapes that have symmetry
across So along each of the axes Sx, Sy, Sz (e.g., geometry::Box,
geometry::Sphere, etc.) For meshes, it depends on how the mesh is
defined. For more discussion on the nuances of geometry::Mesh and
geometry::Convex calculations CalcSpatialInertia(const
geometry::TriangleSurfaceMesh<double>&,double) "see below".

Returns ``M_BBo_B``:
    The spatial inertia of the hypothetical body implied by the given
    ``shape``.

Raises:
    RuntimeError if ``shape`` is an instance of geometry::HalfSpace or
    geometry::MeshcatCone.)""";
        // Source: drake/multibody/tree/geometry_spatial_inertia.h
        const char* doc_mesh =
R"""(Computes the SpatialInertia of a body made up of a homogeneous
material (of given ``density`` in kg/m³) uniformly distributed in the
volume of the given ``mesh``.

The ``mesh`` is defined in its canonical frame M and the body in frame
B. The two frames are coincident and aligned (i.e., X_MB = I).

For the resultant spatial inertia to be meaningful, the ``mesh`` must
satisfy certain requirements:

- The mesh must *fully* enclose a volume (no cracks, no open manifolds,
etc.).
- All triangles must be "wound" such that their normals point outward
(according to the right-hand rule based on vertex winding).

Drake currently doesn't validate these requirements on the mesh.
Instead, it does a best-faith effort to compute a spatial inertia. For
some "bad" meshes, the SpatialInertia will be objectively physically
invalid. For others, the SpatialInertia will appear physically valid,
but be meaningless because it does not accurately represent the mesh.

Raises:
    RuntimeError if the resulting spatial inertia is obviously
    physically invalid. See
    multibody::SpatialInertia::IsPhysicallyValid().)""";
      } CalcSpatialInertia;
      // Symbol: drake::multibody::CurvilinearJoint
      struct /* CurvilinearJoint */ {
        // Source: drake/multibody/tree/curvilinear_joint.h
        const char* doc =
R"""(A Joint that allows a body to move along a piecewise constant
curvature path contained in a plane.

The path is specified as a PiecewiseConstantCurvatureTrajectory, refer
to that class documentation for further details on parameterization,
conventions and notation used.

This joint grants a single degree of freedom q that corresponds to the
length s (in meters) along the path. The generalized velocity v = q̇
corresponds to the magnitude of the tangential velocity. We denote
with F a frame on a "parent" body and with M a frame on a "child"
body. For a given trajectory, this joint prescribes X_FM(q) =
PiecewiseConstantCurvatureTrajectory::CalcPose(q).

Frame M is defined according to the convention documented in
PiecewiseConstantCurvatureTrajectory. That is, axis Mx is the tangent
to the trajectory, Mz equals the (constant) normal p̂ to the plane,
and My = Mz x Mx. It is not required that M coincides with F at
distance q = 0.

If the specified trajectory is periodic, the joint prescribes a
trajectory of cycle length L that satisfies X_FP(s) = X_FP(s + k⋅L) ∀
k ∈ ℤ.

By default, the joint position limits are the endpoints for aperiodic
paths, and (-∞, ∞) for periodic paths.

See also:
    trajectories::PiecewiseConstantCurvatureTrajectory)""";
        // Symbol: drake::multibody::CurvilinearJoint::AddInForce
        struct /* AddInForce */ {
          // Source: drake/multibody/tree/curvilinear_joint.h
          const char* doc =
R"""(Adds into a MultibodyForces a generalized force on this joint.

A generalized force for a curvilinear joint is equivalent to a force
in Newtons applied along the path tangent direction (x-axis of frame
M).

Parameter ``context``:
    The context of the model this joint belongs to.

Parameter ``force``:
    The force to be applied, in Newtons.

Parameter ``forces``:
    The MultibodyForces object to which the generalized force is
    added.)""";
        } AddInForce;
        // Symbol: drake::multibody::CurvilinearJoint::CurvilinearJoint<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/curvilinear_joint.h
          const char* doc_5args =
R"""(Constructor to create a curvilinear joint between two bodies so that
frame F attached to the parent body P and frame M attached to the
child body B, move relatively to one another along a planar
curvilinear path. See this class's documentation for further details
on the definition of these frames and the curvilinear path.

This constructor signature creates a joint where the joint velocity
and acceleration limits are ``(-∞, ∞)``. Position limits are ``(0,
L)`` with L the length of the trajectory. If the trajectory is
periodic, position limits are ``(-∞, ∞)``.

The first three arguments to this constructor are those of the Joint
class constructor. See the Joint class's documentation for details.
The additional parameters are:

Parameter ``curvilinear_path``:
    The curvilinear path for this joint, along which the child frame M
    moves relative to the parent frame F.

Parameter ``damping``:
    Viscous damping coefficient, in N⋅s/m, used to model losses within
    the joint. The damping force (in N) is modeled as ``f =
    -damping⋅v`` along the tangent to the curve, i.e. opposing motion,
    with v the tangential velocity for ``this`` joint (see
    get_tangential_velocity()).

Raises:
    RuntimeError if damping is negative)""";
          // Source: drake/multibody/tree/curvilinear_joint.h
          const char* doc_7args =
R"""(Constructor to create a curvilinear joint between two bodies so that
frame F attached to the parent body P and frame M attached to the
child body B, move relatively to one another along a planar
curvilinear path. See this class's documentation for further details
on the definition of these frames and the path.

The first three arguments to this constructor are those of the Joint
class constructor. See the Joint class's documentation for details.
The additional parameters are:

Parameter ``curvilinear_path``:
    The curvilinear path for this joint, along which the child frame M
    moves relative to the parent frame F.

Parameter ``pos_lower_limit``:
    Lower position limit, in meters, for the distance coordinate (see
    get_distance()).

Parameter ``pos_upper_limit``:
    Upper position limit, in meters, for the distance coordinate (see
    get_distance()).

Parameter ``damping``:
    Viscous damping coefficient, in N⋅s/m, used to model losses within
    the joint. The damping force (in N) is modeled as ``f =
    -damping⋅v`` along the tangent to the curve, i.e. opposing motion,
    with v the tangential velocity for ``this`` joint (see
    get_tangential_velocity()).

Raises:
    RuntimeError if damping is negative.

Raises:
    RuntimeError if pos_lower_limit > pos_upper_limit.)""";
        } ctor;
        // Symbol: drake::multibody::CurvilinearJoint::DoAddInDamping
        struct /* DoAddInDamping */ {
          // Source: drake/multibody/tree/curvilinear_joint.h
          const char* doc =
R"""(Joint<T> override called through public NVI, Joint::AddInDamping().
Arguments already checked to be valid by Joint::AddInDamping().

Adds a dissipative force according to the viscous law ``f = -d⋅v``,
where d is the damping coefficient (see default_damping()) and v the
tangential velocity along the path.

Parameter ``context``:
    The context of the model this joint belongs to.

Parameter ``forces``:
    The MultibodyForces object to which the damping force is added.)""";
        } DoAddInDamping;
        // Symbol: drake::multibody::CurvilinearJoint::DoAddInOneForce
        struct /* DoAddInOneForce */ {
          // Source: drake/multibody/tree/curvilinear_joint.h
          const char* doc =
R"""(Joint<T> override called through public NVI, Joint::AddInForce().
Arguments already checked to be valid by Joint::AddInForce().

Parameter ``joint_dof``:
    The joint degree of freedom index, on which the force is added,
    which must be 0.

Parameter ``joint_tau``:
    The force along the path's tangential axis to be added, in
    Newtons, applied to the child body.

Parameter ``forces``:
    The MultibodyForces object to which the force is added.

See also:
    The public NVI AddInOneForce() for details.)""";
        } DoAddInOneForce;
        // Symbol: drake::multibody::CurvilinearJoint::GetDamping
        struct /* GetDamping */ {
          // Source: drake/multibody/tree/curvilinear_joint.h
          const char* doc =
R"""(Returns the Context dependent damping coefficient stored as a
parameter in ``context``. Refer to default_damping() for details.

Parameter ``context``:
    The context of the model this joint belongs to.

Returns:
    The damping coefficient stored in the context, in N⋅s/m.)""";
        } GetDamping;
        // Symbol: drake::multibody::CurvilinearJoint::SetDamping
        struct /* SetDamping */ {
          // Source: drake/multibody/tree/curvilinear_joint.h
          const char* doc =
R"""(Sets the value of the viscous damping coefficient for this joint,
stored as a parameter in ``context``. Refer to default_damping() for
details.

Parameter ``context``:
    The context of the model this joint belongs to.

Parameter ``damping``:
    The damping coefficient to be set in N⋅s/m.

Raises:
    RuntimeError if ``damping`` is negative.)""";
        } SetDamping;
        // Symbol: drake::multibody::CurvilinearJoint::acceleration_lower_limit
        struct /* acceleration_lower_limit */ {
          // Source: drake/multibody/tree/curvilinear_joint.h
          const char* doc =
R"""(Returns the acceleration lower limit for ``this`` joint in m/s².)""";
        } acceleration_lower_limit;
        // Symbol: drake::multibody::CurvilinearJoint::acceleration_upper_limit
        struct /* acceleration_upper_limit */ {
          // Source: drake/multibody/tree/curvilinear_joint.h
          const char* doc =
R"""(Returns the acceleration upper limit for ``this`` joint in m/s².)""";
        } acceleration_upper_limit;
        // Symbol: drake::multibody::CurvilinearJoint::default_damping
        struct /* default_damping */ {
          // Source: drake/multibody/tree/curvilinear_joint.h
          const char* doc =
R"""(Returns ``this`` joint's default damping constant in N⋅s/m.)""";
        } default_damping;
        // Symbol: drake::multibody::CurvilinearJoint::get_default_distance
        struct /* get_default_distance */ {
          // Source: drake/multibody/tree/curvilinear_joint.h
          const char* doc =
R"""(Gets the default travel distance along the path.)""";
        } get_default_distance;
        // Symbol: drake::multibody::CurvilinearJoint::get_distance
        struct /* get_distance */ {
          // Source: drake/multibody/tree/curvilinear_joint.h
          const char* doc =
R"""(Gets the travel distance of the joint from the provided context.

Parameter ``context``:
    The context of the model this joint belongs to.

Returns:
    The distance coordinate of the joint stored in the context.)""";
        } get_distance;
        // Symbol: drake::multibody::CurvilinearJoint::get_tangential_velocity
        struct /* get_tangential_velocity */ {
          // Source: drake/multibody/tree/curvilinear_joint.h
          const char* doc =
R"""(Gets the tangential velocity in meters per second, i.e. the rate of
change of this joint's travel distance (see get_distance()) from the
provided context.

Parameter ``context``:
    The context of the model this joint belongs to.

Returns:
    The tangential velocity as stored in the provided context.)""";
        } get_tangential_velocity;
        // Symbol: drake::multibody::CurvilinearJoint::get_trajectory
        struct /* get_trajectory */ {
          // Source: drake/multibody/tree/curvilinear_joint.h
          const char* doc =
R"""(Returns:
    A reference to the underlying trajectory.)""";
        } get_trajectory;
        // Symbol: drake::multibody::CurvilinearJoint::position_lower_limit
        struct /* position_lower_limit */ {
          // Source: drake/multibody/tree/curvilinear_joint.h
          const char* doc =
R"""(Returns the position lower limit for ``this`` joint in m.)""";
        } position_lower_limit;
        // Symbol: drake::multibody::CurvilinearJoint::position_upper_limit
        struct /* position_upper_limit */ {
          // Source: drake/multibody/tree/curvilinear_joint.h
          const char* doc =
R"""(Returns the position upper limit for ``this`` joint in m.)""";
        } position_upper_limit;
        // Symbol: drake::multibody::CurvilinearJoint::set_default_damping
        struct /* set_default_damping */ {
          // Source: drake/multibody/tree/curvilinear_joint.h
          const char* doc =
R"""(Sets the default value of viscous damping for this joint, in N⋅s/m.

Raises:
    RuntimeError if damping is negative.

Raises:
    RuntimeError if this element is not associated with a
    MultibodyPlant.

Precondition:
    the MultibodyPlant must not be finalized.)""";
        } set_default_damping;
        // Symbol: drake::multibody::CurvilinearJoint::set_default_distance
        struct /* set_default_distance */ {
          // Source: drake/multibody/tree/curvilinear_joint.h
          const char* doc =
R"""(Sets the default travel distance of this joint.

Parameter ``distance``:
    The desired default distance of the joint in meters.)""";
        } set_default_distance;
        // Symbol: drake::multibody::CurvilinearJoint::set_distance
        struct /* set_distance */ {
          // Source: drake/multibody/tree/curvilinear_joint.h
          const char* doc =
R"""(Sets the travel distance of the joint from the provided context.

Parameter ``context``:
    The context of the model this joint belongs to.

Parameter ``distance``:
    The travel distance to be set in the context.

Returns:
    const reference to this joint.)""";
        } set_distance;
        // Symbol: drake::multibody::CurvilinearJoint::set_random_distance_distribution
        struct /* set_random_distance_distribution */ {
          // Source: drake/multibody/tree/curvilinear_joint.h
          const char* doc =
R"""(Sets the random distribution for the distance along the path.

Parameter ``distance``:
    Expression defining the random distance distribution.)""";
        } set_random_distance_distribution;
        // Symbol: drake::multibody::CurvilinearJoint::set_tangential_velocity
        struct /* set_tangential_velocity */ {
          // Source: drake/multibody/tree/curvilinear_joint.h
          const char* doc =
R"""(Sets the tangential velocity of the joint from the provided context.

Parameter ``context``:
    The context of the model this joint belongs to.

Parameter ``tangential_velocity``:
    The tangential velocity to be set in the context.

Returns:
    const reference to this joint.)""";
        } set_tangential_velocity;
        // Symbol: drake::multibody::CurvilinearJoint::type_name
        struct /* type_name */ {
          // Source: drake/multibody/tree/curvilinear_joint.h
          const char* doc = R"""()""";
        } type_name;
        // Symbol: drake::multibody::CurvilinearJoint::velocity_lower_limit
        struct /* velocity_lower_limit */ {
          // Source: drake/multibody/tree/curvilinear_joint.h
          const char* doc =
R"""(Returns the velocity lower limit for ``this`` joint in m/s.)""";
        } velocity_lower_limit;
        // Symbol: drake::multibody::CurvilinearJoint::velocity_upper_limit
        struct /* velocity_upper_limit */ {
          // Source: drake/multibody/tree/curvilinear_joint.h
          const char* doc =
R"""(Returns the velocity upper limit for ``this`` joint in m/s.)""";
        } velocity_upper_limit;
      } CurvilinearJoint;
      // Symbol: drake::multibody::DeformableBody
      struct /* DeformableBody */ {
        // Source: drake/multibody/tree/deformable_body.h
        const char* doc =
R"""(The DeformableBody class represents a single deformable element within
a MultibodyPlant. It encapsulates the mesh, physical properties, and
finite-element model required to simulate deformable behavior. It
manages: - Unique identification (DeformableBodyIndex,
DeformableBodyId) and naming - Geometry association for collision and
visualization - Construction of a FEM model with configurable
constitutive parameters - Storage of reference vertex positions and
system state indices - Application of boundary conditions and
constraints - Registration and retrieval of external forces (including
gravity) - Enabling/disabling of dynamics at runtime

This class is not meant to be created by end users and it must be
created exclusively by DeformableModel through
DeformableModel::RegisterDeformableBody.)""";
        // Symbol: drake::multibody::DeformableBody::AddFixedConstraint
        struct /* AddFixedConstraint */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc =
R"""(Defines a fixed constraint between this deformable body and a rigid
body B. Such a fixed constraint is modeled as distance holonomic
constraints:

p_PᵢQᵢ(q) = 0 for each constrained vertex Pᵢ

where Pᵢ is the i-th vertex of the deformable body (A) under
constraint and Qᵢ is a point rigidly affixed to the rigid body B. To
specify the constraint, we put the reference mesh M of this body A in
B's body frame with the given pose ``X_BA`` and prescribe a shape G
with pose ``X_BG`` in B's body frame. All vertices Pᵢ in M that are
inside (or on the surface of) G are subject to the fixed constraints
with Qᵢ being coincident with Pᵢ when M is in pose X_BA. p_PᵢQᵢ(q)
denotes the relative position of point Qᵢ with respect to point Pᵢ as
a function of the configuration of the model q. Imposing this
constraint forces Pᵢ and Qᵢ to be coincident for each vertex i of the
deformable body specified to be under constraint.

Parameter ``body_B``:
    The rigid body under constraint.

Parameter ``X_BA``:
    The pose of this deformable body A's reference mesh in B's body
    frame.

Parameter ``shape_G``:
    The prescribed geometry shape, attached to rigid body B, used to
    determine which vertices of this deformable body A is under
    constraint.

Parameter ``X_BG``:
    The fixed pose of the geometry frame of the given ``shape`` in
    body B's frame.

Returns:
    the unique id of the newly added constraint.

Raises:
    RuntimeError unless ``body_B`` is registered with the same
    multibody tree owning this deformable body.

Raises:
    RuntimeError if no constraint is added (i.e. no vertex of the
    deformable body is inside the given ``shape`` with the given
    poses).

Raises:
    RuntimeError if this element is not associated with a
    MultibodyPlant.)""";
        } AddFixedConstraint;
        // Symbol: drake::multibody::DeformableBody::CalcCenterOfMassPositionInWorld
        struct /* CalcCenterOfMassPositionInWorld */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc =
R"""(Calculates the body's center of mass position in world frame W.

Parameter ``context``:
    The context associated with the MultibodyPlant that owns this
    body.

Returns ``p_WBcm_W``:
    the body's center of mass position, measured and expressed in the
    world frame W.

Raises:
    RuntimeError if ``context`` does not belong to the MultibodyPlant
    that owns this body.)""";
        } CalcCenterOfMassPositionInWorld;
        // Symbol: drake::multibody::DeformableBody::CalcCenterOfMassTranslationalVelocityInWorld
        struct /* CalcCenterOfMassTranslationalVelocityInWorld */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc =
R"""(Calculates the body's center of mass translational velocity in world
frame W.

Parameter ``context``:
    The context associated with the MultibodyPlant that owns this
    body.

Returns ``v_WScm_W``:
    Scm's translational velocity in frame W, expressed in W, where Scm
    is the center of mass of this body.

Raises:
    RuntimeError if ``context`` does not belong to the MultibodyPlant
    that owns this body.)""";
        } CalcCenterOfMassTranslationalVelocityInWorld;
        // Symbol: drake::multibody::DeformableBody::CalcEffectiveAngularVelocity
        struct /* CalcEffectiveAngularVelocity */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc =
R"""(Using an angular momentum analogy, calculates an "effective" angular
velocity for this body about its center of mass, measured and
expressed in the world frame W. The effective angular velocity is
computed using an angular momentum equation that assumes the body is a
rigid body (albeit we know it is deformable).

H_WBcm_W = I_BBcm_W * w_WBcm_W

for which when solved for w_WBcm_W gives

w_WBcm_W = inverse(I_BBcm_W) * H_WBcm_W

where H_WBcm_W is the body's angular momentum about its center of mass
Bcm measured and expressed in the world frame W.

Parameter ``context``:
    The context associated with the MultibodyPlant that owns this
    body.

Returns ``w_WBcm_W``:
    the body's effective angular velocity about Bcm, measured and
    expressed in the world frame W.

Raises:
    RuntimeError if ``context`` does not belong to the MultibodyPlant
    that owns this body.)""";
        } CalcEffectiveAngularVelocity;
        // Symbol: drake::multibody::DeformableBody::DeformableBody<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::multibody::DeformableBody::Disable
        struct /* Disable */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc =
R"""(Disables this deformable body in the given context. Disabling a
deformable body sets its vertex velocities and accelerations to zero
and freezes its vertex positions. A disabled deformable body is not
subject to any constraint (e.g. frictional contact constraint or fixed
constraint); it does not move under the influence of external forces
(e.g. gravity); and it does not necessarily satisfy the prescribed
boundary condition (if any). On the flip side, a disabled deformable
body does not affect the dynamics of other bodies, even if the
collision between the disabled body's geometry and other geometries is
not filtered. Effectively, the physics of the deformable body stop
being computed. The deformable body can be enabled by calling
Enable(). Calling Disable() on a body which is already disabled has no
effect.

See also:
    Enable().

Raises:
    RuntimeError if the passed in context isn't compatible with the
    MultibodyPlant that owns this body.

Raises:
    RuntimeError if context is null.)""";
        } Disable;
        // Symbol: drake::multibody::DeformableBody::Enable
        struct /* Enable */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc =
R"""(Enables this deformable body in the given context. Calling Enable() on
a body which is already enabled has no effect.

See also:
    Disable().

Raises:
    RuntimeError if the passed in context isn't compatible with the
    MultibodyPlant that owns this body.

Raises:
    RuntimeError if context is null.)""";
        } Enable;
        // Symbol: drake::multibody::DeformableBody::GetPositions
        struct /* GetPositions */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc =
R"""(Copies out the matrix of vertex positions for this deformable body in
the provided ``context``.

Parameter ``context``:
    The context associated with the MultibodyPlant that owns this
    body.

Returns ``q``:
    A 3×N matrix containing the positions of all vertices of the body.

Raises:
    RuntimeError if ``context`` does not belong to the MultibodyPlant
    that owns this body.)""";
        } GetPositions;
        // Symbol: drake::multibody::DeformableBody::GetPositionsAndVelocities
        struct /* GetPositionsAndVelocities */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc =
R"""(Copies out the matrix of vertex positions and velocities for this
deformable body in the provided ``context``. The first N columns are
the positions and the next N columns are the velocities.

Parameter ``context``:
    The context associated with the MultibodyPlant that owns this
    body.

Returns:
    A 3x2N matrix containing the positions and velocities of all
    vertices of the body.

Raises:
    RuntimeError if ``context`` does not belong to the MultibodyPlant
    that owns this body.)""";
        } GetPositionsAndVelocities;
        // Symbol: drake::multibody::DeformableBody::GetVelocities
        struct /* GetVelocities */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc =
R"""(Copies out the matrix of vertex velocities for this deformable body in
the provided ``context``.

Parameter ``context``:
    The context associated with the MultibodyPlant that owns this
    body.

Returns ``v``:
    A 3×N matrix containing the velocities of all vertices of the
    body.

Raises:
    RuntimeError if ``context`` does not belong to the MultibodyPlant
    that owns this body.)""";
        } GetVelocities;
        // Symbol: drake::multibody::DeformableBody::SetPositions
        struct /* SetPositions */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc =
R"""(Sets the vertex positions of this deformable body in the provided
``context``.

Parameter ``out``:
    ] context The context associated with the MultibodyPlant that owns
    this body.

Parameter ``q``:
    A 3×N matrix of vertex positions.

Raises:
    RuntimeError if any of the following conditions are met: 1.
    ``context`` is nullptr. 2. ``context`` does not belong to the
    MultibodyPlant that owns this body. 3. The number of columns of
    ``q`` does not match the number of vertices of this body. 4. ``q``
    contains non-finite values.)""";
        } SetPositions;
        // Symbol: drake::multibody::DeformableBody::SetPositionsAndVelocities
        struct /* SetPositionsAndVelocities */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc =
R"""(Sets the vertex positions and velocities of this deformable body in
the provided ``context``.

Parameter ``out``:
    ] context The context associated with the MultibodyPlant that owns
    this body.

Parameter ``q``:
    A 3×N matrix of vertex positions.

Parameter ``v``:
    A 3×N matrix of vertex velocities.

Raises:
    RuntimeError if any of the following conditions are met: 1.
    ``context`` is nullptr. 2. ``context`` does not belong to the
    MultibodyPlant that owns this body. 3. The number of columns of
    ``q`` or ``v`` does not match the number of vertices of this body.
    4. ``q`` or ``v`` contains non-finite values.)""";
        } SetPositionsAndVelocities;
        // Symbol: drake::multibody::DeformableBody::SetVelocities
        struct /* SetVelocities */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc =
R"""(Sets the vertex velocities of this deformable body in the provided
``context``.

Parameter ``out``:
    ] context The context associated with the MultibodyPlant that owns
    this body.

Parameter ``v``:
    A 3×N matrix of vertex velocities.

Raises:
    RuntimeError if any of the following conditions are met: 1.
    ``context`` is nullptr. 2. ``context`` does not belong to the
    MultibodyPlant that owns this body. 3. The number of columns of
    ``v`` does not match the number of vertices of this body. 4. ``v``
    contains non-finite values.)""";
        } SetVelocities;
        // Symbol: drake::multibody::DeformableBody::SetWallBoundaryCondition
        struct /* SetWallBoundaryCondition */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc =
R"""(Sets wall boundary conditions for this deformable body. All vertices
of the mesh of the deformable body whose reference positions are
inside the prescribed open half space are put under zero displacement
boundary conditions. The open half space is defined by a plane with
outward normal n_W. A vertex V is considered to be subject to the
boundary condition if n̂ ⋅ p_QV < 0 where Q is a point on the plane
and n̂ is normalized n_W.

Parameter ``p_WQ``:
    The position of a point Q on the plane in the world frame.

Parameter ``n_W``:
    Outward normal to the half space expressed in the world frame.

Precondition:
    n_W.norm() > 1e-10.

Warning:
    Roundoff error may cause a point very near the defining plane to
    be mischaracterized as to which side of the plane it is on.)""";
        } SetWallBoundaryCondition;
        // Symbol: drake::multibody::DeformableBody::body_id
        struct /* body_id */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc = R"""(Returns the unique body id.)""";
        } body_id;
        // Symbol: drake::multibody::DeformableBody::config
        struct /* config */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc =
R"""(Returns physical parameters of this deformable body.)""";
        } config;
        // Symbol: drake::multibody::DeformableBody::discrete_state_index
        struct /* discrete_state_index */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc =
R"""(Returns the index of the discrete state associated with this
deformable body in the MultibodyPlant that owns the body.)""";
        } discrete_state_index;
        // Symbol: drake::multibody::DeformableBody::external_forces
        struct /* external_forces */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc =
R"""(Returns all the external forces acting on this deformable body.)""";
        } external_forces;
        // Symbol: drake::multibody::DeformableBody::fem_model
        struct /* fem_model */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc =
R"""(Returns the FemModel for this deformable body.)""";
        } fem_model;
        // Symbol: drake::multibody::DeformableBody::fem_state_cache_index
        struct /* fem_state_cache_index */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc =
R"""(Returns the cache index for the FemState of this deformable body.)""";
        } fem_state_cache_index;
        // Symbol: drake::multibody::DeformableBody::fixed_constraint_specs
        struct /* fixed_constraint_specs */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc =
R"""((Internal use only) Returns a reference to the fixed constraints
registered with this deformable body.)""";
        } fixed_constraint_specs;
        // Symbol: drake::multibody::DeformableBody::geometry_id
        struct /* geometry_id */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc =
R"""(Returns the geometry id of the deformable geometry used to simulate
this deformable body.)""";
        } geometry_id;
        // Symbol: drake::multibody::DeformableBody::get_default_pose
        struct /* get_default_pose */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc =
R"""(Returns the default pose of the simulated geometry (in its reference
configuration) in the world frame W. This returns pose last set by
set_default_pose(), or the pose of the geometry in the world frame W
when the body is registered if set_default_pose() has not been called.)""";
        } get_default_pose;
        // Symbol: drake::multibody::DeformableBody::has_fixed_constraint
        struct /* has_fixed_constraint */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc =
R"""(Returns true if this deformable body is under any fixed constraint.)""";
        } has_fixed_constraint;
        // Symbol: drake::multibody::DeformableBody::index
        struct /* index */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc = R"""(Returns this element's unique index.)""";
        } index;
        // Symbol: drake::multibody::DeformableBody::is_enabled
        struct /* is_enabled */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc =
R"""(Returns:
    true if this deformable body is enabled.

Raises:
    RuntimeError if the passed in context isn't compatible with the
    MultibodyPlant that owns this body.)""";
        } is_enabled;
        // Symbol: drake::multibody::DeformableBody::is_enabled_parameter_index
        struct /* is_enabled_parameter_index */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc =
R"""(Returns the index of the boolean parameter indicating whether this
deformable body is enabled.)""";
        } is_enabled_parameter_index;
        // Symbol: drake::multibody::DeformableBody::name
        struct /* name */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc = R"""(Returns the name of the body.)""";
        } name;
        // Symbol: drake::multibody::DeformableBody::num_dofs
        struct /* num_dofs */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc =
R"""(Returns the number of degrees of freedom (DoFs) of this body.)""";
        } num_dofs;
        // Symbol: drake::multibody::DeformableBody::reference_positions
        struct /* reference_positions */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc =
R"""(Returns the reference positions of the vertices of the deformable body
identified by the given ``id``. The reference positions are the
positions of the vertices of the mesh geometry representing the body
at registration time, measured and expressed in the world frame. The
reference positions are represented as a VectorX with 3N values where
N is the number of vertices. The x-, y-, and z-positions (measured and
expressed in the world frame) of the j-th vertex are 3j, 3j + 1, and
3j + 2 in the VectorX.)""";
        } reference_positions;
        // Symbol: drake::multibody::DeformableBody::scoped_name
        struct /* scoped_name */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc =
R"""(Returns scoped name of this body. Neither of the two pieces of the
name will be empty (the scope name and the element name).

Raises:
    RuntimeError if this element is not associated with a
    MultibodyPlant.)""";
        } scoped_name;
        // Symbol: drake::multibody::DeformableBody::set_default_pose
        struct /* set_default_pose */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc =
R"""(Sets the default pose of the simulated geometry (in its reference
configuration) in the world frame W.

Parameter ``X_WD``:
    The default pose of the simulated geometry in the world frame W.)""";
        } set_default_pose;
        // Symbol: drake::multibody::DeformableBody::set_parallelism
        struct /* set_parallelism */ {
          // Source: drake/multibody/tree/deformable_body.h
          const char* doc =
R"""((Internal use only) Configures the parallelism that ``this``
DeformableBody uses when opportunities for parallel computation
arises.)""";
        } set_parallelism;
      } DeformableBody;
      // Symbol: drake::multibody::DeformableBodyId
      struct /* DeformableBodyId */ {
        // Source: drake/multibody/tree/multibody_tree_indexes.h
        const char* doc =
R"""(Type used to identify a deformable body by id within a multibody
plant.)""";
      } DeformableBodyId;
      // Symbol: drake::multibody::DeformableBodyIndex
      struct /* DeformableBodyIndex */ {
        // Source: drake/multibody/tree/multibody_tree_indexes.h
        const char* doc =
R"""(Type used to identify a deformable body by index within a multibody
plant.)""";
      } DeformableBodyIndex;
      // Symbol: drake::multibody::DoorHinge
      struct /* DoorHinge */ {
        // Source: drake/multibody/tree/door_hinge.h
        const char* doc =
R"""(This ForceElement models a revolute DoorHinge joint that could exhibit
different force/torque characteristics at different states due to the
existence of different type of torques on the joint. This class
implements a "christmas tree" accumulation of these different torques
in an empirical and unprincipled way. Specifically, different curves
are assigned to different torques to mimic their evolution based on
the joint state and some prespecified parameters.

Torques considered in this implementation include: * torsional spring
torque (τ_ts) -- position dependent * catch torque (τ_c) -- position
dependent * dynamic friction torque (τ_df) -- velocity dependent *
static friction torque (τ_sf) -- velocity dependent * viscous friction
torque (τ_vf) -- velocity dependent

We then implement two curves to approximate the progression of
different torques. A curve ``s(t, x) = tanh(x/t)`` uses the ``tanh``
function to approximate a step curve ({`x<0`: -1 ; ``x>0``: 1})
outside of ``-t < x < t``. The curve ``doublet(t, x) = 2 * s * (1 −
s²)`` is the second derivative of ``s`` scaled by ``-t²``, which
yields a lump at negative ``x`` that integrates to -1 and a lump at
positive ``x`` that integrates to 1. Finally, the total external
torque on the hinge joint would be:

``τ = τ_ts + τ_c + τ_df + τ_sf + τ_vf``.

where ``τ_ts = -k_ts * (q − qs₀)``, `τ_c = k_c * doublet(qc₀/2, q −
qc₀/2)``, `τ_df = -k_df * s(k_q̇₀, q̇)``, `τ_sf = -k_sf *
doublet(k_q̇₀, q̇)` and ``τ_vf = -k_vf * q̇``. The door is assumed to
be closed at ``q=0``, opening in the positive-q direction. Note that,
the sign of the torques depends on two elements: one is the sign of
the torque related constants and another one is the sign of the
assigned curves. For example, as defined above, the static friction
torque ``τ_sf`` should be opposite to the direction of the velocity
q̇. The catch torque ``τ_c`` should be negative when ``q < qc₀/2`` and
positive otherwise. This class applies all hinge-originating forces,
so it can be used instead of the SDF viscous damping. The users could
change the values of these different elements to obtain different
characteristics for the DoorHinge joint that the users want to model.
A jupyter notebook tool is also provided to help the users visualize
the curves and design parameters. Run ``bazel run
//bindings/pydrake/multibody:examples/door_hinge_inspector`` to bring
up the notebook.

**To give an example**, a common dishwasher door has a frictional
torque sufficient for it to rest motionless at any angle, a catch at
the top to hold it in place, a dashpot (viscous friction source) to
prevent it from swinging too fast, and a spring to counteract some of
its mass. Two figures are provided to illustrate the dishwasher
DoorHinge torque with the given default parameters. Figure 1 shows the
static characteristic of the dishwasher door. At q = 0, there exists a
negative catch torque to prevent the door from moving. After that, the
torsional spring torque will dominate to compensate part of the door
gravity. Figure 2 shows the dynamic feature of the dishwasher door at
q = 30 deg. It shows the door can be closed easily since the torque is
small when the velocity is negative. However, whenever the door
intends to open further, there will be a counter torque to prevent
that movement, which therefore keeps the door at rest. Note that, due
to the gravity, the dishwasher door will be fully open eventually.
This process can be really slow because of the default
``motion_threshold`` is set to be very small. You can change the
``motion_threshold`` parameter to adjust the time. @image html
drake/multibody/tree/images/torque_vs_angle.svg "Figure 1" @image html
drake/multibody/tree/images/torque_vs_velocity.svg "Figure 2")""";
        // Symbol: drake::multibody::DoorHinge::CalcConservativePower
        struct /* CalcConservativePower */ {
          // Source: drake/multibody/tree/door_hinge.h
          const char* doc = R"""()""";
        } CalcConservativePower;
        // Symbol: drake::multibody::DoorHinge::CalcHingeConservativePower
        struct /* CalcHingeConservativePower */ {
          // Source: drake/multibody/tree/door_hinge.h
          const char* doc =
R"""(Calculate the total conservative power with the given ``angle``,
``angular_rate``, and the internal DoorHingeConfig.)""";
        } CalcHingeConservativePower;
        // Symbol: drake::multibody::DoorHinge::CalcHingeFrictionalTorque
        struct /* CalcHingeFrictionalTorque */ {
          // Source: drake/multibody/tree/door_hinge.h
          const char* doc =
R"""(Calculates the total frictional torque with the given ``angular_rate``
and the internal DoorHingeConfig.)""";
        } CalcHingeFrictionalTorque;
        // Symbol: drake::multibody::DoorHinge::CalcHingeNonConservativePower
        struct /* CalcHingeNonConservativePower */ {
          // Source: drake/multibody/tree/door_hinge.h
          const char* doc =
R"""(Calculate the total non-conservative power with the given
``angular_rate`` and the internal DoorHingeConfig.)""";
        } CalcHingeNonConservativePower;
        // Symbol: drake::multibody::DoorHinge::CalcHingeSpringTorque
        struct /* CalcHingeSpringTorque */ {
          // Source: drake/multibody/tree/door_hinge.h
          const char* doc =
R"""(Calculate the total spring related torque with the given ``angle`` and
the internal DoorHingeConfig.)""";
        } CalcHingeSpringTorque;
        // Symbol: drake::multibody::DoorHinge::CalcHingeStoredEnergy
        struct /* CalcHingeStoredEnergy */ {
          // Source: drake/multibody/tree/door_hinge.h
          const char* doc =
R"""(Calculate the total potential energy of the DoorHinge ForceElement
with the given ``angle`` and the internal DoorHingeConfig.)""";
        } CalcHingeStoredEnergy;
        // Symbol: drake::multibody::DoorHinge::CalcHingeTorque
        struct /* CalcHingeTorque */ {
          // Source: drake/multibody/tree/door_hinge.h
          const char* doc =
R"""(Calculate the total torque with the given ``angle`` and
``angular_rate`` and the internal DoorHingeConfig.)""";
        } CalcHingeTorque;
        // Symbol: drake::multibody::DoorHinge::CalcNonConservativePower
        struct /* CalcNonConservativePower */ {
          // Source: drake/multibody/tree/door_hinge.h
          const char* doc = R"""()""";
        } CalcNonConservativePower;
        // Symbol: drake::multibody::DoorHinge::CalcPotentialEnergy
        struct /* CalcPotentialEnergy */ {
          // Source: drake/multibody/tree/door_hinge.h
          const char* doc = R"""()""";
        } CalcPotentialEnergy;
        // Symbol: drake::multibody::DoorHinge::DoCalcAndAddForceContribution
        struct /* DoCalcAndAddForceContribution */ {
          // Source: drake/multibody/tree/door_hinge.h
          const char* doc = R"""()""";
        } DoCalcAndAddForceContribution;
        // Symbol: drake::multibody::DoorHinge::DoCloneToScalar
        struct /* DoCloneToScalar */ {
          // Source: drake/multibody/tree/door_hinge.h
          const char* doc = R"""()""";
        } DoCloneToScalar;
        // Symbol: drake::multibody::DoorHinge::DoShallowClone
        struct /* DoShallowClone */ {
          // Source: drake/multibody/tree/door_hinge.h
          const char* doc = R"""()""";
        } DoShallowClone;
        // Symbol: drake::multibody::DoorHinge::DoorHinge<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/door_hinge.h
          const char* doc =
R"""(Constructs a hinge force element with parameters ``config`` applied to
the specified ``joint``. It will throw an exception if the
DoorHingeConfig is invalid.)""";
        } ctor;
        // Symbol: drake::multibody::DoorHinge::config
        struct /* config */ {
          // Source: drake/multibody/tree/door_hinge.h
          const char* doc = R"""()""";
        } config;
        // Symbol: drake::multibody::DoorHinge::joint
        struct /* joint */ {
          // Source: drake/multibody/tree/door_hinge.h
          const char* doc =
R"""(Raises:
    RuntimeError if this element is not associated with a
    MultibodyPlant.)""";
        } joint;
      } DoorHinge;
      // Symbol: drake::multibody::DoorHingeConfig
      struct /* DoorHingeConfig */ {
        // Source: drake/multibody/tree/door_hinge.h
        const char* doc =
R"""(Configuration structure for the DoorHinge.)""";
        // Symbol: drake::multibody::DoorHingeConfig::DoorHingeConfig
        struct /* ctor */ {
          // Source: drake/multibody/tree/door_hinge.h
          const char* doc =
R"""(Initialize to empirically reasonable values measured approximately by
banging on the door of a dishwasher with a force gauge.)""";
        } ctor;
        // Symbol: drake::multibody::DoorHingeConfig::Serialize
        struct /* Serialize */ {
          // Source: drake/multibody/tree/door_hinge.h
          const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
        } Serialize;
        // Symbol: drake::multibody::DoorHingeConfig::catch_torque
        struct /* catch_torque */ {
          // Source: drake/multibody/tree/door_hinge.h
          const char* doc =
R"""(k_c maximum catch torque applied over ``catch_width`` [Nm]. It should
be non-negative.)""";
        } catch_torque;
        // Symbol: drake::multibody::DoorHingeConfig::catch_width
        struct /* catch_width */ {
          // Source: drake/multibody/tree/door_hinge.h
          const char* doc =
R"""(qc₀ measured from closed (q=0) position [radian]. It should be
non-negative.)""";
        } catch_width;
        // Symbol: drake::multibody::DoorHingeConfig::dynamic_friction_torque
        struct /* dynamic_friction_torque */ {
          // Source: drake/multibody/tree/door_hinge.h
          const char* doc =
R"""(k_df maximum dynamic friction torque measured opposite direction of
motion [Nm]. It should be non-negative.)""";
        } dynamic_friction_torque;
        // Symbol: drake::multibody::DoorHingeConfig::motion_threshold
        struct /* motion_threshold */ {
          // Source: drake/multibody/tree/door_hinge.h
          const char* doc =
R"""(k_q̇₀ motion threshold to start to apply friction torques [rad/s]. It
should be non-negative. Realistic frictional force is very stiff,
reversing entirely over zero change in position or velocity, which
kills integrators. We approximate it with a continuous function. This
constant [rad/s] is the scaling factor on that function -- very
approximately the rad/s at which half of the full frictional force is
applied. This number is nonphysical; make it small but not so small
that the simulation vibrates or explodes.)""";
        } motion_threshold;
        // Symbol: drake::multibody::DoorHingeConfig::spring_constant
        struct /* spring_constant */ {
          // Source: drake/multibody/tree/door_hinge.h
          const char* doc =
R"""(k_ts torsional spring constant measured toward the spring zero angle
[Nm/rad]. It should be non-negative.)""";
        } spring_constant;
        // Symbol: drake::multibody::DoorHingeConfig::spring_zero_angle_rad
        struct /* spring_zero_angle_rad */ {
          // Source: drake/multibody/tree/door_hinge.h
          const char* doc =
R"""(qs₀ measured outward from the closed position [radian].)""";
        } spring_zero_angle_rad;
        // Symbol: drake::multibody::DoorHingeConfig::static_friction_torque
        struct /* static_friction_torque */ {
          // Source: drake/multibody/tree/door_hinge.h
          const char* doc =
R"""(k_sf maximum static friction measured opposite direction of motion
[Nm]. It should be non-negative.)""";
        } static_friction_torque;
        // Symbol: drake::multibody::DoorHingeConfig::viscous_friction
        struct /* viscous_friction */ {
          // Source: drake/multibody/tree/door_hinge.h
          const char* doc =
R"""(k_vf viscous friction measured opposite direction of motion [Nm]. It
should be non-negative.)""";
        } viscous_friction;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("catch_torque", catch_torque.doc),
            std::make_pair("catch_width", catch_width.doc),
            std::make_pair("dynamic_friction_torque", dynamic_friction_torque.doc),
            std::make_pair("motion_threshold", motion_threshold.doc),
            std::make_pair("spring_constant", spring_constant.doc),
            std::make_pair("spring_zero_angle_rad", spring_zero_angle_rad.doc),
            std::make_pair("static_friction_torque", static_friction_torque.doc),
            std::make_pair("viscous_friction", viscous_friction.doc),
          };
        }
      } DoorHingeConfig;
      // Symbol: drake::multibody::FixedOffsetFrame
      struct /* FixedOffsetFrame */ {
        // Source: drake/multibody/tree/fixed_offset_frame.h
        const char* doc =
R"""(%FixedOffsetFrame represents a material frame F whose pose is fixed
with respect to a *parent* material frame P. The pose offset is given
by a spatial transform ``X_PF``, which is constant after construction.
For instance, we could rigidly attach a frame F to move with a rigid
body B at a fixed pose ``X_BF``, where B is the RigidBodyFrame
associated with body B. Thus, the World frame pose ``X_WF`` of a
FixedOffsetFrame F depends only on the World frame pose ``X_WP`` of
its parent P, and the constant pose ``X_PF``, with ``X_WF=X_WP*X_PF``.

For more information about spatial transforms, see
multibody_spatial_pose.)""";
        // Symbol: drake::multibody::FixedOffsetFrame::DoCalcPoseInBodyFrame
        struct /* DoCalcPoseInBodyFrame */ {
          // Source: drake/multibody/tree/fixed_offset_frame.h
          const char* doc = R"""()""";
        } DoCalcPoseInBodyFrame;
        // Symbol: drake::multibody::FixedOffsetFrame::DoCalcRotationMatrixInBodyFrame
        struct /* DoCalcRotationMatrixInBodyFrame */ {
          // Source: drake/multibody/tree/fixed_offset_frame.h
          const char* doc = R"""()""";
        } DoCalcRotationMatrixInBodyFrame;
        // Symbol: drake::multibody::FixedOffsetFrame::DoCloneToScalar
        struct /* DoCloneToScalar */ {
          // Source: drake/multibody/tree/fixed_offset_frame.h
          const char* doc =
R"""(Precondition:
    The parent frame to this frame already has a clone in
    ``tree_clone``.)""";
        } DoCloneToScalar;
        // Symbol: drake::multibody::FixedOffsetFrame::DoShallowClone
        struct /* DoShallowClone */ {
          // Source: drake/multibody/tree/fixed_offset_frame.h
          const char* doc = R"""()""";
        } DoShallowClone;
        // Symbol: drake::multibody::FixedOffsetFrame::FixedOffsetFrame<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/fixed_offset_frame.h
          const char* doc_4args =
R"""(Creates a material Frame F whose pose is fixed with respect to its
parent material Frame P. The pose is given by a spatial transform
``X_PF``; see class documentation for more information.

Parameter ``name``:
    The name of this frame. Cannot be empty.

Parameter ``P``:
    The frame to which this frame is attached with a fixed pose.

Parameter ``X_PF``:
    The *default* transform giving the pose of F in P, therefore only
    the value (as a RigidTransform<double>) is provided.

Parameter ``model_instance``:
    The model instance to which this frame belongs to. If unspecified,
    will use P.model_instance().)""";
          // Source: drake/multibody/tree/fixed_offset_frame.h
          const char* doc_3args =
R"""(Creates a material Frame F whose pose is fixed with respect to the
RigidBodyFrame B of the given RigidBody, which serves as F's parent
frame. The pose is given by a RigidTransform ``X_BF``; see class
documentation for more information.

Parameter ``name``:
    The name of this frame. Cannot be empty.

Parameter ``bodyB``:
    The body whose RigidBodyFrame B is to be F's parent frame.

Parameter ``X_BF``:
    The transform giving the pose of F in B.)""";
        } ctor;
        // Symbol: drake::multibody::FixedOffsetFrame::GetFixedPoseInBodyFrame
        struct /* GetFixedPoseInBodyFrame */ {
          // Source: drake/multibody/tree/fixed_offset_frame.h
          const char* doc =
R"""(Returns:
    The default fixed pose in the body frame.)""";
        } GetFixedPoseInBodyFrame;
        // Symbol: drake::multibody::FixedOffsetFrame::GetFixedRotationMatrixInBodyFrame
        struct /* GetFixedRotationMatrixInBodyFrame */ {
          // Source: drake/multibody/tree/fixed_offset_frame.h
          const char* doc =
R"""(Returns:
    The default rotation matrix of this fixed pose in the body frame.)""";
        } GetFixedRotationMatrixInBodyFrame;
        // Symbol: drake::multibody::FixedOffsetFrame::GetPoseInParentFrame
        struct /* GetPoseInParentFrame */ {
          // Source: drake/multibody/tree/fixed_offset_frame.h
          const char* doc =
R"""(Returns the rigid transform X_PF that characterizes ``this`` frame F's
pose in its parent frame P.

Parameter ``context``:
    of the multibody plant associated with this frame.)""";
        } GetPoseInParentFrame;
        // Symbol: drake::multibody::FixedOffsetFrame::SetPoseInParentFrame
        struct /* SetPoseInParentFrame */ {
          // Source: drake/multibody/tree/fixed_offset_frame.h
          const char* doc =
R"""(Sets the pose of ``this`` frame F in its parent frame P.

Parameter ``context``:
    of the multibody plant associated with this frame.

Parameter ``X_PF``:
    Rigid transform that characterizes ``this`` frame F's pose
    (orientation and position) in its parent frame P.)""";
        } SetPoseInParentFrame;
        // Symbol: drake::multibody::FixedOffsetFrame::parent_frame
        struct /* parent_frame */ {
          // Source: drake/multibody/tree/fixed_offset_frame.h
          const char* doc =
R"""(Returns:
    The parent frame to which this frame is attached.)""";
        } parent_frame;
      } FixedOffsetFrame;
      // Symbol: drake::multibody::ForceDensityField
      struct /* ForceDensityField */ {
        // Source: drake/multibody/tree/force_density_field.h
        const char* doc =
R"""(Implementations of the ForceDensityFieldBase class should inherit from
this class. This class provides the functionality for a force density
field to depend on context-dependent quantities. It also provides the
functionality to declare system resources in a MultibodyPlant.)""";
        // Symbol: drake::multibody::ForceDensityField::DeclareAbstractInputPort
        struct /* DeclareAbstractInputPort */ {
          // Source: drake/multibody/tree/force_density_field.h
          const char* doc = R"""()""";
        } DeclareAbstractInputPort;
        // Symbol: drake::multibody::ForceDensityField::DeclareCacheEntry
        struct /* DeclareCacheEntry */ {
          // Source: drake/multibody/tree/force_density_field.h
          const char* doc =
R"""(Protected LeafSystem methods exposed to declare system resources in a
MultibodyPlant. DoDeclareCacheEntries() and DoDeclareInputPorts() can
use these to declare cache entries and input ports.)""";
        } DeclareCacheEntry;
        // Symbol: drake::multibody::ForceDensityField::DeclareSystemResources
        struct /* DeclareSystemResources */ {
          // Source: drake/multibody/tree/force_density_field.h
          const char* doc = R"""()""";
        } DeclareSystemResources;
        // Symbol: drake::multibody::ForceDensityField::DeclareVectorInputPort
        struct /* DeclareVectorInputPort */ {
          // Source: drake/multibody/tree/force_density_field.h
          const char* doc = R"""()""";
        } DeclareVectorInputPort;
        // Symbol: drake::multibody::ForceDensityField::DoDeclareCacheEntries
        struct /* DoDeclareCacheEntries */ {
          // Source: drake/multibody/tree/force_density_field.h
          const char* doc =
R"""(NVI implementations for declaring system resources. Defaults to no-op.
Derived classes should override the default implementation if the
external force field is Context-dependent.)""";
        } DoDeclareCacheEntries;
        // Symbol: drake::multibody::ForceDensityField::DoDeclareInputPorts
        struct /* DoDeclareInputPorts */ {
          // Source: drake/multibody/tree/force_density_field.h
          const char* doc = R"""()""";
        } DoDeclareInputPorts;
        // Symbol: drake::multibody::ForceDensityField::ForceDensityField<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/force_density_field.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::multibody::ForceDensityField::has_parent_system
        struct /* has_parent_system */ {
          // Source: drake/multibody/tree/force_density_field.h
          const char* doc =
R"""(Returns true iff ``this`` external force is owned by a MultibodyPlant.)""";
        } has_parent_system;
        // Symbol: drake::multibody::ForceDensityField::parent_system_or_throw
        struct /* parent_system_or_throw */ {
          // Source: drake/multibody/tree/force_density_field.h
          const char* doc =
R"""(Returns the owning MultibodyPlant LeafSystem.

Raises:
    RuntimeError if ``this`` force density field is not owned by any
    system.)""";
        } parent_system_or_throw;
      } ForceDensityField;
      // Symbol: drake::multibody::ForceElement
      struct /* ForceElement */ {
        // Source: drake/multibody/tree/force_element.h
        const char* doc =
R"""(A ForceElement allows modeling state and time dependent forces in a
MultibodyTree model. Examples of such forces are springs, dampers,
drag and gravity. Forces that depend on accelerations such as virtual
mass cannot be modeled with a ForceElement. This abstract class
provides an API that all force elements subclasses must implement in
order to be fully defined. These are:

- CalcAndAddForceContribution(): computes the force contribution of a force
  element in a MultibodyTree model.
- CalcPotentialEnergy(): computes a force element potential energy
  contribution.
- CalcConservativePower(): computes the power generated by conservative
  forces.
- CalcNonConservativePower(): computes the power dissipated by
  non-conservative forces.)""";
        // Symbol: drake::multibody::ForceElement::CalcAndAddForceContribution
        struct /* CalcAndAddForceContribution */ {
          // Source: drake/multibody/tree/force_element.h
          const char* doc =
R"""((Advanced) Computes the force contribution for ``this`` force element
and **adds** it to the output arrays of forces. Depending on their
model, different force elements may write into the array of spatial
forces ``F_B_W`` or the array of generalized forces ``tau``.

Parameter ``context``:
    The context containing the state of the MultibodyTree model.

Parameter ``pc``:
    A position kinematics cache object already updated to be in sync
    with ``context``.

Parameter ``vc``:
    A velocity kinematics cache object already updated to be in sync
    with ``context``.

Parameter ``forces``:
    A pointer to a valid, non nullptr, multibody forces object. On
    output ``this`` force element adds its contribution into
    ``forces``. This method will abort if the ``forces`` pointer is
    null or if the forces object is not compatible with ``this``
    MultibodyTree, see MultibodyForces::CheckInvariants().

Precondition:
    The position kinematics ``pc`` must have been previously updated
    with a call to CalcPositionKinematicsCache().

Precondition:
    The velocity kinematics ``vc`` must have been previously updated
    with a call to CalcVelocityKinematicsCache().)""";
        } CalcAndAddForceContribution;
        // Symbol: drake::multibody::ForceElement::CalcConservativePower
        struct /* CalcConservativePower */ {
          // Source: drake/multibody/tree/force_element.h
          const char* doc =
R"""((Advanced) Calculates and returns the power generated by conservative
force elements or zero if ``this`` force element is non-conservative.
This quantity is defined to be positive when the potential energy is
decreasing. In other words, if ``PE`` is the potential energy as
defined by CalcPotentialEnergy(), then the conservative power, ``Pc``,
is ``Pc = -d(PE)/dt``.

See also:
    CalcPotentialEnergy(), CalcNonConservativePower())""";
        } CalcConservativePower;
        // Symbol: drake::multibody::ForceElement::CalcNonConservativePower
        struct /* CalcNonConservativePower */ {
          // Source: drake/multibody/tree/force_element.h
          const char* doc =
R"""((Advanced) Calculates the rate at which mechanical energy is being
generated (positive) or dissipated (negative) *other than* by
conversion between potential and kinetic energy. Integrating this
quantity yields work W, and the total energy ``E = PE + KE - W``
should be conserved by any physically-correct model, to within
integration accuracy of W.

See also:
    CalcConservativePower())""";
        } CalcNonConservativePower;
        // Symbol: drake::multibody::ForceElement::CalcPotentialEnergy
        struct /* CalcPotentialEnergy */ {
          // Source: drake/multibody/tree/force_element.h
          const char* doc =
R"""((Advanced) Calculates the potential energy currently stored given the
configuration provided in ``context``. Non-conservative force elements
will return zero.

Parameter ``context``:
    The context containing the state of the MultibodyTree model.

Parameter ``pc``:
    A position kinematics cache object already updated to be in sync
    with ``context``.

Precondition:
    The position kinematics ``pc`` must have been previously updated
    with a call to CalcPositionKinematicsCache().

Returns:
    For conservative force models, the potential energy stored by
    ``this`` force element. For non-conservative force models, zero.

See also:
    CalcConservativePower())""";
        } CalcPotentialEnergy;
        // Symbol: drake::multibody::ForceElement::CloneToScalar
        struct /* CloneToScalar */ {
          // Source: drake/multibody/tree/force_element.h
          const char* doc = R"""()""";
        } CloneToScalar;
        // Symbol: drake::multibody::ForceElement::DoCalcAndAddForceContribution
        struct /* DoCalcAndAddForceContribution */ {
          // Source: drake/multibody/tree/force_element.h
          const char* doc =
R"""(This method is called only from the public non-virtual
CalcAndAddForceContributions() which will already have error-checked
the parameters so you don't have to. Refer to the documentation for
CalcAndAddForceContribution() for details describing the purpose and
parameters of this method. It assumes ``forces`` to be a valid pointer
to a MultibodyForces object compatible with the MultibodyTree model
owning ``this`` force element.

Precondition:
    The position kinematics ``pc`` must have been previously updated
    with a call to CalcPositionKinematicsCache().

Precondition:
    The velocity kinematics ``vc`` must have been previously updated
    with a call to CalcVelocityKinematicsCache().)""";
        } DoCalcAndAddForceContribution;
        // Symbol: drake::multibody::ForceElement::DoCloneToScalar
        struct /* DoCloneToScalar */ {
          // Source: drake/multibody/tree/force_element.h
          const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Clones this ForceElement (templated on T) to a mobilizer templated on
``double``.)""";
        } DoCloneToScalar;
        // Symbol: drake::multibody::ForceElement::DoDeclareForceElementParameters
        struct /* DoDeclareForceElementParameters */ {
          // Source: drake/multibody/tree/force_element.h
          const char* doc =
R"""(Called by DoDeclareParameters(). Derived classes may choose to
override to declare their sub-class specific parameters.)""";
        } DoDeclareForceElementParameters;
        // Symbol: drake::multibody::ForceElement::DoSetDefaultForceElementParameters
        struct /* DoSetDefaultForceElementParameters */ {
          // Source: drake/multibody/tree/force_element.h
          const char* doc =
R"""(Called by DoSetDefaultParameters(). Derived classes may choose to
override to set their sub-class specific parameters.)""";
        } DoSetDefaultForceElementParameters;
        // Symbol: drake::multibody::ForceElement::DoShallowClone
        struct /* DoShallowClone */ {
          // Source: drake/multibody/tree/force_element.h
          const char* doc = R"""(NVI for ShallowClone().)""";
        } DoShallowClone;
        // Symbol: drake::multibody::ForceElement::ForceElement<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/force_element.h
          const char* doc =
R"""(Default constructor for a generic force element.)""";
        } ctor;
        // Symbol: drake::multibody::ForceElement::ShallowClone
        struct /* ShallowClone */ {
          // Source: drake/multibody/tree/force_element.h
          const char* doc = R"""()""";
        } ShallowClone;
        // Symbol: drake::multibody::ForceElement::index
        struct /* index */ {
          // Source: drake/multibody/tree/force_element.h
          const char* doc = R"""(Returns this element's unique index.)""";
        } index;
      } ForceElement;
      // Symbol: drake::multibody::ForceElementIndex
      struct /* ForceElementIndex */ {
        // Source: drake/multibody/tree/multibody_tree_indexes.h
        const char* doc =
R"""(Type used to identify force elements by index within a multibody
plant.)""";
      } ForceElementIndex;
      // Symbol: drake::multibody::Frame
      struct /* Frame */ {
        // Source: drake/multibody/tree/frame.h
        const char* doc =
R"""(%Frame is an abstract class representing a *material frame* (also
called a *physical frame*) of its underlying RigidBody. The Frame's
origin is a material point of its RigidBody, and its axes have fixed
directions in that body. A Frame's pose (position and orientation)
with respect to its RigidBodyFrame may be parameterized, but is fixed
(not time or state dependent) once parameters have been set.

An important characteristic of a Frame is that forces or torques
applied to a Frame are applied to the Frame's underlying RigidBody.
Force-producing elements like joints, actuators, and constraints
usually employ two Frames, with one Frame connected to one body and
the other connected to a different body. Every Frame F can report the
RigidBody B to which it is attached and its pose X_BF with respect to
B's RigidBodyFrame.

A Frame's pose in World (or relative to other frames) is always
calculated starting with its pose relative to its underlying
RigidBodyFrame. Subclasses derived from Frame differ in how kinematic
calculations are performed. For example, the angular velocity of a
FixedOffsetFrame or RigidBodyFrame is identical to the angular
velocity of its underlying body, whereas the translational velocity of
a FixedOffsetFrame differs from that of a RigidBodyFrame.

Frame provides methods for obtaining its current orientation,
position, motion, etc. from a Context passed to those methods.)""";
        // Symbol: drake::multibody::Frame::CalcAngularVelocity
        struct /* CalcAngularVelocity */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""(Calculates ``this`` frame F's angular velocity measured in a frame M,
expressed in a frame E.

Parameter ``context``:
    contains the state of the multibody system.

Parameter ``measured_in_frame``:
    which is frame M (the frame in which ``this`` angular velocity is
    to be measured).

Parameter ``expressed_in_frame``:
    which is frame E (the frame in which the returned angular velocity
    is to be expressed).

Returns:
    ω_MF_E, ``this`` frame F's angular velocity ω measured in frame M,
    expressed in frame E.

See also:
    EvalAngularVelocityInWorld() to evaluate ω_WF_W (``this`` frame
    F's angular velocity ω measured and expressed in the world frame
    W).)""";
        } CalcAngularVelocity;
        // Symbol: drake::multibody::Frame::CalcOffsetPoseInBody
        struct /* CalcOffsetPoseInBody */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""(Given the offset pose ``X_FQ`` of a frame Q in ``this`` frame F, this
method computes the pose ``X_BQ`` of frame Q in the body frame B to
which this frame is attached. In other words, if the pose of ``this``
frame F in the body frame B is ``X_BF``, this method computes the pose
``X_BQ`` of frame Q in the body frame B as ``X_BQ = X_BF * X_FQ``. In
particular, if ``this`` **is** the body frame B, i.e. ``X_BF`` is the
identity transformation, this method directly returns ``X_FQ``.
Specific frame subclasses can override this method to provide faster
implementations if needed.)""";
        } CalcOffsetPoseInBody;
        // Symbol: drake::multibody::Frame::CalcOffsetRotationMatrixInBody
        struct /* CalcOffsetRotationMatrixInBody */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""(Calculates and returns the rotation matrix ``R_BQ`` that relates body
frame B to frame Q via ``this`` intermediate frame F, i.e., ``R_BQ =
R_BF * R_FQ`` (B is the body frame to which ``this`` frame F is
attached).

Parameter ``R_FQ``:
    rotation matrix that relates frame F to frame Q.)""";
        } CalcOffsetRotationMatrixInBody;
        // Symbol: drake::multibody::Frame::CalcPose
        struct /* CalcPose */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""(Computes and returns the pose ``X_MF`` of ``this`` frame F in measured
in ``frame_M`` as a function of the state of the model stored in
``context``.

See also:
    CalcPoseInWorld().)""";
        } CalcPose;
        // Symbol: drake::multibody::Frame::CalcPoseInBodyFrame
        struct /* CalcPoseInBodyFrame */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""(Returns the pose ``X_BF`` of ``this`` frame F in the body frame B
associated with this frame. In particular, if ``this`` **is** the body
frame B, this method directly returns the identity transformation.
Note that this ONLY depends on the Parameters in the context; it does
not depend on time, input, state, etc.)""";
        } CalcPoseInBodyFrame;
        // Symbol: drake::multibody::Frame::CalcPoseInWorld
        struct /* CalcPoseInWorld */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""(Computes and returns the pose ``X_WF`` of ``this`` frame F in the
world frame W as a function of the state of the model stored in
``context``.

Note:
    RigidBody::EvalPoseInWorld() provides a more efficient way to
    obtain the pose for a body frame.)""";
        } CalcPoseInWorld;
        // Symbol: drake::multibody::Frame::CalcRelativeSpatialAcceleration
        struct /* CalcRelativeSpatialAcceleration */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""(Calculates ``this`` frame C's spatial acceleration relative to another
frame B, measured in a frame M, expressed in a frame E.

Parameter ``context``:
    contains the state of the multibody system.

Parameter ``other_frame``:
    which is frame B.

Parameter ``measured_in_frame``:
    which is frame M.

Parameter ``expressed_in_frame``:
    which is frame E.

Returns:
    A_M_BC_E = A_MC_E - A_MB_E, frame C's spatial acceleration
    relative to frame B, measured in frame M, expressed in frame E.

In general, A_M_BC = DtW(V_M_BC), the time-derivative in frame M of
frame C's spatial velocity relative to frame B. The rotational part of
the returned quantity is α_MC_E - α_MB_E = DtM(ω_BC)_E. Note: For 3D
analysis, DtM(ω_BC) ≠ α_BC. The translational part of the returned
quantity is a_M_BoCo_E (Co's translational acceleration relative to
Bo, measured in frame M, expressed in frame E).


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    α_MC_E - α_MB_E = DtM(ω_MC)_E - DtM(ω_MB)_E = DtM(ω_BC)_E
     a_M_BoCo_E = a_MCo_E - a_MBo_E = DtM(v_MCo) - DtM(v_MBo) = Dt²M(p_BoCo)_E

.. raw:: html

    </details>

where Dt²M(p_BoCo)_E is the 2ⁿᵈ time-derivative in frame M of p_BoCo
(the position vector from Bo to Co), and this result is expressed in
frame E.

Note:
    The calculation of the 2ⁿᵈ time-derivative of the distance between
    Bo and Co can be done with relative translational acceleration,
    but this calculation does not depend on the measured-in-frame,
    hence in this case, consider
    CalcRelativeSpatialAccelerationInWorld() since it is faster.

See also:
    CalcSpatialAccelerationInWorld(), CalcSpatialAcceleration(), and
    CalcRelativeSpatialAccelerationInWorld().)""";
        } CalcRelativeSpatialAcceleration;
        // Symbol: drake::multibody::Frame::CalcRelativeSpatialAccelerationInWorld
        struct /* CalcRelativeSpatialAccelerationInWorld */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""(Calculates ``this`` frame C's spatial acceleration relative to another
frame B, measured and expressed in the world frame W.

Parameter ``context``:
    contains the state of the multibody system.

Parameter ``other_frame``:
    which is frame B.

Returns:
    A_W_BC_W = A_WC_W - A_WB_W, frame C's spatial acceleration
    relative to frame B, measured and expressed in the world frame W.

In general, A_W_BC = DtW(V_W_BC), the time-derivative in the world
frame W of frame C's spatial velocity relative to frame B. The
rotational part of the returned quantity is α_WC_W - α_WB_W =
DtW(ω_BC)_W. For 3D analysis, DtW(ω_BC) ≠ α_BC. The translational part
of the returned quantity is a_W_BoCo_W (Co's translational
acceleration relative to Bo, measured and expressed in world frame W).


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    α_WC_W - α_WB_W = DtW(ω_WC)_W - DtW(ω_WB)_W = DtW(ω_BC)_W
     a_W_BoCo_W = a_WCo_W - a_WBo_W = DtW(v_WCo) - DtW(v_WBo) = Dt²W(p_BoCo)_W

.. raw:: html

    </details>

where Dt²W(p_BoCo)_W is the 2ⁿᵈ time-derivative in frame W of p_BoCo
(the position vector from Bo to Co), and this result is expressed in
frame W.

Note:
    The method CalcSpatialAccelerationInWorld() is more efficient and
    coherent if any of ``this``, other_frame, or the world frame W are
    the same.

See also:
    CalcSpatialAccelerationInWorld(),
    CalcRelativeSpatialAcceleration().)""";
        } CalcRelativeSpatialAccelerationInWorld;
        // Symbol: drake::multibody::Frame::CalcRelativeSpatialVelocity
        struct /* CalcRelativeSpatialVelocity */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""(Calculates ``this`` frame C's spatial velocity relative to another
frame B, measured in a frame M, expressed in a frame E.

Parameter ``context``:
    contains the state of the multibody system.

Parameter ``other_frame``:
    which is frame B.

Parameter ``measured_in_frame``:
    which is frame M.

Parameter ``expressed_in_frame``:
    which is frame E.

Returns:
    V_M_BC_E = V_MC_E - V_MB_E, frame C's spatial velocity relative to
    frame B, measured in frame M, expressed in frame E. The rotational
    part of the returned quantity is ω_BC_E (C's angular velocity
    measured in B and expressed in E). The translational part is
    v_M_BoCo_E (Co's translational velocity relative to Bo, measured
    in M, and expressed in E).


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ω_BC_E = ω_MC_E - ω_MB_E
     v_M_BoCo_E = v_MCo_E - v_MBo_E = DtM(p_BoCo)

.. raw:: html

    </details>

where DtM(p_BoCo) is the time-derivative in frame M of p_BoCo
(position vector from Bo to Co), and this vector is expressed in frame
E.

Note:
    The method CalcSpatialVelocity() is more efficient and coherent if
    any of ``this``, other_frame, or measured_in_frame are the same.
    Also, the value of V_M_BoCo does not depend on the
    measured_in_frame if Bo and Co are coincident (i.e., p_BoCo = 0),
    in which case consider the more efficient method
    CalcRelativeSpatialVelocityInWorld(). Lastly, the calculation of
    elongation between Bo and Co can be done with relative
    translational velocity, but elongation does not depend on the
    measured-in-frame (hence consider
    CalcRelativeSpatialVelocityInWorld()).

See also:
    CalcSpatialVelocityInWorld(), CalcSpatialVelocity(), and
    CalcRelativeSpatialVelocityInWorld().)""";
        } CalcRelativeSpatialVelocity;
        // Symbol: drake::multibody::Frame::CalcRelativeSpatialVelocityInWorld
        struct /* CalcRelativeSpatialVelocityInWorld */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""(Calculates ``this`` frame C's spatial velocity relative to another
frame B, measured and expressed in the world frame W.

Parameter ``context``:
    contains the state of the multibody system.

Parameter ``other_frame``:
    which is frame B.

Returns:
    V_W_BC_W = V_WC_W - V_WB_W, frame C's spatial velocity relative to
    frame B, measured and expressed in the world frame W. The
    rotational part of the returned quantity is ω_BC_W (C's angular
    velocity measured in B and expressed in W). The translational part
    is v_W_BoCo_W (Co's translational velocity relative to Bo,
    measured and expressed in world frame W).


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ω_BC_W  = ω_WC_W - ω_WB_W
     v_W_BoCo_W = v_WCo_W - v_WBo_W = DtW(p_BoCo)

.. raw:: html

    </details>

where DtW(p_BoCo) is the time-derivative in frame W of p_BoCo
(position vector from Bo to Co), and this vector is expressed in frame
W.

Note:
    The method CalcSpatialVelocityInWorld() is more efficient and
    coherent if any of ``this``, other_frame, or the world frame W are
    the same.

See also:
    CalcSpatialVelocityInWorld() and CalcRelativeSpatialVelocity().)""";
        } CalcRelativeSpatialVelocityInWorld;
        // Symbol: drake::multibody::Frame::CalcRotationMatrix
        struct /* CalcRotationMatrix */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""(Calculates and returns the rotation matrix ``R_MF`` that relates
``frame_M`` and ``this`` frame F as a function of the state stored in
``context``.)""";
        } CalcRotationMatrix;
        // Symbol: drake::multibody::Frame::CalcRotationMatrixInBodyFrame
        struct /* CalcRotationMatrixInBodyFrame */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""(Returns the rotation matrix ``R_BF`` that relates body frame B to
``this`` frame F (B is the body frame to which ``this`` frame F is
attached).

Note:
    If ``this`` is B, this method returns the identity RotationMatrix.
    Note that this ONLY depends on the Parameters in the context; it
    does not depend on time, input, state, etc.)""";
        } CalcRotationMatrixInBodyFrame;
        // Symbol: drake::multibody::Frame::CalcRotationMatrixInWorld
        struct /* CalcRotationMatrixInWorld */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""(Calculates and returns the rotation matrix ``R_WF`` that relates the
world frame W and ``this`` frame F as a function of the state stored
in ``context``.)""";
        } CalcRotationMatrixInWorld;
        // Symbol: drake::multibody::Frame::CalcSpatialAcceleration
        struct /* CalcSpatialAcceleration */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""(Calculates ``this`` frame F's spatial acceleration measured in a frame
M, expressed in a frame E.

Parameter ``context``:
    contains the state of the multibody system.

Parameter ``measured_in_frame``:
    which is frame M.

Parameter ``expressed_in_frame``:
    which is frame E.

Returns:
    A_MF_E, ``this`` frame F's spatial acceleration measured in frame
    M, expressed in frame E. The rotational part of the returned
    quantity is α_MF_E (frame F's angular acceleration α measured in
    frame M, expressed in frame E). The translational part is a_MFo_E
    (translational acceleration of frame F's origin point Fo, measured
    in frame M, expressed in frame E). Although α_MF is defined below
    in terms of DtM(ω_MF), the time-derivative in frame M of ω_MF, the
    actual calculation of α_MF avoids differentiation. Similarly for
    the definition vs. calculation for a_MFo.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    α_MF = DtM(ω_MF)           ω_MF is frame F's angular velocity in frame M.
     a_MFo = DtM(v_MFo)    v_MF is Fo's translational acceleration in frame M.

.. raw:: html

    </details>

See also:
    CalcSpatialAccelerationInWorld() and CalcSpatialVelocity().)""";
        } CalcSpatialAcceleration;
        // Symbol: drake::multibody::Frame::CalcSpatialAccelerationInWorld
        struct /* CalcSpatialAccelerationInWorld */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""(Calculates ``this`` frame F's spatial acceleration measured and
expressed in the world frame W.

Parameter ``context``:
    contains the state of the multibody system.

Returns:
    A_WF_W, ``this`` frame F's spatial acceleration measured and
    expressed in the world frame W. The rotational part of the
    returned quantity is α_WF_E (frame F's angular acceleration α
    measured and expressed in the world frame W). The translational
    part is a_WFo_W (translational acceleration of frame F's origin
    point Fo, measured and expressed in the world frame W).

Note:
    RigidBody::EvalSpatialAccelerationInWorld() provides a more
    efficient way to obtain a body frame's spatial acceleration
    measured in the world frame.

Note:
    When cached values are out of sync with the state stored in
    context, this method performs an expensive forward dynamics
    computation, whereas once evaluated, successive calls to this
    method are inexpensive.

See also:
    CalcSpatialAcceleration() and CalcSpatialVelocityInWorld().)""";
        } CalcSpatialAccelerationInWorld;
        // Symbol: drake::multibody::Frame::CalcSpatialVelocity
        struct /* CalcSpatialVelocity */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""(Calculates ``this`` frame F's spatial velocity measured in a frame M,
expressed in a frame E.

Parameter ``context``:
    contains the state of the multibody system.

Parameter ``frame_M``:
    which is the measured_in_frame.

Parameter ``frame_E``:
    which is the expressed_in_frame.

Returns:
    V_MF_E, ``this`` frame F's spatial velocity measured in frame M,
    expressed in frame E. The rotational part of the returned quantity
    is ω_MF_E (frame F's angular velocity ω measured in frame M,
    expressed in frame E). The translational part is v_MFo_E
    (translational velocity v of frame F's origin point Fo, measured
    in frame M, expressed in frame E).

See also:
    CalcSpatialVelocityInWorld(), CalcRelativeSpatialVelocity(), and
    CalcSpatialAcceleration().)""";
        } CalcSpatialVelocity;
        // Symbol: drake::multibody::Frame::CalcSpatialVelocityInWorld
        struct /* CalcSpatialVelocityInWorld */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""(Calculates ``this`` frame F's spatial velocity measured and expressed
in the world frame W.

Parameter ``context``:
    contains the state of the multibody system.

Returns:
    V_WF_W, ``this`` frame F's spatial velocity measured and expressed
    in the world frame W. The rotational part of the returned quantity
    is ω_WF_W (frame F's angular velocity ω measured and expressed in
    the world frame W). The translational part is v_WFo_W
    (translational velocity v of frame F's origin point Fo, measured
    and expressed in the world frame W).

Note:
    RigidBody::EvalSpatialVelocityInWorld() provides a more efficient
    way to obtain a body frame's spatial velocity measured in the
    world frame.

See also:
    CalcSpatialVelocity(), CalcRelativeSpatialVelocityInWorld(), and
    CalcSpatialAccelerationInWorld().)""";
        } CalcSpatialVelocityInWorld;
        // Symbol: drake::multibody::Frame::CloneToScalar
        struct /* CloneToScalar */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""((Advanced) NVI to DoCloneToScalar() templated on the scalar type of
the new clone to be created. This method is mostly intended to be
called by MultibodyTree::CloneToScalar(). Most users should not call
this clone method directly but rather clone the entire parent
MultibodyTree if needed.

See also:
    MultibodyTree::CloneToScalar())""";
        } CloneToScalar;
        // Symbol: drake::multibody::Frame::DoCalcOffsetPoseInBody
        struct /* DoCalcOffsetPoseInBody */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc = R"""()""";
        } DoCalcOffsetPoseInBody;
        // Symbol: drake::multibody::Frame::DoCalcOffsetRotationMatrixInBody
        struct /* DoCalcOffsetRotationMatrixInBody */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc = R"""()""";
        } DoCalcOffsetRotationMatrixInBody;
        // Symbol: drake::multibody::Frame::DoCalcPoseInBodyFrame
        struct /* DoCalcPoseInBodyFrame */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc = R"""()""";
        } DoCalcPoseInBodyFrame;
        // Symbol: drake::multibody::Frame::DoCalcRotationMatrixInBodyFrame
        struct /* DoCalcRotationMatrixInBodyFrame */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc = R"""()""";
        } DoCalcRotationMatrixInBodyFrame;
        // Symbol: drake::multibody::Frame::DoCloneToScalar
        struct /* DoCloneToScalar */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Clones this Frame (templated on T) to a frame templated on ``double``.)""";
        } DoCloneToScalar;
        // Symbol: drake::multibody::Frame::DoDeclareFrameParameters
        struct /* DoDeclareFrameParameters */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""(Called by DoDeclareParameters(). Derived classes may choose to
override to declare their sub-class specific parameters.)""";
        } DoDeclareFrameParameters;
        // Symbol: drake::multibody::Frame::DoSetDefaultFrameParameters
        struct /* DoSetDefaultFrameParameters */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""(Called by DoSetDefaultParameters(). Derived classes may choose to
override to set their sub-class specific parameters.)""";
        } DoSetDefaultFrameParameters;
        // Symbol: drake::multibody::Frame::DoShallowClone
        struct /* DoShallowClone */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc = R"""(NVI for ShallowClone().)""";
        } DoShallowClone;
        // Symbol: drake::multibody::Frame::EvalAngularVelocityInWorld
        struct /* EvalAngularVelocityInWorld */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""(Evaluates ``this`` frame F's angular velocity measured and expressed
in the world frame W.

Parameter ``context``:
    contains the state of the multibody system.

Returns:
    ω_WF_W (frame F's angular velocity ω measured and expressed in the
    world frame W).

See also:
    CalcAngularVelocity() to calculate ω_MF_E (``this`` frame F's
    angular velocity ω measured in a frame M and expressed in a frame
    E).)""";
        } EvalAngularVelocityInWorld;
        // Symbol: drake::multibody::Frame::EvalPoseInBodyFrame
        struct /* EvalPoseInBodyFrame */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""(Returns a reference to the body-relative pose X_BF giving the pose of
this Frame with respect to its body's RigidBodyFrame. This may depend
on parameters in the Context but not on time or state. The first time
this is called after a parameter change will precalculate offset poses
for all Frames into the Context's cache; subsequent calls on any Frame
are very fast.)""";
        } EvalPoseInBodyFrame;
        // Symbol: drake::multibody::Frame::Frame<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""(Only derived classes can use this constructor. It creates a Frame
object attached to ``body`` and puts the frame in the body's model
instance.)""";
        } ctor;
        // Symbol: drake::multibody::Frame::GetFixedOffsetPoseInBody
        struct /* GetFixedOffsetPoseInBody */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""(Variant of CalcOffsetPoseInBody() that given the offset pose ``X_FQ``
of a frame Q in ``this`` frame F, returns the pose ``X_BQ`` of frame Q
in the body frame B to which this frame is attached.

Raises:
    RuntimeError if called on a Frame that does not have a fixed
    offset in the body frame.)""";
        } GetFixedOffsetPoseInBody;
        // Symbol: drake::multibody::Frame::GetFixedPoseInBodyFrame
        struct /* GetFixedPoseInBodyFrame */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""(Variant of CalcPoseInBodyFrame() that returns the fixed pose ``X_BF``
of ``this`` frame F in the body frame B associated with this frame.

Raises:
    RuntimeError if called on a Frame that does not have a fixed
    offset in the body frame.)""";
        } GetFixedPoseInBodyFrame;
        // Symbol: drake::multibody::Frame::GetFixedRotationMatrixInBody
        struct /* GetFixedRotationMatrixInBody */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""(Calculates and returns the rotation matrix ``R_BQ`` that relates body
frame B to frame Q via ``this`` intermediate frame F, i.e., ``R_BQ =
R_BF * R_FQ`` (B is the body frame to which ``this`` frame F is
attached).

Parameter ``R_FQ``:
    rotation matrix that relates frame F to frame Q.

Raises:
    RuntimeError if ``this`` frame F is a Frame that does not have a
    fixed offset in the body frame B (i.e., ``R_BF`` is not constant).)""";
        } GetFixedRotationMatrixInBody;
        // Symbol: drake::multibody::Frame::GetFixedRotationMatrixInBodyFrame
        struct /* GetFixedRotationMatrixInBodyFrame */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""(Returns the rotation matrix ``R_BF`` that relates body frame B to
``this`` frame F (B is the body frame to which ``this`` frame F is
attached).

Raises:
    RuntimeError if ``this`` frame F is a Frame that does not have a
    fixed offset in the body frame B (i.e., ``R_BF`` is not constant).
    Frame sub-classes that have a constant ``R_BF`` must override this
    method. An example of a frame sub-class not implementing this
    method would be that of a frame on a soft body, for which its pose
    in the body frame depends on the state of deformation of the body.)""";
        } GetFixedRotationMatrixInBodyFrame;
        // Symbol: drake::multibody::Frame::ShallowClone
        struct /* ShallowClone */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""((Internal use only) Returns a shallow clone (i.e., dependent elements
such as bodies are aliased, not copied) that is not associated with
any MbT (so the assigned index, if any, is discarded).)""";
        } ShallowClone;
        // Symbol: drake::multibody::Frame::body
        struct /* body */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""(Returns a const reference to the body associated to this Frame.)""";
        } body;
        // Symbol: drake::multibody::Frame::get_X_BF
        struct /* get_X_BF */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""((Internal use only) Given an already up-to-date frame body pose cache,
extract X_BF for this Frame from it.

Note:
    Be sure you have called MultibodyTreeSystem::EvalFrameBodyPoses()
    since the last parameter change; we can't check here.

Returns ``X_BF``:
    pose of this frame in its body's frame)""";
        } get_X_BF;
        // Symbol: drake::multibody::Frame::get_X_FB
        struct /* get_X_FB */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""((Internal use only) Given an already up-to-date frame body pose cache,
extract X_FB (=X_BF⁻¹) for this Frame from it.

Note:
    Be sure you have called MultibodyTreeSystem::EvalFrameBodyPoses()
    since the last parameter change; we can't check here.

Returns ``X_FB``:
    inverse of this frame's pose in its body's frame)""";
        } get_X_FB;
        // Symbol: drake::multibody::Frame::get_body_pose_index_in_cache
        struct /* get_body_pose_index_in_cache */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""((Internal use only) Retrieve this Frame's body pose index in the
cache.)""";
        } get_body_pose_index_in_cache;
        // Symbol: drake::multibody::Frame::index
        struct /* index */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc = R"""(Returns this element's unique index.)""";
        } index;
        // Symbol: drake::multibody::Frame::is_X_BF_identity
        struct /* is_X_BF_identity */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""((Internal use only) Given an already up-to-date frame body pose cache,
returns whether X_BF (and thus X_FB) is exactly identity. This is
precomputed in the cache so is very fast to check.

Note:
    Be sure you have called MultibodyTreeSystem::EvalFrameBodyPoses()
    since the last parameter change; we can't check here.

See also:
    get_X_BF())""";
        } is_X_BF_identity;
        // Symbol: drake::multibody::Frame::is_body_frame
        struct /* is_body_frame */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""(Returns true if ``this`` is the body frame.)""";
        } is_body_frame;
        // Symbol: drake::multibody::Frame::is_world_frame
        struct /* is_world_frame */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""(Returns true if ``this`` is the world frame.)""";
        } is_world_frame;
        // Symbol: drake::multibody::Frame::name
        struct /* name */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""(Returns the name of this frame. The name will never be empty.)""";
        } name;
        // Symbol: drake::multibody::Frame::scoped_name
        struct /* scoped_name */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""(Returns scoped name of this frame. Neither of the two pieces of the
name will be empty (the scope name and the element name).

Raises:
    RuntimeError if this element is not associated with a
    MultibodyPlant.)""";
        } scoped_name;
        // Symbol: drake::multibody::Frame::set_body_pose_index_in_cache
        struct /* set_body_pose_index_in_cache */ {
          // Source: drake/multibody/tree/frame.h
          const char* doc =
R"""((Internal use only) A Frame's pose-in-parent X_PF can be
parameterized, the parent's pose may also be parameterized, and so on.
Thus the calculation of this frame's pose in its body (X_BF) can be
expensive. There is a cache entry that holds the calculated X_BF,
evaluated whenever parameters change. This allows us to grab X_BF as a
const reference rather than having to extract and reformat parameters,
and compose with parent and ancestor poses at runtime.

At the time parameters are allocated we assign a slot in the body pose
cache entry to each Frame and record its index using this function.
(The index for a RigidBodyFrame will refer to an identity transform.)
Note that the body pose index is not necessarily the same as the Frame
index because all RigidBodyFrames can share an entry. (Of course if
you know you are working with a RigidBodyFrame you don't need to ask
about its body pose!))""";
        } set_body_pose_index_in_cache;
      } Frame;
      // Symbol: drake::multibody::FrameIndex
      struct /* FrameIndex */ {
        // Source: drake/multibody/tree/multibody_tree_indexes.h
        const char* doc =
R"""(Type used to identify frames by index in a multibody plant.)""";
      } FrameIndex;
      // Symbol: drake::multibody::GravityForceField
      struct /* GravityForceField */ {
        // Source: drake/multibody/tree/force_density_field.h
        const char* doc =
R"""(A uniform gravitational force density field for a uniform density
object. The force density f [N/m³] is given by the product of mass
density ρ [kg/m³] and gravity vector g [m/s²].)""";
        // Symbol: drake::multibody::GravityForceField::GravityForceField<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/force_density_field.h
          const char* doc =
R"""(Constructs a uniform gravitational force density field for a uniform
density object with the given ``gravity_vector`` [m/s²] and
``mass_density`` [kg/m³] in the reference (undeformed) configuration
where the reference (undeformed) configuration is defined by the input
mesh provided by the user.)""";
        } ctor;
      } GravityForceField;
      // Symbol: drake::multibody::JacobianWrtVariable
      struct /* JacobianWrtVariable */ {
        // Source: drake/multibody/tree/multibody_tree.h
        const char* doc =
R"""(Enumeration that indicates whether the Jacobian is partial
differentiation with respect to q̇ (time-derivatives of generalized
positions) or with respect to v (generalized velocities).)""";
        // Symbol: drake::multibody::JacobianWrtVariable::kQDot
        struct /* kQDot */ {
          // Source: drake/multibody/tree/multibody_tree.h
          const char* doc = R"""(J = ∂V/∂q̇)""";
        } kQDot;
        // Symbol: drake::multibody::JacobianWrtVariable::kV
        struct /* kV */ {
          // Source: drake/multibody/tree/multibody_tree.h
          const char* doc = R"""(J = ∂V/∂v)""";
        } kV;
      } JacobianWrtVariable;
      // Symbol: drake::multibody::Joint
      struct /* Joint */ {
        // Source: drake/multibody/tree/joint.h
        const char* doc =
R"""(A Joint models the kinematical relationship which characterizes the
possible relative motion between two bodies. The two bodies connected
by this Joint object are referred to as *parent* and *child* bodies.
The parent/child ordering defines the sign conventions for the
generalized coordinates and the coordinate ordering for multi-DOF
joints. A Joint is a model of a physical kinematic constraint between
two bodies, a constraint that in the real physical system does not
specify a tree ordering. @image html
drake/multibody/plant/images/BodyParentChildJoint.png width=50%

In Drake we define a frame F rigidly attached to the parent body P
with pose ``X_PF`` and a frame M rigidly attached to the child body B
with pose ``X_BM``. A Joint object specifies a kinematic relation
between frames F and M, which in turn imposes a kinematic relation
between bodies P and B.

Typical joints include the ball joint, to allow unrestricted rotations
about a given point, the revolute joint, that constraints two bodies
to rotate about a given common axis, etc.

Consider the following example to build a simple pendulum system:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    MultibodyPlant<double> plant(0.0);
    // ... Code here to setup quantities below as mass, com, etc. ...
    const RigidBody<double>& pendulum =
      plant.AddRigidBody(SpatialInertia<double>(mass, com, unit_inertia));
    // We will connect the pendulum body to the world using a RevoluteJoint.
    // In this simple case the parent body P is the model's world body and frame
    // F IS the world frame.
    // Additionally, we need to specify the pose of frame M on the pendulum's
    // body frame B.
    // Say we declared and initialized X_BM...
    const RevoluteJoint<double>& elbow =
      plant.AddJoint<RevoluteJoint>(
        "Elbow",                /* joint name 
        plant.world_body(),     /* parent body 
        {},                     /* frame F IS the world frame W 
        pendulum,               /* child body, the pendulum 
        X_BM,                   /* pose of frame M in the body frame B 
        Vector3d::UnitZ());     /* revolute axis in this case

.. raw:: html

    </details>

Warning:
    Do not ever attempt to instantiate and manipulate Joint objects on
    the stack; it will fail. Add joints to your plant using the
    provided API MultibodyPlant::AddJoint() as in the example above.

Note:
    To developers: this is the base class for all concrete Joint
    types. Extending this class to add a new Joint type necessarily
    requires working with internal implementation classes for which we
    cannot guarantee API stability due to the need for ongoing
    improvements to these performance-critical classes. So while our
    usual stability guarantees apply to the Joint ``public`` API, the
    ``protected`` API here is subject to change when the underlying
    internal objects change. Our release notes will say when we have
    made changes that might affect your Joint implementations, but we
    won't necessarily be able to provide a deprecation period.)""";
        // Symbol: drake::multibody::Joint::AddInDamping
        struct /* AddInDamping */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Adds into ``forces`` the force due to damping within ``this`` joint.

Parameter ``context``:
    The context storing the state and parameters for the model to
    which ``this`` joint belongs.

Parameter ``forces``:
    On return, this method will add the force due to damping within
    ``this`` joint. This method aborts if ``forces`` is ``nullptr`` or
    if ``forces`` does not have the right sizes to accommodate a set
    of forces for the model to which this joint belongs.)""";
        } AddInDamping;
        // Symbol: drake::multibody::Joint::AddInOneForce
        struct /* AddInOneForce */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Adds into ``forces`` a force along the one of the joint's degrees of
freedom indicated by index ``joint_dof``. The meaning for this degree
of freedom and even its dimensional units depend on the specific joint
sub-class. For a RevoluteJoint for instance, ``joint_dof`` can only be
0 since revolute joints's motion subspace only has one degree of
freedom, while the units of ``joint_tau`` are those of torque (N⋅m in
the MKS system of units). For multi-dof joints please refer to the
documentation provided by specific joint sub-classes regarding the
meaning of ``joint_dof``.

Parameter ``context``:
    The context storing the state and parameters for the model to
    which ``this`` joint belongs.

Parameter ``joint_dof``:
    Index specifying one of the degrees of freedom for this joint. The
    index must be in the range ``0 <= joint_dof < num_velocities()``
    or otherwise this method will abort.

Parameter ``joint_tau``:
    Generalized force corresponding to the degree of freedom indicated
    by ``joint_dof`` to be added into ``forces``.

Parameter ``forces``:
    On return, this method will add force ``joint_tau`` for the degree
    of freedom ``joint_dof`` into the output ``forces``. This method
    aborts if ``forces`` is ``nullptr`` or if ``forces`` doest not
    have the right sizes to accommodate a set of forces for the model
    to which this joint belongs.)""";
        } AddInOneForce;
        // Symbol: drake::multibody::Joint::Build
        struct /* Build */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc = R"""()""";
        } Build;
        // Symbol: drake::multibody::Joint::CloneToScalar
        struct /* CloneToScalar */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc = R"""()""";
        } CloneToScalar;
        // Symbol: drake::multibody::Joint::DoAddInDamping
        struct /* DoAddInDamping */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Adds into MultibodyForces the forces due to damping within ``this``
joint. How forces are added to a MultibodyTree model depends on the
underlying implementation of a particular joint (for instance,
mobilizer vs. constraint) and therefore specific Joint subclasses must
provide a definition for this method. The default implementation is a
no-op for joints with no damping.)""";
        } DoAddInDamping;
        // Symbol: drake::multibody::Joint::DoAddInOneForce
        struct /* DoAddInOneForce */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Adds into ``forces`` a force along the one of the joint's degrees of
freedom given by ``joint_dof``. How forces are added to a
MultibodyTree model depends on the underlying implementation of a
particular joint and therefore specific Joint subclasses must provide
a definition for this method. For instance, a revolute joint could be
modeled with a single generalized coordinate for the angular rotation
(implemented through a RevoluteMobilizer) or it could be modeled using
a constraint that only allows rotation about the joint's axis but that
constrains the motion in the other five degrees of freedom. This
method is only called by the public NVI AddInOneForce() and therefore
input arguments were checked to be valid.

See also:
    The public NVI AddInOneForce() for details.)""";
        } DoAddInOneForce;
        // Symbol: drake::multibody::Joint::DoCloneToScalar
        struct /* DoCloneToScalar */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc_was_unable_to_choose_unambiguous_names =
R"""(@name Methods to make a clone, optionally templated on different
scalar types. Clones this Joint (templated on T) to a joint templated
on ``double``.)""";
        } DoCloneToScalar;
        // Symbol: drake::multibody::Joint::DoGetDefaultPosePair
        struct /* DoGetDefaultPosePair */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Implementation of the NVI GetDefaultPose(). This is optional for Joint
subclasses *except* for floating (6 dof) Joints. The subclass should
convert its default_positions to pose X_FM and return that as a
(quaternion, translation) pair. If the subclass already uses
(quaternion, translation) as generalized coordinates (i.e. it's a
quaternion_floating_joint) it must return those exactly (don't convert
to a transform first).)""";
        } DoGetDefaultPosePair;
        // Symbol: drake::multibody::Joint::DoGetOnePosition
        struct /* DoGetOnePosition */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Implementation of the NVI GetOnePosition() that must only be
implemented by those joint subclasses that have a single degree of
freedom. The default implementation for all other joints is to abort
with an appropriate message. Revolute and prismatic are examples of
joints that will want to implement this method.)""";
        } DoGetOnePosition;
        // Symbol: drake::multibody::Joint::DoGetOneVelocity
        struct /* DoGetOneVelocity */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Implementation of the NVI GetOneVelocity() that must only be
implemented by those joint subclasses that have a single degree of
freedom. The default implementation for all other joints is to abort
with an appropriate message. Revolute and prismatic are examples of
joints that will want to implement this method.)""";
        } DoGetOneVelocity;
        // Symbol: drake::multibody::Joint::DoSetDefaultPosePair
        struct /* DoSetDefaultPosePair */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Implementation of the NVI SetDefaultPose(). This is optional for Joint
subclasses *except* for floating (6 dof) Joints. The subclass should
convert the input to the closest equivalent in generalized coordinates
and invoke set_default_positions() to record them. If the subclass
already uses (quaternion, translation) as generalized coordinates
(i.e. it's a quaternion_floating_joint) it must store those exactly.)""";
        } DoSetDefaultPosePair;
        // Symbol: drake::multibody::Joint::DoSetTopology
        struct /* DoSetTopology */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc = R"""()""";
        } DoSetTopology;
        // Symbol: drake::multibody::Joint::DoShallowClone
        struct /* DoShallowClone */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(NVI for ShallowClone(). The public Joint::ShallowClone in this base
class is responsible for copying the mutable Joint data (damping, all
limits, default positions, etc.) into the return value. The subclass
only needs to handle subclass-specific details.)""";
        } DoShallowClone;
        // Symbol: drake::multibody::Joint::GetDampingVector
        struct /* GetDampingVector */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Returns the Context dependent damping coefficients stored as
parameters in ``context``. Refer to default_damping_vector() for
details.

Parameter ``context``:
    The context storing the state and parameters for the model to
    which ``this`` joint belongs.)""";
        } GetDampingVector;
        // Symbol: drake::multibody::Joint::GetDefaultPose
        struct /* GetDefaultPose */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Returns this joint's default pose as a RigidTransform X_FM.

Note:
    Currently this is implemented only for floating (6 dof) joints
    which can represent any pose.

Raises:
    RuntimeError if called for any joint type that does not implement
    this function.

Returns ``X_FM``:
    The default pose as a rigid transform.

See also:
    default_positions() to see the generalized positions q₀ that this
    joint used to generate the returned transform.

See also:
    GetDefaultPosePair() for an alternative using a quaternion)""";
        } GetDefaultPose;
        // Symbol: drake::multibody::Joint::GetDefaultPosePair
        struct /* GetDefaultPosePair */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""((Advanced) This is the same as GetDefaultPose() except it returns this
joint's default pose as a (quaternion, translation vector) pair.

Note:
    Currently this is implemented only for floating (6 dof) joints
    which can represent any pose.

Note:
    For a QuaternionFloatingJoint the return will be bit-identical to
    the pose provided to SetDefaultPosePair(). For any other floating
    (6 dof) joint the pose will be numerically equivalent (i.e. within
    roundoff) but not identical. For other joint types it will be some
    approximation.

Returns ``q_FM``:
    ,p_FM The default pose as a (quaternion, translation) pair.

Raises:
    RuntimeError if called for any joint type that does not implement
    this function.

See also:
    GetDefaultPose())""";
        } GetDefaultPosePair;
        // Symbol: drake::multibody::Joint::GetMobilizerInUse
        struct /* GetMobilizerInUse */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc = R"""()""";
        } GetMobilizerInUse;
        // Symbol: drake::multibody::Joint::GetOnePosition
        struct /* GetOnePosition */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Returns the position coordinate for joints with a single degree of
freedom.

Raises:
    RuntimeError if the joint does not have a single degree of
    freedom.)""";
        } GetOnePosition;
        // Symbol: drake::multibody::Joint::GetOneVelocity
        struct /* GetOneVelocity */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Returns the velocity coordinate for joints with a single degree of
freedom.

Raises:
    RuntimeError if the joint does not have a single degree of
    freedom.)""";
        } GetOneVelocity;
        // Symbol: drake::multibody::Joint::GetPose
        struct /* GetPose */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Returns this joint's current pose using its position coordinates q
taken from the given ``context`` and converting that to a
RigidTransform X_FM(q).

Note:
    The returned pose may not match the transform that was supplied to
    SetPose() since in general joints (other than 6 dof joints) cannot
    represent arbitrary poses.

Note:
    All joint types support this function.

Raises:
    RuntimeError if called for any joint type that does not implement
    this function.

Returns ``X_FM``:
    The current pose as a rigid transform.

See also:
    GetPositions() to see the generalized positions q that this joint
    used to generate the returned transform.)""";
        } GetPose;
        // Symbol: drake::multibody::Joint::GetPosePair
        struct /* GetPosePair */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""((Advanced) This is the same as GetPose() except it returns this
joint's pose in the given ``context`` as a (quaternion, translation
vector) pair.

Note:
    All joint types support this function.

Note:
    For a QuaternionFloatingJoint the return will be bit-identical to
    the pose provided to SetPosePair(). For any other floating (6 dof)
    joint the pose will be numerically equivalent (i.e. within
    roundoff) but not identical. For other joint types it will be some
    approximation.

Returns ``q_FM``:
    ,p_FM The pose as a (quaternion, translation) pair.

Raises:
    RuntimeError if the containing MultibodyPlant has not yet been
    finalized.

See also:
    GetPose())""";
        } GetPosePair;
        // Symbol: drake::multibody::Joint::GetPositions
        struct /* GetPositions */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Returns the current value in the given ``context`` of the generalized
coordinates q for this joint.

Raises:
    RuntimeError if the containing MultibodyPlant has not yet been
    finalized.)""";
        } GetPositions;
        // Symbol: drake::multibody::Joint::GetSpatialVelocity
        struct /* GetSpatialVelocity */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Given the generalized positions q and generalized velocities v for
this joint in the given ``context``, returns the cross-joint spatial
velocity V_FM.

Note:
    All joint types support this function.

Returns ``V_FM``:
    the spatial velocity across this joint.

Raises:
    RuntimeError if the containing MultibodyPlant has not yet been
    finalized.

See also:
    GetVelocities() to see the generalized velocities v that this
    joint used to generate the returned spatial velocity.)""";
        } GetSpatialVelocity;
        // Symbol: drake::multibody::Joint::GetVelocities
        struct /* GetVelocities */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Returns the current value in the given ``context`` of the generalized
velocities v for this joint.

Raises:
    RuntimeError if the containing MultibodyPlant has not yet been
    finalized.)""";
        } GetVelocities;
        // Symbol: drake::multibody::Joint::Joint<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc_10args =
R"""(Creates a joint between two Frame objects which imposes a given
kinematic relation between frame F attached on the parent body P and
frame M attached on the child body B. The joint will be assigned to
the model instance from ``frame_on_child`` (this is the typical
convention for joints between the world and a model, or between two
models (e.g. an arm to a gripper)). See this class's documentation for
further details.

Parameter ``name``:
    A string with a name identifying ``this`` joint.

Parameter ``frame_on_parent``:
    The frame F attached on the parent body connected by this joint.

Parameter ``frame_on_child``:
    The frame M attached on the child body connected by this joint.

Parameter ``damping``:
    A vector of viscous damping coefficients, of size
    num_velocities(). See default_damping_vector() for details.

Parameter ``pos_lower_limits``:
    A vector storing the lower limit for each generalized position. It
    must have the same size as ``pos_upper_limit``. A value equal to
    -∞ implies no lower limit.

Parameter ``pos_upper_limits``:
    A vector storing the upper limit for each generalized position. It
    must have the same size as ``pos_lower_limit``. A value equal to
    +∞ implies no upper limit.

Parameter ``vel_lower_limits``:
    A vector storing the lower limit for each generalized velocity. It
    must have the same size as ``vel_upper_limit``. A value equal to
    -∞ implies no lower limit.

Parameter ``vel_upper_limits``:
    A vector storing the upper limit for each generalized velocity. It
    must have the same size as ``vel_lower_limit``. A value equal to
    +∞ implies no upper limit.

Parameter ``acc_lower_limits``:
    A vector storing the lower limit for each generalized
    acceleration. It must have the same size as ``acc_upper_limit``. A
    value equal to -∞ implies no lower limit.

Parameter ``acc_upper_limits``:
    A vector storing the upper limit for each generalized
    acceleration. It must have the same size as ``acc_lower_limit``. A
    value equal to +∞ implies no upper limit.)""";
          // Source: drake/multibody/tree/joint.h
          const char* doc_9args =
R"""(Additional constructor overload for joints with zero damping. Refer to
the more general constructor signature taking damping for further
details on the rest of the arguments for this constructor.)""";
        } ctor;
        // Symbol: drake::multibody::Joint::Lock
        struct /* Lock */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Lock the joint. Its generalized velocities will be 0 until it is
unlocked. If actuated, its PD controllers will be ignored and thus
will have no effect on the reported actuation output nor reaction
forces.)""";
        } Lock;
        // Symbol: drake::multibody::Joint::MakeUniqueOffsetFrameName
        struct /* MakeUniqueOffsetFrameName */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc = R"""()""";
        } MakeUniqueOffsetFrameName;
        // Symbol: drake::multibody::Joint::SetDampingVector
        struct /* SetDampingVector */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Sets the value of the viscous damping coefficients for this joint,
stored as parameters in ``context``. Refer to default_damping_vector()
for details.

Parameter ``context``:
    The context storing the state and parameters for the model to
    which ``this`` joint belongs.

Parameter ``damping``:
    The vector of damping values.

Raises:
    RuntimeError if damping.size() != num_velocities().

Raises:
    RuntimeError if any of the damping coefficients is negative.

Note:
    Some multi-dof joints may have specific semantics for their
    damping vector that are not enforced here. For instance,
    QuaternionFloatingJoint assumes identical damping values for all 3
    angular velocity components and identical damping values for all 3
    translational velocity components. It will thus use
    ``angular_damping = damping[0]`` and ``translational_damping =
    damping[3]``. Refer to the particular subclass for more semantic
    information.)""";
        } SetDampingVector;
        // Symbol: drake::multibody::Joint::SetDefaultPose
        struct /* SetDefaultPose */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Sets this joint's default generalized positions q₀ such that the pose
of the child frame M in the parent frame F best matches the given
pose. The pose is given by a RigidTransform ``X_FM``, but a joint will
represent pose differently.

Note:
    Currently this is implemented only for floating (6 dof) joints
    which can represent any pose.

Raises:
    RuntimeError if called for any joint type that does not implement
    this function.

See also:
    default_positions() to see the resulting q₀ after this call.

See also:
    SetDefaultPosePair() for an alternative using a quaternion)""";
        } SetDefaultPose;
        // Symbol: drake::multibody::Joint::SetDefaultPosePair
        struct /* SetDefaultPosePair */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""((Advanced) This is the same as SetDefaultPose() except it takes the
pose as a (quaternion, translation vector) pair. A
QuaternionFloatingJoint will store this pose bit-identically; an
RpyFloatingJoint will store it to within floating point precision; any
other joint will approximate it consistent with that joint's mobility.

Note:
    Currently this is implemented only for floating (6 dof) joints
    which can represent any pose.

Raises:
    RuntimeError if called for any joint type that does not implement
    this function.

See also:
    SetDefaultPose())""";
        } SetDefaultPosePair;
        // Symbol: drake::multibody::Joint::SetPose
        struct /* SetPose */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Sets in the given ``context`` this joint's generalized positions q
such that the pose of the child frame M in the parent frame F best
matches the given pose. The pose is given by a RigidTransform X_FM,
but a joint will represent pose differently. Drake's "floating" (6
dof) joints can represent any pose, but other joints may only be able
to approximate X_FM. See the individual joint descriptions for
specifics.

Note:
    Currently this is implemented only for floating (6 dof) joints
    which can represent any pose.

Raises:
    RuntimeError if called for any joint type that does not implement
    this function.

Raises:
    RuntimeError if the containing MultibodyPlant has not yet been
    finalized.

Precondition:
    ``context`` is not null.

See also:
    GetPositions() to see the resulting q after this call.

See also:
    SetPosePair() for an alternative using a quaternion.)""";
        } SetPose;
        // Symbol: drake::multibody::Joint::SetPosePair
        struct /* SetPosePair */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""((Advanced) This is the same as SetPose() except it takes the pose as a
(quaternion, translation vector) pair. A QuaternionFloatingJoint will
store this pose bit-identically; any other joint will approximate it.

Note:
    Currently this is implemented only for floating (6 dof) joints
    which can represent any pose.

Raises:
    RuntimeError if called for any joint type that does not implement
    this function.

Raises:
    RuntimeError if the containing MultibodyPlant has not yet been
    finalized.

Precondition:
    ``context`` is not null.

See also:
    SetPose())""";
        } SetPosePair;
        // Symbol: drake::multibody::Joint::SetPositions
        struct /* SetPositions */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Sets in the given ``context`` the generalized position coordinates q
for this joint to ``positions``.

Note:
    The values in ``positions`` are NOT constrained to be within
    position_lower_limits() and position_upper_limits().

Raises:
    RuntimeError if the dimension of ``positions`` does not match
    num_positions().

Raises:
    RuntimeError if the containing MultibodyPlant has not yet been
    finalized.

Precondition:
    ``context`` is not null.)""";
        } SetPositions;
        // Symbol: drake::multibody::Joint::SetSpatialVelocity
        struct /* SetSpatialVelocity */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Sets in the given ``context`` this joint's generalized velocities v
such that the spatial velocity of the child frame M in the parent
frame F best matches the given spatial velocity. The velocity is
provided as a spatial velocity V_FM, but a joint may represent
velocity differently. Drake's "floating" (6 dof) joints can represent
any spatial velocity, but other joints may only be able to approximate
V_FM. See the individual joint descriptions for specifics.

Note:
    Currently this is implemented only for floating (6 dof) joints
    which can represent any spatial velocity.

Raises:
    RuntimeError if called for any joint type that does not implement
    this function.

Raises:
    RuntimeError if the containing MultibodyPlant has not yet been
    finalized.

Precondition:
    ``context`` is not null.

See also:
    GetVelocities() to see the resulting v after this call.)""";
        } SetSpatialVelocity;
        // Symbol: drake::multibody::Joint::SetVelocities
        struct /* SetVelocities */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Sets in the given ``context`` the generalized velocity coordinates v
for this joint to ``velocities``.

Note:
    The values in ``velocities`` are NOT constrained to be within
    velocity_lower_limits() and velocity_upper_limits().

Raises:
    RuntimeError if the dimension of ``velocities`` does not match
    num_velocities().

Raises:
    RuntimeError if the containing MultibodyPlant has not yet been
    finalized.

Precondition:
    ``context`` is not null.)""";
        } SetVelocities;
        // Symbol: drake::multibody::Joint::ShallowClone
        struct /* ShallowClone */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc = R"""()""";
        } ShallowClone;
        // Symbol: drake::multibody::Joint::Unlock
        struct /* Unlock */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Unlock the joint. If actuated, its PD controllers (if any) will no
longer be ignored.)""";
        } Unlock;
        // Symbol: drake::multibody::Joint::acceleration_lower_limits
        struct /* acceleration_lower_limits */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc = R"""(Returns the acceleration lower limits.)""";
        } acceleration_lower_limits;
        // Symbol: drake::multibody::Joint::acceleration_upper_limits
        struct /* acceleration_upper_limits */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc = R"""(Returns the acceleration upper limits.)""";
        } acceleration_upper_limits;
        // Symbol: drake::multibody::Joint::can_rotate
        struct /* can_rotate */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Returns true if this joint's mobility allows relative rotation of the
two frames associated with this joint.

Precondition:
    the MultibodyPlant must be finalized.

See also:
    can_translate())""";
        } can_rotate;
        // Symbol: drake::multibody::Joint::can_translate
        struct /* can_translate */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Returns true if this joint's mobility allows relative translation of
the two frames associated with this joint.

Precondition:
    the MultibodyPlant must be finalized.

See also:
    can_rotate())""";
        } can_translate;
        // Symbol: drake::multibody::Joint::child_body
        struct /* child_body */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Returns a const reference to the child body B.)""";
        } child_body;
        // Symbol: drake::multibody::Joint::default_damping_vector
        struct /* default_damping_vector */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Returns all default damping coefficients for joints that model viscous
damping, of size num_velocities(). Joints that do not model damping
return a zero vector of size num_velocities(). If vj is the vector of
generalized velocities for this joint, of size num_velocities(),
viscous damping models a generalized force at the joint of the form
tau = -diag(dj)⋅vj, with dj the vector returned by this function. The
units of the coefficients will depend on the specific joint type. For
instance, for a revolute joint where vj is an angular velocity with
units of rad/s and tau having units of N⋅m, the coefficient of viscous
damping has units of N⋅m⋅s. Refer to each joint's documentation for
further details.)""";
        } default_damping_vector;
        // Symbol: drake::multibody::Joint::default_positions
        struct /* default_positions */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Returns the default generalized position coordinates q₀. These will be
the values set with set_default_positions() if any; otherwise, they
will be the "zero configuration" for this joint type (as defined by
the particular joint type).

Note:
    The default generalized velocities v₀ are zero for every joint.)""";
        } default_positions;
        // Symbol: drake::multibody::Joint::do_get_num_positions
        struct /* do_get_num_positions */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Implementation of the NVI num_positions(), see num_positions() for
details.

Note:
    Implementations must meet the styleguide requirements for
    snake_case accessor methods.)""";
        } do_get_num_positions;
        // Symbol: drake::multibody::Joint::do_get_num_velocities
        struct /* do_get_num_velocities */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Implementation of the NVI num_velocities(), see num_velocities() for
details.

Note:
    Implementations must meet the styleguide requirements for
    snake_case accessor methods.)""";
        } do_get_num_velocities;
        // Symbol: drake::multibody::Joint::do_get_position_start
        struct /* do_get_position_start */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Implementation of the NVI position_start(), see position_start() for
details.

Note:
    Implementations must meet the styleguide requirements for
    snake_case accessor methods.)""";
        } do_get_position_start;
        // Symbol: drake::multibody::Joint::do_get_position_suffix
        struct /* do_get_position_suffix */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Implementation of the NVI position_suffix(), see position_suffix() for
details. The suffix should contain only alphanumeric characters (e.g.
'wx' not '_wx' or '.wx').)""";
        } do_get_position_suffix;
        // Symbol: drake::multibody::Joint::do_get_velocity_start
        struct /* do_get_velocity_start */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Implementation of the NVI velocity_start(), see velocity_start() for
details. Note that this must be the offset within just the velocity
vector, *not* within the composite state vector.

Note:
    Implementations must meet the styleguide requirements for
    snake_case accessor methods.)""";
        } do_get_velocity_start;
        // Symbol: drake::multibody::Joint::do_get_velocity_suffix
        struct /* do_get_velocity_suffix */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Implementation of the NVI velocity_suffix(), see velocity_suffix() for
details. The suffix should contain only alphanumeric characters (e.g.
'wx' not '_wx' or '.wx').)""";
        } do_get_velocity_suffix;
        // Symbol: drake::multibody::Joint::do_set_default_positions
        struct /* do_set_default_positions */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Implementation of the NVI set_default_positions(), see
set_default_positions() for details. It is the responsibility of the
subclass to ensure that its joint implementation (i.e., mobilizer),
should it have one, is updated with ``default_positions``. Note that
the Joint base class also stores default_positions (as a VectorX); the
implementing mobilizer should have the same value but as a fixed-size
vector.

Note:
    Implementations must meet the styleguide requirements for
    snake_case accessor methods.)""";
        } do_set_default_positions;
        // Symbol: drake::multibody::Joint::frame_on_child
        struct /* frame_on_child */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Returns a const reference to the frame M attached on the child body B.)""";
        } frame_on_child;
        // Symbol: drake::multibody::Joint::frame_on_parent
        struct /* frame_on_parent */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Returns a const reference to the frame F attached on the parent body
P.)""";
        } frame_on_parent;
        // Symbol: drake::multibody::Joint::get_mobilizer_downcast
        struct /* get_mobilizer_downcast */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""((Internal use only) Returns the mobilizer implementing this joint,
downcast to its specific type.

Precondition:
    A mobilizer has been created for this Joint.

Precondition:
    ConcreteMobilizer must exactly match the dynamic type of the
    mobilizer associated with this Joint, or be a base class of the
    dynamic type. This requirement is (only) checked in Debug builds.)""";
        } get_mobilizer_downcast;
        // Symbol: drake::multibody::Joint::get_mutable_mobilizer_downcast
        struct /* get_mutable_mobilizer_downcast */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""((Internal use only) Mutable flavor of get_mobilizer_downcast().)""";
        } get_mutable_mobilizer_downcast;
        // Symbol: drake::multibody::Joint::has_mobilizer
        struct /* has_mobilizer */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""((Internal use only) Returns true if this Joint has an implementing
Mobilizer.)""";
        } has_mobilizer;
        // Symbol: drake::multibody::Joint::index
        struct /* index */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc = R"""(Returns this element's unique index.)""";
        } index;
        // Symbol: drake::multibody::Joint::is_locked
        struct /* is_locked */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Returns:
    true if the joint is locked, false otherwise.)""";
        } is_locked;
        // Symbol: drake::multibody::Joint::name
        struct /* name */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc = R"""(Returns the name of this joint.)""";
        } name;
        // Symbol: drake::multibody::Joint::num_positions
        struct /* num_positions */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Returns the number of generalized positions describing this joint.)""";
        } num_positions;
        // Symbol: drake::multibody::Joint::num_velocities
        struct /* num_velocities */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Returns the number of generalized velocities describing this joint.)""";
        } num_velocities;
        // Symbol: drake::multibody::Joint::ordinal
        struct /* ordinal */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Returns this element's unique ordinal. The joint's ordinal is a unique
index into contiguous containers that have an entry for each Joint,
such as the vector valued reaction forces (see
MultibodyPlant::get_reaction_forces_output_port()). The ordinal value
will be updated (if needed) when joints are removed from the parent
plant so that the set of ordinal values is a bijection with [0,
num_joints()). Ordinals are assigned in the order that joints are
added to the plant, thus a set of joints sorted by ordinal has the
same ordering as if it were sorted by JointIndex. If joints have been
removed from the plant, do *not* use index() to access contiguous
containers with entries per Joint.)""";
        } ordinal;
        // Symbol: drake::multibody::Joint::parent_body
        struct /* parent_body */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Returns a const reference to the parent body P.)""";
        } parent_body;
        // Symbol: drake::multibody::Joint::position_lower_limits
        struct /* position_lower_limits */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc = R"""(Returns the position lower limits.)""";
        } position_lower_limits;
        // Symbol: drake::multibody::Joint::position_start
        struct /* position_start */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Returns the index to the first generalized position for this joint
within the vector q of generalized positions for the full multibody
system.)""";
        } position_start;
        // Symbol: drake::multibody::Joint::position_suffix
        struct /* position_suffix */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Returns a string suffix (e.g. to be appended to the name()) to
identify the `k`th position in this joint. ``position_index_in_joint``
must be in [0, num_positions()).

Precondition:
    the MultibodyPlant must be finalized.)""";
        } position_suffix;
        // Symbol: drake::multibody::Joint::position_upper_limits
        struct /* position_upper_limits */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc = R"""(Returns the position upper limits.)""";
        } position_upper_limits;
        // Symbol: drake::multibody::Joint::set_acceleration_limits
        struct /* set_acceleration_limits */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Sets the acceleration limits to ``lower_limits`` and ``upper_limits``.

Raises:
    RuntimeError if the dimension of ``lower_limits`` or
    ``upper_limits`` does not match num_velocities().

Raises:
    RuntimeError if any of ``lower_limits`` is larger than the
    corresponding term in ``upper_limits``.)""";
        } set_acceleration_limits;
        // Symbol: drake::multibody::Joint::set_default_damping_vector
        struct /* set_default_damping_vector */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Sets the default value of the viscous damping coefficients for this
joint. Refer to default_damping_vector() for details.

Raises:
    RuntimeError if damping.size() != num_velocities().

Raises:
    RuntimeError if any of the damping coefficients is negative.

Raises:
    RuntimeError if this element is not associated with a
    MultibodyPlant.

Precondition:
    the MultibodyPlant must not be finalized.)""";
        } set_default_damping_vector;
        // Symbol: drake::multibody::Joint::set_default_positions
        struct /* set_default_positions */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Sets the default generalized position coordinates q₀ to
``default_positions``.

Note:
    The values in ``default_positions`` are NOT constrained to be
    within position_lower_limits() and position_upper_limits().

Note:
    The default generalized velocities v₀ are zero for every joint.

Raises:
    RuntimeError if the dimension of ``default_positions`` does not
    match num_positions().)""";
        } set_default_positions;
        // Symbol: drake::multibody::Joint::set_position_limits
        struct /* set_position_limits */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Sets the position limits to ``lower_limits`` and ``upper_limits``.

Raises:
    RuntimeError if the dimension of ``lower_limits`` or
    ``upper_limits`` does not match num_positions().

Raises:
    RuntimeError if any of ``lower_limits`` is larger than the
    corresponding term in ``upper_limits``.

Note:
    Setting the position limits does not affect the
    ``default_positions()``, regardless of whether the current
    ``default_positions()`` satisfy the new position limits.)""";
        } set_position_limits;
        // Symbol: drake::multibody::Joint::set_velocity_limits
        struct /* set_velocity_limits */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Sets the velocity limits to ``lower_limits`` and ``upper_limits``.

Raises:
    RuntimeError if the dimension of ``lower_limits`` or
    ``upper_limits`` does not match num_velocities().

Raises:
    RuntimeError if any of ``lower_limits`` is larger than the
    corresponding term in ``upper_limits``.)""";
        } set_velocity_limits;
        // Symbol: drake::multibody::Joint::tree_frames
        struct /* tree_frames */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Utility for concrete joint implementations to use to select the
inboard/outboard frames for a tree in the spanning forest, given
whether they should be reversed from the parent/child frames that are
members of this Joint object.)""";
        } tree_frames;
        // Symbol: drake::multibody::Joint::type_name
        struct /* type_name */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Returns a string identifying the type of ``this`` joint, such as
"revolute" or "prismatic".)""";
        } type_name;
        // Symbol: drake::multibody::Joint::velocity_lower_limits
        struct /* velocity_lower_limits */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc = R"""(Returns the velocity lower limits.)""";
        } velocity_lower_limits;
        // Symbol: drake::multibody::Joint::velocity_start
        struct /* velocity_start */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Returns the index to the first generalized velocity for this joint
within the vector v of generalized velocities for the full multibody
system.)""";
        } velocity_start;
        // Symbol: drake::multibody::Joint::velocity_suffix
        struct /* velocity_suffix */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc =
R"""(Returns a string suffix (e.g. to be appended to the name()) to
identify the `k`th velocity in this joint. ``velocity_index_in_joint``
must be in [0, num_velocities()).

Precondition:
    the MultibodyPlant must be finalized.)""";
        } velocity_suffix;
        // Symbol: drake::multibody::Joint::velocity_upper_limits
        struct /* velocity_upper_limits */ {
          // Source: drake/multibody/tree/joint.h
          const char* doc = R"""(Returns the velocity upper limits.)""";
        } velocity_upper_limits;
      } Joint;
      // Symbol: drake::multibody::JointActuator
      struct /* JointActuator */ {
        // Source: drake/multibody/tree/joint_actuator.h
        const char* doc =
R"""(The JointActuator class is mostly a simple bookkeeping structure to
represent an actuator acting on a given Joint. It helps to flag
whether a given Joint is actuated or not so that MultibodyTree clients
can apply forces on actuated joints through their actuators, see
AddInOneForce().)""";
        // Symbol: drake::multibody::JointActuator::AddInOneForce
        struct /* AddInOneForce */ {
          // Source: drake/multibody/tree/joint_actuator.h
          const char* doc =
R"""(Adds into ``forces`` a force along one of the degrees of freedom of
the Joint actuated by ``this`` actuator. The meaning for this degree
of freedom, sign conventions and even its dimensional units depend on
the specific joint sub-class being actuated. For a RevoluteJoint for
instance, ``joint_dof`` can only be 0 since revolute joints's motion
subspace only has one degree of freedom, while the units of ``tau``
are those of torque (N⋅m in the MKS system of units). For multi-dof
joints please refer to the documentation provided by specific joint
sub-classes regarding the meaning of ``joint_dof``.

Parameter ``context``:
    The context storing the state and parameters for the model to
    which ``this`` joint belongs.

Parameter ``joint_dof``:
    Index specifying one of the degrees of freedom for this joint. The
    index must be in the range ``0 <= joint_dof < num_inputs()`` or
    otherwise this method will throw an exception.

Parameter ``tau``:
    Generalized force corresponding to the degree of freedom indicated
    by ``joint_dof`` to be added into ``forces``. Refer to the
    specific Joint sub-class documentation for details on the meaning
    and units for this degree of freedom.

Parameter ``forces``:
    On return, this method will add force ``tau`` for the degree of
    freedom ``joint_dof`` into the output ``forces``. This method
    aborts if ``forces`` is ``nullptr`` or if ``forces`` doest not
    have the right sizes to accommodate a set of forces for the model
    to which this actuator belongs.)""";
        } AddInOneForce;
        // Symbol: drake::multibody::JointActuator::CloneToScalar
        struct /* CloneToScalar */ {
          // Source: drake/multibody/tree/joint_actuator.h
          const char* doc = R"""()""";
        } CloneToScalar;
        // Symbol: drake::multibody::JointActuator::JointActuator<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/joint_actuator.h
          const char* doc =
R"""(Creates an actuator for ``joint`` with the given ``name``. The name
must be unique within the given multibody model. This is enforced by
MultibodyPlant::AddJointActuator().

Parameter ``name``:
    A string with a name identifying ``this`` actuator.

Parameter ``joint``:
    The ``joint`` that the created actuator will act on.

Parameter ``effort_limit``:
    The maximum effort for the actuator. It must be strictly positive,
    otherwise an RuntimeError is thrown. If +∞, the actuator has no
    limit, which is the default. The effort limit has physical units
    in accordance to the joint type it actuates. For instance, it will
    have units of N⋅m (torque) for revolute joints while it will have
    units of N (force) for prismatic joints.)""";
        } ctor;
        // Symbol: drake::multibody::JointActuator::SetGearRatio
        struct /* SetGearRatio */ {
          // Source: drake/multibody/tree/joint_actuator.h
          const char* doc =
R"""(Sets the associated gear ratio value for this actuator in ``context``.
See reflected_inertia.)""";
        } SetGearRatio;
        // Symbol: drake::multibody::JointActuator::SetRotorInertia
        struct /* SetRotorInertia */ {
          // Source: drake/multibody/tree/joint_actuator.h
          const char* doc =
R"""(Sets the associated rotor inertia value for this actuator in
``context``. See reflected_inertia.)""";
        } SetRotorInertia;
        // Symbol: drake::multibody::JointActuator::calc_reflected_inertia
        struct /* calc_reflected_inertia */ {
          // Source: drake/multibody/tree/joint_actuator.h
          const char* doc =
R"""(Calculates the reflected inertia value for this actuator in
``context``. See reflected_inertia. Note that this ONLY depends on the
Parameters in the context; it does not depend on time, input, state,
etc.)""";
        } calc_reflected_inertia;
        // Symbol: drake::multibody::JointActuator::default_gear_ratio
        struct /* default_gear_ratio */ {
          // Source: drake/multibody/tree/joint_actuator.h
          const char* doc =
R"""(Gets the default value for this actuator's gear ratio. See
reflected_inertia.)""";
        } default_gear_ratio;
        // Symbol: drake::multibody::JointActuator::default_reflected_inertia
        struct /* default_reflected_inertia */ {
          // Source: drake/multibody/tree/joint_actuator.h
          const char* doc =
R"""(Returns the default value for this actuator's reflected inertia. It is
calculated as ρ²⋅Iᵣ, where ρ is the default gear ratio and Iᵣ is the
default rotor inertia for this actuator. See reflected_inertia.)""";
        } default_reflected_inertia;
        // Symbol: drake::multibody::JointActuator::default_rotor_inertia
        struct /* default_rotor_inertia */ {
          // Source: drake/multibody/tree/joint_actuator.h
          const char* doc =
R"""(Gets the default value for this actuator's rotor inertia. See
reflected_inertia.)""";
        } default_rotor_inertia;
        // Symbol: drake::multibody::JointActuator::effort_limit
        struct /* effort_limit */ {
          // Source: drake/multibody/tree/joint_actuator.h
          const char* doc = R"""(Returns the actuator effort limit.)""";
        } effort_limit;
        // Symbol: drake::multibody::JointActuator::gear_ratio
        struct /* gear_ratio */ {
          // Source: drake/multibody/tree/joint_actuator.h
          const char* doc =
R"""(Returns the associated gear ratio value for this actuator, stored in
``context``. See reflected_inertia. Note that this ONLY depends on the
Parameters in the context; it does not depend on time, input, state,
etc.)""";
        } gear_ratio;
        // Symbol: drake::multibody::JointActuator::get_actuation_vector
        struct /* get_actuation_vector */ {
          // Source: drake/multibody/tree/joint_actuator.h
          const char* doc =
R"""(Gets the actuation values for ``this`` actuator from the actuation
vector u for the entire plant model.

Returns:
    a reference to a nv-dimensional vector, where nv is the number of
    velocity variables of joint().

Raises:
    RuntimeError if this element is not associated with a
    MultibodyPlant.)""";
        } get_actuation_vector;
        // Symbol: drake::multibody::JointActuator::get_controller_gains
        struct /* get_controller_gains */ {
          // Source: drake/multibody/tree/joint_actuator.h
          const char* doc =
R"""(Returns a reference to the controller gains for this actuator.

Precondition:
    has_controller() is ``True``.)""";
        } get_controller_gains;
        // Symbol: drake::multibody::JointActuator::has_controller
        struct /* has_controller */ {
          // Source: drake/multibody/tree/joint_actuator.h
          const char* doc =
R"""(Returns ``True`` if any non-zero controller gains have been specified
with a call to set_controller_gains().

Note:
    A controller for a given model instance can be *disarmed* if the
    desired state input port for its model instance is not connected.
    When a PD controller is disarmed, it has no effect on the
    MultibodyPlant's dynamics, as if there was no PD controller
    (still, this method returns ``True`` whenever non-zero gains were
    set with set_controller_gains().) See pd_controllers_and_ports for
    further details.)""";
        } has_controller;
        // Symbol: drake::multibody::JointActuator::index
        struct /* index */ {
          // Source: drake/multibody/tree/joint_actuator.h
          const char* doc = R"""(Returns this element's unique index.)""";
        } index;
        // Symbol: drake::multibody::JointActuator::input_start
        struct /* input_start */ {
          // Source: drake/multibody/tree/joint_actuator.h
          const char* doc =
R"""(Returns the index to the first element for this joint actuator /
within the vector of actuation inputs for the full multibody / system.
Returns -1 if this JointActuator hasn't been added to a
MultibodyPlant.)""";
        } input_start;
        // Symbol: drake::multibody::JointActuator::joint
        struct /* joint */ {
          // Source: drake/multibody/tree/joint_actuator.h
          const char* doc =
R"""(Returns a reference to the joint actuated by this JointActuator.

Raises:
    RuntimeError if this element is not associated with a
    MultibodyPlant.)""";
        } joint;
        // Symbol: drake::multibody::JointActuator::name
        struct /* name */ {
          // Source: drake/multibody/tree/joint_actuator.h
          const char* doc = R"""(Returns the name of the actuator.)""";
        } name;
        // Symbol: drake::multibody::JointActuator::num_inputs
        struct /* num_inputs */ {
          // Source: drake/multibody/tree/joint_actuator.h
          const char* doc =
R"""(Returns the number of inputs associated with this actuator. This is
always the number of degrees of freedom of the actuated joint.)""";
        } num_inputs;
        // Symbol: drake::multibody::JointActuator::rotor_inertia
        struct /* rotor_inertia */ {
          // Source: drake/multibody/tree/joint_actuator.h
          const char* doc =
R"""(Returns the associated rotor inertia value for this actuator, stored
in ``context``. See reflected_inertia. Note that this ONLY depends on
the Parameters in the context; it does not depend on time, input,
state, etc.)""";
        } rotor_inertia;
        // Symbol: drake::multibody::JointActuator::set_actuation_vector
        struct /* set_actuation_vector */ {
          // Source: drake/multibody/tree/joint_actuator.h
          const char* doc =
R"""(Given the actuation values ``u_actuator`` for ``this`` actuator,
updates the actuation vector ``u`` for the entire multibody model to
which this actuator belongs to.

Parameter ``u_actuator``:
    Actuation values for ``this`` actuator. It must be of size equal
    to num_inputs(). For units and sign conventions refer to the
    specific Joint sub-class documentation.

Parameter ``u``:
    Actuation values for the entire plant model to which ``this``
    actuator belongs to. The actuation value in ``u`` for ``this``
    actuator must be found at offset input_start(). Only values
    corresponding to this actuator are changed.

Raises:
    RuntimeError if ``u_actuator.size() != this->num_inputs()``.

Raises:
    RuntimeError if u is nullptr.

Raises:
    RuntimeError if this element is not associated with a
    MultibodyPlant.

Raises:
    RuntimeError if ``u.size() !=
    this->GetParentPlant().num_actuated_dofs()``.)""";
        } set_actuation_vector;
        // Symbol: drake::multibody::JointActuator::set_actuator_dof_start
        struct /* set_actuator_dof_start */ {
          // Source: drake/multibody/tree/joint_actuator.h
          const char* doc = R"""()""";
        } set_actuator_dof_start;
        // Symbol: drake::multibody::JointActuator::set_controller_gains
        struct /* set_controller_gains */ {
          // Source: drake/multibody/tree/joint_actuator.h
          const char* doc =
R"""(Set controller gains for this joint actuator. This enables the
modeling of a simple PD controller of the form: ũ = -Kp⋅(q − qd) -
Kd⋅(v − vd) + u_ff u = max(−e, min(e, ũ)) where qd and vd are the
desired configuration and velocity for joint(), Kp and Kd are the
proportional and derivative gains specified in ``gains``, u_ff is the
feedforward actuation and ``e`` corresponds to effort_limit().

The gains must be finite and non-negative. Setting both gains to zero
will remove the controller (has_controller() will return false).

For simulation, feedforward actuation can be provided through
MultibodyPlant::get_actuation_input_port(). Desired configuration and
velocity are specified through
MultibodyPlant::get_desired_state_input_port().

PD control is currently only supported for a discrete time plant.
Attempting to use non-zero gains on a continuous time plant will
result in an exception. See pd_controllers_and_ports for further
details.)""";
        } set_controller_gains;
        // Symbol: drake::multibody::JointActuator::set_default_gear_ratio
        struct /* set_default_gear_ratio */ {
          // Source: drake/multibody/tree/joint_actuator.h
          const char* doc =
R"""(Sets the default value for this actuator's gear ratio. See
reflected_inertia.)""";
        } set_default_gear_ratio;
        // Symbol: drake::multibody::JointActuator::set_default_rotor_inertia
        struct /* set_default_rotor_inertia */ {
          // Source: drake/multibody/tree/joint_actuator.h
          const char* doc =
R"""(Sets the default value for this actuator's rotor inertia. See
reflected_inertia.)""";
        } set_default_rotor_inertia;
        // Symbol: drake::multibody::JointActuator::set_effort_limit
        struct /* set_effort_limit */ {
          // Source: drake/multibody/tree/joint_actuator.h
          const char* doc =
R"""(Sets the actuator effort limit. (To clear the limit, set it to +∞.))""";
        } set_effort_limit;
      } JointActuator;
      // Symbol: drake::multibody::JointActuatorIndex
      struct /* JointActuatorIndex */ {
        // Source: drake/multibody/tree/multibody_tree_indexes.h
        const char* doc =
R"""(Type used to identify actuators by index within a multibody plant.)""";
      } JointActuatorIndex;
      // Symbol: drake::multibody::JointIndex
      struct /* JointIndex */ {
        // Source: drake/multibody/tree/multibody_tree_indexes.h
        const char* doc =
R"""(Type used to identify joints by index within a multibody plant.)""";
      } JointIndex;
      // Symbol: drake::multibody::JointOrdinal
      struct /* JointOrdinal */ {
        // Source: drake/multibody/tree/multibody_tree_indexes.h
        const char* doc =
R"""(Type used to identify joints by ordinal within a multibody plant.)""";
      } JointOrdinal;
      // Symbol: drake::multibody::LinearBushingRollPitchYaw
      struct /* LinearBushingRollPitchYaw */ {
        // Source: drake/multibody/tree/linear_bushing_roll_pitch_yaw.h
        const char* doc =
R"""(This ForceElement models a massless flexible bushing that connects a
frame A of a link (body) L0 to a frame C of a link (body) L1. The
bushing can apply a torque and force due to stiffness (spring) and
dissipation (damper) properties. Frame B is the bushing frame whose
origin Bo is halfway between Ao (A's origin) and Co (C's origin) and
whose unit vectors 𝐁𝐱, 𝐁𝐲, 𝐁𝐳 are "halfway" (in an angle-axis sense)
between the unit vectors of frame A and frame C. Frame B is a
"floating" frame in the sense that it is calculated from the position
and orientation of frames A and C (B is not welded to the bushing).

@image html drake/multibody/tree/images/LinearBushingRollPitchYaw.png
width=80%

The set of forces on frame C from the bushing is equivalent to a
torque 𝐭 on frame C and a force 𝐟 applied to a point Cp of C. The set
of forces on frame A from the bushing is equivalent to a torque −𝐭 on
frame A and a force −𝐟 applied to a point Ap of A. Points Ap and Cp
are coincident with Bo (frame B's origin).

This "quasi-symmetric" bushing force/torque model was developed at
Toyota Research Institute and has advantages compared to traditional
bushing models because it employs a bushing-centered "symmetric" frame
B and it ensures the moment of −𝐟 on A about Ao is equal to the moment
of 𝐟 on C about Co. Traditional models differ as they lack a
"symmetric" frame B and apply −𝐟 at Ao, which means the moment of −𝐟
on A about Ao is always zero. Note: This bushing model is not fully
symmetric since the orientation between frames A and C is
parameterized with roll-pitch-yaw angles [q₀ q₁ q₂]. Since these
angles have an inherent sequence, they are not mathematically
symmetric.

The torque model depends on spring-damper "gimbal" torques ``τ ≜ [τ₀
τ₁ τ₂]`` which themselves depend on roll-pitch-yaw angles ``q ≜ [q₀ q₁
q₂]`` and rates ``q̇ = [q̇₀ q̇₁ q̇₂]`` via a diagonal torque-stiffness
matrix K₀₁₂ and a diagonal torque-damping matrix D₀₁₂ as


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ⌈ τ₀ ⌉     ⌈k₀    0    0⌉ ⌈ q₀ ⌉     ⌈d₀    0    0⌉ ⌈ q̇₀ ⌉
    τ ≜ | τ₁ | = − | 0   k₁    0| | q₁ |  −  | 0   d₁    0| | q̇₁ |
        ⌊ τ₂ ⌋     ⌊ 0    0   k₂⌋ ⌊ q₂ ⌋     ⌊ 0    0   d₂⌋ ⌊ q̇₂ ⌋

.. raw:: html

    </details>

where k₀, k₁, k₂ and d₀, d₁, d₂ are torque stiffness and damping
constants and must have non-negative values.

Note:
    τ does not represent a vector expressed in one frame. Instead it
    is regarded as a 3x1 array of torque scalars associated with
    roll-pitch yaw.

Note:
    As discussed in the Advanced section below, τ is not 𝐭 ``(τ ≠
    𝐭)``.

Note:
    This is a "linear" bushing model as gimbal torque τ varies
    linearly with q and q̇ as τ = τᴋ + τᴅ where τᴋ = −K₀₁₂ ⋅ q and τᴅ
    = −D₀₁₂ ⋅ q̇.

The bushing model for the net force 𝐟 on frame C from the bushing
depends on scalars x, y, z which are defined so 𝐫 (the position vector
from Ao to Co) can be expressed in frame B as ``𝐫 ≜ p_AoCo = [x y z]ʙ
= x 𝐁𝐱 + y 𝐁𝐲 + z 𝐁𝐳``. The model for 𝐟 uses a diagonal
force-stiffness matrix Kxyᴢ, a diagonal force-damping matrix Dxyᴢ, and
defines fx, fy, fz so ``𝐟 = [fx fy fz]ʙ``.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ⌈ fx ⌉      ⌈kx    0    0⌉ ⌈ x ⌉     ⌈dx    0    0⌉ ⌈ ẋ ⌉
    | fy | =  − | 0   ky    0| | y |  −  | 0   dy    0| | ẏ |
    ⌊ fz ⌋      ⌊ 0    0   kz⌋ ⌊ z ⌋     ⌊ 0    0   dz⌋ ⌊ ż ⌋

.. raw:: html

    </details>

where kx, ky, kz and dx, dy, dz are force stiffness and damping
constants and must have non-negative values.

Note:
    This is a "linear" bushing model as the force 𝐟 varies linearly
    with 𝐫 and 𝐫̇̇ as 𝐟 = 𝐟ᴋ + 𝐟ᴅ where 𝐟ᴋ = −Kxyz ⋅ 𝐫 and 𝐟ᴅ = −Dxyz
    ⋅ 𝐫̇̇.

This bushing's constructor sets the torque stiffness/damping constants
``[k₀ k₁ k₂]`` and ``[d₀ d₁ d₂]`` and the force stiffness/damping
constants ``[kx ky kz]`` and ``[dx dy dz]``. The examples below
demonstrate how to model various joints that have a flexible (e.g.,
rubber) mount. The damping values below with ? may be set to 0 or a
reasonable positive number.

Bushing type | torque constants | force constants
--------------------------------|:--------------------|:------------------
z-axis revolute joint | k₀₁₂ = ``[k₀ k₁ 0]`` | kxyz = ``[kx ky kz]`` ^
| d₀₁₂ = ``[d₀ d₁ ?]`` | dxyz = ``[dx dy dz]`` x-axis prismatic joint
| k₀₁₂ = ``[k₀ k₁ k₂]`` | kxyz = ``[0 ky kz]`` ^ | d₀₁₂ = ``[d₀ d₁
d₂]`` | dxyz = ``[? dy dz]`` Ball and socket joint | k₀₁₂ = ``[0 0
0]`` | kxyz = ``[kx ky kz]`` ^ | d₀₁₂ = ``[? ? ?]`` | dxyz = ``[dx dy
dz]`` Weld/fixed joint | k₀₁₂ = ``[k₀ k₁ k₂]`` | kxyz = ``[kx ky kz]``
^ | d₀₁₂ = ``[d₀ d₁ d₂]`` | dxyz = ``[dx dy dz]``

Angles q₀, q₁, q₂ are calculated from frame C's orientation relative
to frame A, with ``[−π < q₀ ≤ π, −π/2 ≤ q₁ ≤ π/2, −π < q₂ ≤ π]``,
hence, there is no angle wrapping and torque stiffness has a limited
range. Gimbal torques τ can be discontinuous if one of q₀, q₁, q₂ is
discontinuous and its associated torque spring constant is nonzero.
For example, τ₂ is discontinuous if ``k₂ ≠ 0`` and the bushing has a
large rotation so q₂ jumps from ``≈ −π to π``. τ can also be
discontinuous if one of q̇₀, q̇₁, q̇₂ is discontinuous and its
associated torque damper constant is nonzero. For example, τ₀ is
discontinuous if ``d₀ ≠ 0`` and q̇₀ is undefined (which occurs when
``pitch = q₁ = π/2``). Note: Due to the relationship of 𝐭 to τ shown
below, 𝐭 is discontinuous if τ is discontinuous.

As shown below, there are multiple ways to estimate torque and force
stiffness and damping constants. Use a method or combination of
methods appropriate for your application. For example, some methods
are more useful for a real physical bushing whereas other methods
(called "penalty methods") can be more useful when replacing an ideal
joint (such as a revolute or fixed/weld joint) with a bushing.

Consider a penalty method if you want a bushing to substitute for a
"hard" constraint (e.g., an ideal joint). Since a bushing is
inherently compliant it will violate a hard constraint somewhat. The
stiffer the bushing, the more accurately it enforces the hard
constraint, but at a cost of more computational time. To balance
accuracy versus time, consider your tolerance for constraint errors.
For example, is it OK for your bushing to displace xₘₐₓ = 1 mm for an
estimated Fxₘₐₓ = 100 N? Also, one way to choose a force damping
constant dx is by choosing a "reasonably small" settling time tₛ,
where settling time tₛ is the interval of time for a system to settle
to within 1% (0.01) of an equilibrium solution). Is tₛ = 0.01 s
negligible for a robot arm with a 10 s reach maneuver?

**** How to choose a torque stiffness constant k₀ or damping constant
d₀. The estimate of stiffness k₀ depends on whether you are modeling a
physical bushing (consider stiffness methods 1 or 2 below) or whether
you are using a bushing to replace an ideal joint such as a revolute
or fixed/weld joint (consider stiffness "penalty methods" 3 or 4
below). 1. Use a static experiment, e.g., apply a known moment load
Mx, measure the associated angular displacement Δq (radians), and
estimate k₀ = Mx / Δq. 2. Use FEA (finite element analysis) software
to estimate k₀. 3. Pick a desired maximum angular displacement qₘₐₓ,
estimate a maximum moment load Mxₘₐₓ, and estimate ``k₀ = Mxₘₐₓ /
qₘₐₓ`` (units of N*m/rad). 4. Choose a characteristic moment of
inertia I₀ (directionally dependent), choose a desired angular
frequency ωₙ > 0 (in rad/s) and estimate ``k₀ = I₀ ωₙ²`` (units of
N*m/rad).

The estimate of damping d₀ depends on whether you are modeling a
physical bushing (consider damping method 1 below) or whether you are
using a bushing to enforce a constraint (consider damping methods 2 or
3 below). 1. Use experiments to estimate a damping ratio ζ and
settling time tₛ. Compute "undamped natural frequency" ωₙ from ζ and
tₛ (as shown below in the Advanced section), then ``d₀ = 2 ζ k₀ / ωₙ``
(units of N*m*s/rad). 2. Choose a damping ratio ζ (e.g., ζ = 1,
critical damping) and a desired settling time tₛ, calculate ωₙ (as
shown below in the Advanced section), then ``d₀ = 2 ζ k₀ / ωₙ`` (units
of N*m*s/rad). 3. Choose a damping ratio ζ (e.g., ζ = 1, critical
damping), estimate a characteristic moment of inertia and calculate
``d₀ = 2 ζ √(I₀ k₀)``.

Refer to Advanced_bushing_stiffness_and_damping "Advanced bushing
stiffness and damping" for more details.

**** How to choose a force stiffness constant kx or damping constant
dx. The estimate of stiffness kx depends on whether you are modeling a
real bushing (consider stiffness methods 1 or 2 below) or whether you
are using a bushing to replace an ideal joint such as a revolute or
fixed/weld joint (consider stiffness "penalty methods" 3 or 4 below).
1. Use a static experiment, e.g., apply a known force load Fx, measure
the associated displacement (stretch) Δx (in meters), and estimate kx
= Fx / Δx. 2. Use FEA (finite element analysis) software to estimate
kx (units of N/m). 3. Pick a desired maximum displacement xₘₐₓ,
estimate a maximum force load Fxₘₐₓ, and estimate ``kx = Fxₘₐₓ /
xₘₐₓ`` (units of N/m). 4. Choose a characteristic mass m (which may be
directionally dependent), choose a desired angular frequency ωₙ > 0
(in rad/s) and estimate ``kx = m ωₙ²`` (units of N/m).

The estimate of damping dx depends on whether you are modeling a
physical bushing (consider damping method 1 below) or whether you are
using a bushing to enforce a constraint (consider damping methods 2 or
3 below). 1. Use experiments to estimate a damping ratio ζ and
settling time tₛ. Compute "undamped natural frequency" ωₙ from ζ and
tₛ (as shown below in the Advanced section), then ``dx = 2 ζ kx / ωₙ``
(units of N*s/m). 2. Choose a damping ratio ζ (e.g., ζ = 1, critical
damping) and a desired settling time tₛ, calculate ωₙ (as shown below
in the Advanced section), then ``dx = 2 ζ kx / ωₙ`` (units of N*s/m).
3. Choose a damping ratio ζ (e.g., ζ = 1, critical damping), estimate
a characteristic mass m and calculate ``dx = 2 ζ √(m kx)`` (units of
N*s/m).

Refer to Advanced_bushing_stiffness_and_damping "Advanced bushing
stiffness and damping" for more details.

**** Advanced: Relationship of 𝐭 to τ. To understand how "gimbal
torques" τ relate to 𝐭, it helps to remember that the RollPitchYaw
class documentation states that a Space-fixed (extrinsic) X-Y-Z
rotation with roll-pitch-yaw angles [q₀ q₁ q₂] is equivalent to a
Body-fixed (intrinsic) Z-Y-X rotation by yaw-pitch-roll angles [q₂ q₁
q₀]. In the context of "gimbal torques", the Body-fixed Z-Y-X rotation
sequence with angles [q₂ q₁ q₀] is physical meaningful as it produces
torques associated with successive frames in a gimbal as τ₂ 𝐀𝐳, τ₁ 𝐏𝐲,
τ₀ 𝐂𝐱, where each of 𝐀𝐳, 𝐏𝐲, 𝐂𝐱 are unit vectors associated with a
frame in the yaw-pitch-roll rotation sequence and 𝐏𝐲 is a unit vector
of the "pitch" intermediate frame. As described earlier, torque 𝐭 is
the moment of the bushing forces on frame C about Cp. Scalars tx, ty,
tz are defined so 𝐭 can be expressed ``𝐭 = [tx ty tz]ᴀ = tx 𝐀𝐱 + ty 𝐀𝐲
+ tz 𝐀𝐳``. As shown in code documentation, the relationship of [tx ty
tz] to [τ₀ τ₁ τ₂] was found by equating 𝐭's power to τ's power as 𝐭 ⋅
w_AC = τ ⋅ q̇.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ⌈ tx ⌉      ⌈ τ₀ ⌉            ⌈ cos(q₂)/cos(q₁)  sin(q₂)/cos(q₁)   0 ⌉
    | ty | = Nᵀ | τ₁ |  where N = |   −sin(q2)            cos(q2)      0 |
    ⌊ tz ⌋      ⌊ τ₂ ⌋            ⌊ cos(q₂)*tan(q₁)   sin(q₂)*tan(q₁)  1 ⌋

.. raw:: html

    </details>

**** Advanced: More on how to choose bushing stiffness and damping
constants. The basics on how to choose bushing stiffness and damping
constants are at: - Basic_bushing_torque_stiffness_and_damping "How to
choose torque stiffness and damping constants" -
Basic_bushing_force_stiffness_and_damping "How to choose force
stiffness and damping constants"

The list below provides more detail on: The performance tradeoff
between high stiffness and long simulation time; loads that affect
estimates of Mxₘₐₓ or Fxₘₐₓ; and how a linear 2ⁿᵈ-order ODE provides
insight on how to experimentally determine stiffness and damping
constants. - Stiffness [k₀ k₁ k₂] and [kx ky kz] affect simulation
time and accuracy. Generally, a stiffer bushing better resembles an
ideal joint (e.g., a revolute joint or fixed/weld joint). However
(depending on integrator), a stiffer bushing usually increases
numerical integration time. - An estimate for a maximum load Mxₘₐₓ or
Fxₘₐₓ accounts for gravity forces, applied forces, inertia forces
(centripetal, Coriolis, gyroscopic), etc. - One way to determine
physical stiffness and damping constants is through the mathematical
intermediaries ωₙ (units of rad/s) and ζ (no units). The constant ωₙ
(called "undamped natural frequency" or "angular frequency") and
constant ζ (called "damping ratio") relate to the physical constants
mass m, damping constant dx, and stiffness constant kx via the
following prototypical linear constant-coefficient 2ⁿᵈ-order ODEs.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    m ẍ +     dx ẋ +  kx x = 0   or alternatively as
       ẍ + 2 ζ ωₙ ẋ + ωₙ² x = 0   where ωₙ² = kx/m,  ζ = dx / (2 √(m kx))

.. raw:: html

    </details>

ωₙ and ζ also appear in the related ODEs for rotational systems,
namely


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    I₀ q̈ +     d₀ q̇ +  k₀ q = 0   or alternatively as
        q̈ + 2 ζ ωₙ q̇ + ωₙ² q = 0   where ωₙ² = k₀/I₀,  ζ = d₀ / (2 √(I₀ k₀))

.. raw:: html

    </details>

One way to determine ωₙ is from settling time tₛ which approximates
the time for a system to settle to within a specified settling ratio
of an equilibrium solutions. Typical values for settling ratio are 1%
(0.01), 2% (0.02), 5% (0.05), and 10% (0.10). - When ζ < 0.7
(underdamped), a commonly used approximation is ωₙ ≈
-ln(settling_ratio) / (ζ tₛ) which for settling ratios 0.01 and 0.05
give ωₙ ≈ 4.6 / (ζ tₛ) and ωₙ ≈ 3 / (ζ tₛ). Another commonly used
approximation is ωₙ ≈ -ln(settling_ratio √(1- ζ²)) / (ζ tₛ). See
https://en.wikipedia.org/wiki/Settling_time or the book Modern Control
Engineering by Katsuhiko Ogata. Although these approximate formulas
for ωₙ are common, they are somewhat inaccurate. Settling time for
underdamped systems is discontinuous and requires solving a nonlinear
algebraic equation (an iterative process). For more information, see
http://www.scielo.org.co/pdf/rfiua/n66/n66a09.pdf [Ramos-Paja, et. al
2012], "Accurate calculation of settling time in second order systems:
a photovoltaic application". Another reference is
https://courses.grainger.illinois.edu/ece486/sp2020/laboratory/docs/lab2/estimates.html
- When ζ ≈ 1 (critically damped), ωₙ is determined by choosing a
settling ratio and then solving for (ωₙ tₛ) via the nonlinear
algebraic equation (1 + ωₙ tₛ)*exp(-ωₙ tₛ) = settling_ratio. Settling
ratio | ωₙ -------------- | ------------- 0.01 | 6.64 / tₛ 0.02 | 5.83
/ tₛ 0.05 | 4.74 / tₛ 0.10 | 3.89 / tₛ See
https://electronics.stackexchange.com/questions/296567/over-and-critically-damped-systems-settling-time
- When ζ ≥ 1.01 (overdamped), ωₙ ≈ -ln(2 settling_ratio sz/s₂) / (s₁
tₛ) where sz = √(ζ² - 1), s₁ = ζ - sz, s₂ = ζ + sz. The derivation and
approximation error estimates for this overdamped settling time
formula is ApproximateOverdampedSettlingTime "below".

- For a real physical bushing, an experiment is one way to estimate damping
constants.  For example, to estimate a torque damping constant d₀ associated
with underdamped vibrations (damping ratio 0 < ζ < 1), attach the bushing to
a massive rod, initially displace the rod by angle Δq, release the rod and
measure q(t).  From the q(t) measurement, estimate decay ratio (the ratio of
successive peak heights above the final steady-state value) calculate
logarithmic decrement δ = -ln(decay_ratio), calculate damping ratio
ζ = √(δ² / (4π² + δ²)), then calculate d₀ using d₀ = 2 ζ √(I₀ k₀) or
d₀ = 2 ζ k₀ / ωₙ. For more information, see
https://en.wikipedia.org/wiki/Damping_ratio#Logarithmic_decrement

**** Derivation: Approximate formula for overdamped settling time.
Since a literature reference for this formula was not found, the
derivation below was done at TRI (it has not been peer reviewed). This
formula results from the "dominant pole" solution in the prototypical
constant-coefficient linear 2ⁿᵈ-order ODE. For ẋ(0) = 0, mathematics
shows poles p₁ = -ωₙ s₁, p₂ = -ωₙ s₂, where sz = √(ζ² - 1), s₁ = ζ -
sz, s₂ = ζ + sz. and


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    x(t) / x(0) = p₂/(p₂-p₁) exp(p₁ t) - p₁/(p₂-p₁) exp(p₂ t)
                 = s₂/(s₂-s₁) exp(p₁ t) - s₁/(s₂-s₁) exp(p₂ t)
                 =  k/( k-1 ) exp(p₁ t) -  1/( k-1 ) exp(p₂ t) where k = s₂ / s₁
                 ≈  k/( k-1 ) exp(p₁ t)                        since p₁ > p₂

.. raw:: html

    </details>

Note: k = s₂ / s₁ is real, k > 0, s₂ = k s₁, and p₁ > p₂ (p₁ is less
negative then p₂), so exp(p₁ t) decays to zero slower than exp(p₂ t)
and exp(p₁ t) ≫ exp(p₂ t) for sufficiently large t. Hence we assume
exp(p₂ t) ≈ 0 (which is why p₁ is called the "dominant pole"). Next,


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    k/(k - 1) = s₂ / s₁ / (s₂/s₁ -1) = s₂ / (s₂ - s₁) = s₂ / (2 sz),  so
      x(t) / x(0)  ≈  s₂ / (2 sz) exp(-s₁ ωₙ t),                        hence
      settling_ratio ≈ s₂ / (2 sz) exp(-s₁ ωₙ tₛ),                      finally
      ωₙ ≈ -ln(settling_ratio 2 sz / s₂) / (s₁ tₛ)

.. raw:: html

    </details>

The table below shows that there is little error in this approximate
formula for various settling ratios and ζ, particularly for ζ ≥ 1.1.
For 1.0 ≤ ζ < 1.1, the critical damping estimates of ωₙ work well.
Settling ratio | ζ = 1.01 | ζ = 1.1 | ζ = 1.2 | ζ = 1.3 | ζ = 1.5
-------------- | -------- | ------- | ------- | ------- | --------
0.01 | 1.98% | 0.005% | 2.9E-5% | 1.6E-7% | 2.4E-12% 0.02 | 2.91% |
0.016% | 1.8E-4% | 2.1E-6% | 1.6E-10% 0.05 | 5.10% | 0.076% | 2.3E-3%
| 7.0E-5% | 4.4E-8% 0.10 | 8.28% | 0.258% | 1.6E-2% | 1.0E-3% |
3.3E-6% Note: There is a related derivation in the reference below,
however, it needlessly makes the oversimplified approximation k/(k -
1) ≈ 1.
https://electronics.stackexchange.com/questions/296567/over-and-critically-damped-systems-settling-time

Note:
    The complete theory for this bushing is documented in the source
    code. Please look there if you want more information.

Template parameter ``T``:
    The underlying scalar type. Must be a valid Eigen scalar.

See also:
    math::RollPitchYaw for definitions of roll, pitch, yaw ``[q₀ q₁
    q₂]``.

Note:
    Per issue #12982, do not directly or indirectly call the following
    methods as they have not yet been implemented and throw an
    exception: CalcPotentialEnergy(), CalcConservativePower(),
    CalcNonConservativePower().)""";
        // Symbol: drake::multibody::LinearBushingRollPitchYaw::CalcBushingSpatialForceOnFrameA
        struct /* CalcBushingSpatialForceOnFrameA */ {
          // Source: drake/multibody/tree/linear_bushing_roll_pitch_yaw.h
          const char* doc =
R"""(Calculate F_A_A, the bushing's spatial force on frame A expressed in
A. F_A_A contains two vectors: the moment of all bushing forces on A
about Ao (−𝐭 + p_AoAp × −𝐟) and the net bushing force on A (−𝐟
expressed in A).

Parameter ``context``:
    The state of the multibody system.

See also:
    CalcBushingSpatialForceOnFrameC().

Raises:
    RuntimeError if pitch angle is near gimbal-lock. For more info,

See also:
    RollPitchYaw::DoesCosPitchAngleViolateGimbalLockTolerance().)""";
        } CalcBushingSpatialForceOnFrameA;
        // Symbol: drake::multibody::LinearBushingRollPitchYaw::CalcBushingSpatialForceOnFrameC
        struct /* CalcBushingSpatialForceOnFrameC */ {
          // Source: drake/multibody/tree/linear_bushing_roll_pitch_yaw.h
          const char* doc =
R"""(Calculate F_C_C, the bushing's spatial force on frame C expressed in
C. F_C_C contains two vectors: the moment of all bushing forces on C
about Co (𝐭 + p_CoCp × 𝐟) and the resultant bushing force on C (𝐟
expressed in C).

Parameter ``context``:
    The state of the multibody system.

See also:
    CalcBushingSpatialForceOnFrameA().

Raises:
    RuntimeError if pitch angle is near gimbal-lock. For more info,

See also:
    RollPitchYaw::DoesCosPitchAngleViolateGimbalLockTolerance().)""";
        } CalcBushingSpatialForceOnFrameC;
        // Symbol: drake::multibody::LinearBushingRollPitchYaw::GetForceDampingConstants
        struct /* GetForceDampingConstants */ {
          // Source: drake/multibody/tree/linear_bushing_roll_pitch_yaw.h
          const char* doc =
R"""(Returns the force damping constants ``[dx dy dz]`` (units of N*s/m)
stored in ``context``.)""";
        } GetForceDampingConstants;
        // Symbol: drake::multibody::LinearBushingRollPitchYaw::GetForceStiffnessConstants
        struct /* GetForceStiffnessConstants */ {
          // Source: drake/multibody/tree/linear_bushing_roll_pitch_yaw.h
          const char* doc =
R"""(Returns the force stiffness constants ``[kx ky kz]`` (units of N/m)
stored in ``context``.)""";
        } GetForceStiffnessConstants;
        // Symbol: drake::multibody::LinearBushingRollPitchYaw::GetTorqueDampingConstants
        struct /* GetTorqueDampingConstants */ {
          // Source: drake/multibody/tree/linear_bushing_roll_pitch_yaw.h
          const char* doc =
R"""(Returns the torque damping constants ``[d₀ d₁ d₂]`` (units of
N*m*s/rad) stored in ``context``.)""";
        } GetTorqueDampingConstants;
        // Symbol: drake::multibody::LinearBushingRollPitchYaw::GetTorqueStiffnessConstants
        struct /* GetTorqueStiffnessConstants */ {
          // Source: drake/multibody/tree/linear_bushing_roll_pitch_yaw.h
          const char* doc =
R"""(Returns the torque stiffness constants ``[k₀ k₁ k₂]`` (units of
N*m/rad) stored in ``context``.)""";
        } GetTorqueStiffnessConstants;
        // Symbol: drake::multibody::LinearBushingRollPitchYaw::LinearBushingRollPitchYaw<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/linear_bushing_roll_pitch_yaw.h
          const char* doc =
R"""(Construct a LinearBushingRollPitchYaw B that connects frames A and C,
where frame A is welded to a link L0 and frame C is welded to a link
L1.

Parameter ``frameA``:
    frame A of link (body) L0 that connects to bushing B.

Parameter ``frameC``:
    frame C of link (body) L1 that connects to bushing B.

Parameter ``torque_stiffness_constants``:
    ``[k₀ k₁ k₂]`` multiply the roll-pitch-yaw angles ``[q₀ q₁ q₂]``
    to produce the spring portion of the "gimbal" torques τ₀, τ₁, τ₂.
    The SI units of ``k₀, k₁, k₂`` are N*m/rad.

Parameter ``torque_damping_constants``:
    ``[d₀ d₁ d₂]`` multiply the roll-pitch-yaw rates ``[q̇₀ q̇₁ q̇₂]``
    to produce the damper portion of the "gimbal" torques τ₀, τ₁, τ₂.
    The SI units of ``d₀, d₁, d₂`` are N*m*s/rad.

Parameter ``force_stiffness_constants``:
    ``[kx ky kz]`` multiply the bushing displacements ``[x y z]`` to
    form 𝐟ᴋ, the spring portion of the force 𝐟 = [fx fy fz]ʙ. The SI
    units of ``kx, ky, kz`` are N/m.

Parameter ``force_damping_constants``:
    ``[dx dy dz]`` multiply the bushing displacement rates ``[ẋ ẏ
    ż]`` to form 𝐟ᴅ, the damper portion of the force 𝐟 = [fx fy fz]ʙ.
    The SI units of ``dx, dy, dz`` are N*s/m.

Note:
    The LinearBushingRollPitchYaw class documentation describes the
    stiffness and damping constants.

Note:
    The net moment on C about Co is affected by both the gimbal torque
    and the moment of 𝐟 about Co. Similarly, for the net moment on A
    about Ao.

Note:
    math::RollPitchYaw describes the roll pitch yaw angles q₀, q₁, q₂.
    The position from Ao to Co is p_AoCo_B = x 𝐁𝐱 + y 𝐁𝐲 + z 𝐁𝐳 = [x y
    z]ʙ.

Note:
    The ModelInstanceIndex assigned to this by the constructor is the
    one assigned to frame C, i.e., frameC.model_instance().

Precondition:
    All the stiffness and damping constants must be non-negative.)""";
        } ctor;
        // Symbol: drake::multibody::LinearBushingRollPitchYaw::SetForceDampingConstants
        struct /* SetForceDampingConstants */ {
          // Source: drake/multibody/tree/linear_bushing_roll_pitch_yaw.h
          const char* doc =
R"""(Sets the force damping constants ``[dx dy dz]`` (units of N*s/m) in
``context``.)""";
        } SetForceDampingConstants;
        // Symbol: drake::multibody::LinearBushingRollPitchYaw::SetForceStiffnessConstants
        struct /* SetForceStiffnessConstants */ {
          // Source: drake/multibody/tree/linear_bushing_roll_pitch_yaw.h
          const char* doc =
R"""(Sets the force stiffness constants ``[kx ky kz]`` (units of N/m) in
``context``.)""";
        } SetForceStiffnessConstants;
        // Symbol: drake::multibody::LinearBushingRollPitchYaw::SetTorqueDampingConstants
        struct /* SetTorqueDampingConstants */ {
          // Source: drake/multibody/tree/linear_bushing_roll_pitch_yaw.h
          const char* doc =
R"""(Sets the torque damping constants ``[d₀ d₁ d₂]`` (units of N*m*s/rad)
in ``context``.)""";
        } SetTorqueDampingConstants;
        // Symbol: drake::multibody::LinearBushingRollPitchYaw::SetTorqueStiffnessConstants
        struct /* SetTorqueStiffnessConstants */ {
          // Source: drake/multibody/tree/linear_bushing_roll_pitch_yaw.h
          const char* doc =
R"""(Sets the torque stiffness constants ``[k₀ k₁ k₂]`` (units of N*m/rad)
in ``context``.)""";
        } SetTorqueStiffnessConstants;
        // Symbol: drake::multibody::LinearBushingRollPitchYaw::force_damping_constants
        struct /* force_damping_constants */ {
          // Source: drake/multibody/tree/linear_bushing_roll_pitch_yaw.h
          const char* doc =
R"""(Returns the default force damping constants ``[dx dy dz]`` (units of
N*s/m). Refer to Basic_bushing_force_stiffness_and_damping "How to
choose force stiffness and damping constants" for more details.)""";
        } force_damping_constants;
        // Symbol: drake::multibody::LinearBushingRollPitchYaw::force_stiffness_constants
        struct /* force_stiffness_constants */ {
          // Source: drake/multibody/tree/linear_bushing_roll_pitch_yaw.h
          const char* doc =
R"""(Returns the default force stiffness constants ``[kx ky kz]`` (units of
N/m). Refer to Basic_bushing_force_stiffness_and_damping "How to
choose force stiffness and damping constants" for more details.)""";
        } force_stiffness_constants;
        // Symbol: drake::multibody::LinearBushingRollPitchYaw::frameA
        struct /* frameA */ {
          // Source: drake/multibody/tree/linear_bushing_roll_pitch_yaw.h
          const char* doc =
R"""(Returns frame A, which is the frame that is welded to link (body) L0
and attached to the bushing.

Raises:
    RuntimeError if this element is not associated with a
    MultibodyPlant.)""";
        } frameA;
        // Symbol: drake::multibody::LinearBushingRollPitchYaw::frameC
        struct /* frameC */ {
          // Source: drake/multibody/tree/linear_bushing_roll_pitch_yaw.h
          const char* doc =
R"""(Returns frame C, which is the frame that is welded to link (body) L1
and attached to the bushing.

Raises:
    RuntimeError if this element is not associated with a
    MultibodyPlant.)""";
        } frameC;
        // Symbol: drake::multibody::LinearBushingRollPitchYaw::link0
        struct /* link0 */ {
          // Source: drake/multibody/tree/linear_bushing_roll_pitch_yaw.h
          const char* doc =
R"""(Returns link (body) L0 (frame A is welded to link L0).)""";
        } link0;
        // Symbol: drake::multibody::LinearBushingRollPitchYaw::link1
        struct /* link1 */ {
          // Source: drake/multibody/tree/linear_bushing_roll_pitch_yaw.h
          const char* doc =
R"""(Returns link (body) L1 (frame C is welded to link L1).)""";
        } link1;
        // Symbol: drake::multibody::LinearBushingRollPitchYaw::torque_damping_constants
        struct /* torque_damping_constants */ {
          // Source: drake/multibody/tree/linear_bushing_roll_pitch_yaw.h
          const char* doc =
R"""(Returns the default torque damping constants ``[d₀ d₁ d₂]`` (units of
N*m*s/rad). Refer to Basic_bushing_torque_stiffness_and_damping "How
to choose torque stiffness and damping constants" for more details.)""";
        } torque_damping_constants;
        // Symbol: drake::multibody::LinearBushingRollPitchYaw::torque_stiffness_constants
        struct /* torque_stiffness_constants */ {
          // Source: drake/multibody/tree/linear_bushing_roll_pitch_yaw.h
          const char* doc =
R"""(Returns the default torque stiffness constants ``[k₀ k₁ k₂]`` (units
of N*m/rad). Refer to Basic_bushing_torque_stiffness_and_damping "How
to choose torque stiffness and damping constants" for more details.)""";
        } torque_stiffness_constants;
      } LinearBushingRollPitchYaw;
      // Symbol: drake::multibody::LinearSpringDamper
      struct /* LinearSpringDamper */ {
        // Source: drake/multibody/tree/linear_spring_damper.h
        const char* doc =
R"""(This ForceElement models a spring-damper attached between two points
on two different bodies. Given a point P on a body A and a point Q on
a body B with positions p_AP and p_BQ, respectively, this
spring-damper applies equal and opposite forces on bodies A and B
according to:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    f_AP = (k⋅(ℓ - ℓ₀) + c⋅dℓ/dt)⋅r̂
      f_BQ = -f_AP

.. raw:: html

    </details>

where ``ℓ = ‖p_WQ - p_WP‖`` is the current length of the spring, dℓ/dt
its rate of change, ``r̂ = (p_WQ - p_WP) / ℓ`` is the normalized
vector from P to Q, ℓ₀ is the free length of the spring and k and c
are the stiffness and damping of the spring-damper, respectively. This
ForceElement is meant to model finite free length springs attached
between two points. In this typical arrangement springs are usually
pre-loaded, meaning they apply a non-zero spring force in the static
configuration of the system. Thus, neither the free length ℓ₀ nor the
current length ℓ of the spring can ever be zero. The length of the
spring approaching zero would incur in a non-physical configuration
and therefore this element throws a RuntimeError exception in that
case. Note that:

- The applied force is always along the line connecting points P and
Q. - Damping always dissipates energy. - Forces on bodies A and B are
equal and opposite according to Newton's third law.)""";
        // Symbol: drake::multibody::LinearSpringDamper::CalcConservativePower
        struct /* CalcConservativePower */ {
          // Source: drake/multibody/tree/linear_spring_damper.h
          const char* doc = R"""()""";
        } CalcConservativePower;
        // Symbol: drake::multibody::LinearSpringDamper::CalcNonConservativePower
        struct /* CalcNonConservativePower */ {
          // Source: drake/multibody/tree/linear_spring_damper.h
          const char* doc = R"""()""";
        } CalcNonConservativePower;
        // Symbol: drake::multibody::LinearSpringDamper::CalcPotentialEnergy
        struct /* CalcPotentialEnergy */ {
          // Source: drake/multibody/tree/linear_spring_damper.h
          const char* doc = R"""()""";
        } CalcPotentialEnergy;
        // Symbol: drake::multibody::LinearSpringDamper::DoCalcAndAddForceContribution
        struct /* DoCalcAndAddForceContribution */ {
          // Source: drake/multibody/tree/linear_spring_damper.h
          const char* doc = R"""()""";
        } DoCalcAndAddForceContribution;
        // Symbol: drake::multibody::LinearSpringDamper::DoCloneToScalar
        struct /* DoCloneToScalar */ {
          // Source: drake/multibody/tree/linear_spring_damper.h
          const char* doc = R"""()""";
        } DoCloneToScalar;
        // Symbol: drake::multibody::LinearSpringDamper::DoShallowClone
        struct /* DoShallowClone */ {
          // Source: drake/multibody/tree/linear_spring_damper.h
          const char* doc = R"""()""";
        } DoShallowClone;
        // Symbol: drake::multibody::LinearSpringDamper::LinearSpringDamper<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/linear_spring_damper.h
          const char* doc =
R"""(Constructor for a spring-damper between a point P on ``bodyA`` and a
point Q on ``bodyB``. Point P is defined by its position ``p_AP`` as
measured and expressed in the body frame A and similarly, point Q is
defined by its position p_BQ as measured and expressed in body frame
B. The remaining parameters define:

Parameter ``free_length``:
    The free length of the spring ℓ₀, in meters, at which the spring
    applies no forces. Since this force element is meant to model
    finite length springs, ℓ₀ must be strictly positive.

Parameter ``stiffness``:
    The stiffness k of the spring in N/m. It must be non-negative.

Parameter ``damping``:
    The damping c of the damper in N⋅s/m. It must be non-negative.
    Refer to this class's documentation for further details.

Raises:
    RuntimeError if ``free_length`` is negative or zero.

Raises:
    RuntimeError if ``stiffness`` is negative.

Raises:
    RuntimeError if ``damping`` is negative.)""";
        } ctor;
        // Symbol: drake::multibody::LinearSpringDamper::bodyA
        struct /* bodyA */ {
          // Source: drake/multibody/tree/linear_spring_damper.h
          const char* doc = R"""()""";
        } bodyA;
        // Symbol: drake::multibody::LinearSpringDamper::bodyB
        struct /* bodyB */ {
          // Source: drake/multibody/tree/linear_spring_damper.h
          const char* doc = R"""()""";
        } bodyB;
        // Symbol: drake::multibody::LinearSpringDamper::damping
        struct /* damping */ {
          // Source: drake/multibody/tree/linear_spring_damper.h
          const char* doc = R"""()""";
        } damping;
        // Symbol: drake::multibody::LinearSpringDamper::free_length
        struct /* free_length */ {
          // Source: drake/multibody/tree/linear_spring_damper.h
          const char* doc = R"""()""";
        } free_length;
        // Symbol: drake::multibody::LinearSpringDamper::p_AP
        struct /* p_AP */ {
          // Source: drake/multibody/tree/linear_spring_damper.h
          const char* doc =
R"""(The position p_AP of point P on body A as measured and expressed in
body frame A.)""";
        } p_AP;
        // Symbol: drake::multibody::LinearSpringDamper::p_BQ
        struct /* p_BQ */ {
          // Source: drake/multibody/tree/linear_spring_damper.h
          const char* doc =
R"""(The position p_BQ of point Q on body B as measured and expressed in
body frame B.)""";
        } p_BQ;
        // Symbol: drake::multibody::LinearSpringDamper::stiffness
        struct /* stiffness */ {
          // Source: drake/multibody/tree/linear_spring_damper.h
          const char* doc = R"""()""";
        } stiffness;
      } LinearSpringDamper;
      // Symbol: drake::multibody::ModelInstanceIndex
      struct /* ModelInstanceIndex */ {
        // Source: drake/multibody/tree/multibody_tree_indexes.h
        const char* doc =
R"""(Type used to identify model instances by index within a multibody
plant.)""";
      } ModelInstanceIndex;
      // Symbol: drake::multibody::MultibodyConstraintId
      struct /* MultibodyConstraintId */ {
        // Source: drake/multibody/tree/multibody_tree_indexes.h
        const char* doc =
R"""(Type used to identify constraint by id within a multibody plant.)""";
      } MultibodyConstraintId;
      // Symbol: drake::multibody::MultibodyElement
      struct /* MultibodyElement */ {
        // Source: drake/multibody/tree/multibody_element.h
        const char* doc =
R"""(A class representing an element (subcomponent) of a MultibodyPlant or
(internally) a MultibodyTree. Examples of multibody elements are
bodies, joints, force elements, and actuators. After a Finalize()
call, multibody elements get assigned a type-specific index that
uniquely identifies them. By convention, every subclass of
MultibodyElement provides an ``index()`` member function that returns
the assigned index, e.g.,


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    /** Returns this element's unique index. 
    BodyIndex index() const { return this->template index_impl<BodyIndex>(); }

.. raw:: html

    </details>

Some multibody elements are added during Finalize() and are not part
of the user-specified model. These are called "ephemeral" elements and
can be identified using the ``is_ephemeral()`` function here. Examples
include - free joints added to connect lone bodies or free-floating
trees to World - fixed offset frames added when joints are modeled by
mobilizers - all mobilizers.)""";
        // Symbol: drake::multibody::MultibodyElement::DeclareAbstractParameter
        struct /* DeclareAbstractParameter */ {
          // Source: drake/multibody/tree/multibody_element.h
          const char* doc =
R"""(To be used by MultibodyElement-derived objects when declaring
parameters in their implementation of DoDeclareParameters(). For an
example, see Joint::DoDeclareParameters().)""";
        } DeclareAbstractParameter;
        // Symbol: drake::multibody::MultibodyElement::DeclareCacheEntries
        struct /* DeclareCacheEntries */ {
          // Source: drake/multibody/tree/multibody_element.h
          const char* doc =
R"""((Advanced) Declares all cache entries needed by this element. This
method is called by MultibodyTree on ``this`` element during
MultibodyTree::Finalize(). It subsequently calls
DoDeclareCacheEntries(). Custom elements that need to declare cache
entries must override DoDeclareCacheEntries().)""";
        } DeclareCacheEntries;
        // Symbol: drake::multibody::MultibodyElement::DeclareCacheEntry
        struct /* DeclareCacheEntry */ {
          // Source: drake/multibody/tree/multibody_element.h
          const char* doc =
R"""(To be used by MultibodyElement-derived objects when declaring cache
entries in their implementation of DoDeclareCacheEntries(). For an
example, see DeformableBody::DoDeclareCacheEntries().)""";
        } DeclareCacheEntry;
        // Symbol: drake::multibody::MultibodyElement::DeclareDiscreteState
        struct /* DeclareDiscreteState */ {
          // Source: drake/multibody/tree/multibody_element.h
          const char* doc_1args =
R"""(Declares MultibodyTreeSystem discrete states. NVI to the virtual
method DoDeclareDiscreteState().

Parameter ``tree_system``:
    A mutable copy of the parent MultibodyTreeSystem.

Precondition:
    'tree_system' must be the same as the parent tree system (what's
    returned from GetParentTreeSystem()).)""";
          // Source: drake/multibody/tree/multibody_element.h
          const char* doc_2args =
R"""(To be used by MultibodyElement-derived objects when declaring discrete
states in their implementation of DoDeclareDiscreteStates(). For an
example, see DeformableBody::DoDeclareDiscreteStates().)""";
        } DeclareDiscreteState;
        // Symbol: drake::multibody::MultibodyElement::DeclareNumericParameter
        struct /* DeclareNumericParameter */ {
          // Source: drake/multibody/tree/multibody_element.h
          const char* doc =
R"""(To be used by MultibodyElement-derived objects when declaring
parameters in their implementation of DoDeclareParameters(). For an
example, see RigidBody::DoDeclareParameters().)""";
        } DeclareNumericParameter;
        // Symbol: drake::multibody::MultibodyElement::DeclareParameters
        struct /* DeclareParameters */ {
          // Source: drake/multibody/tree/multibody_element.h
          const char* doc =
R"""(Declares MultibodyTreeSystem Parameters at
MultibodyTreeSystem::Finalize() time. NVI to the virtual method
DoDeclareParameters().

Parameter ``tree_system``:
    A mutable copy of the parent MultibodyTreeSystem.

Precondition:
    'tree_system' must be the same as the parent tree system (what's
    returned from GetParentTreeSystem()).)""";
        } DeclareParameters;
        // Symbol: drake::multibody::MultibodyElement::DoDeclareCacheEntries
        struct /* DoDeclareCacheEntries */ {
          // Source: drake/multibody/tree/multibody_element.h
          const char* doc =
R"""(Derived classes must override this method to declare cache entries
needed by ``this`` element. The default implementation is a no-op.)""";
        } DoDeclareCacheEntries;
        // Symbol: drake::multibody::MultibodyElement::DoDeclareDiscreteState
        struct /* DoDeclareDiscreteState */ {
          // Source: drake/multibody/tree/multibody_element.h
          const char* doc =
R"""(Implementation of the NVI DeclareDiscreteState().
MultibodyElement-derived objects may override to declare their
specific state variables.)""";
        } DoDeclareDiscreteState;
        // Symbol: drake::multibody::MultibodyElement::DoDeclareParameters
        struct /* DoDeclareParameters */ {
          // Source: drake/multibody/tree/multibody_element.h
          const char* doc =
R"""(Implementation of the NVI DeclareParameters().
MultibodyElement-derived objects may override to declare their
specific parameters.)""";
        } DoDeclareParameters;
        // Symbol: drake::multibody::MultibodyElement::DoSetDefaultParameters
        struct /* DoSetDefaultParameters */ {
          // Source: drake/multibody/tree/multibody_element.h
          const char* doc =
R"""(Implementation of the NVI SetDefaultParameters().
MultibodyElement-derived objects may override to set default values of
their specific parameters.)""";
        } DoSetDefaultParameters;
        // Symbol: drake::multibody::MultibodyElement::DoSetTopology
        struct /* DoSetTopology */ {
          // Source: drake/multibody/tree/multibody_element.h
          const char* doc =
R"""(Implementation of the NVI SetTopology(). For advanced use only for
developers implementing new MultibodyTree components.)""";
        } DoSetTopology;
        // Symbol: drake::multibody::MultibodyElement::GetParentPlant
        struct /* GetParentPlant */ {
          // Source: drake/multibody/tree/multibody_element.h
          const char* doc =
R"""(Returns the MultibodyPlant that owns this MultibodyElement.

Note:
    You can only invoke this method if you have a definition of
    MultibodyPlant available. That is, you must include
    ``drake/multibody/plant/multibody_plant.h`` in the translation
    unit that invokes this method; multibody_element.h cannot do that
    for you.

Raises:
    RuntimeError if there is no MultibodyPlant owner.)""";
        } GetParentPlant;
        // Symbol: drake::multibody::MultibodyElement::GetParentTreeSystem
        struct /* GetParentTreeSystem */ {
          // Source: drake/multibody/tree/multibody_element.h
          const char* doc =
R"""(Returns a constant reference to the parent MultibodyTreeSystem that
owns the parent MultibodyTree that owns this element.

Raises:
    RuntimeError if has_parent_tree() is false.)""";
        } GetParentTreeSystem;
        // Symbol: drake::multibody::MultibodyElement::MultibodyElement<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/multibody_element.h
          const char* doc_0args =
R"""(Default constructor made protected so that sub-classes can still
declare their default constructors if they need to.)""";
          // Source: drake/multibody/tree/multibody_element.h
          const char* doc_1args =
R"""(Constructor which allows specifying a model instance.)""";
          // Source: drake/multibody/tree/multibody_element.h
          const char* doc_2args =
R"""(Both the model instance and element index are specified.)""";
        } ctor;
        // Symbol: drake::multibody::MultibodyElement::SetDefaultParameters
        struct /* SetDefaultParameters */ {
          // Source: drake/multibody/tree/multibody_element.h
          const char* doc =
R"""(Sets default values of parameters belonging to each MultibodyElement
in ``parameters`` at a call to
MultibodyTreeSystem::SetDefaultParameters().

Parameter ``parameters``:
    A mutable collections of parameters in a context.

Precondition:
    parameters != nullptr)""";
        } SetDefaultParameters;
        // Symbol: drake::multibody::MultibodyElement::SetTopology
        struct /* SetTopology */ {
          // Source: drake/multibody/tree/multibody_element.h
          const char* doc =
R"""((Internal use only) Gives MultibodyElement-derived objects the
opportunity to set data members that depend on topology and coordinate
assignments having been finalized. This is invoked at the end of
MultibodyTree::Finalize(). NVI to pure virtual method DoSetTopology().)""";
        } SetTopology;
        // Symbol: drake::multibody::MultibodyElement::get_parent_tree
        struct /* get_parent_tree */ {
          // Source: drake/multibody/tree/multibody_element.h
          const char* doc =
R"""(Returns a constant reference to the parent MultibodyTree that owns
this element.

Precondition:
    has_parent_tree is true.)""";
        } get_parent_tree;
        // Symbol: drake::multibody::MultibodyElement::has_parent_tree
        struct /* has_parent_tree */ {
          // Source: drake/multibody/tree/multibody_element.h
          const char* doc =
R"""(Returns true if this multibody element has a parent tree, otherwise
false.)""";
        } has_parent_tree;
        // Symbol: drake::multibody::MultibodyElement::index_impl
        struct /* index_impl */ {
          // Source: drake/multibody/tree/multibody_element.h
          const char* doc = R"""(Returns this element's unique index.)""";
        } index_impl;
        // Symbol: drake::multibody::MultibodyElement::is_ephemeral
        struct /* is_ephemeral */ {
          // Source: drake/multibody/tree/multibody_element.h
          const char* doc =
R"""(Returns ``True`` if this MultibodyElement was added during Finalize()
rather than something a user added. (See class comments.))""";
        } is_ephemeral;
        // Symbol: drake::multibody::MultibodyElement::model_instance
        struct /* model_instance */ {
          // Source: drake/multibody/tree/multibody_element.h
          const char* doc =
R"""(Returns the ModelInstanceIndex of the model instance to which this
element belongs.)""";
        } model_instance;
        // Symbol: drake::multibody::MultibodyElement::ordinal_impl
        struct /* ordinal_impl */ {
          // Source: drake/multibody/tree/multibody_element.h
          const char* doc =
R"""(Returns this element's unique ordinal.

Note:
    The int64_t default is present for backwards compatibility but you
    should not use it. Instead, define a ThingOrdinal specialization
    of TypeSafeIndex for any element Thing that has a meaningful
    ordinal. Then use that type explicitly in Thing's public
    ``ordinal()`` method.)""";
        } ordinal_impl;
        // Symbol: drake::multibody::MultibodyElement::set_is_ephemeral
        struct /* set_is_ephemeral */ {
          // Source: drake/multibody/tree/multibody_element.h
          const char* doc =
R"""((Internal use only) Sets the ``is_ephemeral`` flag to the indicated
value. The default if this is never called is ``False``. Any element
that is added during Finalize() should set this flag to ``True``.)""";
        } set_is_ephemeral;
      } MultibodyElement;
      // Symbol: drake::multibody::MultibodyForces
      struct /* MultibodyForces */ {
        // Source: drake/multibody/tree/multibody_forces.h
        const char* doc =
R"""(A class to hold a set of forces applied to a MultibodyTree system.
Forces can include generalized forces as well as body spatial forces.
MultibodyPlant::CalcGeneralizedForces() can be used to compute the
*total* generalized force, combining generalized_forces() and
body_forces().)""";
        // Symbol: drake::multibody::MultibodyForces::AddInForces
        struct /* AddInForces */ {
          // Source: drake/multibody/tree/multibody_forces.h
          const char* doc =
R"""(Adds into ``this`` the force contribution stored in ``addend``.)""";
        } AddInForces;
        // Symbol: drake::multibody::MultibodyForces::CheckHasRightSizeForModel
        struct /* CheckHasRightSizeForModel */ {
          // Source: drake/multibody/tree/multibody_forces.h
          const char* doc_1args_plant =
R"""(Utility that checks that the forces stored by ``this`` object have the
proper sizes to represent the set of forces for the given ``plant``.

Returns:
    true if ``this`` forces object has the proper sizes for the given
    ``plant``.)""";
          // Source: drake/multibody/tree/multibody_forces.h
          const char* doc_1args_model = R"""((Advanced) Tree overload.)""";
        } CheckHasRightSizeForModel;
        // Symbol: drake::multibody::MultibodyForces::MultibodyForces<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/multibody_forces.h
          const char* doc_1args_plant =
R"""(Constructs a force object to store a set of forces to be applied to
the multibody model for ``plant``. Forces are initialized to zero,
meaning no forces are applied. ``plant`` must have been already
finalized with MultibodyPlant::Finalize() or this constructor will
abort.)""";
          // Source: drake/multibody/tree/multibody_forces.h
          const char* doc_1args_model = R"""((Advanced) Tree overload.)""";
          // Source: drake/multibody/tree/multibody_forces.h
          const char* doc_2args_nb_nv =
R"""(Number of bodies and number of generalized velocities overload. This
constructor is useful for constructing the MultibodyForces structure
before a MultibodyPlant has been constructed.)""";
        } ctor;
        // Symbol: drake::multibody::MultibodyForces::SetZero
        struct /* SetZero */ {
          // Source: drake/multibody/tree/multibody_forces.h
          const char* doc =
R"""(Sets ``this`` to store zero forces (no applied forces).)""";
        } SetZero;
        // Symbol: drake::multibody::MultibodyForces::body_forces
        struct /* body_forces */ {
          // Source: drake/multibody/tree/multibody_forces.h
          const char* doc =
R"""((Advanced) Returns a constant reference to the vector of spatial body
forces ``F_BBo_W`` on each body B in the model, at the body's frame
origin ``Bo``, expressed in the world frame W.

Note:
    Entries are ordered by MobodIndex.)""";
        } body_forces;
        // Symbol: drake::multibody::MultibodyForces::generalized_forces
        struct /* generalized_forces */ {
          // Source: drake/multibody/tree/multibody_forces.h
          const char* doc =
R"""((Advanced) Returns a constant reference to the vector of generalized
forces stored by ``this`` forces object.)""";
        } generalized_forces;
        // Symbol: drake::multibody::MultibodyForces::mutable_body_forces
        struct /* mutable_body_forces */ {
          // Source: drake/multibody/tree/multibody_forces.h
          const char* doc =
R"""((Advanced) Mutable version of body_forces().)""";
        } mutable_body_forces;
        // Symbol: drake::multibody::MultibodyForces::mutable_generalized_forces
        struct /* mutable_generalized_forces */ {
          // Source: drake/multibody/tree/multibody_forces.h
          const char* doc =
R"""((Advanced) Mutable version of generalized_forces().)""";
        } mutable_generalized_forces;
        // Symbol: drake::multibody::MultibodyForces::num_bodies
        struct /* num_bodies */ {
          // Source: drake/multibody/tree/multibody_forces.h
          const char* doc =
R"""(Returns the number of bodies for which ``this`` force object applies.
Determined at construction from the given model MultibodyTree object.)""";
        } num_bodies;
        // Symbol: drake::multibody::MultibodyForces::num_velocities
        struct /* num_velocities */ {
          // Source: drake/multibody/tree/multibody_forces.h
          const char* doc =
R"""(Returns the number of generalized velocities for the model to which
these forces apply. The number of generalized forces in a multibody
model always equals the number of generalized velocities. Determined
at construction from the given model MultibodyTree object.)""";
        } num_velocities;
      } MultibodyForces;
      // Symbol: drake::multibody::PdControllerGains
      struct /* PdControllerGains */ {
        // Source: drake/multibody/tree/joint_actuator.h
        const char* doc =
R"""(PD controller gains. This enables the modeling of a simple low level
PD controllers, see JointActuator::set_controller_gains().)""";
        // Symbol: drake::multibody::PdControllerGains::d
        struct /* d */ {
          // Source: drake/multibody/tree/joint_actuator.h
          const char* doc = R"""()""";
        } d;
        // Symbol: drake::multibody::PdControllerGains::p
        struct /* p */ {
          // Source: drake/multibody/tree/joint_actuator.h
          const char* doc = R"""()""";
        } p;
      } PdControllerGains;
      // Symbol: drake::multibody::PlanarJoint
      struct /* PlanarJoint */ {
        // Source: drake/multibody/tree/planar_joint.h
        const char* doc =
R"""(This joint models a planar joint allowing two bodies to translate and
rotate relative to one another in a plane with three degrees of
freedom. That is, given a frame F attached to the parent body P and a
frame M attached to the child body B (see the Joint class's
documentation), this joint allows frame M to translate within the x-y
plane of frame F and to rotate about the z-axis, with M's z-axis Mz
and F's z-axis Fz coincident at all times. The translations along the
x- and y-axes of F, the rotation about the z-axis and their rates
specify the state of the joint. Zero (x, y, θ) corresponds to frames F
and M being coincident and aligned. Translation (x, y) is defined to
be positive in the direction of the respective axes and the rotation θ
is defined to be positive according to the right-hand-rule with the
thumb aligned in the direction of frame F's z-axis.)""";
        // Symbol: drake::multibody::PlanarJoint::PlanarJoint<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/planar_joint.h
          const char* doc =
R"""(Constructor to create a planar joint between two bodies so that frame
F attached to the parent body P and frame M attached to the child body
B translate and rotate as described in the class's documentation. This
constructor signature creates a joint with no joint limits, i.e. the
joint position, velocity and acceleration limits are the pair ``(-∞,
∞)``. These can be set using the Joint methods set_position_limits(),
set_velocity_limits() and set_acceleration_limits(). The first three
arguments to this constructor are those of the Joint class
constructor. See the Joint class's documentation for details. The
additional parameters are:

Parameter ``damping``:
    Viscous damping coefficient, in N⋅s/m for translation and N⋅m⋅s
    for rotation, used to model losses within the joint. See
    documentation of default_damping() for details on modelling of the
    damping force and torque.

Raises:
    RuntimeError if any element of damping is negative.)""";
        } ctor;
        // Symbol: drake::multibody::PlanarJoint::default_damping
        struct /* default_damping */ {
          // Source: drake/multibody/tree/planar_joint.h
          const char* doc =
R"""(Returns ``this`` joint's default damping constant in N⋅s/m for the
translational degrees and N⋅m⋅s for the rotational degree. The damping
force (in N) is modeled as ``fᵢ = -dampingᵢ⋅vᵢ, i = 1, 2`` i.e.
opposing motion, with vᵢ the translation rates along the i-th axis for
``this`` joint (see get_translational_velocity()) and fᵢ the force on
child body B at Mo and expressed in F. That is, f_BMo_F = (f₁, f₂).
The damping torque (in N⋅m) is modeled as ``τ = -damping₃⋅ω`` i.e.
opposing motion, with ω the angular rate for ``this`` joint (see
get_angular_velocity()) and τ the torque on child body B expressed in
frame F as t_B_F = τ⋅Fz_F.)""";
        } default_damping;
        // Symbol: drake::multibody::PlanarJoint::get_angular_velocity
        struct /* get_angular_velocity */ {
          // Source: drake/multibody/tree/planar_joint.h
          const char* doc =
R"""(Gets the rate of change, in radians per second, of ``this`` joint's
angle θ from ``context``. See class documentation for the definition
of this angle.

Parameter ``context``:
    The context of the model this joint belongs to.

Returns ``theta_dot``:
    The rate of change of ``this`` joint's angle θ as stored in the
    ``context``.)""";
        } get_angular_velocity;
        // Symbol: drake::multibody::PlanarJoint::get_default_rotation
        struct /* get_default_rotation */ {
          // Source: drake/multibody/tree/planar_joint.h
          const char* doc =
R"""(Gets the default angle for ``this`` joint.

Returns ``theta``:
    The default angle of ``this`` joint.)""";
        } get_default_rotation;
        // Symbol: drake::multibody::PlanarJoint::get_default_translation
        struct /* get_default_translation */ {
          // Source: drake/multibody/tree/planar_joint.h
          const char* doc =
R"""(Gets the default position for ``this`` joint.

Returns ``p_FoMo_F``:
    The default position of ``this`` joint.)""";
        } get_default_translation;
        // Symbol: drake::multibody::PlanarJoint::get_rotation
        struct /* get_rotation */ {
          // Source: drake/multibody/tree/planar_joint.h
          const char* doc =
R"""(Gets the angle θ of ``this`` joint from ``context``.

Parameter ``context``:
    The context of the model this joint belongs to.

Returns ``theta``:
    The angle of ``this`` joint stored in the ``context``. See class
    documentation for details.)""";
        } get_rotation;
        // Symbol: drake::multibody::PlanarJoint::get_translation
        struct /* get_translation */ {
          // Source: drake/multibody/tree/planar_joint.h
          const char* doc =
R"""(Gets the position of ``this`` joint from ``context``.

Parameter ``context``:
    The context of the model this joint belongs to.

Returns ``p_FoMo_F``:
    The position of ``this`` joint stored in the ``context`` ordered
    as (x, y). See class documentation for details.)""";
        } get_translation;
        // Symbol: drake::multibody::PlanarJoint::get_translational_velocity
        struct /* get_translational_velocity */ {
          // Source: drake/multibody/tree/planar_joint.h
          const char* doc =
R"""(Gets the translational velocity v_FoMo_F, in meters per second, of
``this`` joint's Mo measured and expressed in frame F from
``context``.

Parameter ``context``:
    The context of the model this joint belongs to.

Returns ``v_FoMo_F``:
    The translational velocity of ``this`` joint as stored in the
    ``context``.)""";
        } get_translational_velocity;
        // Symbol: drake::multibody::PlanarJoint::set_angular_velocity
        struct /* set_angular_velocity */ {
          // Source: drake/multibody/tree/planar_joint.h
          const char* doc =
R"""(Sets the rate of change, in radians per second, of ``this`` joint's
angle θ (see class documentation) to ``theta_dot``. The new rate of
change gets stored in ``context``.

Parameter ``context``:
    The context of the model this joint belongs to.

Parameter ``theta_dot``:
    The desired rates of change of ``this`` joint's angle in radians
    per second.

Returns:
    a constant reference to ``this`` joint.)""";
        } set_angular_velocity;
        // Symbol: drake::multibody::PlanarJoint::set_default_pose
        struct /* set_default_pose */ {
          // Source: drake/multibody/tree/planar_joint.h
          const char* doc =
R"""(Sets the default position and angle of this joint.

Parameter ``p_FoMo_F``:
    The desired default position of the joint

Parameter ``theta``:
    The desired default angle of the joint)""";
        } set_default_pose;
        // Symbol: drake::multibody::PlanarJoint::set_default_rotation
        struct /* set_default_rotation */ {
          // Source: drake/multibody/tree/planar_joint.h
          const char* doc =
R"""(Sets the default angle of this joint.

Parameter ``theta``:
    The desired default angle of the joint)""";
        } set_default_rotation;
        // Symbol: drake::multibody::PlanarJoint::set_default_translation
        struct /* set_default_translation */ {
          // Source: drake/multibody/tree/planar_joint.h
          const char* doc =
R"""(Sets the default position of this joint.

Parameter ``p_FoMo_F``:
    The desired default position of the joint)""";
        } set_default_translation;
        // Symbol: drake::multibody::PlanarJoint::set_pose
        struct /* set_pose */ {
          // Source: drake/multibody/tree/planar_joint.h
          const char* doc =
R"""(Sets the ``context`` so that the position of ``this`` joint equals
``p_FoMo_F`` and its angle equals ``theta``.

Parameter ``context``:
    The context of the model this joint belongs to.

Parameter ``p_FoMo_F``:
    The desired position in meters to be stored in ``context`` ordered
    as (x, y). See class documentation for details.

Parameter ``theta``:
    The desired angle in radians to be stored in ``context``. See
    class documentation for details.

Returns:
    a constant reference to ``this`` joint.)""";
        } set_pose;
        // Symbol: drake::multibody::PlanarJoint::set_random_pose_distribution
        struct /* set_random_pose_distribution */ {
          // Source: drake/multibody/tree/planar_joint.h
          const char* doc =
R"""(Sets the random distribution that the position and angle of this joint
will be randomly sampled from. See class documentation for details on
the definition of the position and angle.)""";
        } set_random_pose_distribution;
        // Symbol: drake::multibody::PlanarJoint::set_rotation
        struct /* set_rotation */ {
          // Source: drake/multibody/tree/planar_joint.h
          const char* doc =
R"""(Sets the ``context`` so that the angle θ of ``this`` joint equals
``theta``.

Parameter ``context``:
    The context of the model this joint belongs to.

Parameter ``theta``:
    The desired angle in radians to be stored in ``context``. See
    class documentation for details.

Returns:
    a constant reference to ``this`` joint.)""";
        } set_rotation;
        // Symbol: drake::multibody::PlanarJoint::set_translation
        struct /* set_translation */ {
          // Source: drake/multibody/tree/planar_joint.h
          const char* doc =
R"""(Sets the ``context`` so that the position of ``this`` joint equals
``p_FoMo_F``.

Parameter ``context``:
    The context of the model this joint belongs to.

Parameter ``p_FoMo_F``:
    The desired position in meters to be stored in ``context`` ordered
    as (x, y). See class documentation for details.

Returns:
    a constant reference to ``this`` joint.)""";
        } set_translation;
        // Symbol: drake::multibody::PlanarJoint::set_translational_velocity
        struct /* set_translational_velocity */ {
          // Source: drake/multibody/tree/planar_joint.h
          const char* doc =
R"""(Sets the translational velocity, in meters per second, of this
``this`` joint's Mo measured and expressed in frame F to ``v_FoMo_F``.
The new translational velocity gets stored in ``context``.

Parameter ``context``:
    The context of the model this joint belongs to.

Parameter ``v_FoMo_F``:
    The desired translational velocity of ``this`` joint in meters per
    second.

Returns:
    a constant reference to ``this`` joint.)""";
        } set_translational_velocity;
        // Symbol: drake::multibody::PlanarJoint::type_name
        struct /* type_name */ {
          // Source: drake/multibody/tree/planar_joint.h
          const char* doc = R"""()""";
        } type_name;
      } PlanarJoint;
      // Symbol: drake::multibody::PrismaticJoint
      struct /* PrismaticJoint */ {
        // Source: drake/multibody/tree/prismatic_joint.h
        const char* doc =
R"""(This Joint allows two bodies to translate relative to one another
along a common axis. That is, given a frame Jp attached to the parent
body P and a frame Jc attached to the child body C (see the Joint
class's documentation), this Joint allows frames Jp and Jc to
translate with respect to each other along an axis â. The translation
distance is defined positive when child body C translates along the
direction of â. Axis vector â is constant and has the same
components in both frames Jp and Jc, that is, ``â_Jp = â_Jc``.)""";
        // Symbol: drake::multibody::PrismaticJoint::AddInForce
        struct /* AddInForce */ {
          // Source: drake/multibody/tree/prismatic_joint.h
          const char* doc =
R"""(Adds into ``multibody_forces`` a given ``force``, in Newtons, for
``this`` joint that is to be applied along the joint's axis. The force
is defined to be positive in the direction along this joint's axis.
That is, a positive force causes a positive translational acceleration
along the joint's axis.)""";
        } AddInForce;
        // Symbol: drake::multibody::PrismaticJoint::DoAddInDamping
        struct /* DoAddInDamping */ {
          // Source: drake/multibody/tree/prismatic_joint.h
          const char* doc =
R"""(Joint<T> override called through public NVI, Joint::AddInDamping().
Therefore arguments were already checked to be valid. This method adds
into ``forces`` a dissipative force according to the viscous law ``f =
-d⋅v``, with d the damping coefficient (see default_damping()).)""";
        } DoAddInDamping;
        // Symbol: drake::multibody::PrismaticJoint::DoAddInOneForce
        struct /* DoAddInOneForce */ {
          // Source: drake/multibody/tree/prismatic_joint.h
          const char* doc =
R"""(Joint<T> virtual override called through public NVI,
Joint::AddInForce(). Therefore arguments were already checked to be
valid. For a PrismaticJoint, we must always have ``joint_dof = 0``
since there is only a single degree of freedom (num_velocities() ==
1). ``joint_tau`` is the linear force applied along the joint's axis,
on the body declared as child (according to the prismatic joint's
constructor) at the origin of the child frame (which is coincident
with the origin of the parent frame at all times).)""";
        } DoAddInOneForce;
        // Symbol: drake::multibody::PrismaticJoint::GetDamping
        struct /* GetDamping */ {
          // Source: drake/multibody/tree/prismatic_joint.h
          const char* doc =
R"""(Returns the Context dependent damping coefficient stored as a
parameter in ``context``. Refer to default_damping() for details.

Parameter ``context``:
    The context storing the state and parameters for the model to
    which ``this`` joint belongs.)""";
        } GetDamping;
        // Symbol: drake::multibody::PrismaticJoint::PrismaticJoint<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/prismatic_joint.h
          const char* doc =
R"""(Constructor to create a prismatic joint between two bodies so that
frame Jp attached to the parent body P and frame Jc attached to the
child body C, translate relatively to one another along a common axis.
See this class's documentation for further details on the definition
of these frames and translation distance. The first three arguments to
this constructor are those of the Joint class constructor. See the
Joint class's documentation for details. The additional parameter
``axis`` is:

Parameter ``axis``:
    A vector in ℝ³ specifying the translation axis for this joint.
    Given that frame Jc only translates with respect to Jp and there
    is no relative rotation, the components of ``axis`` in either
    frame Jp or Jc are exactly the same, that is, ``axis_Jp =
    axis_Jc``. This vector can have any length, only the direction is
    used.

Parameter ``pos_lower_limit``:
    Lower position limit, in meters, for the translation coordinate
    (see get_translation()).

Parameter ``pos_upper_limit``:
    Upper position limit, in meters, for the translation coordinate
    (see get_translation()).

Parameter ``damping``:
    Viscous damping coefficient, in N⋅s/m, used to model losses within
    the joint. The damping force (in N) is modeled as ``f =
    -damping⋅v``, i.e. opposing motion, with v the translational speed
    for ``this`` joint (see get_translation_rate()).

Raises:
    RuntimeError if the L2 norm of ``axis`` is less than the square
    root of machine epsilon.

Raises:
    RuntimeError if damping is negative.

Raises:
    RuntimeError if pos_lower_limit > pos_upper_limit.)""";
        } ctor;
        // Symbol: drake::multibody::PrismaticJoint::SetDamping
        struct /* SetDamping */ {
          // Source: drake/multibody/tree/prismatic_joint.h
          const char* doc =
R"""(Sets the value of the viscous damping coefficient for this joint,
stored as a parameter in ``context``. Refer to default_damping() for
details.

Parameter ``context``:
    The context storing the state and parameters for the model to
    which ``this`` joint belongs.

Parameter ``damping``:
    The damping value.

Raises:
    RuntimeError if ``damping`` is negative.)""";
        } SetDamping;
        // Symbol: drake::multibody::PrismaticJoint::acceleration_lower_limit
        struct /* acceleration_lower_limit */ {
          // Source: drake/multibody/tree/prismatic_joint.h
          const char* doc =
R"""(Returns the acceleration lower limit for ``this`` joint in meters per
second squared.)""";
        } acceleration_lower_limit;
        // Symbol: drake::multibody::PrismaticJoint::acceleration_upper_limit
        struct /* acceleration_upper_limit */ {
          // Source: drake/multibody/tree/prismatic_joint.h
          const char* doc =
R"""(Returns the acceleration upper limit for ``this`` joint in meters per
second squared.)""";
        } acceleration_upper_limit;
        // Symbol: drake::multibody::PrismaticJoint::default_damping
        struct /* default_damping */ {
          // Source: drake/multibody/tree/prismatic_joint.h
          const char* doc =
R"""(Returns ``this`` joint's default damping constant in N⋅s/m.)""";
        } default_damping;
        // Symbol: drake::multibody::PrismaticJoint::get_default_translation
        struct /* get_default_translation */ {
          // Source: drake/multibody/tree/prismatic_joint.h
          const char* doc =
R"""(Gets the default translation. Wrapper for the more general
``Joint::default_positions()``.

Returns:
    The default translation of ``this`` stored in
    ``default_positions_``.)""";
        } get_default_translation;
        // Symbol: drake::multibody::PrismaticJoint::get_translation
        struct /* get_translation */ {
          // Source: drake/multibody/tree/prismatic_joint.h
          const char* doc =
R"""(Gets the translation distance of ``this`` mobilizer from ``context``.

Parameter ``context``:
    The context of the MultibodyTree this joint belongs to.

Returns:
    The translation coordinate of ``this`` joint read from
    ``context``.)""";
        } get_translation;
        // Symbol: drake::multibody::PrismaticJoint::get_translation_rate
        struct /* get_translation_rate */ {
          // Source: drake/multibody/tree/prismatic_joint.h
          const char* doc =
R"""(Gets the rate of change, in meters per second, of ``this`` joint's
translation distance (see get_translation()) from ``context``.

Parameter ``context``:
    The context of the MultibodyTree this joint belongs to.

Returns:
    The rate of change of ``this`` joint's translation read from
    ``context``.)""";
        } get_translation_rate;
        // Symbol: drake::multibody::PrismaticJoint::position_lower_limit
        struct /* position_lower_limit */ {
          // Source: drake/multibody/tree/prismatic_joint.h
          const char* doc =
R"""(Returns the position lower limit for ``this`` joint in meters.)""";
        } position_lower_limit;
        // Symbol: drake::multibody::PrismaticJoint::position_upper_limit
        struct /* position_upper_limit */ {
          // Source: drake/multibody/tree/prismatic_joint.h
          const char* doc =
R"""(Returns the position upper limit for ``this`` joint in meters.)""";
        } position_upper_limit;
        // Symbol: drake::multibody::PrismaticJoint::set_default_damping
        struct /* set_default_damping */ {
          // Source: drake/multibody/tree/prismatic_joint.h
          const char* doc =
R"""(Sets the default value of viscous damping for this joint, in N⋅s/m.

Raises:
    RuntimeError if damping is negative.

Raises:
    RuntimeError if this element is not associated with a
    MultibodyPlant.

Precondition:
    the MultibodyPlant must not be finalized.)""";
        } set_default_damping;
        // Symbol: drake::multibody::PrismaticJoint::set_default_translation
        struct /* set_default_translation */ {
          // Source: drake/multibody/tree/prismatic_joint.h
          const char* doc =
R"""(Sets the ``default_positions`` of this joint (in this case a single
translation)

Parameter ``translation``:
    The desired default translation of the joint)""";
        } set_default_translation;
        // Symbol: drake::multibody::PrismaticJoint::set_random_translation_distribution
        struct /* set_random_translation_distribution */ {
          // Source: drake/multibody/tree/prismatic_joint.h
          const char* doc = R"""()""";
        } set_random_translation_distribution;
        // Symbol: drake::multibody::PrismaticJoint::set_translation
        struct /* set_translation */ {
          // Source: drake/multibody/tree/prismatic_joint.h
          const char* doc =
R"""(Sets ``context`` so that the generalized coordinate corresponding to
the translation distance of ``this`` joint equals ``translation``.

Parameter ``context``:
    The context of the MultibodyTree this joint belongs to.

Parameter ``translation``:
    The desired translation in meters to be stored in ``context``.

Returns:
    a constant reference to ``this`` joint.)""";
        } set_translation;
        // Symbol: drake::multibody::PrismaticJoint::set_translation_rate
        struct /* set_translation_rate */ {
          // Source: drake/multibody/tree/prismatic_joint.h
          const char* doc =
R"""(Sets the rate of change, in meters per second, of ``this`` joint's
translation distance to ``translation_dot``. The new rate of change
``translation_dot`` gets stored in ``context``.

Parameter ``context``:
    The context of the MultibodyTree this joint belongs to.

Parameter ``translation_dot``:
    The desired rate of change of ``this`` joints's translation in
    meters per second.

Returns:
    a constant reference to ``this`` joint.)""";
        } set_translation_rate;
        // Symbol: drake::multibody::PrismaticJoint::translation_axis
        struct /* translation_axis */ {
          // Source: drake/multibody/tree/prismatic_joint.h
          const char* doc =
R"""(Returns the axis of translation for ``this`` joint as a unit vector.
Since the components of this axis in either frame Jp or Jc are the
same (see this class's documentation for frame definitions) then,
``axis = axis_Jp = axis_Jc``.)""";
        } translation_axis;
        // Symbol: drake::multibody::PrismaticJoint::type_name
        struct /* type_name */ {
          // Source: drake/multibody/tree/prismatic_joint.h
          const char* doc = R"""()""";
        } type_name;
        // Symbol: drake::multibody::PrismaticJoint::velocity_lower_limit
        struct /* velocity_lower_limit */ {
          // Source: drake/multibody/tree/prismatic_joint.h
          const char* doc =
R"""(Returns the velocity lower limit for ``this`` joint in meters per
second.)""";
        } velocity_lower_limit;
        // Symbol: drake::multibody::PrismaticJoint::velocity_upper_limit
        struct /* velocity_upper_limit */ {
          // Source: drake/multibody/tree/prismatic_joint.h
          const char* doc =
R"""(Returns the velocity upper limit for ``this`` joint in meters per
second.)""";
        } velocity_upper_limit;
      } PrismaticJoint;
      // Symbol: drake::multibody::PrismaticSpring
      struct /* PrismaticSpring */ {
        // Source: drake/multibody/tree/prismatic_spring.h
        const char* doc =
R"""(This ForceElement models a linear spring attached to a PrismaticJoint
and applies a force to that joint according to


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    f = -k⋅(x - x₀)

.. raw:: html

    </details>

where x₀ is the nominal (zero spring force) position in meters, x is
the joint position in meters, f is the spring force in Newtons and k
is the spring constant in N/m. Note that joint damping exists within
the PrismaticJoint itself, and so is not included here.

Note:
    This is different from the LinearSpringDamper: this
    PrismaticSpring is associated with a joint, while the
    LinearSpringDamper connects two bodies.)""";
        // Symbol: drake::multibody::PrismaticSpring::CalcConservativePower
        struct /* CalcConservativePower */ {
          // Source: drake/multibody/tree/prismatic_spring.h
          const char* doc = R"""()""";
        } CalcConservativePower;
        // Symbol: drake::multibody::PrismaticSpring::CalcNonConservativePower
        struct /* CalcNonConservativePower */ {
          // Source: drake/multibody/tree/prismatic_spring.h
          const char* doc = R"""()""";
        } CalcNonConservativePower;
        // Symbol: drake::multibody::PrismaticSpring::CalcPotentialEnergy
        struct /* CalcPotentialEnergy */ {
          // Source: drake/multibody/tree/prismatic_spring.h
          const char* doc = R"""()""";
        } CalcPotentialEnergy;
        // Symbol: drake::multibody::PrismaticSpring::PrismaticSpring<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/prismatic_spring.h
          const char* doc =
R"""(Constructor for a linear spring attached to the given prismatic joint.

Parameter ``nominal_position``:
    The nominal position of the spring x₀, in meters, at which the
    spring applies no force. This is measured the same way as the
    generalized position of the prismatic joint.

Parameter ``stiffness``:
    The stiffness k of the spring in N/m.

Raises:
    RuntimeError if ``stiffness`` is (strictly) negative.)""";
        } ctor;
        // Symbol: drake::multibody::PrismaticSpring::joint
        struct /* joint */ {
          // Source: drake/multibody/tree/prismatic_spring.h
          const char* doc =
R"""(Returns the joint associated with this spring.

Raises:
    RuntimeError if this element is not associated with a
    MultibodyPlant.)""";
        } joint;
        // Symbol: drake::multibody::PrismaticSpring::nominal_position
        struct /* nominal_position */ {
          // Source: drake/multibody/tree/prismatic_spring.h
          const char* doc = R"""()""";
        } nominal_position;
        // Symbol: drake::multibody::PrismaticSpring::stiffness
        struct /* stiffness */ {
          // Source: drake/multibody/tree/prismatic_spring.h
          const char* doc = R"""()""";
        } stiffness;
      } PrismaticSpring;
      // Symbol: drake::multibody::QuaternionFloatingJoint
      struct /* QuaternionFloatingJoint */ {
        // Source: drake/multibody/tree/quaternion_floating_joint.h
        const char* doc =
R"""(This Joint allows two bodies to move freely relatively to one another.
That is, given a frame F attached to the parent body P and a frame M
attached to the child body B (see the Joint class's documentation),
this Joint allows frame M to translate and rotate freely with respect
to F, introducing six degrees of freedom. This Joint introduces four
generalized positions to describe the orientation ``R_FM`` of frame M
in F with a quaternion ``q_FM``, and three generalized positions to
describe the translation of frame M's origin in F with a position
vector ``p_FM``. The seven entries of the configuration vector q are
ordered ``(q_FM, p_FM)`` with the quaternion, ordered wxyz (scalar
then vector), preceding the translation vector. As generalized
velocities, this Joint introduces the angular velocity ``w_FM`` of
frame M in F and the linear velocity ``v_FM`` of frame M's origin in
frame F, ordered ``(w_FM, v_FM)``.)""";
        // Symbol: drake::multibody::QuaternionFloatingJoint::DoAddInDamping
        struct /* DoAddInDamping */ {
          // Source: drake/multibody/tree/quaternion_floating_joint.h
          const char* doc =
R"""(Joint<T> override called through public NVI, Joint::AddInDamping().
Therefore arguments were already checked to be valid. This method adds
into the translational component of ``forces`` for ``this`` joint a
dissipative force according to the viscous law ``f = -d⋅v``, with d
the damping coefficient (see default_translational_damping()). This
method also adds into the angular component of ``forces`` for ``this``
joint a dissipative torque according to the viscous law ``τ = -d⋅ω``,
with d the damping coefficient (see default_angular_damping()).)""";
        } DoAddInDamping;
        // Symbol: drake::multibody::QuaternionFloatingJoint::DoAddInOneForce
        struct /* DoAddInOneForce */ {
          // Source: drake/multibody/tree/quaternion_floating_joint.h
          const char* doc =
R"""(Joint<T> override called through public NVI, Joint::AddInForce().
Adding forces per-dof makes no physical sense. Therefore, this method
throws an exception if invoked.)""";
        } DoAddInOneForce;
        // Symbol: drake::multibody::QuaternionFloatingJoint::GetPose
        struct /* GetPose */ {
          // Source: drake/multibody/tree/quaternion_floating_joint.h
          const char* doc =
R"""(Returns the pose ``X_FM`` of the outboard frame M as measured and
expressed in the inboard frame F. Refer to the documentation for this
class for details.

Parameter ``context``:
    A Context for the MultibodyPlant this joint belongs to.

Returns ``X_FM``:
    The pose of frame M in frame F.)""";
        } GetPose;
        // Symbol: drake::multibody::QuaternionFloatingJoint::QuaternionFloatingJoint<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/quaternion_floating_joint.h
          const char* doc =
R"""(Constructor for a QuaternionFloatingJoint granting six degrees of
freedom to an outboard frame M attached to the child body B with
respect to an inboard frame F attached to the parent body P. The
orientation of frame M in F is represented by a quaternion ``q_FM``
while the position of F in M is given by a position vector ``p_FM``
expressed in frame F. See this class's documentation for further
details on the definition of these frames, get_quaternion() and
get_translation() for an explanation of the configuration of this
joint, and get_angular_velocity() and get_translational_velocity() for
an explanation of the generalized velocities.

This constructor signature creates a joint with no joint limits, i.e.
the joint position, velocity and acceleration limits are the pair
``(-∞, ∞)``. These can be set using the Joint methods
set_position_limits(), set_velocity_limits() and
set_acceleration_limits().

The first three arguments to this constructor are those of the Joint
class constructor. See the Joint class's documentation for details.
The additional parameters are:

Parameter ``angular_damping``:
    Viscous damping coefficient in N⋅m⋅s for the angular component of
    this joint's velocity, used to model losses within the joint. See
    documentation of default_angular_damping() for details on
    modelling of the damping force.

Parameter ``translational_damping``:
    Viscous damping coefficient in N⋅s/m for the translational
    component of this joint's velocity, used to model losses within
    the joint. See documentation of default_translational_damping()
    for details on modelling of the damping force.

Raises:
    RuntimeError if angular_damping is negative.

Raises:
    RuntimeError if translational_damping is negative.)""";
        } ctor;
        // Symbol: drake::multibody::QuaternionFloatingJoint::SetOrientation
        struct /* SetOrientation */ {
          // Source: drake/multibody/tree/quaternion_floating_joint.h
          const char* doc =
R"""(Sets the quaternion in ``context`` so this joint's orientation is
consistent with the given ``R_FM`` rotation matrix.

Parameter ``context``:
    A Context for the MultibodyPlant this joint belongs to.

Parameter ``R_FM``:
    The rotation matrix relating the orientation of frame F and frame
    M.

Returns:
    a constant reference to ``this`` joint.)""";
        } SetOrientation;
        // Symbol: drake::multibody::QuaternionFloatingJoint::SetPose
        struct /* SetPose */ {
          // Source: drake/multibody/tree/quaternion_floating_joint.h
          const char* doc =
R"""(Sets ``context`` to store ``X_FM`` the pose of frame M measured and
expressed in frame F.

Parameter ``context``:
    A Context for the MultibodyPlant this joint belongs to.

Parameter ``X_FM``:
    The desired pose of frame M in frame F to be stored in
    ``context``.

Returns:
    a constant reference to ``this`` joint.)""";
        } SetPose;
        // Symbol: drake::multibody::QuaternionFloatingJoint::SetQuaternion
        struct /* SetQuaternion */ {
          // Source: drake/multibody/tree/quaternion_floating_joint.h
          const char* doc =
R"""(Sets ``context`` so that the orientation of frame M in F is given by
the input quaternion ``q_FM``.

Parameter ``context``:
    A Context for the MultibodyPlant this joint belongs to.

Parameter ``q_FM``:
    Quaternion relating frames F and M to be stored in ``context``.

Returns:
    a constant reference to ``this`` joint.)""";
        } SetQuaternion;
        // Symbol: drake::multibody::QuaternionFloatingJoint::SetTranslation
        struct /* SetTranslation */ {
          // Source: drake/multibody/tree/quaternion_floating_joint.h
          const char* doc =
R"""(For this joint, stores the position vector ``p_FM`` in ``context``.

Parameter ``context``:
    A Context for the MultibodyPlant this joint belongs to.

Parameter ``p_FM``:
    position vector p_FoMo_F from Fo (inboard frame F's origin) to Mo
    (outboard frame M's origin), expressed in frame F.

Returns:
    a constant reference to ``this`` joint.)""";
        } SetTranslation;
        // Symbol: drake::multibody::QuaternionFloatingJoint::default_angular_damping
        struct /* default_angular_damping */ {
          // Source: drake/multibody/tree/quaternion_floating_joint.h
          const char* doc =
R"""(Returns ``this`` joint's default angular damping constant in N⋅m⋅s.
The damping torque (in N⋅m) is modeled as ``τ = -damping⋅ω``, i.e.
opposing motion, with ω the angular velocity of frame M in F (see
get_angular_velocity()) and τ the torque on child body B (to which M
is rigidly attached).)""";
        } default_angular_damping;
        // Symbol: drake::multibody::QuaternionFloatingJoint::default_translational_damping
        struct /* default_translational_damping */ {
          // Source: drake/multibody/tree/quaternion_floating_joint.h
          const char* doc =
R"""(Returns ``this`` joint's default translational damping constant in
N⋅s/m. The damping force (in N) is modeled as ``f = -damping⋅v`` i.e.
opposing motion, with v the translational velocity of frame M in F
(see get_translational_velocity()) and f the force on child body B at
Mo.)""";
        } default_translational_damping;
        // Symbol: drake::multibody::QuaternionFloatingJoint::get_angular_velocity
        struct /* get_angular_velocity */ {
          // Source: drake/multibody/tree/quaternion_floating_joint.h
          const char* doc =
R"""(Retrieves from ``context`` the angular velocity ``w_FM`` of the child
frame M in the parent frame F, expressed in F.

Parameter ``context``:
    A Context for the MultibodyPlant this joint belongs to.

Returns ``w_FM``:
    A vector in ℝ³ with the angular velocity of the child frame M in
    the parent frame F, expressed in F. Refer to this class's
    documentation for further details and definitions of these frames.)""";
        } get_angular_velocity;
        // Symbol: drake::multibody::QuaternionFloatingJoint::get_default_quaternion
        struct /* get_default_quaternion */ {
          // Source: drake/multibody/tree/quaternion_floating_joint.h
          const char* doc =
R"""(Gets the default quaternion ``q_FM`` for ``this`` joint.

Returns:
    The default quaternion ``q_FM`` of ``this``.)""";
        } get_default_quaternion;
        // Symbol: drake::multibody::QuaternionFloatingJoint::get_default_translation
        struct /* get_default_translation */ {
          // Source: drake/multibody/tree/quaternion_floating_joint.h
          const char* doc =
R"""(Returns this joint's default translation as the position vector
p_FoMo_F from Fo (inboard frame F's origin) to Mo (outboard frame M's
origin), expressed in inboard frame F.

Returns ``This``:
    joint's default translation as the position vector p_FM.)""";
        } get_default_translation;
        // Symbol: drake::multibody::QuaternionFloatingJoint::get_quaternion
        struct /* get_quaternion */ {
          // Source: drake/multibody/tree/quaternion_floating_joint.h
          const char* doc =
R"""(Gets the quaternion ``q_FM`` that represents the orientation of
outboard frame M in the inboard frame F. Refer to the documentation
for this class for details.

Parameter ``context``:
    A Context for the MultibodyPlant this joint belongs to.

Returns ``q_FM``:
    The quaternion representing the orientation of frame M in F.)""";
        } get_quaternion;
        // Symbol: drake::multibody::QuaternionFloatingJoint::get_translation
        struct /* get_translation */ {
          // Source: drake/multibody/tree/quaternion_floating_joint.h
          const char* doc =
R"""(Returns the position vector p_FoMo_F from Fo (inboard frame F's
origin) to Mo (outboard frame M's origin), expressed in inboard frame
F.

Parameter ``context``:
    contains the state of the multibody system.

Note:
    Class documentation describes inboard frame F and outboard frame
    M.

Returns ``p_FM``:
    The position vector from Fo (frame F's origin) to Mo (frame M's
    origin), expressed in frame F.)""";
        } get_translation;
        // Symbol: drake::multibody::QuaternionFloatingJoint::get_translational_velocity
        struct /* get_translational_velocity */ {
          // Source: drake/multibody/tree/quaternion_floating_joint.h
          const char* doc =
R"""(Retrieves from ``context`` the translational velocity ``v_FM`` of the
child frame M's origin as measured and expressed in the parent frame
F.

Parameter ``context``:
    A Context for the MultibodyPlant this joint belongs to.

Returns ``v_FM``:
    A vector in ℝ³ with the translational velocity of the origin of
    child frame M in the parent frame F, expressed in F. Refer to this
    class's documentation for further details and definitions of these
    frames.)""";
        } get_translational_velocity;
        // Symbol: drake::multibody::QuaternionFloatingJoint::set_angular_velocity
        struct /* set_angular_velocity */ {
          // Source: drake/multibody/tree/quaternion_floating_joint.h
          const char* doc =
R"""(Sets in ``context`` the state for ``this`` joint so that the angular
velocity of the child frame M in the parent frame F is ``w_FM``.

Parameter ``context``:
    A Context for the MultibodyPlant this joint belongs to.

Parameter ``w_FM``:
    A vector in ℝ³ with the angular velocity of the child frame M in
    the parent frame F, expressed in F. Refer to this class's
    documentation for further details and definitions of these frames.

Returns:
    a constant reference to ``this`` joint.)""";
        } set_angular_velocity;
        // Symbol: drake::multibody::QuaternionFloatingJoint::set_default_quaternion
        struct /* set_default_quaternion */ {
          // Source: drake/multibody/tree/quaternion_floating_joint.h
          const char* doc =
R"""(Sets the default quaternion ``q_FM`` of this joint.

Parameter ``q_FM``:
    The desired default quaternion of the joint.)""";
        } set_default_quaternion;
        // Symbol: drake::multibody::QuaternionFloatingJoint::set_default_translation
        struct /* set_default_translation */ {
          // Source: drake/multibody/tree/quaternion_floating_joint.h
          const char* doc =
R"""(Sets this joint's default position vector ``p_FM``.

Parameter ``p_FM``:
    position vector p_FoMo_F from Fo (inboard frame F's origin) to Mo
    (outboard frame M's origin), expressed in frame F.)""";
        } set_default_translation;
        // Symbol: drake::multibody::QuaternionFloatingJoint::set_random_quaternion_distribution
        struct /* set_random_quaternion_distribution */ {
          // Source: drake/multibody/tree/quaternion_floating_joint.h
          const char* doc =
R"""((Advanced) Sets the random distribution that the orientation of this
joint will be randomly sampled from. If a translation (position)
distribution has already been set with stochastic variables, it will
remain so. Otherwise translation will be set to this joint's zero
configuration. See get_quaternion() for details on the orientation
representation.

Note:
    Use caution when setting a quaternion distribution. A naive
    uniform sampling of each component will not lead to a uniform
    sampling of the unit sphere. See
    ``set_random_quaternion_distribution_to_uniform()`` for the most
    common case of uniformly sampling rotations.)""";
        } set_random_quaternion_distribution;
        // Symbol: drake::multibody::QuaternionFloatingJoint::set_random_quaternion_distribution_to_uniform
        struct /* set_random_quaternion_distribution_to_uniform */ {
          // Source: drake/multibody/tree/quaternion_floating_joint.h
          const char* doc =
R"""(Sets the random distribution such that the orientation of this joint
will be randomly sampled using uniformly sampled rotations.)""";
        } set_random_quaternion_distribution_to_uniform;
        // Symbol: drake::multibody::QuaternionFloatingJoint::set_random_translation_distribution
        struct /* set_random_translation_distribution */ {
          // Source: drake/multibody/tree/quaternion_floating_joint.h
          const char* doc =
R"""(For this joint, sets the random distribution that the translation of
this joint will be randomly sampled from. If a quaternion distribution
has already been set with stochastic variables, it will remain so.
Otherwise the quaternion will be set to this joint's zero orientation.
See get_translation() for details on the translation representation.)""";
        } set_random_translation_distribution;
        // Symbol: drake::multibody::QuaternionFloatingJoint::set_translational_velocity
        struct /* set_translational_velocity */ {
          // Source: drake/multibody/tree/quaternion_floating_joint.h
          const char* doc =
R"""(Sets in ``context`` the state for ``this`` joint so that the
translational velocity of the child frame M's origin in the parent
frame F is ``v_FM``.

Parameter ``context``:
    A Context for the MultibodyPlant this joint belongs to.

Parameter ``v_FM``:
    A vector in ℝ³ with the translational velocity of the child frame
    M's origin in the parent frame F, expressed in F. Refer to this
    class's documentation for further details and definitions of these
    frames.

Returns:
    a constant reference to ``this`` joint.)""";
        } set_translational_velocity;
        // Symbol: drake::multibody::QuaternionFloatingJoint::type_name
        struct /* type_name */ {
          // Source: drake/multibody/tree/quaternion_floating_joint.h
          const char* doc =
R"""(Returns the name of this joint type: "quaternion_floating")""";
        } type_name;
      } QuaternionFloatingJoint;
      // Symbol: drake::multibody::RevoluteJoint
      struct /* RevoluteJoint */ {
        // Source: drake/multibody/tree/revolute_joint.h
        const char* doc =
R"""(This Joint allows two bodies to rotate relatively to one another
around a common axis. That is, given a frame Jp attached to the parent
body P and a frame Jc attached to the child body C (see the Joint
class's documentation), this Joint allows frames Jp and Jc to rotate
with respect to each other about an axis â. The rotation angle's sign
is defined such that child body C rotates about axis â according to
the right hand rule, with thumb aligned in the axis direction. Axis
vector â is constant and has the same components in both frames Jp
and Jc, that is, ``â_Jp = â_Jc``.)""";
        // Symbol: drake::multibody::RevoluteJoint::AddInTorque
        struct /* AddInTorque */ {
          // Source: drake/multibody/tree/revolute_joint.h
          const char* doc =
R"""(Adds into ``forces`` a given ``torque`` for ``this`` joint that is to
be applied about the joint's axis. The torque is defined to be
positive according to the right-hand-rule with the thumb aligned in
the direction of ``this`` joint's axis. That is, a positive torque
causes a positive rotational acceleration according to the
right-hand-rule around the joint's axis.

Note:
    A torque is the moment of a set of forces whose resultant is zero.)""";
        } AddInTorque;
        // Symbol: drake::multibody::RevoluteJoint::DoAddInDamping
        struct /* DoAddInDamping */ {
          // Source: drake/multibody/tree/revolute_joint.h
          const char* doc =
R"""(Joint<T> override called through public NVI, Joint::AddInDamping().
Therefore arguments were already checked to be valid. This method adds
into ``forces`` a dissipative torque according to the viscous law ``τ
= -d⋅ω``, with d the damping coefficient (see default_damping()).)""";
        } DoAddInDamping;
        // Symbol: drake::multibody::RevoluteJoint::DoAddInOneForce
        struct /* DoAddInOneForce */ {
          // Source: drake/multibody/tree/revolute_joint.h
          const char* doc =
R"""(Joint<T> override called through public NVI, Joint::AddInForce().
Therefore arguments were already checked to be valid. For a
RevoluteJoint, we must always have ``joint_dof = 0`` since there is
only a single degree of freedom (num_velocities() == 1). ``joint_tau``
is the torque applied about the joint's axis, on the body declared as
child (according to the revolute joint's constructor) at the origin of
the child frame (which is coincident with the origin of the parent
frame at all times). The torque is defined to be positive according to
the right-hand-rule with the thumb aligned in the direction of
``this`` joint's axis. That is, a positive torque causes a positive
rotational acceleration (of the child body frame) according to the
right-hand-rule around the joint's axis.)""";
        } DoAddInOneForce;
        // Symbol: drake::multibody::RevoluteJoint::GetDamping
        struct /* GetDamping */ {
          // Source: drake/multibody/tree/revolute_joint.h
          const char* doc =
R"""(Returns the Context dependent damping coefficient stored as a
parameter in ``context``. Refer to default_damping() for details.

Parameter ``context``:
    The context storing the state and parameters for the model to
    which ``this`` joint belongs.)""";
        } GetDamping;
        // Symbol: drake::multibody::RevoluteJoint::RevoluteJoint<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/revolute_joint.h
          const char* doc_5args =
R"""(Constructor to create a revolute joint between two bodies so that
frame Jp attached to the parent body P and frame Jc attached to the
child body C, rotate relatively to one another about a common axis.
See this class's documentation for further details on the definition
of these frames and rotation angle. This constructor signature creates
a joint with no joint limits, i.e. the joint position, velocity and
acceleration limits are the pair ``(-∞, ∞)``. The first three
arguments to this constructor are those of the Joint class
constructor. See the Joint class's documentation for details. The
additional parameters are:

Parameter ``axis``:
    A vector in ℝ³ specifying the axis of revolution for this joint.
    Given that frame Jc only rotates with respect to Jp and their
    origins are coincident at all times, the components of ``axis`` in
    either frame Jp or Jc are exactly the same, that is, ``axis_Jp =
    axis_Jc``. In other words, ``axis_Jp`` (or ``axis_Jc``) is the
    eigenvector of ``R_JpJc`` with eigenvalue equal to one. This
    vector can have any length, only the direction is used. This
    method aborts if ``axis`` is the zero vector.

Parameter ``damping``:
    Viscous damping coefficient, in N⋅m⋅s, used to model losses within
    the joint. The damping torque (in N⋅m) is modeled as ``τ =
    -damping⋅ω``, i.e. opposing motion, with ω the angular rate for
    ``this`` joint (see get_angular_rate()).

Raises:
    RuntimeError if damping is negative.)""";
          // Source: drake/multibody/tree/revolute_joint.h
          const char* doc_7args =
R"""(Constructor to create a revolute joint between two bodies so that
frame Jp attached to the parent body P and frame Jc attached to the
child body C, rotate relatively to one another about a common axis.
See this class's documentation for further details on the definition
of these frames and rotation angle. The first three arguments to this
constructor are those of the Joint class constructor. See the Joint
class's documentation for details. The additional parameters are:

Parameter ``axis``:
    A vector in ℝ³ specifying the axis of revolution for this joint.
    Given that frame Jc only rotates with respect to Jp and their
    origins are coincident at all times, the components of ``axis`` in
    either frame Jp or Jc are exactly the same, that is, ``axis_Jp =
    axis_Jc``. In other words, ``axis_Jp`` (or ``axis_Jc``) is the
    eigenvector of ``R_JpJc`` with eigenvalue equal to one. This
    vector can have any length, only the direction is used.

Parameter ``pos_lower_limit``:
    Lower position limit, in radians, for the rotation coordinate (see
    get_angle()).

Parameter ``pos_upper_limit``:
    Upper position limit, in radians, for the rotation coordinate (see
    get_angle()).

Parameter ``damping``:
    Viscous damping coefficient, in N⋅m⋅s, used to model losses within
    the joint. The damping torque (in N⋅m) is modeled as ``τ =
    -damping⋅ω``, i.e. opposing motion, with ω the angular rate for
    ``this`` joint (see get_angular_rate()).

Raises:
    RuntimeError if the L2 norm of ``axis`` is less than the square
    root of machine epsilon.

Raises:
    RuntimeError if damping is negative.

Raises:
    RuntimeError if pos_lower_limit > pos_upper_limit.)""";
        } ctor;
        // Symbol: drake::multibody::RevoluteJoint::SetDamping
        struct /* SetDamping */ {
          // Source: drake/multibody/tree/revolute_joint.h
          const char* doc =
R"""(Sets the value of the viscous damping coefficient for this joint,
stored as a parameter in ``context``. Refer to default_damping() for
details.

Parameter ``context``:
    The context storing the state and parameters for the model to
    which ``this`` joint belongs.

Parameter ``damping``:
    The damping value.

Raises:
    RuntimeError if ``damping`` is negative.)""";
        } SetDamping;
        // Symbol: drake::multibody::RevoluteJoint::acceleration_lower_limit
        struct /* acceleration_lower_limit */ {
          // Source: drake/multibody/tree/revolute_joint.h
          const char* doc =
R"""(Returns the acceleration lower limit for ``this`` joint in radians /
s².)""";
        } acceleration_lower_limit;
        // Symbol: drake::multibody::RevoluteJoint::acceleration_upper_limit
        struct /* acceleration_upper_limit */ {
          // Source: drake/multibody/tree/revolute_joint.h
          const char* doc =
R"""(Returns the acceleration upper limit for ``this`` joint in radians /
s².)""";
        } acceleration_upper_limit;
        // Symbol: drake::multibody::RevoluteJoint::default_damping
        struct /* default_damping */ {
          // Source: drake/multibody/tree/revolute_joint.h
          const char* doc =
R"""(Returns ``this`` joint's default damping constant in N⋅m⋅s.)""";
        } default_damping;
        // Symbol: drake::multibody::RevoluteJoint::get_angle
        struct /* get_angle */ {
          // Source: drake/multibody/tree/revolute_joint.h
          const char* doc =
R"""(Gets the rotation angle of ``this`` mobilizer from ``context``.

Parameter ``context``:
    The context of the MultibodyTree this joint belongs to.

Returns:
    The angle coordinate of ``this`` joint stored in the ``context``.)""";
        } get_angle;
        // Symbol: drake::multibody::RevoluteJoint::get_angular_rate
        struct /* get_angular_rate */ {
          // Source: drake/multibody/tree/revolute_joint.h
          const char* doc =
R"""(Gets the rate of change, in radians per second, of ``this`` joint's
angle (see get_angle()) from ``context``.

Parameter ``context``:
    The context of the MultibodyTree this joint belongs to.

Returns:
    The rate of change of ``this`` joint's angle as stored in the
    ``context``.)""";
        } get_angular_rate;
        // Symbol: drake::multibody::RevoluteJoint::get_default_angle
        struct /* get_default_angle */ {
          // Source: drake/multibody/tree/revolute_joint.h
          const char* doc =
R"""(Gets the default rotation angle. Wrapper for the more general
``Joint::default_positions()``.

Returns:
    The default angle of ``this`` stored in ``default_positions_``)""";
        } get_default_angle;
        // Symbol: drake::multibody::RevoluteJoint::position_lower_limit
        struct /* position_lower_limit */ {
          // Source: drake/multibody/tree/revolute_joint.h
          const char* doc =
R"""(Returns the position lower limit for ``this`` joint in radians.)""";
        } position_lower_limit;
        // Symbol: drake::multibody::RevoluteJoint::position_upper_limit
        struct /* position_upper_limit */ {
          // Source: drake/multibody/tree/revolute_joint.h
          const char* doc =
R"""(Returns the position upper limit for ``this`` joint in radians.)""";
        } position_upper_limit;
        // Symbol: drake::multibody::RevoluteJoint::revolute_axis
        struct /* revolute_axis */ {
          // Source: drake/multibody/tree/revolute_joint.h
          const char* doc =
R"""(Returns the axis of revolution of ``this`` joint as a unit vector.
Since the measures of this axis in either frame F or M are the same
(see this class's documentation for frame definitions) then, ``axis =
axis_Jp = axis_Jc``.)""";
        } revolute_axis;
        // Symbol: drake::multibody::RevoluteJoint::set_angle
        struct /* set_angle */ {
          // Source: drake/multibody/tree/revolute_joint.h
          const char* doc =
R"""(Sets the ``context`` so that the generalized coordinate corresponding
to the rotation angle of ``this`` joint equals ``angle``.

Parameter ``context``:
    The context of the MultibodyTree this joint belongs to.

Parameter ``angle``:
    The desired angle in radians to be stored in ``context``.

Returns:
    a constant reference to ``this`` joint.)""";
        } set_angle;
        // Symbol: drake::multibody::RevoluteJoint::set_angular_rate
        struct /* set_angular_rate */ {
          // Source: drake/multibody/tree/revolute_joint.h
          const char* doc =
R"""(Sets the rate of change, in radians per second, of this ``this``
joint's angle to ``angle``. The new rate of change ``angle`` gets
stored in ``context``.

Parameter ``context``:
    The context of the MultibodyTree this joint belongs to.

Parameter ``angle``:
    The desired rate of change of ``this`` joints's angle in radians
    per second. (Should have been named ``rate`` or ``angular_rate``.)

Returns:
    a constant reference to ``this`` joint.)""";
        } set_angular_rate;
        // Symbol: drake::multibody::RevoluteJoint::set_default_angle
        struct /* set_default_angle */ {
          // Source: drake/multibody/tree/revolute_joint.h
          const char* doc =
R"""(Sets the ``default_positions`` of this joint (in this case a single
angle).

Parameter ``angle``:
    The desired default angle of the joint)""";
        } set_default_angle;
        // Symbol: drake::multibody::RevoluteJoint::set_default_damping
        struct /* set_default_damping */ {
          // Source: drake/multibody/tree/revolute_joint.h
          const char* doc =
R"""(Sets the default value of viscous damping for this joint, in N⋅m⋅s.

Raises:
    RuntimeError if damping is negative.

Raises:
    RuntimeError if this element is not associated with a
    MultibodyPlant.

Precondition:
    the MultibodyPlant must not be finalized.)""";
        } set_default_damping;
        // Symbol: drake::multibody::RevoluteJoint::set_random_angle_distribution
        struct /* set_random_angle_distribution */ {
          // Source: drake/multibody/tree/revolute_joint.h
          const char* doc = R"""()""";
        } set_random_angle_distribution;
        // Symbol: drake::multibody::RevoluteJoint::type_name
        struct /* type_name */ {
          // Source: drake/multibody/tree/revolute_joint.h
          const char* doc = R"""()""";
        } type_name;
        // Symbol: drake::multibody::RevoluteJoint::velocity_lower_limit
        struct /* velocity_lower_limit */ {
          // Source: drake/multibody/tree/revolute_joint.h
          const char* doc =
R"""(Returns the velocity lower limit for ``this`` joint in radians / s.)""";
        } velocity_lower_limit;
        // Symbol: drake::multibody::RevoluteJoint::velocity_upper_limit
        struct /* velocity_upper_limit */ {
          // Source: drake/multibody/tree/revolute_joint.h
          const char* doc =
R"""(Returns the velocity upper limit for ``this`` joint in radians / s.)""";
        } velocity_upper_limit;
      } RevoluteJoint;
      // Symbol: drake::multibody::RevoluteSpring
      struct /* RevoluteSpring */ {
        // Source: drake/multibody/tree/revolute_spring.h
        const char* doc =
R"""(This ForceElement models a torsional spring attached to a
RevoluteJoint and applies a torque to that joint


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    τ = -k⋅(θ - θ₀)

.. raw:: html

    </details>

where θ₀ is the nominal joint position. Note that joint damping exists
within the RevoluteJoint itself, and so is not included here.

The k (stiffness) and θ₀ (nominal angle) specified in the constructor
are kept as default values. These parameters are stored within the
context and can be accessed and set by context dependent
getters/setters.)""";
        // Symbol: drake::multibody::RevoluteSpring::CalcConservativePower
        struct /* CalcConservativePower */ {
          // Source: drake/multibody/tree/revolute_spring.h
          const char* doc = R"""()""";
        } CalcConservativePower;
        // Symbol: drake::multibody::RevoluteSpring::CalcNonConservativePower
        struct /* CalcNonConservativePower */ {
          // Source: drake/multibody/tree/revolute_spring.h
          const char* doc = R"""()""";
        } CalcNonConservativePower;
        // Symbol: drake::multibody::RevoluteSpring::CalcPotentialEnergy
        struct /* CalcPotentialEnergy */ {
          // Source: drake/multibody/tree/revolute_spring.h
          const char* doc = R"""()""";
        } CalcPotentialEnergy;
        // Symbol: drake::multibody::RevoluteSpring::DoCalcAndAddForceContribution
        struct /* DoCalcAndAddForceContribution */ {
          // Source: drake/multibody/tree/revolute_spring.h
          const char* doc = R"""()""";
        } DoCalcAndAddForceContribution;
        // Symbol: drake::multibody::RevoluteSpring::DoCloneToScalar
        struct /* DoCloneToScalar */ {
          // Source: drake/multibody/tree/revolute_spring.h
          const char* doc = R"""()""";
        } DoCloneToScalar;
        // Symbol: drake::multibody::RevoluteSpring::DoShallowClone
        struct /* DoShallowClone */ {
          // Source: drake/multibody/tree/revolute_spring.h
          const char* doc = R"""()""";
        } DoShallowClone;
        // Symbol: drake::multibody::RevoluteSpring::GetNominalAngle
        struct /* GetNominalAngle */ {
          // Source: drake/multibody/tree/revolute_spring.h
          const char* doc =
R"""(Returns the Context dependent nominal angle θ₀ stored as a parameter
in ``context``.

Parameter ``context``:
    The context storing the state and parameters for the model to
    which ``this`` spring belongs.

Returns ``returns``:
    the nominal angle θ₀ in radians.)""";
        } GetNominalAngle;
        // Symbol: drake::multibody::RevoluteSpring::GetStiffness
        struct /* GetStiffness */ {
          // Source: drake/multibody/tree/revolute_spring.h
          const char* doc =
R"""(Returns the Context dependent stiffness coefficient k stored as a
parameter in ``context``.

Parameter ``context``:
    The context storing the state and parameters for the model to
    which ``this`` spring belongs.

Returns ``returns``:
    the stiffness k in N⋅m/rad stored within the context.)""";
        } GetStiffness;
        // Symbol: drake::multibody::RevoluteSpring::RevoluteSpring<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/revolute_spring.h
          const char* doc =
R"""(Constructor for a spring attached to the given joint

Parameter ``nominal_angle``:
    The nominal angle of the spring θ₀, in radians, at which the
    spring applies no moment.

Parameter ``stiffness``:
    The stiffness k of the spring in N⋅m/rad.

Raises:
    RuntimeError if ``stiffness`` is negative.)""";
        } ctor;
        // Symbol: drake::multibody::RevoluteSpring::SetNominalAngle
        struct /* SetNominalAngle */ {
          // Source: drake/multibody/tree/revolute_spring.h
          const char* doc =
R"""(Sets the value of the nominal angle θ₀ for this force element, stored
as a parameter in ``context``.

Parameter ``context``:
    The context storing the state and parameters for the model to
    which ``this`` spring belongs.

Parameter ``nominal_angle``:
    The nominal angle θ₀ in radians stored within the context.)""";
        } SetNominalAngle;
        // Symbol: drake::multibody::RevoluteSpring::SetStiffness
        struct /* SetStiffness */ {
          // Source: drake/multibody/tree/revolute_spring.h
          const char* doc =
R"""(Sets the value of the linear stiffness coefficient k for this force
element, stored as a parameter in ``context``.

Parameter ``context``:
    The context storing the state and parameters for the model to
    which ``this`` spring belongs.

Parameter ``stiffness``:
    The stiffness value k with units N⋅m/rad.

Raises:
    RuntimeError if ``stiffness`` is negative.)""";
        } SetStiffness;
        // Symbol: drake::multibody::RevoluteSpring::default_nominal_angle
        struct /* default_nominal_angle */ {
          // Source: drake/multibody/tree/revolute_spring.h
          const char* doc =
R"""(Returns the default spring reference angle θ₀ in radians.)""";
        } default_nominal_angle;
        // Symbol: drake::multibody::RevoluteSpring::default_stiffness
        struct /* default_stiffness */ {
          // Source: drake/multibody/tree/revolute_spring.h
          const char* doc =
R"""(Returns the default stiffness constant k in N⋅m/rad.)""";
        } default_stiffness;
        // Symbol: drake::multibody::RevoluteSpring::joint
        struct /* joint */ {
          // Source: drake/multibody/tree/revolute_spring.h
          const char* doc =
R"""(Returns the joint associated with this spring.

Raises:
    RuntimeError if this element is not associated with a
    MultibodyPlant.)""";
        } joint;
      } RevoluteSpring;
      // Symbol: drake::multibody::RigidBody
      struct /* RigidBody */ {
        // Source: drake/multibody/tree/rigid_body.h
        const char* doc =
R"""(The term **rigid body** implies that the deformations of the body
under consideration are so small that they have no significant effect
on the overall motions of the body and therefore deformations can be
neglected. If deformations are neglected, the distance between any two
points on the rigid body remains constant at all times. This
invariance of the distance between two arbitrary points is often taken
as the definition of a rigid body in classical treatments of multibody
mechanics [Goldstein 2001]. It can be demonstrated that the
unconstrained three-dimensional motions of a rigid body can be
described by six coordinates and thus it is often said that a free
body in space has six **degrees of freedom**. These degrees of freedom
obey the Newton-Euler equations of motion. However, within a
MultibodyTree, a RigidBody is *not* free in space; instead, it is
assigned a limited number of degrees of freedom (0-6) with respect to
its parent body in the multibody tree by its Mobilizer (also called a
"tree joint" or "inboard joint"). Additional constraints on
permissible motion can be added using Constraint objects to remove
more degrees of freedom.

- [Goldstein 2001] H Goldstein, CP Poole, JL Safko, Classical Mechanics
                   (3rd Edition), Addison-Wesley, 2001.)""";
        // Symbol: drake::multibody::RigidBody::AddInForce
        struct /* AddInForce */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""(Adds the SpatialForce on this RigidBody B, applied at point P and
expressed in a frame E into ``forces``.

Parameter ``context``:
    The context containing the current state of the model.

Parameter ``p_BP_E``:
    The position of point P in B, expressed in a frame E.

Parameter ``F_Bp_E``:
    The spatial force to be applied on body B at point P, expressed in
    frame E.

Parameter ``frame_E``:
    The expressed-in frame E.

Parameter ``forces``:
    A multibody forces objects that on output will have ``F_Bp_E``
    added.

Raises:
    RuntimeError if ``forces`` is nullptr or if it is not consistent
    with the model to which this body belongs.)""";
        } AddInForce;
        // Symbol: drake::multibody::RigidBody::AddInForceInWorld
        struct /* AddInForceInWorld */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""(Adds the SpatialForce on this RigidBody B, applied at body B's origin
Bo and expressed in the world frame W into ``forces``.)""";
        } AddInForceInWorld;
        // Symbol: drake::multibody::RigidBody::CalcCenterOfMassInBodyFrame
        struct /* CalcCenterOfMassInBodyFrame */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""(Gets this body's center of mass position from the given context.

Parameter ``context``:
    contains the state of the multibody system.

Returns:
    p_BoBcm_B position vector from Bo (this rigid body B's origin) to
    Bcm (B's center of mass), expressed in B.

Precondition:
    the context makes sense for use by this RigidBody.)""";
        } CalcCenterOfMassInBodyFrame;
        // Symbol: drake::multibody::RigidBody::CalcCenterOfMassTranslationalAccelerationInWorld
        struct /* CalcCenterOfMassTranslationalAccelerationInWorld */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""(Calculates Bcm's translational acceleration in the world frame W.

Parameter ``context``:
    The context contains the state of the model.

Returns ``a_WBcm_W``:
    The translational acceleration of Bcm (this rigid body's center of
    mass) in the world frame W, expressed in W.

Note:
    When cached values are out of sync with the state stored in
    context, this method performs an expensive forward dynamics
    computation, whereas once evaluated, successive calls to this
    method are inexpensive.)""";
        } CalcCenterOfMassTranslationalAccelerationInWorld;
        // Symbol: drake::multibody::RigidBody::CalcCenterOfMassTranslationalVelocityInWorld
        struct /* CalcCenterOfMassTranslationalVelocityInWorld */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""(Calculates Bcm's translational velocity in the world frame W.

Parameter ``context``:
    The context contains the state of the model.

Returns ``v_WBcm_W``:
    The translational velocity of Bcm (this rigid body's center of
    mass) in the world frame W, expressed in W.)""";
        } CalcCenterOfMassTranslationalVelocityInWorld;
        // Symbol: drake::multibody::RigidBody::CalcSpatialInertiaInBodyFrame
        struct /* CalcSpatialInertiaInBodyFrame */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""(Gets this body's spatial inertia about its origin from the given
context.

Parameter ``context``:
    contains the state of the multibody system.

Returns:
    M_BBo_B spatial inertia of this rigid body B about Bo (B's
    origin), expressed in B. M_BBo_B contains properties related to
    B's mass, the position vector from Bo to Bcm (B's center of mass),
    and G_BBo_B (B's unit inertia about Bo expressed in B).

Precondition:
    the context makes sense for use by this RigidBody.)""";
        } CalcSpatialInertiaInBodyFrame;
        // Symbol: drake::multibody::RigidBody::CloneToScalar
        struct /* CloneToScalar */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""((Advanced) This method is mostly intended to be called by
MultibodyTree::CloneToScalar(). Most users should not call this clone
method directly but rather clone the entire parent MultibodyTree if
needed.

See also:
    MultibodyTree::CloneToScalar())""";
        } CloneToScalar;
        // Symbol: drake::multibody::RigidBody::EvalPoseInWorld
        struct /* EvalPoseInWorld */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""(Returns the pose ``X_WB`` of this RigidBody B in the world frame W as
a function of the state of the model stored in ``context``.)""";
        } EvalPoseInWorld;
        // Symbol: drake::multibody::RigidBody::EvalSpatialAccelerationInWorld
        struct /* EvalSpatialAccelerationInWorld */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""(Evaluates A_WB, this body B's SpatialAcceleration in the world frame
W.

Parameter ``context``:
    Contains the state of the model.

Returns ``A_WB_W``:
    this body B's spatial acceleration in the world frame W, expressed
    in W (for point Bo, the body's origin).

Note:
    When cached values are out of sync with the state stored in
    context, this method performs an expensive forward dynamics
    computation, whereas once evaluated, successive calls to this
    method are inexpensive.)""";
        } EvalSpatialAccelerationInWorld;
        // Symbol: drake::multibody::RigidBody::EvalSpatialVelocityInWorld
        struct /* EvalSpatialVelocityInWorld */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""(Evaluates V_WB, this body B's SpatialVelocity in the world frame W.

Parameter ``context``:
    Contains the state of the model.

Returns ``V_WB_W``:
    this body B's spatial velocity in the world frame W, expressed in
    W (for point Bo, the body frame's origin).)""";
        } EvalSpatialVelocityInWorld;
        // Symbol: drake::multibody::RigidBody::GetForceInWorld
        struct /* GetForceInWorld */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""(Gets the SpatialForce on this RigidBody B from ``forces`` as F_BBo_W:
applied at body B's origin Bo and expressed in world frame W.)""";
        } GetForceInWorld;
        // Symbol: drake::multibody::RigidBody::Lock
        struct /* Lock */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""(For a floating base RigidBody, lock its inboard joint. Its generalized
velocities will be 0 until it is unlocked.

Raises:
    RuntimeError if this body is not a floating base body.)""";
        } Lock;
        // Symbol: drake::multibody::RigidBody::RigidBody<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc_2args =
R"""(Constructs a RigidBody named ``body_name`` with the given default
SpatialInertia.

Parameter ``body_name``:
    A name associated with this body.

Parameter ``M_BBo_B``:
    Spatial inertia of this body B about the frame's origin ``Bo`` and
    expressed in the body frame B. When not provided, defaults to
    zero.

Note:
    See multibody_spatial_inertia for details on the monogram notation
    used for spatial inertia quantities.)""";
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc_3args =
R"""(Constructs a RigidBody named ``body_name`` with the given default
SpatialInertia.

Parameter ``body_name``:
    A name associated with this body.

Parameter ``model_instance``:
    The model instance associated with this body.

Parameter ``M_BBo_B``:
    Spatial inertia of this body B about the frame's origin ``Bo`` and
    expressed in the body frame B. When not provided, defaults to
    zero.

Note:
    See multibody_spatial_inertia for details on the monogram notation
    used for spatial inertia quantities.)""";
        } ctor;
        // Symbol: drake::multibody::RigidBody::SetCenterOfMassInBodyFrame
        struct /* SetCenterOfMassInBodyFrame */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""((Advanced) Sets this body's center of mass position while preserving
its inertia about its body origin.

Parameter ``out``:
    ] context contains the state of the multibody system. It is
    modified to store the updated com (center of mass position).

Parameter ``com``:
    position vector from Bo (this body B's origin) to Bcm (B's center
    of mass), expressed in B.

Note:
    This function changes B's center of mass position **without**
    modifying G_BBo_B (B's unit inertia about Bo, expressed in B).
    Since this use case is very unlikely, consider using
    SetSpatialInertiaInBodyFrame() or
    SetCenterOfMassInBodyFrameAndPreserveCentralInertia().

Precondition:
    the context makes sense for use by this RigidBody.

Raises:
    RuntimeError if context is null.

Warning:
    Do not use this function unless it is needed (think twice).)""";
        } SetCenterOfMassInBodyFrame;
        // Symbol: drake::multibody::RigidBody::SetCenterOfMassInBodyFrameAndPreserveCentralInertia
        struct /* SetCenterOfMassInBodyFrameAndPreserveCentralInertia */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""(Sets this body's center of mass position while preserving its inertia
about its center of mass.

Parameter ``out``:
    ] context contains the state of the multibody system. It is
    modified to store the updated center_of_mass_position and the
    updated G_BBo_B (this body B's unit inertia about B's origin Bo,
    expressed in B).

Parameter ``center_of_mass_position``:
    position vector from Bo to Bcm (B's center of mass), expressed in
    B.

Note:
    G_BBo_B is modified to ensure B's inertia about Bcm is unchanged.
    Although this function can work well when B's mass is concentrated
    at (or mostly near) a single point, it has **questionable**
    utility to generally account for inertia changes due to arbitrary
    center of mass changes. Consider using
    SetSpatialInertiaInBodyFrame() instead.

Precondition:
    the context makes sense for use by this RigidBody.

Raises:
    RuntimeError if context is null.)""";
        } SetCenterOfMassInBodyFrameAndPreserveCentralInertia;
        // Symbol: drake::multibody::RigidBody::SetMass
        struct /* SetMass */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""(For this RigidBody B, sets its mass stored in ``context`` to ``mass``.

Parameter ``context``:
    contains the state of the multibody system.

Parameter ``mass``:
    mass of this rigid body B.

Note:
    This function changes this body B's mass and appropriately scales
    I_BBo_B (B's rotational inertia about Bo, expressed in B).

Precondition:
    the context makes sense for use by this RigidBody.

Raises:
    RuntimeError if context is null.)""";
        } SetMass;
        // Symbol: drake::multibody::RigidBody::SetSpatialInertiaInBodyFrame
        struct /* SetSpatialInertiaInBodyFrame */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""(For this RigidBody B, sets its SpatialInertia that is stored in
``context`` to ``M_Bo_B``.

Parameter ``context``:
    contains the state of the multibody system.

Parameter ``M_Bo_B``:
    spatial inertia of this rigid body B about Bo (B's origin),
    expressed in B. M_Bo_B contains properties related to B's mass,
    the position vector from Bo to Bcm (B's center of mass), and
    G_Bo_B (B's unit inertia about Bo expressed in B).

Precondition:
    the context makes sense for use by this RigidBody.

Raises:
    RuntimeError if context is null.)""";
        } SetSpatialInertiaInBodyFrame;
        // Symbol: drake::multibody::RigidBody::Unlock
        struct /* Unlock */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""(For a floating base RigidBody, unlock its inboard joint.

Raises:
    RuntimeError if this body is not a floating base body.)""";
        } Unlock;
        // Symbol: drake::multibody::RigidBody::body_frame
        struct /* body_frame */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""(Returns a const reference to the associated BodyFrame.)""";
        } body_frame;
        // Symbol: drake::multibody::RigidBody::default_com
        struct /* default_com */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""(Returns the default value of this RigidBody's center of mass as
measured and expressed in its body frame. This value is initially
supplied at construction when specifying this body's SpatialInertia.

Returns ``p_BoBcm_B``:
    The position of this rigid body B's center of mass ``Bcm``
    measured from Bo (B's frame origin) and expressed in B (body B's
    frame).)""";
        } default_com;
        // Symbol: drake::multibody::RigidBody::default_mass
        struct /* default_mass */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""(Returns this RigidBody's default mass, which is initially supplied at
construction when specifying this body's SpatialInertia.

Note:
    In general, a rigid body's mass can be a constant property stored
    in this rigid body's SpatialInertia or a parameter that is stored
    in a Context. The default constant mass value is used to
    initialize the mass parameter in the Context.)""";
        } default_mass;
        // Symbol: drake::multibody::RigidBody::default_rotational_inertia
        struct /* default_rotational_inertia */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""(Gets the default value of this body B's rotational inertia about Bo
(B's origin), expressed in B (this body's body frame). This value is
calculated from the SpatialInertia supplied at construction of this
body.

Returns ``I_BBo_B``:
    body B's rotational inertia about Bo, expressed in B.)""";
        } default_rotational_inertia;
        // Symbol: drake::multibody::RigidBody::default_spatial_inertia
        struct /* default_spatial_inertia */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""(Gets the default value of this body B's SpatialInertia about Bo (B's
origin) and expressed in B (this body's frame).

Returns ``M_BBo_B``:
    body B's spatial inertia about Bo, expressed in B.)""";
        } default_spatial_inertia;
        // Symbol: drake::multibody::RigidBody::default_unit_inertia
        struct /* default_unit_inertia */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""(Returns the default value of this body B's unit inertia about Bo (body
B's origin), expressed in B (this body's body frame). This value is
initially supplied at construction when specifying this body's
SpatialInertia.

Returns ``G_BBo_B``:
    rigid body B's unit inertia about Bo, expressed in B.)""";
        } default_unit_inertia;
        // Symbol: drake::multibody::RigidBody::floating_position_suffix
        struct /* floating_position_suffix */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""(Returns a string suffix (e.g. to be appended to the name()) to
identify the ``k`th position in the floating base body.
`position_index_in_body`` must be in [0, 7) if
``has_quaternion_dofs()`` is true, otherwise in [0, 6).

Raises:
    RuntimeError if called pre-finalize

Precondition:
    this is a floating base body

See also:
    is_floating_base_body(), has_quaternion_dofs()

See also:
    MultibodyPlant::Finalize())""";
        } floating_position_suffix;
        // Symbol: drake::multibody::RigidBody::floating_positions_start
        struct /* floating_positions_start */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""((Advanced) For floating base bodies (see is_floating_base_body()),
returns the index of this RigidBody's first generalized position in
the vector q of generalized position coordinates for a MultibodyPlant
model. Positions q for this RigidBody are then contiguous starting at
this index. When a floating RigidBody is modeled with quaternion
coordinates (see has_quaternion_dofs()), the four consecutive entries
in the state starting at this index correspond to the quaternion that
parametrizes this RigidBody's orientation.

Raises:
    RuntimeError if called pre-finalize

Precondition:
    this is a floating base body

See also:
    is_floating_base_body(), has_quaternion_dofs()

See also:
    MultibodyPlant::Finalize())""";
        } floating_positions_start;
        // Symbol: drake::multibody::RigidBody::floating_velocities_start_in_v
        struct /* floating_velocities_start_in_v */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""((Advanced) For floating base bodies (see is_floating_base_body()),
returns the index of this RigidBody's first generalized velocity in
the vector v of generalized velocities for a MultibodyPlant model.
Velocities v for this RigidBody are then contiguous starting at this
index.

Raises:
    RuntimeError if called pre-finalize

Precondition:
    this is a floating base body

See also:
    is_floating_base_body(), MultibodyPlant::Finalize())""";
        } floating_velocities_start_in_v;
        // Symbol: drake::multibody::RigidBody::floating_velocity_suffix
        struct /* floating_velocity_suffix */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""(Returns a string suffix (e.g. to be appended to the name()) to
identify the ``k`th velocity in the floating base body.
`velocity_index_in_body`` must be in [0,6).

Raises:
    RuntimeError if called pre-finalize

Precondition:
    this is a floating base body

See also:
    is_floating_base_body(), MultibodyPlant::Finalize())""";
        } floating_velocity_suffix;
        // Symbol: drake::multibody::RigidBody::get_angular_acceleration_in_world
        struct /* get_angular_acceleration_in_world */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""((Advanced) Extract this body's angular acceleration in world,
expressed in world.

Parameter ``ac``:
    velocity kinematics cache.

Returns ``alpha_WB_W``:
    B's angular acceleration in world W, expressed in W.)""";
        } get_angular_acceleration_in_world;
        // Symbol: drake::multibody::RigidBody::get_angular_velocity_in_world
        struct /* get_angular_velocity_in_world */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""((Advanced) Extract this body's angular velocity in world, expressed in
world.

Parameter ``vc``:
    velocity kinematics cache.

Returns ``w_WB_W``:
    rigid body B's angular velocity in world W, expressed in W.)""";
        } get_angular_velocity_in_world;
        // Symbol: drake::multibody::RigidBody::get_mass
        struct /* get_mass */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""(Gets this body's mass from the given context.

Parameter ``context``:
    contains the state of the multibody system.

Precondition:
    the context makes sense for use by this RigidBody.)""";
        } get_mass;
        // Symbol: drake::multibody::RigidBody::get_origin_acceleration_in_world
        struct /* get_origin_acceleration_in_world */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""((Advanced) Extract acceleration of this body's origin in world,
expressed in world.

Parameter ``ac``:
    acceleration kinematics cache.

Returns ``a_WBo_W``:
    acceleration of body origin Bo in world W, expressed in W.)""";
        } get_origin_acceleration_in_world;
        // Symbol: drake::multibody::RigidBody::get_origin_position_in_world
        struct /* get_origin_position_in_world */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""((Advanced) Extract the position vector from world origin to this
body's origin, expressed in world.

Parameter ``pc``:
    position kinematics cache.

Returns ``p_WoBo_W``:
    position vector from Wo (world origin) to Bo (this body's origin)
    expressed in W (world).)""";
        } get_origin_position_in_world;
        // Symbol: drake::multibody::RigidBody::get_origin_velocity_in_world
        struct /* get_origin_velocity_in_world */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""((Advanced) Extract the velocity of this body's origin in world,
expressed in world.

Parameter ``vc``:
    velocity kinematics cache.

Returns ``v_WBo_W``:
    velocity of Bo (body origin) in world W, expressed in W.)""";
        } get_origin_velocity_in_world;
        // Symbol: drake::multibody::RigidBody::get_pose_in_world
        struct /* get_pose_in_world */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""((Advanced) Extract this body's pose in world (from the position
kinematics).

Parameter ``pc``:
    position kinematics cache.

Returns ``X_WB``:
    pose of rigid body B in world frame W.)""";
        } get_pose_in_world;
        // Symbol: drake::multibody::RigidBody::get_rotation_matrix_in_world
        struct /* get_rotation_matrix_in_world */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""((Advanced) Extract the RotationMatrix relating the world frame to this
body's frame.

Parameter ``pc``:
    position kinematics cache.

Returns ``R_WB``:
    rotation matrix relating rigid body B in world frame W.)""";
        } get_rotation_matrix_in_world;
        // Symbol: drake::multibody::RigidBody::get_spatial_acceleration_in_world
        struct /* get_spatial_acceleration_in_world */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""((Advanced) Returns A_WB, this RigidBody B's SpatialAcceleration in the
world frame W.

Parameter ``ac``:
    acceleration kinematics cache.

Returns ``A_WB_W``:
    this rigid body B's spatial acceleration in the world frame W,
    expressed in W (for point Bo, the body frame's origin).)""";
        } get_spatial_acceleration_in_world;
        // Symbol: drake::multibody::RigidBody::get_spatial_velocity_in_world
        struct /* get_spatial_velocity_in_world */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""((Advanced) Returns V_WB, this RigidBody B's SpatialVelocity in the
world frame W.

Parameter ``vc``:
    velocity kinematics cache.

Returns ``V_WB_W``:
    this rigid body B's spatial velocity in the world frame W,
    expressed in W (for point Bo, the body frame's origin).)""";
        } get_spatial_velocity_in_world;
        // Symbol: drake::multibody::RigidBody::has_quaternion_dofs
        struct /* has_quaternion_dofs */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""((Advanced) If ``True``, this body's generalized position coordinates q
include a quaternion, which occupies the first four elements of q.
Note that this does not imply that the body is floating base body
since it may have fewer than 6 dofs or its inboard body could be
something other than World.

Raises:
    RuntimeError if called pre-finalize

See also:
    is_floating_base_body(), MultibodyPlant::Finalize())""";
        } has_quaternion_dofs;
        // Symbol: drake::multibody::RigidBody::index
        struct /* index */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc = R"""(Returns this element's unique index.)""";
        } index;
        // Symbol: drake::multibody::RigidBody::is_floating
        struct /* is_floating */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc_deprecated =
R"""((Deprecated.)

Deprecated:
    Use is_floating_base_body() instead. This will be removed from
    Drake on or after 2026-06-01.)""";
        } is_floating;
        // Symbol: drake::multibody::RigidBody::is_floating_base_body
        struct /* is_floating_base_body */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""((Advanced) Returns ``True`` if this body is a *floating base body*,
meaning it had no explicit joint to a parent body so is mobilized by
an automatically-added (ephemeral) floating (6 dof) joint to World.

Note:
    A floating base body is not necessarily modeled with a quaternion
    mobilizer, see has_quaternion_dofs(). Alternative options include
    a roll-pitch-yaw (rpy) parametrization of rotations, see
    RpyFloatingMobilizer.

Raises:
    RuntimeError if called pre-finalize,

See also:
    MultibodyPlant::Finalize())""";
        } is_floating_base_body;
        // Symbol: drake::multibody::RigidBody::is_locked
        struct /* is_locked */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""(Determines whether this RigidBody is currently locked to its inboard
(parent) RigidBody. This is not limited to floating base bodies but
generally Joint::is_locked() is preferable otherwise.

Returns:
    true if the body is locked, false otherwise.)""";
        } is_locked;
        // Symbol: drake::multibody::RigidBody::mobod_index
        struct /* mobod_index */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""((Advanced) Returns the index of the mobilized body ("mobod") in the
computational directed forest structure of the owning MultibodyTree to
which this RigidBody belongs. This serves as the BodyNode index and
the index into all associated quantities.)""";
        } mobod_index;
        // Symbol: drake::multibody::RigidBody::name
        struct /* name */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""(Gets the ``name`` associated with this rigid body. The name will never
be empty.)""";
        } name;
        // Symbol: drake::multibody::RigidBody::scoped_name
        struct /* scoped_name */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc =
R"""(Returns scoped name of this body. Neither of the two pieces of the
name will be empty (the scope name and the element name).

Raises:
    RuntimeError if this element is not associated with a
    MultibodyPlant.)""";
        } scoped_name;
      } RigidBody;
      // Symbol: drake::multibody::RigidBodyFrame
      struct /* RigidBodyFrame */ {
        // Source: drake/multibody/tree/rigid_body.h
        const char* doc =
R"""(A RigidBodyFrame is a material Frame that serves as the unique
reference frame for a RigidBody.

Each RigidBody B has a unique body frame for which we use the same
symbol B (with meaning clear from context). We represent a body frame
by a RigidBodyFrame object that is created whenever a RigidBody is
constructed and is owned by the RigidBody. All properties of a
RigidBody are defined with respect to its RigidBodyFrame, including
its mass properties and attachment locations for joints, constraints,
actuators, geometry and so on. Run time motion of the body is defined
with respect to the motion of its body frame.

Note that the body frame associated with a rigid body does not
necessarily need to be located at its center of mass nor does it need
to be aligned with the body's principal axes, although, in practice,
it frequently is.

A RigidBodyFrame and RigidBody are tightly coupled concepts; neither
makes sense without the other. Therefore, a RigidBodyFrame instance is
constructed in conjunction with its RigidBody and cannot be
constructed anywhere else. However, you can still access the frame
associated with a body, see RigidBody::body_frame(). This access is
more than a convenience; you can use the RigidBodyFrame to define
other frames on the body and to attach other multibody elements to it.)""";
        // Symbol: drake::multibody::RigidBodyFrame::DoCalcOffsetPoseInBody
        struct /* DoCalcOffsetPoseInBody */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc = R"""()""";
        } DoCalcOffsetPoseInBody;
        // Symbol: drake::multibody::RigidBodyFrame::DoCalcOffsetRotationMatrixInBody
        struct /* DoCalcOffsetRotationMatrixInBody */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc = R"""()""";
        } DoCalcOffsetRotationMatrixInBody;
        // Symbol: drake::multibody::RigidBodyFrame::DoCalcPoseInBodyFrame
        struct /* DoCalcPoseInBodyFrame */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc = R"""()""";
        } DoCalcPoseInBodyFrame;
        // Symbol: drake::multibody::RigidBodyFrame::DoCalcRotationMatrixInBodyFrame
        struct /* DoCalcRotationMatrixInBodyFrame */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc = R"""()""";
        } DoCalcRotationMatrixInBodyFrame;
        // Symbol: drake::multibody::RigidBodyFrame::DoCloneToScalar
        struct /* DoCloneToScalar */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc = R"""()""";
        } DoCloneToScalar;
        // Symbol: drake::multibody::RigidBodyFrame::DoShallowClone
        struct /* DoShallowClone */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc = R"""()""";
        } DoShallowClone;
        // Symbol: drake::multibody::RigidBodyFrame::GetFixedOffsetPoseInBody
        struct /* GetFixedOffsetPoseInBody */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc = R"""()""";
        } GetFixedOffsetPoseInBody;
        // Symbol: drake::multibody::RigidBodyFrame::GetFixedPoseInBodyFrame
        struct /* GetFixedPoseInBodyFrame */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc = R"""()""";
        } GetFixedPoseInBodyFrame;
        // Symbol: drake::multibody::RigidBodyFrame::GetFixedRotationMatrixInBody
        struct /* GetFixedRotationMatrixInBody */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc = R"""()""";
        } GetFixedRotationMatrixInBody;
        // Symbol: drake::multibody::RigidBodyFrame::GetFixedRotationMatrixInBodyFrame
        struct /* GetFixedRotationMatrixInBodyFrame */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc = R"""()""";
        } GetFixedRotationMatrixInBodyFrame;
        // Symbol: drake::multibody::RigidBodyFrame::RigidBodyFrame<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/rigid_body.h
          const char* doc = R"""()""";
        } ctor;
      } RigidBodyFrame;
      // Symbol: drake::multibody::RotationalInertia
      struct /* RotationalInertia */ {
        // Source: drake/multibody/tree/rotational_inertia.h
        const char* doc =
R"""(This class describes the mass distribution (inertia properties) of a
body or composite body about a particular point. Herein, "composite
body" means one body or a collection of bodies that are welded
together. In this documentation, "body" and "composite body" are used
interchangeably.

A **rigid** body's mass distribution is described by three quantities:
the body's mass; the body's center of mass; and the body's rotational
inertia about a particular point. The term **rotational inertia** is
used here and by [Jain 2010] to distinguish from a body's **spatial
inertia**. In this class, a 3x3 **inertia matrix** I represents a
body's rotational inertia about a point and expressed in a frame. More
specifically, ``I_BP_E`` is the inertia matrix of a body B about-point
P and expressed-in frame E (herein frame E's orthogonal unit vectors
Ex, Ey, Ez are denoted 𝐱̂, 𝐲̂, 𝐳̂).


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    | Ixx Ixy Ixz |
    I = | Ixy Iyy Iyz |
        | Ixz Iyz Izz |

.. raw:: html

    </details>

The moments of inertia Ixx, Iyy, Izz and products of inertia Ixy, Ixz,
Iyz are defined in terms of the mass dm of a differential volume of
the body. The position of dm from about-point P is xx̂ + yŷ + zẑ =
[x, y, z]_E.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    Ixx = ∫ (y² + z²) dm
    Iyy = ∫ (x² + z²) dm
    Izz = ∫ (x² + y²) dm
    Ixy = - ∫ x y dm
    Ixz = - ∫ x z dm
    Iyz = - ∫ y z dm

.. raw:: html

    </details>

We use the negated convention for products of inertia, so that I
serves to relate angular velocity ω and angular momentum h via ``h = I
⋅ ω``. Ensure your products of inertia follow this negative sign
convention.

The 3x3 inertia matrix is symmetric and its diagonal elements (moments
of inertia) and off-diagonal elements (products of inertia) are
associated with a body (or composite body) S, an about-point P, and an
expressed-in frame E (𝐱̂, 𝐲̂, 𝐳̂̂). A rotational inertia is
ill-defined unless there is a body S, about-point P, and expressed-in
frame E. The user of this class is responsible for tracking the body
S, about-point P and expressed-in frame E (none of these are stored in
this class).

Note:
    This class does not store the about-point nor the expressed-in
    frame, nor does this class help enforce consistency of the
    about-point or expressed-in frame. To help users of this class
    track the about-point and expressed-in frame, we strongly
    recommend the following notation.

Note:
    In typeset material, use the symbol :math:`[I^{S/P}]_E` to
    represent the rotational inertia (inertia matrix) of a body (or
    composite body) S about-point P, expressed in frame E. In code and
    comments, use the monogram notation ``I_SP_E`` (e.g., as described
    in multibody_spatial_inertia). If the about-point P is fixed to a
    body B, the point is named :math:`B_P` and this appears in
    code/comments as ``Bp``. Examples: ``I_BBp_E`` is rigid body B's
    rotational inertia about-point Bp expressed-in frame E; I_BBo_E is
    B's rotational inertia about-point ``Bo`` (body B's origin)
    expressed-in frame E; and I_BBcm_E is B's inertia matrix
    about-point ``Bcm`` (B's center of mass) expressed-in frame E.

Note:
    The rotational inertia (inertia matrix) can be re-expressed in
    terms of a special frame whose orthogonal unit vectors are
    parallel to **principal axes of inertia** so that the inertia
    matrix is diagonalized with elements called **principal moments of
    inertia**.

Note:
    The formal definition of the inertia matrix :math:`I^{S/P}` of a
    system S about a point P follows the definition of the inertia
    dyadic 𝐈 of S about P, which begins by modeling S with n particles
    S₁ ... Sₙ (e.g., 12 grams of carbon can be modeled with n = 6.02 *
    10²³ molecules/particles). The inertia dyadic 𝐈₁ of one particle
    S₁ about point P is defined [Kane, 1985] in terms of m₁ (mass of
    S₁), ᴾ𝐩ˢ¹ (position vector from P to S₁), and the unit dyadic 𝐔
    which is defined by the property 𝐔 ⋅ 𝐯 = 𝐯 where 𝐯 is is any
    vector (this definition of 𝐔 is analogous to defining the identity
    matrix by the property 𝑰𝒅𝒆𝒏𝒕𝒊𝒕𝒚𝑴𝒂𝒕𝒓𝒊𝒙 * 𝒂𝒏𝒚𝑴𝒂𝒕𝒓𝒊𝒙 = 𝒂𝒏𝒚𝑴𝒂𝒕𝒓𝒊𝒙).


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    𝐈₁ = m₁ * [𝐔 * (ᴾ𝐩ˢ¹ ⋅ ᴾ𝐩ˢ¹)  -  ᴾ𝐩ˢ¹ * ᴾ𝐩ˢ¹]

.. raw:: html

    </details>

Note: The vector dot-product (⋅) above produces a scalar whereas the
vector multiply (*) produces a dyadic which is a 2nd-order tensor
(ᴾ𝐩ˢ¹ * ᴾ𝐩ˢ¹ is similar to the matrix outer-product of a 3x1 matrix
multiplied by a 1x3 matrix). An example inertia dyadic for a single
particle is shown further below. The inertia dyadic 𝐈 of the entire
system S is defined by summing the inertia dyadic of each particle Sᵢ
about P (i = 1, ... n), i.e.,


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    𝐈 = 𝐈₁ + 𝐈₂ + ... 𝐈ₙ

.. raw:: html

    </details>

The elements of the inertia matrix :math:`[I^{S/P}]_E` expressed in
frame E (in terms of orthogonal unit vectors 𝐱̂, 𝐲̂, 𝐳̂̂) are found by
pre-dot multiplying and post-dot multiplying 𝐈 with appropriate unit
vectors.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    Ixx = 𝐱̂ ⋅ 𝐈 ⋅ 𝐱̂     Ixy = 𝐱̂ ⋅ 𝐈 ⋅ 𝐲̂      Ixz = 𝐱̂ ⋅ 𝐈 ⋅ 𝐳̂̂
       Iyx = 𝐲̂ ⋅ 𝐈 ⋅ 𝐱̂     Iyy = 𝐲̂ ⋅ 𝐈 ⋅ 𝐲̂      Iyz = 𝐲̂ ⋅ 𝐈 ⋅ 𝐳̂̂
       Izx = 𝐳̂̂ ⋅ 𝐈 ⋅ 𝐱̂     Izy = 𝐳̂̂ ⋅ 𝐈 ⋅ 𝐲̂      Izz = 𝐳̂̂ ⋅ 𝐈 ⋅ 𝐳̂̂

.. raw:: html

    </details>

The inertia dyadic 𝐈ᴮ of a rigid body B about Bcm (B's center of mass)
is related to various dynamic quantities. For example, B's angular
momentum 𝐇 about Bcm in a frame N and B's kinetic energy KE in N
relate to 𝐈ᴮ by


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    𝐇 = 𝐈ᴮ ⋅ 𝛚
       KE = 1/2 𝛚 ⋅ 𝐈ᴮ ⋅ 𝛚  +  1/2 mᴮ 𝐯 ⋅ 𝐯

.. raw:: html

    </details>

where 𝛚 is B's angular velocity in N, 𝐯 is Bcm's translational
velocity in N, and mᴮ is B's mass. When frame N happens to be a
Newtonian frame (also called an inertial frame or
non-rotating/non-accelerating frame), the moment 𝐓 of all forces on B
about Bcm relates to 𝐈ᴮ and 𝛂 (B's angular acceleration in N) by
Euler's rigid body equation as


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    𝐓 = 𝐈ᴮ ⋅ 𝛂  +  𝛚 × 𝐈ᴮ ⋅ 𝛚

.. raw:: html

    </details>

Example: For a particle Q of mass m whose position vector from a point
O is written in terms of right-handed orthogonal unit vectors 𝐱̂, 𝐲̂,
𝐳̂ (below), the inertia dyadic 𝐈 of particle Q about point O is
defined and calculated


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    𝐩 = x 𝐱̂  +  y 𝐲̂                               (given)
        𝐈 = m * [𝐔 * (𝐩 ⋅ 𝐩)  -  𝐩 * 𝐩]              (definition)
          = m * [𝐔 * (x² + y²)  -  (x𝐱̂ + y𝐲̂̂) * (x𝐱̂ + y𝐲̂)
          = m * [(𝐱̂𝐱̂ + 𝐲̂𝐲̂ + 𝐳̂𝐳̂) * (x² + y²) - (x²𝐱̂𝐱̂ + xy𝐱̂𝐲̂̂ + xy𝐲̂̂𝐱̂ + y²𝐲̂̂𝐲̂̂)]
          = m * [y²𝐱̂𝐱̂ + x²𝐲̂𝐲̂ + (x² + y²)𝐳̂𝐳̂ - xy𝐱̂𝐲̂̂ - xy𝐲̂̂𝐱̂]

.. raw:: html

    </details>

which means the inertia matrix for particle Q about point O for 𝐱̂,
𝐲̂, 𝐳̂ is


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    |  m y²     -m x y         0     |
    I = | -m x y     m x²          0     |
        |    0         0     m (x² + y²) |

.. raw:: html

    </details>

[Kane, 1985] pg. 68. "Dynamics: Theory and Applications," McGraw-Hill
Co., New York, 1985 (with D. A. Levinson). Available for free .pdf
download: https://ecommons.cornell.edu/handle/1813/637

Note:
    Several methods in this class throw a RuntimeError for invalid
    rotational inertia operations in debug releases only. This
    provides speed in a release build while facilitating debugging in
    debug builds. In addition, these validity tests are only performed
    for scalar types for which drake::scalar_predicate<T>::is_bool is
    ``True``. For instance, validity checks are not performed when T
    is symbolic::Expression.

Note:
    The methods of this class satisfy the "basic exception guarantee":
    if an exception is thrown, the program will still be in a valid
    state. Specifically, no resources are leaked, and all objects'
    invariants are intact. Be aware that RotationalInertia objects may
    contain invalid inertia data in cases where input checking is
    skipped.

See also:
    https://en.cppreference.com/w/cpp/language/exceptions

Various methods in this class require numerical (not symbolic) data
types.)""";
        // Symbol: drake::multibody::RotationalInertia::CalcMaximumPossibleMomentOfInertia
        struct /* CalcMaximumPossibleMomentOfInertia */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(Returns the maximum possible moment of inertia for ``this`` rotational
inertia about-point P for any expressed-in frame E.

Remark:
    The maximum moment Imax has range: trace / 3 <= Imax <= trace / 2.

See also:
    Trace())""";
        } CalcMaximumPossibleMomentOfInertia;
        // Symbol: drake::multibody::RotationalInertia::CalcPrincipalMomentsAndAxesOfInertia
        struct /* CalcPrincipalMomentsAndAxesOfInertia */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(Forms the 3 principal moments of inertia and their 3 associated
principal directions for ``this`` rotational inertia.

Returns:
    3 principal moments of inertia [Ixx Iyy Izz], sorted in ascending
    order (Ixx ≤ Iyy ≤ Izz) and a rotation matrix R_EA whose columns
    are the 3 associated principal directions that relate the
    expressed-in frame E to a frame A, where frame E is the
    expressed-in frame for ``this`` rotational inertia I_BP_E (body
    B's rotational inertia about-point P) and frame A contains
    right-handed orthonormal vectors Ax, Ay, Az. The 1ˢᵗ column of
    R_EA is Ax_E (Ax expressed in frame E) which is parallel to the
    principal axis associated with Ixx (the smallest principal moment
    of inertia). Similarly, the 2ⁿᵈ and 3ʳᵈ columns of R_EA are Ay_E
    and Az_E, which are parallel to principal axes associated with Iyy
    and Izz (the intermediate and largest principal moments of
    inertia). If all principal moments of inertia are equal (i.e., Ixx
    = Iyy = Izz), R_EA is the identity matrix.

Raises:
    RuntimeError if the elements of ``this`` rotational inertia cannot
    be converted to a real finite double. For example, an exception is
    thrown if ``this`` contains an erroneous NaN or if scalar type T
    is symbolic.

See also:
    CalcPrincipalMomentsOfInertia() to calculate the principal moments
    of inertia [Ixx Iyy Izz], without calculating the principal
    directions.)""";
        } CalcPrincipalMomentsAndAxesOfInertia;
        // Symbol: drake::multibody::RotationalInertia::CalcPrincipalMomentsOfInertia
        struct /* CalcPrincipalMomentsOfInertia */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(Forms the 3 principal moments of inertia for ``this`` rotational
inertia.

Returns ``The``:
    3 principal moments of inertia [Imin Imed Imax], sorted in
    ascending order (Imin ≤ Imed ≤ Imax).

Raises:
    RuntimeError if the elements of ``this`` rotational inertia cannot
    be converted to a real finite double. For example, an exception is
    thrown if ``this`` contains an erroneous NaN or if scalar type T
    is symbolic.

See also:
    CalcPrincipalMomentsAndAxesOfInertia() to also calculate principal
    moment of inertia directions associated with ``this`` rotational
    inertia.)""";
        } CalcPrincipalMomentsOfInertia;
        // Symbol: drake::multibody::RotationalInertia::CopyToFullMatrix3
        struct /* CopyToFullMatrix3 */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(Gets a full 3x3 matrix copy of this rotational inertia. The returned
copy is symmetric and includes both lower and upper parts of the
matrix.)""";
        } CopyToFullMatrix3;
        // Symbol: drake::multibody::RotationalInertia::CouldBePhysicallyValid
        struct /* CouldBePhysicallyValid */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(Performs several checks to verify whether ``this`` rotational inertia
*could* be physically valid, including:

- No NaN moments or products of inertia.
- Ixx, Iyy, Izz and principal moments are all non-negative.
- Ixx, Iyy  Izz and principal moments satisfy the triangle inequality:
  - ``Ixx + Iyy ≥ Izz``
  - `Ixx + Izz ≥ Iyy`
  - ``Iyy + Izz ≥ Ixx``

Warning:
    These checks are necessary (but NOT sufficient) conditions for a
    rotational inertia to be physically valid. The sufficient
    condition requires a rotational inertia to satisfy the above
    checks *after* ``this`` is shifted to the center of mass, i.e.,
    the sufficient condition requires calling CouldBePhysicallyValid()
    when the about-point is Bcm (the body's center of mass). Note:
    this class does not know its about-point or its center of mass
    location.

Returns:
    ``True`` for a plausible rotational inertia passing the above
    necessary but insufficient checks and ``False`` otherwise.

Raises:
    RuntimeError if principal moments of inertia cannot be calculated
    (eigenvalue solver) or if scalar type T cannot be converted to a
    double.)""";
        } CouldBePhysicallyValid;
        // Symbol: drake::multibody::RotationalInertia::IsFinite
        struct /* IsFinite */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(Returns true if all moments and products in ``this`` rotational
inertia are finite (e.g., no NaNs or infinities), otherwise returns
false.)""";
        } IsFinite;
        // Symbol: drake::multibody::RotationalInertia::IsNaN
        struct /* IsNaN */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(Returns ``True`` if any moment/product in ``this`` rotational inertia
is NaN. Otherwise returns ``False``.)""";
        } IsNaN;
        // Symbol: drake::multibody::RotationalInertia::IsNearlyEqualTo
        struct /* IsNearlyEqualTo */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(Compares ``this`` rotational inertia to ``other`` rotational inertia
within the specified ``precision`` (which is a dimensionless number
specifying the relative precision to which the comparison is
performed). Denoting ``I_maxA`` as the largest element value that can
appear in a valid ``this`` rotational inertia (independent of the
expressed-in frame E) and denoting ``I_maxB`` as the largest element
value that can appear in a valid ``other`` rotational inertia
(independent of the expressed-in frame E), ``this`` and ``other`` are
considered nearly equal to each other, if: ‖this - other‖∞ < precision
* min(I_maxA, I_maxB)

Parameter ``other``:
    Rotational inertia to compare with ``this`` rotational inertia.

Parameter ``precision``:
    is a dimensionless real positive number that is usually based on
    two factors, namely expected accuracy of moments/products of
    inertia (e.g., from end-user or CAD) and/or machine-precision.

Returns:
    ``True`` if the absolute value of each moment/product of inertia
    in ``this`` is within ``epsilon`` of the corresponding moment/
    product absolute value in ``other``. Otherwise returns ``False``.

Note:
    : This method only works if all moments of inertia with scalar
    type T in ``this`` and ``other`` can be converted to a double
    (discarding supplemental scalar data such as derivatives of an
    AutoDiffXd). It fails at runtime if type T cannot be converted to
    ``double``.)""";
        } IsNearlyEqualTo;
        // Symbol: drake::multibody::RotationalInertia::IsZero
        struct /* IsZero */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(Returns ``True`` if all moments and products of inertia are exactly
zero.)""";
        } IsZero;
        // Symbol: drake::multibody::RotationalInertia::MakeFromMomentsAndProductsOfInertia
        struct /* MakeFromMomentsAndProductsOfInertia */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""((Internal use only) Creates a rotational inertia with moments of
inertia Ixx, Iyy, Izz, and with products of inertia Ixy, Ixz, Iyz.

Parameter ``Ixx``:
    , Iyy, Izz Moments of inertia.

Parameter ``Ixy``:
    , Ixz, Iyz Products of inertia.

Parameter ``skip_validity_check``:
    If set to false, the rotational inertia is checked via
    CouldBePhysicallyValid() to ensure it is physically valid. If set
    to true (not generally recommended), the check is skipped (which
    reduces some computational cost). The default value is false.

Raises:
    RuntimeError if skip_validity_check is false and
    CouldBePhysicallyValid() fails.)""";
        } MakeFromMomentsAndProductsOfInertia;
        // Symbol: drake::multibody::RotationalInertia::MinusEqualsUnchecked
        struct /* MinusEqualsUnchecked */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(Subtracts a rotational inertia ``I_BP_E`` from ``this`` rotational
inertia. No check is done to determine if the result is physically
valid.

Parameter ``I_BP_E``:
    Rotational inertia of a body (or composite body) B to be
    subtracted from ``this`` rotational inertia.

Returns:
    A reference to ``this`` rotational inertia. ``this`` changes since
    rotational inertia ``I_BP_E`` has been subtracted from it.

See also:
    operator-().

Warning:
    This operator may produce an invalid rotational inertia. Use
    operator-=() to perform necessary (but insufficient) checks on the
    physical validity of the resulting rotational inertia.

Note:
    : Although this method is mathematically useful, it may result in
    a rotational inertia that is physically invalid. This method helps
    perform intermediate calculations which do not necessarily
    represent a real rotational inertia. For example, an efficient way
    to shift a rotational inertia from an arbitrary point P to an
    arbitrary point Q is mathematical equivalent to a + (b - c).
    Although ``a`` must be physically valid and the result ``a + (b -
    c)`` must be physically valid, the intermediate quantity (b - c)
    is not necessarily physically valid. This method allows (b - c) to
    be calculated without requiring (b - c) to be physically valid.

See also:
    operator-=().)""";
        } MinusEqualsUnchecked;
        // Symbol: drake::multibody::RotationalInertia::MultiplyByScalarSkipValidityCheck
        struct /* MultiplyByScalarSkipValidityCheck */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""((Internal use only) Multiplies a rotational inertia by a scalar.

Parameter ``s``:
    Scalar which multiplies ``this``.

Returns:
    ``this`` rotational inertia multiplied by ``s``.

See also:
    operator*(const T&, const RotationalInertia<T>&).

Note:
    This method works even if ``s`` is negative or ``this`` is
    invalid. This method is useful for error messages associated with
    an invalid inertia.)""";
        } MultiplyByScalarSkipValidityCheck;
        // Symbol: drake::multibody::RotationalInertia::ReExpress
        struct /* ReExpress */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(Re-expresses ``this`` rotational inertia ``I_BP_E`` to ``I_BP_A``
i.e., re-expresses body B's rotational inertia from frame E to frame
A.

Parameter ``R_AE``:
    RotationMatrix relating frames A and E.

Returns ``I_BP_A``:
    Rotational inertia of B about-point P expressed-in frame A.

Raises:
    RuntimeError for Debug builds if the rotational inertia that is
    re-expressed-in frame A violates CouldBePhysicallyValid().

See also:
    ReExpressInPlace())""";
        } ReExpress;
        // Symbol: drake::multibody::RotationalInertia::ReExpressInPlace
        struct /* ReExpressInPlace */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(Re-expresses ``this`` rotational inertia ``I_BP_E`` in place to
``I_BP_A``. In other words, starts with ``this`` rotational inertia of
a body (or composite body) B about-point P expressed-in frame E and
re-expresses to B's rotational inertia about-point P expressed-in
frame A. More concisely, we compute ``I_BP_A = R_AE * I_BP_E *
(R_AE)ᵀ``.

Parameter ``R_AE``:
    RotationMatrix relating frames A and E.

Raises:
    RuntimeError for Debug builds if the rotational inertia that is
    re-expressed-in frame A violates CouldBePhysicallyValid().

See also:
    ReExpress().)""";
        } ReExpressInPlace;
        // Symbol: drake::multibody::RotationalInertia::RotationalInertia<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc_0args =
R"""(Constructs a rotational inertia that has all its moments/products of
inertia equal to NaN (helps quickly detect uninitialized values).)""";
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc_3args =
R"""(Creates a rotational inertia with moments of inertia ``Ixx``, `Iyy`,
``Izz``, and with each product of inertia set to zero.

Raises:
    RuntimeError for Debug builds if not CouldBePhysicallyValid().)""";
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc_6args =
R"""(Creates a rotational inertia with moments of inertia ``Ixx``, `Iyy`,
``Izz``, and with products of inertia ``Ixy``, `Ixz`, ``Iyz``.

Raises:
    RuntimeError for Debug builds if not CouldBePhysicallyValid().)""";
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc_2args =
R"""(Constructs a rotational inertia for a particle Q of mass ``mass``,
whose position vector from about-point P is p_PQ_E (E is expressed-in
frame). This RuntimeError exception only occurs if ``mass`` < 0.

Parameter ``mass``:
    The mass of particle Q.

Parameter ``p_PQ_E``:
    Position from about-point P to Q, expressed-in frame E.

Returns ``I_QP_E``:
    , Q's rotational inertia about-point P expressed-in frame E.

Remark:
    Negating the position vector p_PQ_E has no affect on the result.

Raises:
    RuntimeError for Debug builds if not CouldBePhysicallyValid().)""";
        } ctor;
        // Symbol: drake::multibody::RotationalInertia::SetToNaN
        struct /* SetToNaN */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(Sets ``this`` rotational inertia so all its elements are equal to NaN.
This helps quickly detect uninitialized moments/products of inertia.)""";
        } SetToNaN;
        // Symbol: drake::multibody::RotationalInertia::SetZero
        struct /* SetZero */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(Sets ``this`` rotational inertia so all its moments/products of
inertia are zero, e.g., for convenient initialization before a
computation or for inertia calculations involving a particle
(point-mass). Note: Real 3D massive physical objects have non-zero
moments of inertia.)""";
        } SetZero;
        // Symbol: drake::multibody::RotationalInertia::ShiftFromCenterOfMass
        struct /* ShiftFromCenterOfMass */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(Calculates the rotational inertia that results from shifting ``this``
rotational inertia for a body (or composite body) B from about-point
Bcm (B's center of mass) to about-point Q. I.e., shifts ``I_BBcm_E``
to ``I_BQ_E`` (both are expressed-in frame E).

Parameter ``mass``:
    The mass of body (or composite body) B.

Parameter ``p_BcmQ_E``:
    Position vector from Bcm to Q, expressed-in frame E.

Returns ``I_BQ_E``:
    B's rotational inertia about-point Q expressed-in frame E.

Raises:
    RuntimeError for Debug builds if the rotational inertia that is
    shifted to about-point Q violates CouldBePhysicallyValid().

Remark:
    Negating the position vector p_BcmQ_E has no affect on the result.)""";
        } ShiftFromCenterOfMass;
        // Symbol: drake::multibody::RotationalInertia::ShiftFromCenterOfMassInPlace
        struct /* ShiftFromCenterOfMassInPlace */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(Shifts ``this`` rotational inertia for a body (or composite body) B
from about-point Bcm (B's center of mass) to about-point Q. I.e.,
shifts ``I_BBcm_E`` to ``I_BQ_E`` (both are expressed-in frame E). On
return, ``this`` is modified to be shifted from about-point Bcm to
about-point Q.

Parameter ``mass``:
    The mass of body (or composite body) B.

Parameter ``p_BcmQ_E``:
    Position vector from Bcm to Q, expressed-in frame E.

Raises:
    RuntimeError for Debug builds if the rotational inertia that is
    shifted to about-point Q violates CouldBePhysicallyValid().

Remark:
    Negating the position vector p_BcmQ_E has no affect on the result.)""";
        } ShiftFromCenterOfMassInPlace;
        // Symbol: drake::multibody::RotationalInertia::ShiftToCenterOfMass
        struct /* ShiftToCenterOfMass */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(Calculates the rotational inertia that results from shifting ``this``
rotational inertia for a body (or composite body) B from about-point Q
to about-point ``Bcm`` (B's center of mass). I.e., shifts ``I_BQ_E``
to ``I_BBcm_E`` (both are expressed-in frame E).

Parameter ``mass``:
    The mass of body (or composite body) B.

Parameter ``p_QBcm_E``:
    Position vector from Q to ``Bcm``, expressed-in frame E.

Returns ``I_BBcm_E``:
    B's rotational inertia about-point ``Bcm`` expressed-in frame E.

Raises:
    RuntimeError for Debug builds if the rotational inertia that is
    shifted to about-point ``Bcm`` violates CouldBePhysicallyValid().

Remark:
    Negating the position vector ``p_QBcm_E`` has no affect on the
    result.)""";
        } ShiftToCenterOfMass;
        // Symbol: drake::multibody::RotationalInertia::ShiftToCenterOfMassInPlace
        struct /* ShiftToCenterOfMassInPlace */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(Shifts ``this`` rotational inertia for a body (or composite body) B
from about-point Q to about-point ``Bcm`` (B's center of mass). I.e.,
shifts ``I_BQ_E`` to ``I_BBcm_E`` (both are expressed-in frame E). On
return, ``this`` is shifted from about-point Q to about-point ``Bcm``.

Parameter ``mass``:
    The mass of body (or composite body) B.

Parameter ``p_QBcm_E``:
    Position vector from Q to ``Bcm``, expressed-in frame E.

Raises:
    RuntimeError for Debug builds if the rotational inertia that is
    shifted to about-point ``Bcm`` violates CouldBePhysicallyValid().

Remark:
    Negating the position vector ``p_QBcm_E`` has no affect on the
    result.)""";
        } ShiftToCenterOfMassInPlace;
        // Symbol: drake::multibody::RotationalInertia::ShiftToThenAwayFromCenterOfMass
        struct /* ShiftToThenAwayFromCenterOfMass */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(Calculates the rotational inertia that results from shifting ``this``
rotational inertia for a body (or composite body) B from about-point P
to about-point Q via Bcm (B's center of mass). I.e., shifts ``I_BP_E``
to ``I_BQ_E`` (both are expressed-in frame E).

Parameter ``mass``:
    The mass of body (or composite body) B.

Parameter ``p_PBcm_E``:
    Position vector from P to Bcm, expressed-in frame E.

Parameter ``p_QBcm_E``:
    Position vector from Q to Bcm, expressed-in frame E.

Returns ``I_BQ_E``:
    , B's rotational inertia about-point Q expressed-in frame E.

Raises:
    RuntimeError for Debug builds if the rotational inertia that is
    shifted to about-point Q violates CouldBePhysicallyValid().

Remark:
    Negating either (or both) position vectors p_PBcm_E and p_QBcm_E
    has no affect on the result.)""";
        } ShiftToThenAwayFromCenterOfMass;
        // Symbol: drake::multibody::RotationalInertia::ShiftToThenAwayFromCenterOfMassInPlace
        struct /* ShiftToThenAwayFromCenterOfMassInPlace */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(Shifts ``this`` rotational inertia for a body (or composite body) B
from about-point P to about-point Q via Bcm (B's center of mass).
I.e., shifts ``I_BP_E`` to ``I_BQ_E`` (both are expressed-in frame E).
On return, ``this`` is modified to be shifted from about-point P to
about-point Q.

Parameter ``mass``:
    The mass of body (or composite body) B.

Parameter ``p_PBcm_E``:
    Position vector from P to Bcm, expressed-in frame E.

Parameter ``p_QBcm_E``:
    Position vector from Q to Bcm, expressed-in frame E.

Raises:
    RuntimeError for Debug builds if the rotational inertia that is
    shifted to about-point Q violates CouldBePhysicallyValid().

Remark:
    Negating either (or both) position vectors p_PBcm_E and p_QBcm_E
    has no affect on the result.

Remark:
    This method is more efficient (by 6 multiplications) than first
    shifting to the center of mass, then shifting away, e.g., as
    (ShiftToCenterOfMassInPlace()).ShiftFromCenterOfMassInPlace();)""";
        } ShiftToThenAwayFromCenterOfMassInPlace;
        // Symbol: drake::multibody::RotationalInertia::Trace
        struct /* Trace */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(Returns a rotational inertia's trace (i.e., Ixx + Iyy + Izz, the sum
of the diagonal elements of the inertia matrix). The trace happens to
be invariant to its expressed-in frame (i.e., the trace does not
depend on the frame in which it is expressed). The trace is useful
because the largest moment of inertia Imax has range: trace / 3 <=
Imax <= trace / 2, and the largest possible product of inertia must be
<= Imax / 2. Hence, trace / 3 and trace / 2 give a lower and upper
bound on the largest possible element that can be in a valid
rotational inertia.)""";
        } Trace;
        // Symbol: drake::multibody::RotationalInertia::TriaxiallySymmetric
        struct /* TriaxiallySymmetric */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc = R"""()""";
        } TriaxiallySymmetric;
        // Symbol: drake::multibody::RotationalInertia::cast
        struct /* cast */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(Returns a new RotationalInertia object templated on ``Scalar``
initialized from the values of ``this`` rotational inertia's entries.

Template parameter ``Scalar``:
    The scalar type on which the new rotational inertia will be
    templated.

Note:
    ``RotationalInertia<From>::cast<To>()`` creates a new
    ``RotationalInertia<To>`` from a ``RotationalInertia<From>`` but
    only if type ``To`` is constructible from type ``From``. This cast
    method works in accordance with Eigen's cast method for Eigen's
    Matrix3 that underlies this RotationalInertia. For example, Eigen
    currently allows cast from type double to AutoDiffXd, but not
    vice-versa.)""";
        } cast;
        // Symbol: drake::multibody::RotationalInertia::cols
        struct /* cols */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(For consistency with Eigen's API, the cols() method returns 3.)""";
        } cols;
        // Symbol: drake::multibody::RotationalInertia::get_moments
        struct /* get_moments */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(Returns 3-element vector with moments of inertia [Ixx, Iyy, Izz].)""";
        } get_moments;
        // Symbol: drake::multibody::RotationalInertia::get_products
        struct /* get_products */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(Returns 3-element vector with products of inertia [Ixy, Ixz, Iyz].)""";
        } get_products;
        // Symbol: drake::multibody::RotationalInertia::operator()
        struct /* operator_call */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(Const access to the ``(i, j)`` element of this rotational inertia.

Remark:
    A mutable version of operator() is intentionally absent so as to
    prevent an end-user from directly setting elements. This prevents
    the creation of a non-physical (or non-symmetric) rotational
    inertia.)""";
        } operator_call;
        // Symbol: drake::multibody::RotationalInertia::operator*
        struct /* operator_mul */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(Multiplies ``this`` rotational inertia by a nonnegative scalar (>= 0).
In debug builds, throws RuntimeError if ``nonnegative_scalar`` < 0.

Parameter ``nonnegative_scalar``:
    Nonnegative scalar which multiplies ``this``.

Returns:
    ``this`` rotational inertia multiplied by ``nonnegative_scalar``.

See also:
    operator*=(), operator*(const T&, const RotationalInertia<T>&))""";
        } operator_mul;
        // Symbol: drake::multibody::RotationalInertia::operator*=
        struct /* operator_imul */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(Multiplies ``this`` rotational inertia by a nonnegative scalar (>= 0).
In debug builds, throws RuntimeError if ``nonnegative_scalar`` < 0.

Parameter ``nonnegative_scalar``:
    Nonnegative scalar which multiplies ``this``.

Returns:
    A reference to ``this`` rotational inertia. ``this`` changes since
    ``this`` has been multiplied by ``nonnegative_scalar``.

See also:
    operator*(), operator*(const T&, const RotationalInertia<T>&).)""";
        } operator_imul;
        // Symbol: drake::multibody::RotationalInertia::operator+
        struct /* operator_add */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(Adds a rotational inertia ``I_BP_E`` to ``this`` rotational inertia.
This method requires both rotational inertias (``I_BP_E`` and
``this``) to have the same about-point P and the same expressed-in
frame E.

Parameter ``I_BP_E``:
    Rotational inertia of a body (or composite body) B to be added to
    ``this`` rotational inertia. ``I_BP_E`` and ``this`` must have the
    same about-point P and expressed-in frame E.

Returns:
    The sum of ``this`` rotational inertia and ``I_BP_E``.

See also:
    operator+=().)""";
        } operator_add;
        // Symbol: drake::multibody::RotationalInertia::operator+=
        struct /* operator_iadd */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc = R"""()""";
        } operator_iadd;
        // Symbol: drake::multibody::RotationalInertia::operator-
        struct /* operator_sub */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(Subtracts a rotational inertia ``I_BP_E`` from ``this`` rotational
inertia. This method requires both rotational inertias (``I_BP_E`` and
``this``) to have the same about-point P and the same expressed-in
frame E.

Parameter ``I_BP_E``:
    Rotational inertia of a body (or composite body) B to be
    subtracted from ``this`` rotational inertia. ``I_BP_E`` and
    ``this`` must have the same about-point P and expressed-in frame
    E.

Returns:
    The subtraction of ``I_BP_E`` from ``this`` rotational inertia.

Raises:
    RuntimeError for Debug builds if not CouldBePhysicallyValid().

See also:
    operator-=().

Warning:
    See warning and documentation for operator-=().)""";
        } operator_sub;
        // Symbol: drake::multibody::RotationalInertia::operator-=
        struct /* operator_isub */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(Subtracts a rotational inertia ``I_BP_E`` from ``this`` rotational
inertia. This method requires both rotational inertias (``I_BP_E`` and
``this``) to have the same about-point P and the same expressed-in
frame E. The -= operator updates ``this`` so ``I_BP_E`` is subtracted
from ``this``.

Parameter ``I_BP_E``:
    Rotational inertia of a body (or composite body) B to be
    subtracted from ``this`` rotational inertia. ``I_BP_E`` and
    ``this`` must have the same about-point P and expressed-in frame
    E.

Returns:
    A reference to ``this`` rotational inertia. ``this`` changes since
    rotational inertia ``I_BP_E`` has been subtracted from it.

Raises:
    RuntimeError for Debug builds if not CouldBePhysicallyValid().

See also:
    operator-().

Note:
    This subtract operator is useful for computing rotational inertia
    of a body with a hole. First the rotational inertia of a fully
    solid body S (without the hole) is calculated, then the rotational
    inertia of the hole (treated as a massive solid body B) is
    calculated. The rotational inertia of a composite body C
    (comprised of S and -B) is computed by subtracting B's rotational
    inertia from S's rotational inertia.)""";
        } operator_isub;
        // Symbol: drake::multibody::RotationalInertia::operator/
        struct /* operator_div */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc = R"""()""";
        } operator_div;
        // Symbol: drake::multibody::RotationalInertia::operator/=
        struct /* operator_idiv */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(Divides ``this`` rotational inertia by a positive scalar (> 0). In
debug builds, throws RuntimeError if ``positive_scalar`` <= 0.

Parameter ``positive_scalar``:
    Positive scalar (> 0) which divides ``this``.

Returns:
    A reference to ``this`` rotational inertia. ``this`` changes since
    ``this`` has been divided by ``positive_scalar``.

See also:
    operator/().)""";
        } operator_idiv;
        // Symbol: drake::multibody::RotationalInertia::rows
        struct /* rows */ {
          // Source: drake/multibody/tree/rotational_inertia.h
          const char* doc =
R"""(For consistency with Eigen's API, the rows() method returns 3.)""";
        } rows;
      } RotationalInertia;
      // Symbol: drake::multibody::RpyFloatingJoint
      struct /* RpyFloatingJoint */ {
        // Source: drake/multibody/tree/rpy_floating_joint.h
        const char* doc =
R"""(This Joint allows a rigid body to move freely with respect to its
parent rigid body. This is most commonly used to allow a body to move
freely with respect to the World, but can be used with any parent.
More precisely, given a frame F attached to the parent body P and a
frame M attached to the child body B (see the Joint class's
documentation), this joint allows frame M to translate and rotate
freely with respect to F, introducing six degrees of freedom. However,
unlike the QuaternionFloatingJoint, the orientation of M relative to F
is parameterized with roll-pitch-yaw angles (see warning below). The
generalized coordinates q for this joint are the three orientation
angles followed by three generalized positions to describe the
translation of frame M's origin Mo in F with a position vector
``p_FM``. As generalized velocities, we use the angular velocity
``w_FM`` of frame M in F (*not* the orientation angle time derivatives
q̇) and the linear velocity ``v_FM`` of frame M's origin Mo in frame
F.

Warning:
    Any three-parameter representation of orientation necessarily has
    a singularity somewhere. In this case, the singularity occurs when
    the pitch angle (second generalized coordinate q) is at π/2 + kπ
    (for any integer k), and numerical issues may occur when near one
    of those configurations. If you can't be sure your simulation will
    avoid the singularities, consider using the singularity-free
    QuaternionFloatingJoint instead.)""";
        // Symbol: drake::multibody::RpyFloatingJoint::DoAddInDamping
        struct /* DoAddInDamping */ {
          // Source: drake/multibody/tree/rpy_floating_joint.h
          const char* doc =
R"""(Joint<T> override called through public NVI, Joint::AddInDamping().
Therefore arguments were already checked to be valid. This method adds
into the translational component of ``forces`` for ``this`` joint a
dissipative force according to the viscous law ``f = -d⋅v``, with d
the damping coefficient (see default_translational_damping()). This
method also adds into the angular component of ``forces`` for ``this``
joint a dissipative torque according to the viscous law ``τ = -d⋅ω``,
with d the damping coefficient (see default_angular_damping()).)""";
        } DoAddInDamping;
        // Symbol: drake::multibody::RpyFloatingJoint::DoAddInOneForce
        struct /* DoAddInOneForce */ {
          // Source: drake/multibody/tree/rpy_floating_joint.h
          const char* doc =
R"""(Joint<T> override called through public NVI, Joint::AddInForce().
Adding forces per-dof for this joint is not supported. Therefore, this
method throws an exception if invoked.)""";
        } DoAddInOneForce;
        // Symbol: drake::multibody::RpyFloatingJoint::GetPose
        struct /* GetPose */ {
          // Source: drake/multibody/tree/rpy_floating_joint.h
          const char* doc =
R"""(Returns the pose ``X_FM`` of the outboard frame M as measured and
expressed in the inboard frame F. Refer to the documentation for this
class for details.

Parameter ``context``:
    A Context for the MultibodyPlant this joint belongs to.

Returns ``X_FM``:
    The pose of frame M in frame F.)""";
        } GetPose;
        // Symbol: drake::multibody::RpyFloatingJoint::RpyFloatingJoint<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/rpy_floating_joint.h
          const char* doc =
R"""(Constructor to create an rpy floating joint between two bodies so that
frame F attached to the parent body P and frame M attached to the
child body B move freely relative to one another. See this class's
documentation for further details on the definition of these frames
and the generalized positions q and generalized velocities v for this
joint. This constructor signature creates a joint with no joint
limits, i.e. the joint position, velocity and acceleration limits are
the pair ``(-∞, ∞)``. These can be set using the Joint methods
set_position_limits(), set_velocity_limits() and
set_acceleration_limits().

The first three arguments to this constructor are those of the Joint
class constructor. See the Joint class's documentation for details.
The additional parameters are:

Parameter ``angular_damping``:
    Viscous damping coefficient in N⋅m⋅s for the angular component of
    this joint's velocity, used to model losses within the joint. See
    documentation of default_angular_damping() for details on modeling
    of the damping force.

Parameter ``translational_damping``:
    Viscous damping coefficient in N⋅s/m for the translational
    component of this joint's velocity, used to model losses within
    the joint. See documentation of default_translational_damping()
    for details on modeling of the damping force.

Raises:
    RuntimeError if angular_damping is negative.

Raises:
    RuntimeError if translational_damping is negative.)""";
        } ctor;
        // Symbol: drake::multibody::RpyFloatingJoint::SetOrientation
        struct /* SetOrientation */ {
          // Source: drake/multibody/tree/rpy_floating_joint.h
          const char* doc =
R"""(Sets the roll-pitch-yaw angles in ``context`` so this Joint's
orientation is consistent with the given ``R_FM`` rotation matrix.

Parameter ``context``:
    A Context for the MultibodyPlant this joint belongs to.

Parameter ``R_FM``:
    The rotation matrix giving the orientation of frame M in frame F.

Warning:
    See class documentation for discussion of singular configurations.

Returns:
    a constant reference to this joint.)""";
        } SetOrientation;
        // Symbol: drake::multibody::RpyFloatingJoint::SetPose
        struct /* SetPose */ {
          // Source: drake/multibody/tree/rpy_floating_joint.h
          const char* doc =
R"""(Sets ``context`` to store ``X_FM`` the pose of frame M measured and
expressed in frame F.

Parameter ``context``:
    A Context for the MultibodyPlant this joint belongs to.

Parameter ``X_FM``:
    The desired pose of frame M in frame F to be stored in
    ``context``.

Warning:
    See class documentation for discussion of singular configurations.

Returns:
    a constant reference to ``this`` joint.)""";
        } SetPose;
        // Symbol: drake::multibody::RpyFloatingJoint::SetTranslation
        struct /* SetTranslation */ {
          // Source: drake/multibody/tree/rpy_floating_joint.h
          const char* doc =
R"""(Sets ``context`` to store the translation (position vector) ``p_FM``
of frame M's origin Mo measured and expressed in frame F.

Parameter ``context``:
    A Context for the MultibodyPlant this joint belongs to.

Parameter ``p_FM``:
    The desired position of frame M's origin in frame F, to be stored
    in ``context``.

Returns:
    a constant reference to this joint.)""";
        } SetTranslation;
        // Symbol: drake::multibody::RpyFloatingJoint::default_angular_damping
        struct /* default_angular_damping */ {
          // Source: drake/multibody/tree/rpy_floating_joint.h
          const char* doc =
R"""(Returns this joint's default angular damping constant in N⋅m⋅s. The
damping torque (in N⋅m) is modeled as ``τ = -damping⋅ω``, i.e.
opposing motion, with ω the angular velocity of frame M in F (see
get_angular_velocity()) and τ the torque on child body B (to which M
is rigidly attached).)""";
        } default_angular_damping;
        // Symbol: drake::multibody::RpyFloatingJoint::default_translational_damping
        struct /* default_translational_damping */ {
          // Source: drake/multibody/tree/rpy_floating_joint.h
          const char* doc =
R"""(Returns this joint's default translational damping constant in N⋅s/m.
The damping force (in N) is modeled as ``f = -damping⋅v`` i.e.
opposing motion, with v the translational velocity of frame M in F
(see get_translational_velocity()) and f the force on child body B at
Mo.)""";
        } default_translational_damping;
        // Symbol: drake::multibody::RpyFloatingJoint::get_angles
        struct /* get_angles */ {
          // Source: drake/multibody/tree/rpy_floating_joint.h
          const char* doc =
R"""(Gets the roll-pitch-yaw rotation angles of this joint from
``context``.

The orientation ``R_FM`` of the child frame M in parent frame F is
parameterized with roll-pitch-yaw angles (also known as space-fixed
x-y-z Euler angles and extrinsic angles). That is, the angles θr, θp,
θy, correspond to a sequence of rotations about the Fx, Fy, and Fz
axes of parent frame F, respectively. Mathematically, rotation
``R_FM`` is given in terms of angles θr, θp, θy by:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    R_FM(q) = Rz(θy) * Ry(θp) * Rx(θr)

.. raw:: html

    </details>

where ``Rx(θ)``, `Ry(θ)` and ``Rz(θ)`` correspond to the elemental
rotations in amount of θ about the Fx, Fy and Fz axes respectively.
Zero θr, θp, θy angles corresponds to frames F and M having the same
orientation (their origins may still be separated). Angles θr, θp, θy
are defined to be positive according to the right-hand-rule with the
thumb aligned in the direction of their respective axes.

Note:
    Space ``x-y-z`` angles (extrinsic) are equivalent to Body
    ``z-y-x`` angles (intrinsic).

Parameter ``context``:
    A Context for the MultibodyPlant this joint belongs to.

Returns ``angles``:
    The angle coordinates of this joint stored in the ``context``
    ordered as θr, θp, θy.)""";
        } get_angles;
        // Symbol: drake::multibody::RpyFloatingJoint::get_angular_velocity
        struct /* get_angular_velocity */ {
          // Source: drake/multibody/tree/rpy_floating_joint.h
          const char* doc =
R"""(Retrieves from ``context`` the angular velocity ``w_FM`` of the child
frame M in the parent frame F, expressed in F.

Parameter ``context``:
    A Context for the MultibodyPlant this joint belongs to.

Returns ``w_FM``:
    A vector in ℝ³ with the angular velocity of the child frame M in
    the parent frame F, expressed in F. Refer to this class's
    documentation for further details and definitions of these frames.)""";
        } get_angular_velocity;
        // Symbol: drake::multibody::RpyFloatingJoint::get_default_angles
        struct /* get_default_angles */ {
          // Source: drake/multibody/tree/rpy_floating_joint.h
          const char* doc =
R"""(Gets the default angles for this joint.

Returns ``angles``:
    The default roll-pitch-yaw angles as a Vector3.)""";
        } get_default_angles;
        // Symbol: drake::multibody::RpyFloatingJoint::get_default_translation
        struct /* get_default_translation */ {
          // Source: drake/multibody/tree/rpy_floating_joint.h
          const char* doc =
R"""(Gets the default translation (position vector) ``p_FM`` for this
joint.

Returns ``p_FM``:
    The default translation of this joint.)""";
        } get_default_translation;
        // Symbol: drake::multibody::RpyFloatingJoint::get_translation
        struct /* get_translation */ {
          // Source: drake/multibody/tree/rpy_floating_joint.h
          const char* doc =
R"""(Returns the translation (position vector) ``p_FM`` of the child frame
M's origin Mo as measured and expressed in the parent frame F. Refer
to the class documentation for details.

Parameter ``context``:
    A Context for the MultibodyPlant this joint belongs to.

Returns ``p_FM``:
    The position vector of frame M's origin in frame F.)""";
        } get_translation;
        // Symbol: drake::multibody::RpyFloatingJoint::get_translational_velocity
        struct /* get_translational_velocity */ {
          // Source: drake/multibody/tree/rpy_floating_joint.h
          const char* doc =
R"""(Retrieves from ``context`` the translational velocity ``v_FM`` of the
child frame M's origin as measured and expressed in the parent frame
F.

Parameter ``context``:
    A Context for the MultibodyPlant this joint belongs to.

Returns ``v_FM``:
    A vector in ℝ³ with the translational velocity of the origin of
    child frame M in the parent frame F, expressed in F. Refer to this
    class's documentation for further details and definitions of these
    frames.)""";
        } get_translational_velocity;
        // Symbol: drake::multibody::RpyFloatingJoint::set_angles
        struct /* set_angles */ {
          // Source: drake/multibody/tree/rpy_floating_joint.h
          const char* doc =
R"""(Sets the ``context`` so that the generalized coordinates corresponding
to the roll-pitch-yaw rotation angles of this joint equals ``angles``.

Parameter ``context``:
    A Context for the MultibodyPlant this joint belongs to.

Parameter ``angles``:
    Angles in radians to be stored in ``context`` ordered as θr, θp,
    θy.

Warning:
    See class documentation for discussion of singular configurations.

Returns:
    a constant reference to this joint.

See also:
    get_angles() for details)""";
        } set_angles;
        // Symbol: drake::multibody::RpyFloatingJoint::set_angular_velocity
        struct /* set_angular_velocity */ {
          // Source: drake/multibody/tree/rpy_floating_joint.h
          const char* doc =
R"""(Sets in ``context`` the state for this joint so that the angular
velocity of the child frame M in the parent frame F is ``w_FM``.

Parameter ``context``:
    A Context for the MultibodyPlant this joint belongs to.

Parameter ``w_FM``:
    A vector in ℝ³ with the angular velocity of the child frame M in
    the parent frame F, expressed in F. Refer to this class's
    documentation for further details and definitions of these frames.

Returns:
    a constant reference to this joint.)""";
        } set_angular_velocity;
        // Symbol: drake::multibody::RpyFloatingJoint::set_default_angles
        struct /* set_default_angles */ {
          // Source: drake/multibody/tree/rpy_floating_joint.h
          const char* doc =
R"""(Sets the default roll-pitch-yaw angles of this joint.

Parameter ``angles``:
    the desired default angles of the joint

Warning:
    See class documentation for discussion of singular configurations.)""";
        } set_default_angles;
        // Symbol: drake::multibody::RpyFloatingJoint::set_default_translation
        struct /* set_default_translation */ {
          // Source: drake/multibody/tree/rpy_floating_joint.h
          const char* doc =
R"""(Sets the default translation (position vector) ``p_FM`` of this joint.

Parameter ``p_FM``:
    The desired default translation of the joint.)""";
        } set_default_translation;
        // Symbol: drake::multibody::RpyFloatingJoint::set_random_angles_distribution
        struct /* set_random_angles_distribution */ {
          // Source: drake/multibody/tree/rpy_floating_joint.h
          const char* doc =
R"""(Sets the random distribution from which the roll-pitch-yaw orientation
angles of this joint will be randomly sampled. See the
RpyFloatingJoint class documentation for details on the orientation
representation. If a translation distribution has already been set
with stochastic variables, it will remain so. Otherwise translation
will be set to this joint's zero configuration.

Warning:
    Watch for random pitch angles near the singular configuration for
    this joint (see class documentation).)""";
        } set_random_angles_distribution;
        // Symbol: drake::multibody::RpyFloatingJoint::set_random_translation_distribution
        struct /* set_random_translation_distribution */ {
          // Source: drake/multibody/tree/rpy_floating_joint.h
          const char* doc =
R"""(Sets the random distribution that the translation vector of this joint
will be randomly sampled from. See the RpyFloatingJoint class
documentation for details on the translation representation. If an
angles distribution has has already been set with stochastic
variables, it will remain so. Otherwise angles will be set to this
joint's zero configuration.)""";
        } set_random_translation_distribution;
        // Symbol: drake::multibody::RpyFloatingJoint::set_translational_velocity
        struct /* set_translational_velocity */ {
          // Source: drake/multibody/tree/rpy_floating_joint.h
          const char* doc =
R"""(Sets in ``context`` the state for this joint so that the translational
velocity of the child frame M's origin in the parent frame F is
``v_FM``.

Parameter ``context``:
    A Context for the MultibodyPlant this joint belongs to.

Parameter ``v_FM``:
    A vector in ℝ³ with the translational velocity of the child frame
    M's origin in the parent frame F, expressed in F. Refer to this
    class's documentation for further details and definitions of these
    frames.

Returns:
    a constant reference to this joint.)""";
        } set_translational_velocity;
        // Symbol: drake::multibody::RpyFloatingJoint::type_name
        struct /* type_name */ {
          // Source: drake/multibody/tree/rpy_floating_joint.h
          const char* doc =
R"""(Returns the name of this joint type: "rpy_floating")""";
        } type_name;
      } RpyFloatingJoint;
      // Symbol: drake::multibody::ScopedName
      struct /* ScopedName */ {
        // Source: drake/multibody/tree/scoped_name.h
        const char* doc =
R"""(A delimited string name for a multibody element, e.g.,
"robot1::torso".

The name is composed of two semantically separate pieces -- the
``element`` name is the local name for the element (e.g., a joint,
body, frame, etc.) and the ``namespace`` name is the location of that
element within the tree. For "robot1::torso" the namespace name is
"robot1" and the element name is "torso".

The namespace name typically refers to the model instance name that
contains the element. Some temporary scoped names do not use a
namespace (e.g., temporary values created during input file parsing),
in which case the namespace name can be empty. The namespace name will
never start or end with "::".

The element name is never empty, unless the ScopedName was
default-constructed or moved-from. The element name will never contain
the delimiter string "::".

When there is no namespace, the scoped name does not contain a leading
"::", e.g., for the element name "box" without any namespace, the
scoped name is "box" not "::box".

The namespace name may contain the "::" delimiter in the middle of the
name (possibly multiple times), e.g., for
"robot1::left::arm::end_frame" the namespace name is
"robot1::left::arm" and the element is name "end_frame".

This class does not treat a single colon (":") specially. Those can
appear in either namespace names or element names.)""";
        // Symbol: drake::multibody::ScopedName::Join
        struct /* Join */ {
          // Source: drake/multibody/tree/scoped_name.h
          const char* doc =
R"""(Creates a ScopedName for the given ``name1::name2``. Unlike the
constructor or ScopedName::Make(), this function allows "::" in either
name. Any leading or trailing "::" on the names are removed before
joining. After joining, the final word after all "::"s is the element
name, and everything prior is the namespace name.)""";
        } Join;
        // Symbol: drake::multibody::ScopedName::Make
        struct /* Make */ {
          // Source: drake/multibody/tree/scoped_name.h
          const char* doc =
R"""(Creates a ScopedName for the given ``namespace_name`` and
``element_name``. Returns nullopt if ``namespace_name`` starts or ends
with "::", or if ``element_name`` contains "::" or is empty.

See also:
    ScopedName::Join() for automatic coalescing of "::" tokens.)""";
        } Make;
        // Symbol: drake::multibody::ScopedName::Parse
        struct /* Parse */ {
          // Source: drake/multibody/tree/scoped_name.h
          const char* doc =
R"""(Parses the given ``scoped_name`` string. Any leading or trailing "::"s
on the name are removed (even multiple copies like "::::" are
removed).)""";
        } Parse;
        // Symbol: drake::multibody::ScopedName::ScopedName
        struct /* ctor */ {
          // Source: drake/multibody/tree/scoped_name.h
          const char* doc_0args = R"""(Creates an empty name.)""";
          // Source: drake/multibody/tree/scoped_name.h
          const char* doc_2args =
R"""(Creates a ScopedName for the given ``namespace_name`` and
``element_name``.

Raises:
    RuntimeError if ``namespace_name`` starts or ends with "::".

Raises:
    RuntimeError if ``element_name`` contains "::" or is empty.

See also:
    ScopedName::Make() to use a null return value instead of
    exceptions.

See also:
    ScopedName::Join() for automatic cleanup of "::" tokens.)""";
        } ctor;
        // Symbol: drake::multibody::ScopedName::get_element
        struct /* get_element */ {
          // Source: drake/multibody/tree/scoped_name.h
          const char* doc =
R"""(Returns the element portion of this scoped name, e.g., "torso". This
is the local name of the joint, body, etc. within the model instance.
It is never empty unless this ScopedName was default-constructed or
moved-from.)""";
        } get_element;
        // Symbol: drake::multibody::ScopedName::get_full
        struct /* get_full */ {
          // Source: drake/multibody/tree/scoped_name.h
          const char* doc =
R"""(Returns the full ScopedName as a string, e.g., "robot1::torso". It is
never empty unless this ScopedName was default-constructed or
moved-from.)""";
        } get_full;
        // Symbol: drake::multibody::ScopedName::get_namespace
        struct /* get_namespace */ {
          // Source: drake/multibody/tree/scoped_name.h
          const char* doc =
R"""(Returns the namespace portion of this scoped name, e.g., "robot1".
This is typically the model instance name. This is typically the model
instance name but can be empty (see class overview for details).)""";
        } get_namespace;
        // Symbol: drake::multibody::ScopedName::set_element
        struct /* set_element */ {
          // Source: drake/multibody/tree/scoped_name.h
          const char* doc =
R"""(Replaces the element name of this object, leaving the namespace name
unchanged.

Raises:
    RuntimeError if ``element_name`` contains "::" or is empty.)""";
        } set_element;
        // Symbol: drake::multibody::ScopedName::set_namespace
        struct /* set_namespace */ {
          // Source: drake/multibody/tree/scoped_name.h
          const char* doc =
R"""(Replaces the namespace name of this object, leaving the element name
unchanged. The namespace name is allowed to be empty.

Raises:
    RuntimeError if ``namespace_name`` starts or ends with "::".)""";
        } set_namespace;
        // Symbol: drake::multibody::ScopedName::to_string
        struct /* to_string */ {
          // Source: drake/multibody/tree/scoped_name.h
          const char* doc =
R"""(Returns get_full() as a string value instead of a string_view.)""";
        } to_string;
      } ScopedName;
      // Symbol: drake::multibody::ScrewJoint
      struct /* ScrewJoint */ {
        // Source: drake/multibody/tree/screw_joint.h
        const char* doc =
R"""(This joint models a screw joint allowing two bodies to rotate about
one axis while translating along that same axis with one degree of
freedom. That is, given a frame F attached to the parent body P and a
frame M attached to the child body B (see the Joint class's
documentation), this joint allows frame M to translate (while
rotating) along an axis â. Axis â is constant and has the same
measures in both frames F and M, that is, ``â_F = â_M``. The
rotation about the ``â_F`` axis and its rate specify the state of the
joint. Zero (θ) corresponds to frames F and M being coincident and
aligned. The translation distance is defined positive when child body
B translates along the direction of â, and the rotation θ is defined
to be positive according to the right-hand-rule with the thumb aligned
in the direction of the ``â_F`` axis.)""";
        // Symbol: drake::multibody::ScrewJoint::GetDamping
        struct /* GetDamping */ {
          // Source: drake/multibody/tree/screw_joint.h
          const char* doc =
R"""(Returns the Context dependent damping coefficient stored as a
parameter in ``context``. Refer to default_damping() for details.

Parameter ``context``:
    The context storing the state and parameters for the model to
    which ``this`` joint belongs.)""";
        } GetDamping;
        // Symbol: drake::multibody::ScrewJoint::ScrewJoint<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/screw_joint.h
          const char* doc_5args =
R"""(Constructor to create a screw joint between two bodies so that frame F
attached to the parent body P and frame M attached to the child body B
translate and rotate as described in the class's documentation. This
constructor signature creates a joint with the axis â set to the
z-axis and no joint limits, i.e. the joint angular position, angular
velocity and angular acceleration limits are the pair ``(-∞, ∞)``.
These can be set using the Joint methods set_position_limits(),
set_velocity_limits() and set_acceleration_limits() in radians,
radians/s, radians/s² units. The first three arguments to this
constructor are those of the Joint class constructor. See the Joint
class's documentation for details. The additional parameters are:

Parameter ``screw_pitch``:
    Amount of translation in meters occurring over a one full screw
    revolution. It's domain is (-∞, ∞). When the screw pitch is
    negative, positive rotation will result in translating towards the
    negative direction of z-axis. When the screw pitch is zero, this
    joint will behave like a revolute joint.

Parameter ``damping``:
    Viscous damping coefficient, N⋅m⋅s/rad for rotation, used to model
    losses within the joint. See documentation of default_damping()
    for details on modelling of the damping torque.

Raises:
    RuntimeError if damping is negative.)""";
          // Source: drake/multibody/tree/screw_joint.h
          const char* doc_6args =
R"""(Constructor to create a screw joint between two bodies so that frame F
attached to the parent body P and frame M attached to the child body B
translate and rotate as described in the class's documentation. This
constructor signature creates a joint with no joint limits, i.e. the
joint angular position, angular velocity and angular acceleration
limits are the pair ``(-∞, ∞)``. These can be set using the Joint
methods set_position_limits(), set_velocity_limits() and
set_acceleration_limits() in radians, radians/s, radians/s² units. The
first three arguments to this constructor are those of the Joint class
constructor. See the Joint class's documentation for details. The
additional parameters are:

Parameter ``axis``:
    A vector in ℝ³ specifying the axis of motion for this joint. The
    coordinates of ``axis`` expressed in frames F and M are the same
    at all times, that is, ``axis_F = axis_M``. In other words,
    ``axis_F`` (or ``axis_M``) is the eigenvector of ``R_FM`` with
    eigenvalue equal to one. This vector can have any length, only the
    direction is used.

Parameter ``screw_pitch``:
    Amount of translation in meters occurring over a one full screw
    revolution. It's domain is (-∞, ∞). When the screw pitch is
    negative, positive rotation will result in translating towards the
    negative direction of â-axis. When the screw pitch is zero, this
    joint will behave like a revolute joint.

Parameter ``damping``:
    Viscous damping coefficient, N⋅m⋅s/rad for rotation, used to model
    losses within the joint. See documentation of default_damping()
    for details on modelling of the damping torque.

Raises:
    RuntimeError if the L2 norm of ``axis`` is less than the square
    root of machine epsilon.

Raises:
    RuntimeError if damping is negative.)""";
        } ctor;
        // Symbol: drake::multibody::ScrewJoint::SetDamping
        struct /* SetDamping */ {
          // Source: drake/multibody/tree/screw_joint.h
          const char* doc =
R"""(Sets the value of the viscous damping coefficient for this joint,
stored as a parameter in ``context``. Refer to default_damping() for
details.

Parameter ``context``:
    The context storing the state and parameters for the model to
    which ``this`` joint belongs.

Parameter ``damping``:
    The damping value.

Raises:
    RuntimeError if ``damping`` is negative.)""";
        } SetDamping;
        // Symbol: drake::multibody::ScrewJoint::default_damping
        struct /* default_damping */ {
          // Source: drake/multibody/tree/screw_joint.h
          const char* doc =
R"""(Returns ``this`` joint's default damping constant N⋅m⋅s for the
rotational degree. The damping torque (in N⋅m) is modeled as ``τ =
-damping⋅ω`` i.e. opposing motion, with ω the angular rate for
``this`` joint (see get_angular_velocity()) and τ the torque on child
body B expressed in frame F as t_B_F = τ⋅Fâ_F.)""";
        } default_damping;
        // Symbol: drake::multibody::ScrewJoint::get_angular_velocity
        struct /* get_angular_velocity */ {
          // Source: drake/multibody/tree/screw_joint.h
          const char* doc =
R"""(Gets the rate of change, in radians per second, of ``this`` joint's
angle θ from ``context``. See class documentation for the definition
of this angle.

Parameter ``context``:
    The context of the model this joint belongs to.

Returns ``theta_dot``:
    The rate of change of ``this`` joint's angle θ as stored in the
    ``context``.)""";
        } get_angular_velocity;
        // Symbol: drake::multibody::ScrewJoint::get_default_rotation
        struct /* get_default_rotation */ {
          // Source: drake/multibody/tree/screw_joint.h
          const char* doc =
R"""(Gets the default angle for ``this`` joint.

Returns ``theta``:
    The default angle of ``this`` joint.)""";
        } get_default_rotation;
        // Symbol: drake::multibody::ScrewJoint::get_default_translation
        struct /* get_default_translation */ {
          // Source: drake/multibody/tree/screw_joint.h
          const char* doc =
R"""(Gets the default position for ``this`` joint.

Returns ``z``:
    The default position of ``this`` joint.)""";
        } get_default_translation;
        // Symbol: drake::multibody::ScrewJoint::get_rotation
        struct /* get_rotation */ {
          // Source: drake/multibody/tree/screw_joint.h
          const char* doc =
R"""(Gets the angle θ of ``this`` joint from ``context``.

Parameter ``context``:
    The context of the model this joint belongs to.

Returns ``theta``:
    The angle of ``this`` joint stored in the ``context``. See class
    documentation for details.)""";
        } get_rotation;
        // Symbol: drake::multibody::ScrewJoint::get_translation
        struct /* get_translation */ {
          // Source: drake/multibody/tree/screw_joint.h
          const char* doc =
R"""(Gets the translation of ``this`` joint from ``context``.

Parameter ``context``:
    The context of the model this joint belongs to.

Returns ``z``:
    The translation of ``this`` joint stored in the ``context`` as
    (z). See class documentation for details.)""";
        } get_translation;
        // Symbol: drake::multibody::ScrewJoint::get_translational_velocity
        struct /* get_translational_velocity */ {
          // Source: drake/multibody/tree/screw_joint.h
          const char* doc =
R"""(Gets the translational velocity vz, in meters per second, of ``this``
joint's Mo measured and expressed in frame F from ``context``.

Parameter ``context``:
    The context of the model this joint belongs to.

Returns ``vz``:
    The translational velocity of ``this`` joint as stored in the
    ``context``.)""";
        } get_translational_velocity;
        // Symbol: drake::multibody::ScrewJoint::screw_axis
        struct /* screw_axis */ {
          // Source: drake/multibody/tree/screw_joint.h
          const char* doc =
R"""(Returns the normalized axis of motion of ``this`` joint as a unit
vector. Since the measures of this axis in either frame F or M are the
same (see this class's documentation for frame definitions) then,
``axis = axis_F = axis_M``.)""";
        } screw_axis;
        // Symbol: drake::multibody::ScrewJoint::screw_pitch
        struct /* screw_pitch */ {
          // Source: drake/multibody/tree/screw_joint.h
          const char* doc =
R"""(Returns ``this`` joint's amount of translation in meters occurring
over a one full revolution.)""";
        } screw_pitch;
        // Symbol: drake::multibody::ScrewJoint::set_angular_velocity
        struct /* set_angular_velocity */ {
          // Source: drake/multibody/tree/screw_joint.h
          const char* doc =
R"""(Sets the rate of change, in radians per second, of ``this`` joint's
angle θ (see class documentation) to ``theta_dot``. The new rate of
change gets stored in ``context``.

Parameter ``context``:
    The context of the model this joint belongs to.

Parameter ``theta_dot``:
    The desired rates of change of ``this`` joint's angle in radians
    per second.

Returns:
    a constant reference to ``this`` joint.)""";
        } set_angular_velocity;
        // Symbol: drake::multibody::ScrewJoint::set_default_rotation
        struct /* set_default_rotation */ {
          // Source: drake/multibody/tree/screw_joint.h
          const char* doc =
R"""(Sets the default angle of this joint. This will change the
``default_translation`` too, because they are not independent in this
joint.

Parameter ``theta``:
    The desired default angle of the joint)""";
        } set_default_rotation;
        // Symbol: drake::multibody::ScrewJoint::set_default_translation
        struct /* set_default_translation */ {
          // Source: drake/multibody/tree/screw_joint.h
          const char* doc =
R"""(Sets the default translation of this joint. This will change the
``default_rotation`` too, which are not independent in this joint.

Parameter ``z``:
    The desired default translation of the joint

Raises:
    RuntimeError if pitch is very near zero.)""";
        } set_default_translation;
        // Symbol: drake::multibody::ScrewJoint::set_random_pose_distribution
        struct /* set_random_pose_distribution */ {
          // Source: drake/multibody/tree/screw_joint.h
          const char* doc =
R"""(Sets the random distribution that the angle of this joint will be
randomly sampled from. See class documentation for details on the
definition of the position and angle.)""";
        } set_random_pose_distribution;
        // Symbol: drake::multibody::ScrewJoint::set_rotation
        struct /* set_rotation */ {
          // Source: drake/multibody/tree/screw_joint.h
          const char* doc =
R"""(Sets the ``context`` so that the angle θ of ``this`` joint equals
``theta``.

Parameter ``context``:
    The context of the model this joint belongs to.

Parameter ``theta``:
    The desired angle in radians to be stored in ``context``. See
    class documentation for details.

Returns:
    a constant reference to ``this`` joint.)""";
        } set_rotation;
        // Symbol: drake::multibody::ScrewJoint::set_translation
        struct /* set_translation */ {
          // Source: drake/multibody/tree/screw_joint.h
          const char* doc =
R"""(Sets the ``context`` so that the translation of ``this`` joint equals
to (z).

Parameter ``context``:
    The context of the model this joint belongs to.

Parameter ``z``:
    The desired translation in meters to be stored in ``context`` as
    (z). See class documentation for details.

Returns:
    a constant reference to ``this`` joint.)""";
        } set_translation;
        // Symbol: drake::multibody::ScrewJoint::set_translational_velocity
        struct /* set_translational_velocity */ {
          // Source: drake/multibody/tree/screw_joint.h
          const char* doc =
R"""(Sets the translational velocity, in meters per second, of this
``this`` joint's Mo along frame F's â-axis to ``vz``. The new
translational velocity gets stored in ``context``.

Parameter ``context``:
    The context of the model this joint belongs to.

Parameter ``vz``:
    The desired translational velocity of ``this`` joint in meters per
    second along F frame's â-axis.

Returns:
    a constant reference to ``this`` joint.)""";
        } set_translational_velocity;
        // Symbol: drake::multibody::ScrewJoint::type_name
        struct /* type_name */ {
          // Source: drake/multibody/tree/screw_joint.h
          const char* doc = R"""()""";
        } type_name;
      } ScrewJoint;
      // Symbol: drake::multibody::SpatialInertia
      struct /* SpatialInertia */ {
        // Source: drake/multibody/tree/spatial_inertia.h
        const char* doc =
R"""(This class represents the physical concept of a *Spatial Inertia*. A
spatial inertia (or spatial mass matrix) encapsulates the mass, center
of mass, and rotational inertia of the mass distribution of a body or
composite body S, where with "composite body" we mean a collection of
bodies welded together containing at least one body (throughout this
documentation "body" is many times used instead of "composite body"
but the same concepts apply to a collection of bodies as well.) A
spatial inertia is an element of ℝ⁶ˣ⁶ that is symmetric, and positive
semi-definite. It logically consists of ``3x3`` sub-matrices arranged
like so, [Jain 2010]:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    Spatial mass matrix
              ------------ ------------
           0 |            |            |
           1 |    I_SP    | m p_PScm×  |
           2 |            |            |
              ------------ ------------
           3 |            |            |
           4 | -m p_PScm× |     m Id   |
           5 |            |            |
              ------------ ------------
                   Symbol: M

.. raw:: html

    </details>

where, with the monogram notation described in
multibody_spatial_inertia, ``I_SP`` is the rotational inertia of body
or composite body S computed about a point P, m is the mass of this
composite body, ``p_PScm`` is the position vector from point P to the
center of mass ``Scm`` of the composite body S with ``p_PScm×``
denoting its skew-symmetric cross product matrix (defined such that
``a× b = a.cross(b)``), and ``Id`` is the identity matrix in ℝ³ˣ³. See
Section 2.1, p. 17 of [Jain 2010]. The logical arrangement as shown
above is chosen to be consistent with our logical arrangement for
spatial vectors as documented in multibody_spatial_algebra for which
the rotational component comes first followed by the translational
component.

In typeset material we use the symbol :math:`[M^{S/P}]_E` to represent
the spatial inertia of a body or composite body S about point P,
expressed in frame E. For this inertia, the monogram notation reads
``M_SP_E``. If the point P is fixed to a body B, we write that point
as :math:`B_P` which appears in code and comments as ``Bp``. So if the
body or composite body is B and the about point is ``Bp``, the
monogram notation reads ``M_BBp_E``, which can be abbreviated to
``M_Bp_E`` since the about point ``Bp`` also identifies the body.
Common cases are that the about point is the origin ``Bo`` of the
body, or it's the center of mass ``Bcm`` for which the rotational
inertia in monogram notation would read as ``I_Bo_E`` and ``I_Bcm_E``,
respectively. Given ``M_BP_E`` (:math:`[M^{B/P}]_E`), the rotational
inertia of this spatial inertia is ``I_BP_E`` (:math:`[I^{B/P}]_E`)
and the position vector of the center of mass measured from point P
and expressed in E is ``p_PBcm_E`` (:math:`[^Pp^{B_{cm}}]_E`).

Note:
    This class does not implement any mechanism to track the frame E
    in which a spatial inertia is expressed or about what point is
    computed. Methods and operators on this class have no means to
    determine frame consistency through operations. It is therefore
    the responsibility of users of this class to keep track of frames
    in which operations are performed. We suggest doing that using
    disciplined notation, as described above.

Note:
    Several methods in this class throw a RuntimeError for invalid
    rotational inertia operations in debug releases only. This
    provides speed in a release build while facilitating debugging in
    debug builds. In addition, these validity tests are only performed
    for scalar types for which drake::scalar_predicate<T>::is_bool is
    ``True``. For instance, validity checks are not performed when T
    is symbolic::Expression.

Note:
    The methods of this class satisfy the "basic exception guarantee":
    if an exception is thrown, the program will still be in a valid
    state. Specifically, no resources are leaked, and all objects'
    invariants are intact. Be aware that SpatialInertia objects may
    contain invalid inertia data in cases where input checking is
    skipped.

See also:
    https://en.cppreference.com/w/cpp/language/exceptions

See also:
    To create a spatial inertia of a mesh, see
    CalcSpatialInertia(const geometry::TriangleSurfaceMesh<double>&
    mesh, double density).

See also:
    To create spatial inertia from most of geometry::Shape, see
    CalcSpatialInertia(const geometry::Shape& shape, double density).

See also:
    To create spatial inertia for a set of bodies, see
    MultibodyPlant::CalcSpatialInertia().

- [Jain 2010]  Jain, A., 2010. Robot and multibody dynamics: analysis and
               algorithms. Springer Science & Business Media.)""";
        // Symbol: drake::multibody::SpatialInertia::CalcComMoment
        struct /* CalcComMoment */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Computes the center of mass moment vector ``mass * p_PScm_E`` given
the position vector ``p_PScm_E`` from the *about point* P to the
center of mass ``Scm`` of the body or composite body S, expressed in
frame E. See the documentation of this class for details.)""";
        } CalcComMoment;
        // Symbol: drake::multibody::SpatialInertia::CalcMinimumPhysicalLength
        struct /* CalcMinimumPhysicalLength */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Returns the minimum possible length for the physical extent of the
massive object that underlies this spatial inertia. In other words,
the underlying physical object must have at least two particles whose
distance between each other is greater than or equal to the minimum
possible length.

Note:
    The minimum possible length is equal to the space-diagonal of the
    minimum bounding box for ``this`` spatial inertia, which happens
    to be equal to √(2 * trace of the central unit inertia associated
    with ``this``).

Note:
    Minimum possible length can be used to detect erroneous inertias
    associated with absurdly large objects or to detect errors when
    the minimum possible length is larger than the real physical
    geometry that underlies ``this`` spatial inertia (maybe due to
    inertia conversion errors, e.g., factor of 10⁷ from kg m² to g cm²
    or 10⁹ from kg m² to g mm²). To assess whether the minimum
    possible length is reasonable, it helps to have comparable sizes,
    e.g., the world's largest aircraft carrier has a space-diagonal ≈
    355 m (length ≈ 337 m, width ≈ 78 m, height ≈ 76 m), the largest
    land vehicle (Bagger bucket-wheel excavator) is ≈ 224 m long, the
    largest human object in space (International Space Station) is 109
    m long and 75 m wide, the USA space shuttle is ≈ 37 m long and can
    carry a 15.2 m Canadarm, the world's largest humanoid robot
    (Mononofu) is ≈ 8.5 m tall. Also, minimum possible length can be
    compared to known physical geometry (e.g., realistic collision
    geometry, visual geometry, or physical extents associated with
    body connectivity data and topology), and this comparison can be
    used to warn that a spatial inertia may be physically impossible
    (e.g., underlying geometry is smaller than the minimum possible
    length).)""";
        } CalcMinimumPhysicalLength;
        // Symbol: drake::multibody::SpatialInertia::CalcPrincipalHalfLengthsAndPoseForMinimumBoundingBox
        struct /* CalcPrincipalHalfLengthsAndPoseForMinimumBoundingBox */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Returns three ½-lengths [lmax lmed lmin] sorted in descending order
(lmax ≥ lmed ≥ lmin), orientation, and position of a box whose mass is
concentrated in 8 particles at the box's corners and whose spatial
inertia is equal to ``this`` spatial inertia. The physical geometry of
the actual underlying object must be larger than this box, as this box
is the minimum bounding box for the actual geometry. See
spatial_inertia_equivalent_shapes "Spatial inertia equivalent shapes"
for more details.

Raises:
    RuntimeError if the elements of ``this`` spatial inertia cannot be
    converted to a real finite double. For example, an exception is
    thrown if ``this`` contains an erroneous NaN or if scalar type T
    is symbolic.)""";
        } CalcPrincipalHalfLengthsAndPoseForMinimumBoundingBox;
        // Symbol: drake::multibody::SpatialInertia::CalcPrincipalHalfLengthsAndPoseForSolidBox
        struct /* CalcPrincipalHalfLengthsAndPoseForSolidBox */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Returns three ½-lengths [lmax lmed lmin] sorted in descending order
(lmax ≥ lmed ≥ lmin), orientation, and position of a solid box whose
spatial inertia is equal to ``this`` spatial inertia. See
spatial_inertia_equivalent_shapes "Spatial inertia equivalent shapes"
for more details.

Raises:
    RuntimeError if the elements of ``this`` spatial inertia cannot be
    converted to a real finite double. For example, an exception is
    thrown if ``this`` contains an erroneous NaN or if scalar type T
    is symbolic.)""";
        } CalcPrincipalHalfLengthsAndPoseForSolidBox;
        // Symbol: drake::multibody::SpatialInertia::CalcPrincipalSemiDiametersAndPoseForSolidEllipsoid
        struct /* CalcPrincipalSemiDiametersAndPoseForSolidEllipsoid */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Returns 3 principal semi-diameters [lmax lmed lmin] sorted in
descending order (lmax ≥ lmed ≥ lmin), orientation, and position of a
solid ellipsoid whose spatial inertia is equal to ``this`` spatial
inertia. See spatial_inertia_equivalent_shapes "Spatial inertia
equivalent shapes" for more details.

Raises:
    RuntimeError if the elements of ``this`` spatial inertia cannot be
    converted to a real finite double. For example, an exception is
    thrown if ``this`` contains an erroneous NaN or if scalar type T
    is symbolic.)""";
        } CalcPrincipalSemiDiametersAndPoseForSolidEllipsoid;
        // Symbol: drake::multibody::SpatialInertia::CalcRotationalInertia
        struct /* CalcRotationalInertia */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Computes the rotational inertia ``I_SP_E = mass * G_SP_E`` of this
spatial inertia, computed about point P and expressed in frame E. See
the documentation of this class for details.)""";
        } CalcRotationalInertia;
        // Symbol: drake::multibody::SpatialInertia::CopyToFullMatrix6
        struct /* CopyToFullMatrix6 */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Copy to a full 6x6 matrix representation.)""";
        } CopyToFullMatrix6;
        // Symbol: drake::multibody::SpatialInertia::CreateInvalidityReport
        struct /* CreateInvalidityReport */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""((Internal use only). Returns an optional string if this SpatialInertia
is invalid, otherwise returns an empty optional.)""";
        } CreateInvalidityReport;
        // Symbol: drake::multibody::SpatialInertia::HollowSphereWithDensity
        struct /* HollowSphereWithDensity */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Creates a spatial inertia for a uniform density thin hollow sphere B
about its geometric center Bo (which is coincident with B's center of
mass Bcm).

Parameter ``area_density``:
    mass per unit area (kg/m²).

Parameter ``radius``:
    sphere's radius in meters (the hollow sphere is regarded as an
    infinitesimally thin shell of uniform density).

Returns ``M_BBo_B``:
    B's spatial inertia about Bo, expressed in B. Since B's rotational
    inertia is triaxially symmetric, M_BBo_B = M_BBo_E, i.e., M_BBo
    expressed in frame B is equal to M_BBo expressed in an arbitrary
    frame E.

Note:
    B's rotational inertia about Bo is triaxially symmetric, meaning B
    has an equal moment of inertia about any line passing through Bo.

Raises:
    RuntimeError if area_density or radius is not positive and finite.)""";
        } HollowSphereWithDensity;
        // Symbol: drake::multibody::SpatialInertia::HollowSphereWithMass
        struct /* HollowSphereWithMass */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Creates a spatial inertia for a uniform density hollow sphere B about
its geometric center Bo (which is coincident with B's center of mass
Bcm).

Parameter ``mass``:
    mass of the hollow sphere (kg).

Parameter ``radius``:
    sphere's radius in meters (the hollow sphere is regarded as an
    infinitesimally thin shell of uniform density).

Returns ``M_BBo``:
    B's spatial inertia about Bo. Since B's rotational inertia is
    triaxially symmetric, M_BBo_B = M_BBo_E, i.e., M_BBo expressed in
    frame B is equal to M_BBo expressed in an arbitrary frame E.

Note:
    B's rotational inertia about Bo is triaxially symmetric, meaning B
    has an equal moment of inertia about any line passing through Bo.

Raises:
    RuntimeError if mass or radius is not positive and finite.)""";
        } HollowSphereWithMass;
        // Symbol: drake::multibody::SpatialInertia::IsNaN
        struct /* IsNaN */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Returns ``True`` if any of the elements in this spatial inertia is NaN
and ``False`` otherwise.)""";
        } IsNaN;
        // Symbol: drake::multibody::SpatialInertia::IsPhysicallyValid
        struct /* IsPhysicallyValid */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Performs a number of checks to verify that this is a physically valid
spatial inertia. The checks performed include:

- No NaN entries.
- Non-negative mass.
- Non-negative principal moments about the center of mass.
- Principal moments about the center of mass must satisfy the triangle
  inequality:
  - ``Ixx + Iyy >= Izz``
  - `Ixx + Izz >= Iyy`
  - ``Iyy + Izz >= Ixx``

These are the tests performed by
RotationalInertia::CouldBePhysicallyValid() which become a sufficient
condition when performed on a rotational inertia about a body's center
of mass.

See also:
    RotationalInertia::CouldBePhysicallyValid().)""";
        } IsPhysicallyValid;
        // Symbol: drake::multibody::SpatialInertia::IsZero
        struct /* IsZero */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Returns ``True`` if all of the elements in this spatial inertia are
zero and ``False`` otherwise.)""";
        } IsZero;
        // Symbol: drake::multibody::SpatialInertia::MakeFromCentralInertia
        struct /* MakeFromCentralInertia */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Creates a spatial inertia for a physical body or composite body S
about a point P from a given mass, center of mass, and central
rotational inertia. For example, this method creates a body's
SpatialInertia about its body origin Bo from the body's mass, position
vector from Bo to the body's center of mass, and rotational inertia
about the body's center of mass.

This method checks for the physical validity of the resulting
SpatialInertia with IsPhysicallyValid() and throws a RuntimeError
exception in the event the provided input parameters lead to a
non-physically viable spatial inertia.

Parameter ``mass``:
    The mass of the body or composite body S.

Parameter ``p_PScm_E``:
    The position vector from point P to point ``Scm`` (S's center of
    mass), expressed in a frame E.

Parameter ``I_SScm_E``:
    S's RotationalInertia about Scm, expressed in frame E.

Returns ``M_SP_E``:
    S's spatial inertia about point P, expressed in frame E.)""";
        } MakeFromCentralInertia;
        // Symbol: drake::multibody::SpatialInertia::MakeUnitary
        struct /* MakeUnitary */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""((Internal use only) Creates a spatial inertia whose mass is 1,
position vector to center of mass is zero, and whose rotational
inertia has moments of inertia of 1 and products of inertia of 0.)""";
        } MakeUnitary;
        // Symbol: drake::multibody::SpatialInertia::NaN
        struct /* NaN */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Initializes mass, center of mass and rotational inertia to invalid
NaN's for a quick detection of uninitialized values.)""";
        } NaN;
        // Symbol: drake::multibody::SpatialInertia::PointMass
        struct /* PointMass */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Creates the spatial inertia for a particle Q of mass m about a point
P.

Parameter ``mass``:
    mass of the single particle (units of kg).

Parameter ``position``:
    vector from point P to Q, expressed in a frame B.

Returns ``M_QP_B``:
    particle Q's spatial inertia about P, expressed in frame B.

Raises:
    RuntimeError if mass is not positive and finite.)""";
        } PointMass;
        // Symbol: drake::multibody::SpatialInertia::ReExpress
        struct /* ReExpress */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Given ``this`` spatial inertia ``M_SP_E`` for some body or composite
body S, taken about a point P and expressed in frame E, this method
computes the same inertia re-expressed in another frame A.

Parameter ``R_AE``:
    RotationMatrix relating frames A and E.

Returns ``M_SP_A``:
    The same spatial inertia of S about P but now re-expressed in
    frame A.

See also:
    ReExpressInPlace() for details.)""";
        } ReExpress;
        // Symbol: drake::multibody::SpatialInertia::ReExpressInPlace
        struct /* ReExpressInPlace */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Given ``this`` spatial inertia ``M_SP_E`` for some body or composite
body S, taken about a point P and expressed in frame E, this method
computes the same inertia re-expressed in another frame A. This
operation is performed in-place modifying the original object. On
return, ``this`` is now re-expressed in frame A, that is, ``M_SP_A``.

Parameter ``R_AE``:
    Rotation matrix from frame E to frame A.)""";
        } ReExpressInPlace;
        // Symbol: drake::multibody::SpatialInertia::SetNaN
        struct /* SetNaN */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Sets ``this`` spatial inertia to have NaN entries. Typically used for
quick detection of uninitialized values.)""";
        } SetNaN;
        // Symbol: drake::multibody::SpatialInertia::Shift
        struct /* Shift */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Given ``this`` spatial inertia ``M_SP_E`` for some body or composite
body S, computed about point P, and expressed in frame E, this method
uses the *Parallel Axis Theorem* for spatial inertias to compute the
same spatial inertia about a new point Q. The result still is
expressed in frame E.

See also:
    ShiftInPlace() for more details.

Parameter ``p_PQ_E``:
    Vector from the original about point P to the new about point Q,
    expressed in the same frame E ``this`` spatial inertia is
    expressed in.

Returns ``M_SQ_E``:
    This same spatial inertia for body or composite body S but
    computed about a new point Q.)""";
        } Shift;
        // Symbol: drake::multibody::SpatialInertia::ShiftInPlace
        struct /* ShiftInPlace */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Given ``this`` spatial inertia ``M_SP_E`` for some body or composite
body S, computed about point P, and expressed in frame E, this method
uses the *Parallel Axis Theorem* for spatial inertias to compute the
same spatial inertia about a new point Q. The result still is
expressed in frame E. This operation is performed in-place modifying
the original object. On return, ``this`` is now computed about a new
point Q.

See also:
    Shift() which does not modify this object.

For details see Section 2.1.2, p. 20 of [Jain 2010].

Parameter ``p_PQ_E``:
    position vector from the original about-point P to the new
    about-point Q, expressed in the same frame E that ``this`` spatial
    inertia is expressed in.)""";
        } ShiftInPlace;
        // Symbol: drake::multibody::SpatialInertia::SolidBoxWithDensity
        struct /* SolidBoxWithDensity */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Creates a spatial inertia for a uniform density solid box B about its
geometric center Bo (which is coincident with B's center of mass Bcm).

Parameter ``density``:
    mass per volume (kg/m³).

Parameter ``lx``:
    length of the box in the Bx direction (meters).

Parameter ``ly``:
    length of the box in the By direction (meters).

Parameter ``lz``:
    length of the box in the Bz direction (meters).

Returns ``M_BBo_B``:
    B's spatial inertia about Bo, expressed in B.

Raises:
    RuntimeError if density, lx, ly, or lz is not positive and finite.)""";
        } SolidBoxWithDensity;
        // Symbol: drake::multibody::SpatialInertia::SolidBoxWithMass
        struct /* SolidBoxWithMass */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Creates a spatial inertia for a uniform density solid box B about its
geometric center Bo (which is coincident with B's center of mass Bcm).

Parameter ``mass``:
    mass of the solid box (kg).

Parameter ``lx``:
    length of the box in the Bx direction (meters).

Parameter ``ly``:
    length of the box in the By direction (meters).

Parameter ``lz``:
    length of the box in the Bz direction (meters).

Returns ``M_BBo_B``:
    B's spatial inertia about Bo, expressed in B.

Raises:
    RuntimeError if mass, lx, ly, or lz is not positive and finite.)""";
        } SolidBoxWithMass;
        // Symbol: drake::multibody::SpatialInertia::SolidCapsuleWithDensity
        struct /* SolidCapsuleWithDensity */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Creates a spatial inertia for a uniform density solid capsule B about
its geometric center Bo (which is coincident with B's center of mass
Bcm).

Parameter ``density``:
    mass per volume (kg/m³).

Parameter ``radius``:
    radius of the cylinder/half-sphere parts of the capsule.

Parameter ``length``:
    length of the cylindrical part of the capsule.

Parameter ``unit_vector``:
    unit vector defining the axial direction of the cylindrical part
    of the capsule, expressed in B.

Returns ``M_BBo_B``:
    B's spatial inertia about Bo, expressed in B.

Note:
    B's rotational inertia about Bo is axially symmetric, meaning B
    has an equal moment of inertia about any line that both passes
    through Bo and is perpendicular to unit_vector.

Raises:
    RuntimeError if density, radius, or length is not positive and
    finite or if ‖unit_vector‖ is not within 1.0E-14 of 1.0.)""";
        } SolidCapsuleWithDensity;
        // Symbol: drake::multibody::SpatialInertia::SolidCapsuleWithMass
        struct /* SolidCapsuleWithMass */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Creates a spatial inertia for a uniform density solid capsule B about
its geometric center Bo (which is coincident with B's center of mass
Bcm).

Parameter ``mass``:
    mass of the solid capsule (kg).

Parameter ``radius``:
    radius of the cylinder/half-sphere parts of the capsule.

Parameter ``length``:
    length of the cylindrical part of the capsule.

Parameter ``unit_vector``:
    unit vector defining the axial direction of the cylindrical part
    of the capsule, expressed in B.

Returns ``M_BBo_B``:
    B's spatial inertia about Bo, expressed in B.

Note:
    B's rotational inertia about Bo is axially symmetric, meaning B
    has an equal moment of inertia about any line that both passes
    through Bo and is perpendicular to unit_vector.

Raises:
    RuntimeError if mass, radius, or length is not positive and finite
    or if ‖unit_vector‖ is not within 1.0E-14 of 1.0.)""";
        } SolidCapsuleWithMass;
        // Symbol: drake::multibody::SpatialInertia::SolidCubeWithDensity
        struct /* SolidCubeWithDensity */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Creates a spatial inertia for a uniform density solid cube B about its
geometric center Bo (which is coincident with B's center of mass Bcm).

Parameter ``density``:
    mass per volume (kg/m³).

Parameter ``length``:
    The length of each of the cube's sides (meters).

Returns ``M_BBo_B``:
    B's spatial inertia about Bo, expressed in B. Since B's rotational
    inertia is triaxially symmetric, M_BBo_B = M_BBo_E, i.e., M_BBo
    expressed in frame B is equal to M_BBo expressed in an arbitrary
    frame E.

Note:
    B's rotational inertia about Bo is triaxially symmetric, meaning B
    has an equal moment of inertia about any line passing through Bo.

Raises:
    RuntimeError if density or length is not positive and finite.)""";
        } SolidCubeWithDensity;
        // Symbol: drake::multibody::SpatialInertia::SolidCubeWithMass
        struct /* SolidCubeWithMass */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Creates a spatial inertia for a uniform density solid cube B about its
geometric center Bo (which is coincident with B's center of mass Bcm).

Parameter ``mass``:
    mass of the solid cube (kg).

Parameter ``length``:
    The length of each of the cube's sides (meters).

Returns ``M_BBo_B``:
    B's spatial inertia about Bo, expressed in B. Since B's rotational
    inertia is triaxially symmetric, M_BBo_B = M_BBo_E, i.e., M_BBo
    expressed in frame B is equal to M_BBo expressed in an arbitrary
    frame E.

Note:
    B's rotational inertia about Bo is triaxially symmetric, meaning B
    has an equal moment of inertia about any line passing through Bo.

Raises:
    RuntimeError if mass or length is not positive and finite.)""";
        } SolidCubeWithMass;
        // Symbol: drake::multibody::SpatialInertia::SolidCylinderWithDensity
        struct /* SolidCylinderWithDensity */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Creates a spatial inertia for a uniform density solid cylinder B about
its geometric center Bo (which is coincident with B's center of mass
Bcm).

Parameter ``density``:
    mass per volume (kg/m³).

Parameter ``radius``:
    radius of the cylinder (meters).

Parameter ``length``:
    length of cylinder in unit_vector direction (meters).

Parameter ``unit_vector``:
    unit vector defining the axial direction of the cylinder,
    expressed in B.

Returns ``M_BBo_B``:
    B's spatial inertia about Bo, expressed in B.

Note:
    B's rotational inertia about Bo is axially symmetric, meaning B
    has an equal moment of inertia about any line that both passes
    through Bo and is perpendicular to unit_vector.

Raises:
    RuntimeError if density, radius, or length is not positive and
    finite or if ‖unit_vector‖ is not within 1.0E-14 of 1.0.

See also:
    SolidCylinderWithDensityAboutEnd() to calculate M_BBp_B, B's
    spatial inertia about Bp (at the center of one of the cylinder's
    circular ends).)""";
        } SolidCylinderWithDensity;
        // Symbol: drake::multibody::SpatialInertia::SolidCylinderWithDensityAboutEnd
        struct /* SolidCylinderWithDensityAboutEnd */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Creates a spatial inertia for a uniform-density solid cylinder B about
an end-point Bp of the cylinder's axis (see below for more about Bp).

Parameter ``density``:
    mass per volume (kg/m³).

Parameter ``radius``:
    radius of cylinder (meters).

Parameter ``length``:
    length of cylinder in unit_vector direction (meters).

Parameter ``unit_vector``:
    unit vector parallel to the axis of the cylinder and directed from
    Bp to Bcm (B's center of mass), expressed in B.

Returns ``M_BBp_B``:
    B's spatial inertia about Bp, expressed in B.

Note:
    The position from Bp to Bcm is p_BpBcm = length / 2 * unit_vector.

Note:
    B's rotational inertia about Bp is axially symmetric, meaning B
    has an equal moment of inertia about any line that both passes
    through Bp and is perpendicular to unit_vector.

Raises:
    RuntimeError if density, radius, or length is not positive and
    finite or if ‖unit_vector‖ is not within 1.0E-14 of 1.0.

See also:
    SolidCylinderWithDensity() to calculate M_BBcm_B, B's spatial
    inertia about Bcm (B's center of mass).)""";
        } SolidCylinderWithDensityAboutEnd;
        // Symbol: drake::multibody::SpatialInertia::SolidCylinderWithMass
        struct /* SolidCylinderWithMass */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Creates a spatial inertia for a uniform density solid cylinder B about
its geometric center Bo (which is coincident with B's center of mass
Bcm).

Parameter ``mass``:
    mass of the solid cylinder (kg).

Parameter ``radius``:
    radius of the cylinder (meters).

Parameter ``length``:
    length of cylinder in unit_vector direction (meters).

Parameter ``unit_vector``:
    unit vector defining the axial direction of the cylinder,
    expressed in B.

Returns ``M_BBo_B``:
    B's spatial inertia about Bo, expressed in B.

Note:
    B's rotational inertia about Bo is axially symmetric, meaning B
    has an equal moment of inertia about any line that both passes
    through Bo and is perpendicular to unit_vector.

Raises:
    RuntimeError if mass, radius, or length is not positive and finite
    or if ‖unit_vector‖ is not within 1.0E-14 of 1.0.

See also:
    SolidCylinderWithMassAboutEnd() to calculate M_BBp_B, B's spatial
    inertia about Bp (at the center of one of the cylinder's circular
    ends).)""";
        } SolidCylinderWithMass;
        // Symbol: drake::multibody::SpatialInertia::SolidCylinderWithMassAboutEnd
        struct /* SolidCylinderWithMassAboutEnd */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Creates a spatial inertia for a uniform-density solid cylinder B about
an end-point Bp of the cylinder's axis (see below for more about Bp).

Parameter ``mass``:
    mass of the solid cylinder (kg).

Parameter ``radius``:
    radius of cylinder (meters).

Parameter ``length``:
    length of cylinder in unit_vector direction (meters).

Parameter ``unit_vector``:
    unit vector parallel to the axis of the cylinder and directed from
    Bp to Bcm (B's center of mass), expressed in B.

Returns ``M_BBp_B``:
    B's spatial inertia about Bp, expressed in B.

Note:
    The position from Bp to Bcm is p_BpBcm = length / 2 * unit_vector.

Note:
    B's rotational inertia about Bp is axially symmetric, meaning B
    has an equal moment of inertia about any line that both passes
    through Bp and is perpendicular to unit_vector.

Raises:
    RuntimeError if density, radius, or length is not positive and
    finite or if ‖unit_vector‖ is not within 1.0E-14 of 1.0.

See also:
    SolidCylinderWithMass() to calculate M_BBcm_B, B's spatial inertia
    about Bcm (B's center of mass).)""";
        } SolidCylinderWithMassAboutEnd;
        // Symbol: drake::multibody::SpatialInertia::SolidEllipsoidWithDensity
        struct /* SolidEllipsoidWithDensity */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Creates a spatial inertia for a uniform density solid ellipsoid B
about its geometric center Bo (which is coincident with B's center of
mass Bcm).

Parameter ``density``:
    mass per volume (kg/m³).

Parameter ``a``:
    length of ellipsoid semi-axis in the ellipsoid Bx direction.

Parameter ``b``:
    length of ellipsoid semi-axis in the ellipsoid By direction.

Parameter ``c``:
    length of ellipsoid semi-axis in the ellipsoid Bz direction.

Returns ``M_BBo_B``:
    B's spatial inertia about Bo, expressed in B.

Raises:
    RuntimeError if density, a, b, or c is not positive and finite.)""";
        } SolidEllipsoidWithDensity;
        // Symbol: drake::multibody::SpatialInertia::SolidEllipsoidWithMass
        struct /* SolidEllipsoidWithMass */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Creates a spatial inertia for a uniform density solid ellipsoid B
about its geometric center Bo (which is coincident with B's center of
mass Bcm).

Parameter ``mass``:
    mass of the solid ellipsoid (kg).

Parameter ``a``:
    length of ellipsoid semi-axis in the ellipsoid Bx direction.

Parameter ``b``:
    length of ellipsoid semi-axis in the ellipsoid By direction.

Parameter ``c``:
    length of ellipsoid semi-axis in the ellipsoid Bz direction.

Returns ``M_BBo_B``:
    B's spatial inertia about Bo, expressed in B.

Raises:
    RuntimeError if mass, a, b, or c is not positive and finite.)""";
        } SolidEllipsoidWithMass;
        // Symbol: drake::multibody::SpatialInertia::SolidSphereWithDensity
        struct /* SolidSphereWithDensity */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Creates a spatial inertia for a uniform density solid sphere B about
its geometric center Bo (which is coincident with B's center of mass
Bcm).

Parameter ``density``:
    mass per volume (kg/m³).

Parameter ``radius``:
    sphere's radius (meters).

Returns ``M_BBo``:
    B's spatial inertia about Bo. Since B's rotational inertia is
    triaxially symmetric, M_BBo_B = M_BBo_E, i.e., M_BBo expressed in
    frame B is equal to M_BBo expressed in an arbitrary frame E.

Note:
    B's rotational inertia about Bo is triaxially symmetric, meaning B
    has an equal moment of inertia about any line passing through Bo.

Raises:
    RuntimeError if density or radius is not positive and finite.)""";
        } SolidSphereWithDensity;
        // Symbol: drake::multibody::SpatialInertia::SolidSphereWithMass
        struct /* SolidSphereWithMass */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Creates a spatial inertia for a uniform density solid sphere B about
its geometric center Bo (which is coincident with B's center of mass
Bcm).

Parameter ``mass``:
    mass of the solid sphere (kg).

Parameter ``radius``:
    sphere's radius (meters).

Returns ``M_BBo``:
    B's spatial inertia about Bo. Since B's rotational inertia is
    triaxially symmetric, M_BBo_B = M_BBo_E, i.e., M_BBo expressed in
    frame B is equal to M_BBo expressed in an arbitrary frame E.

Note:
    B's rotational inertia about Bo is triaxially symmetric, meaning B
    has an equal moment of inertia about any line passing through Bo.

Raises:
    RuntimeError if mass or radius is not positive and finite.)""";
        } SolidSphereWithMass;
        // Symbol: drake::multibody::SpatialInertia::SolidTetrahedronAboutPointWithDensity
        struct /* SolidTetrahedronAboutPointWithDensity */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Creates a spatial inertia for a uniform density solid tetrahedron B
about a point A, from which position vectors to B's 4 vertices B0, B1,
B2, B3 are measured (position vectors are all expressed in a common
frame E).

Parameter ``density``:
    mass per volume (kg/m³).

Parameter ``p0``:
    position vector p_AB0_E from point A to B0, expressed in E.

Parameter ``p1``:
    position vector p_AB1_E from point A to B1, expressed in E.

Parameter ``p2``:
    position vector p_AB2_E from point A to B2, expressed in E.

Parameter ``p3``:
    position vector p_AB3_E from point A to B3, expressed in E.

Returns ``M_BA_E``:
    B's spatial inertia about point A, expressed in E.

Note:
    In the common case, point A is Eo (the origin of the expressed-in
    frame E). The example below has point A as Wo (origin of world
    frame W).

Raises:
    RuntimeError if density is not positive and finite.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    double density = 1000;
    Vector3<double> p_WoB0_W(1, 0, 0);
    Vector3<double> p_WoB1_W(2, 0, 0);
    Vector3<double> p_WoB2_W(1, 1, 0);
    Vector3<double> p_WoB3_W(1, 0, 1);
    SpatialInertia<double> M_BWo_W =
        SpatialInertia<double>::SolidTetrahedronAboutPointWithDensity(
            density, p_WoB0_W, p_WoB1_W, p_WoB2_W, p_WoB3_W);

.. raw:: html

    </details>

See also:
    SolidTetrahedronAboutVertexWithDensity() to efficiently calculate
    a spatial inertia about a vertex of B.)""";
        } SolidTetrahedronAboutPointWithDensity;
        // Symbol: drake::multibody::SpatialInertia::SolidTetrahedronAboutVertexWithDensity
        struct /* SolidTetrahedronAboutVertexWithDensity */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""((Advanced) Creates a spatial inertia for a uniform density solid
tetrahedron B about its vertex B0, from which position vectors to B's
other 3 vertices B1, B2, B3 are measured (position vectors are all
expressed in a common frame E).

Parameter ``density``:
    mass per volume (kg/m³).

Parameter ``p1``:
    position vector p_B0B1_E from B0 to B1, expressed in E.

Parameter ``p2``:
    position vector p_B0B2_E from B0 to B2, expressed in E.

Parameter ``p3``:
    position vector p_B0B3_E from B0 to B3, expressed in E.

Returns ``M_BB0_E``:
    B's spatial inertia about its vertex B0, expressed in E.

Raises:
    RuntimeError if density is not positive and finite.

See also:
    SolidTetrahedronAboutPointWithDensity() to calculate a spatial
    inertia about an arbitrary point.)""";
        } SolidTetrahedronAboutVertexWithDensity;
        // Symbol: drake::multibody::SpatialInertia::SpatialInertia<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Constructs a spatial inertia for a physical body or composite body S
about a point P from a given mass, center of mass and rotational
inertia. The center of mass is specified by the position vector
``p_PScm_E`` from point P to the center of mass point ``Scm``,
expressed in a frame E. The rotational inertia is provided as the
UnitInertia ``G_SP_E`` of the body or composite body S computed about
point P and expressed in frame E.

Note:
    The third argument of this constructor is unusual in that it is an
    UnitInertia (not a traditional RotationalInertia) and its inertia
    is about the arbitrary point P (not Scm -- S's center of mass).

See also:
    MakeFromCentralInertia a factory method with traditional utility.

This constructor checks for the physical validity of the resulting
SpatialInertia with IsPhysicallyValid() and throws a RuntimeError
exception in the event the provided input parameters lead to
non-physically viable spatial inertia. Since this check has
non-negligable runtime costs, it can be disabled by setting the
optional argument ``skip_validity_check`` to ``True``.

Parameter ``mass``:
    The mass of the body or composite body S.

Parameter ``p_PScm_E``:
    The position vector from point P to the center of mass of body or
    composite body S expressed in frame E.

Parameter ``G_SP_E``:
    UnitInertia of the body or composite body S computed about origin
    point P and expressed in frame E.

Parameter ``skip_validity_check``:
    If true, skips the validity check described above. Defaults to
    false.)""";
        } ctor;
        // Symbol: drake::multibody::SpatialInertia::ThinRodWithMass
        struct /* ThinRodWithMass */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Creates a spatial inertia for a uniform-density thin rod B about its
center of mass Bcm.

Parameter ``mass``:
    mass of the rod (units of kg).

Parameter ``length``:
    length of the rod (units of meters).

Parameter ``unit_vector``:
    unit vector defining the rod's axial direction, expressed in B.

Returns ``M_BBcm_B``:
    B's spatial inertia about Bcm, expressed in B.

Note:
    B's rotational inertia about Bcm is axially symmetric, meaning B
    has an equal moment of inertia about any line that both passes
    through Bcm and is perpendicular to unit_vector. B has no (zero)
    rotational inertia about the line that passes through Bcm and is
    parallel to unit_vector.

Raises:
    RuntimeError if mass or length is not positive and finite or if
    ‖unit_vector‖ is not within 1.0E-14 of 1.0.

See also:
    ThinRodWithMassAboutEnd() to calculate M_BBp_B, B's spatial
    inertia about Bp (one of the ends of rod B).)""";
        } ThinRodWithMass;
        // Symbol: drake::multibody::SpatialInertia::ThinRodWithMassAboutEnd
        struct /* ThinRodWithMassAboutEnd */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Creates a spatial inertia for a uniform-density thin rod B about one
of its ends.

Parameter ``mass``:
    mass of the rod (units of kg).

Parameter ``length``:
    length of the rod (units of meters).

Parameter ``unit_vector``:
    unit vector defining the rod's axial direction, expressed in B.

Returns ``M_BBp_B``:
    B's spatial inertia about Bp, expressed in B.

Note:
    The position from Bp to Bcm is length / 2 * unit_vector.

Note:
    B's rotational inertia about Bp is axially symmetric, meaning B
    has an equal moment of inertia about any line that both passes
    through Bp and is perpendicular to unit_vector. B has no (zero)
    rotational inertia about the line that passes through Bp and is
    parallel to unit_vector.

Raises:
    RuntimeError if mass or length is not positive and finite or if
    ‖unit_vector‖ is not within 1.0E-14 of 1.0.

See also:
    ThinRodWithMass() to calculate M_BBcm_B, B's spatial inertia about
    Bcm (B's center of mass).)""";
        } ThinRodWithMassAboutEnd;
        // Symbol: drake::multibody::SpatialInertia::Zero
        struct /* Zero */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Initializes mass, center of mass and rotational inertia to zero.)""";
        } Zero;
        // Symbol: drake::multibody::SpatialInertia::cast
        struct /* cast */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Returns a new SpatialInertia object templated on ``Scalar``
initialized from the value of ``this`` spatial inertia.

Template parameter ``Scalar``:
    The scalar type on which the new spatial inertia will be
    templated.

Note:
    ``SpatialInertia<From>::cast<To>()`` creates a new
    ``SpatialInertia<To>`` from a ``SpatialInertia<From>`` but only if
    type ``To`` is constructible from type ``From``. This cast method
    works in accordance with Eigen's cast method for Eigen's objects
    that underlie this SpatialInertia. For example, Eigen currently
    allows cast from type double to AutoDiffXd, but not vice-versa.)""";
        } cast;
        // Symbol: drake::multibody::SpatialInertia::get_com
        struct /* get_com */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Get a constant reference to the position vector ``p_PScm_E`` from the
*about point* P to the center of mass ``Scm`` of the body or composite
body S, expressed in frame E. See the documentation of this class for
details.)""";
        } get_com;
        // Symbol: drake::multibody::SpatialInertia::get_mass
        struct /* get_mass */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Get a constant reference to the mass of this spatial inertia.)""";
        } get_mass;
        // Symbol: drake::multibody::SpatialInertia::get_unit_inertia
        struct /* get_unit_inertia */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Get a constant reference to the unit inertia ``G_SP_E`` of this
spatial inertia, computed about point P and expressed in frame E. See
the documentation of this class for details.)""";
        } get_unit_inertia;
        // Symbol: drake::multibody::SpatialInertia::operator*
        struct /* operator_mul */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc_1args_A_WB_E =
R"""(Multiplies ``this`` spatial inertia ``M_Bo_E`` of a body B about its
frame origin ``Bo`` by the spatial acceleration of the body frame B in
a frame W. Mathematically:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    F_Bo_E = M_Bo_E * A_WB_E

.. raw:: html

    </details>

or, in terms of its rotational and translational components (see this
class's documentation for the block form of a rotational inertia):


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    t_Bo = I_Bo * alpha_WB + m * p_BoBcm x a_WBo
      f_Bo = -m * p_BoBcm x alpha_WB + m * a_WBo

.. raw:: html

    </details>

where ``alpha_WB`` and ``a_WBo`` are the rotational and translational
components of the spatial acceleration ``A_WB``, respectively.

Note:
    The term ``F_Bo_E`` computed by this operator appears in the
    equations of motion for a rigid body which, when written about the
    origin ``Bo`` of the body frame B (which does not necessarily need
    to coincide with the body's center of mass), read as:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    Ftot_BBo = M_Bo_W * A_WB + b_Bo

.. raw:: html

    </details>

where ``Ftot_BBo`` is the total spatial force applied on body B at
``Bo`` that corresponds to the body spatial acceleration ``A_WB`` and
``b_Bo`` contains the velocity dependent gyroscopic terms (see Eq.
2.26, p. 27, in A. Jain's book).)""";
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc_1args_V_WBp_E =
R"""(Multiplies ``this`` spatial inertia ``M_BP_E`` of a body B about a
point P by the spatial velocity ``V_WBp``, in a frame W, of the body
frame B shifted to point P. Mathematically:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    L_WBp_E = M_BP_E * V_WBp_E

.. raw:: html

    </details>

or, in terms of its rotational and translational components (see this
class's documentation for the block form of a rotational inertia):


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    h_WB  = I_Bp * w_WB + m * p_BoBcm x v_WP
      l_WBp = -m * p_BoBcm x w_WB + m * v_WP

.. raw:: html

    </details>

where ``w_WB`` and ``v_WP`` are the rotational and translational
components of the spatial velocity ``V_WBp``, respectively and,
``h_WB`` and ``l_WBp`` are the angular and linear components of the
spatial momentum ``L_WBp``, respectively.

Note:
    It is possible to show that ``M_BP_E.Shift(p_PQ_E) *
    V_WBp_E.Shift(p_PQ_E)`` exactly equals ``L_WBp_E.Shift(p_PQ_E)``.)""";
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc_1args_constEigenMatrixBase =
R"""(Multiplies ``this`` spatial inertia by a set of spatial vectors in M⁶
stored as columns of input matrix ``Mmatrix``. The top three rows of
Mmatrix are expected to store the rotational components while the
bottom three rows are expected to store the translational components.
The output matrix is of the same size as ``Mmatrix`` and each j-th
column stores the spatial vector in F⁶ result of multiplying ``this``
spatial inertia with the j-th column of ``Mmatrix``.)""";
        } operator_mul;
        // Symbol: drake::multibody::SpatialInertia::operator+=
        struct /* operator_iadd */ {
          // Source: drake/multibody/tree/spatial_inertia.h
          const char* doc =
R"""(Adds in a spatial inertia to ``this`` spatial inertia.

Parameter ``M_BP_E``:
    A spatial inertia of some body B to be added to ``this`` spatial
    inertia. It must be defined about the same point P as ``this``
    inertia, and expressed in the same frame E.

Returns:
    A reference to ``this`` spatial inertia, which has been updated to
    include the given spatial inertia ``M_BP_E``.

Note:
    Given that the composition of spatial inertias is not well defined
    for massless bodies, this composition of the spatial inertias
    performs the arithmetic average of the center of mass position
    vector (get_com()) and unit inertia (get_unit_inertia()) when the
    two spatial inertias have zero mass (get_mass()). This is only
    valid in the limit to zero mass for two bodies with the same mass.
    This special case allows the composition of spatial inertias in
    the common case of a kinematic chain of massless bodies.

Warning:
    This operation is only valid if both spatial inertias are computed
    about the same point P and expressed in the same frame E.
    Considering ``this`` spatial inertia to be ``M_SP_E`` for some
    body or composite body S, about some point P, the supplied spatial
    inertia ``M_BP_E`` must be for some other body or composite body B
    about the *same* point P; B's inertia is then included in S.)""";
        } operator_iadd;
      } SpatialInertia;
      // Symbol: drake::multibody::UniformGravityFieldElement
      struct /* UniformGravityFieldElement */ {
        // Source: drake/multibody/tree/uniform_gravity_field_element.h
        const char* doc =
R"""(This ForceElement allows modeling the effect of a uniform gravity
field as felt by bodies on the surface of the Earth. This gravity
field acts on all bodies in the MultibodyTree model.)""";
        // Symbol: drake::multibody::UniformGravityFieldElement::CalcConservativePower
        struct /* CalcConservativePower */ {
          // Source: drake/multibody/tree/uniform_gravity_field_element.h
          const char* doc = R"""()""";
        } CalcConservativePower;
        // Symbol: drake::multibody::UniformGravityFieldElement::CalcGravityGeneralizedForces
        struct /* CalcGravityGeneralizedForces */ {
          // Source: drake/multibody/tree/uniform_gravity_field_element.h
          const char* doc =
R"""(Computes the generalized forces ``tau_g(q)`` due to ``this`` gravity
field element as a function of the generalized positions ``q`` stored
in the input ``context``, for the multibody model to which ``this``
element belongs. ``tau_g(q)`` is defined such that it appears on the
right hand side of the equations of motion together with any other
generalized forces, like so:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    Mv̇ + C(q, v)v = tau_g(q) + tau_app

.. raw:: html

    </details>

where ``tau_app`` includes any other generalized forces applied on the
system.

Parameter ``context``:
    The context storing the state of the multibody model to which this
    element belongs.

Returns:
    tau_g A vector containing the generalized forces due to this
    gravity field force element. The generalized forces are consistent
    with the vector of generalized velocities ``v`` for the parent
    MultibodyTree model so that the inner product ``v⋅tau_g``
    corresponds to the power applied by the gravity forces on the
    mechanical system. That is, ``v⋅tau_g > 0`` corresponds to
    potential energy going into the system, as either mechanical
    kinetic energy, some other potential energy, or heat, and
    therefore to a decrease of potential energy.)""";
        } CalcGravityGeneralizedForces;
        // Symbol: drake::multibody::UniformGravityFieldElement::CalcNonConservativePower
        struct /* CalcNonConservativePower */ {
          // Source: drake/multibody/tree/uniform_gravity_field_element.h
          const char* doc = R"""()""";
        } CalcNonConservativePower;
        // Symbol: drake::multibody::UniformGravityFieldElement::CalcPotentialEnergy
        struct /* CalcPotentialEnergy */ {
          // Source: drake/multibody/tree/uniform_gravity_field_element.h
          const char* doc =
R"""(Computes the total potential energy of all bodies in the model in this
uniform gravity field. The definition of potential energy allows to
arbitrarily choose the zero energy height. This element takes the zero
energy height to be the same as the world's height. That is, a body
will have zero potential energy when its the height of its center of
mass is at the world's origin.)""";
        } CalcPotentialEnergy;
        // Symbol: drake::multibody::UniformGravityFieldElement::DoCalcAndAddForceContribution
        struct /* DoCalcAndAddForceContribution */ {
          // Source: drake/multibody/tree/uniform_gravity_field_element.h
          const char* doc = R"""()""";
        } DoCalcAndAddForceContribution;
        // Symbol: drake::multibody::UniformGravityFieldElement::DoCloneToScalar
        struct /* DoCloneToScalar */ {
          // Source: drake/multibody/tree/uniform_gravity_field_element.h
          const char* doc = R"""()""";
        } DoCloneToScalar;
        // Symbol: drake::multibody::UniformGravityFieldElement::DoShallowClone
        struct /* DoShallowClone */ {
          // Source: drake/multibody/tree/uniform_gravity_field_element.h
          const char* doc = R"""()""";
        } DoShallowClone;
        // Symbol: drake::multibody::UniformGravityFieldElement::UniformGravityFieldElement<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/uniform_gravity_field_element.h
          const char* doc_0args =
R"""(Constructs a uniform gravity field element with a default strength (on
the earth's surface) and direction (-z).)""";
          // Source: drake/multibody/tree/uniform_gravity_field_element.h
          const char* doc_1args =
R"""(Constructs a uniform gravity field element with a strength given by
the acceleration of gravity vector ``g_W``, expressed in the world
frame W.)""";
          // Source: drake/multibody/tree/uniform_gravity_field_element.h
          const char* doc_2args =
R"""(Constructs a uniform gravity field element with a strength given by
the acceleration of gravity vector ``g_W``, expressed in the world
frame W. Gravity is disabled for the set of model instances
``disabled_model_instances``.)""";
        } ctor;
        // Symbol: drake::multibody::UniformGravityFieldElement::gravity_vector
        struct /* gravity_vector */ {
          // Source: drake/multibody/tree/uniform_gravity_field_element.h
          const char* doc =
R"""(Returns the acceleration of the gravity vector in m/s², expressed in
the world frame W.)""";
        } gravity_vector;
        // Symbol: drake::multibody::UniformGravityFieldElement::is_enabled
        struct /* is_enabled */ {
          // Source: drake/multibody/tree/uniform_gravity_field_element.h
          const char* doc =
R"""(Returns:
    ``True`` iff gravity is enabled for ``model_instance``.

See also:
    enable(), disable().

Raises:
    RuntimeError if the model instance is invalid.

Raises:
    RuntimeError if this element is not associated with a
    MultibodyPlant.)""";
        } is_enabled;
        // Symbol: drake::multibody::UniformGravityFieldElement::set_enabled
        struct /* set_enabled */ {
          // Source: drake/multibody/tree/uniform_gravity_field_element.h
          const char* doc =
R"""(Sets is_enabled() for ``model_instance`` to ``is_enabled``.

Raises:
    if the parent model is finalized.

Raises:
    RuntimeError if the model instance is invalid.

Raises:
    RuntimeError if this element is not associated with a
    MultibodyPlant.)""";
        } set_enabled;
        // Symbol: drake::multibody::UniformGravityFieldElement::set_gravity_vector
        struct /* set_gravity_vector */ {
          // Source: drake/multibody/tree/uniform_gravity_field_element.h
          const char* doc =
R"""(Sets the acceleration of gravity vector, expressed in the world frame
W in m/s².)""";
        } set_gravity_vector;
      } UniformGravityFieldElement;
      // Symbol: drake::multibody::UnitInertia
      struct /* UnitInertia */ {
        // Source: drake/multibody/tree/unit_inertia.h
        const char* doc =
R"""(This class is used to represent rotational inertias for unit mass
bodies. Therefore, unlike RotationalInertia whose units are kg⋅m², the
units of a UnitInertia are those of length squared. A unit inertia is
a useful concept to represent the geometric distribution of mass in a
body regardless of the actual value of the body mass. The rotational
inertia of a body can therefore be obtained by multiplying its unit
inertia by its mass. Unit inertia matrices can also be called
**gyration** matrices and therefore we choose to represent them in
source code notation with the capital letter G. In contrast, the
capital letter I is used to represent non-unit mass rotational
inertias. This class restricts the set of allowed operations on a unit
inertia to ensure the unit-mass invariant. For instance,
multiplication by a scalar can only return a general RotationalInertia
but not a UnitInertia.

Note:
    This class has no means to check at construction from user
    provided parameters whether it actually represents the unit
    inertia or gyration matrix of a unit-mass body. However, as
    previously noted, once a unit inertia is created, a number of
    operations are disallowed to ensure the unit-mass invariant. Also
    notice that once a unit inertia is created, it *is* the unit
    inertia of *some* body, perhaps with scaled geometry from the
    user's intention.

Note:
    The methods of this class satisfy the "basic exception guarantee":
    if an exception is thrown, the program will still be in a valid
    state. Specifically, no resources are leaked, and all objects'
    invariants are intact. Be aware that UnitInertia objects may
    contain invalid inertia data in cases where input checking is
    skipped.

See also:
    https://en.cppreference.com/w/cpp/language/exceptions)""";
        // Symbol: drake::multibody::UnitInertia::AxiallySymmetric
        struct /* AxiallySymmetric */ {
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc =
R"""(Returns the unit inertia for a body B for which there exists an axis L
passing through the body's center of mass Bcm having the property that
B's moment of inertia about all lines perpendicular to L are equal.
Bodies with this "axially symmetric inertia" property include
axisymmetric cylinders or cones and propellers with 3⁺ evenly spaced
blades. For a body B with axially symmetric inertia, B's unit inertia
about a point Bp on axis L can be written in terms of a unit_vector
parallel to L; the parallel moment of inertia J about L; and the
perpendicular moment of inertia K about any line perpendicular to axis
L; as:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    G = K * Identity + (J - K) * unit_vector ⊗ unit_vector

.. raw:: html

    </details>

where ``Identity`` is the identity matrix and ⊗ denotes the tensor
product operator. See Mitiguy, P., 2016. Advanced Dynamics & Motion
Simulation.

Parameter ``moment_parallel``:
    (J) B's unit moment of inertia about axis L.

Parameter ``moment_perpendicular``:
    (K) B's unit moment of inertia about Bp for any line perpendicular
    to unit_vector.

Parameter ``unit_vector``:
    unit vector parallel to axis L, expressed in a frame E.

Returns ``G_BBp_E``:
    B's unit inertia about point Bp on B's symmetry axis, expressed in
    the same frame E as the unit_vector is expressed.

Precondition:
    Points Bp and Bcm are both on B's symmetry axis. The actual
    location of these points is not known by this function. However,
    the value of moment_perpendicular (K) is associated with point Bp.

Note:
    B's unit inertia about Bp is axially symmetric, meaning B has an
    equal moment of inertia about any line that both passes through Bp
    and is perpendicular to unit_vector.

Raises:
    RuntimeError if moment_parallel (J) or moment_perpendicular (K) is
    negative or if J > 2 K (violates the triangle inequality, see
    CouldBePhysicallyValid()) or ‖unit_vector‖ is not within 1.0E-14
    of 1.0.)""";
        } AxiallySymmetric;
        // Symbol: drake::multibody::UnitInertia::CalcPrincipalHalfLengthsAndAxesForEquivalentShape
        struct /* CalcPrincipalHalfLengthsAndAxesForEquivalentShape */ {
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc = R"""()""";
        } CalcPrincipalHalfLengthsAndAxesForEquivalentShape;
        // Symbol: drake::multibody::UnitInertia::HollowSphere
        struct /* HollowSphere */ {
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc =
R"""(Computes the unit inertia for a unit-mass hollow sphere of radius
``r`` consisting of an infinitesimally thin shell of uniform density.
The unit inertia is taken about the center of the sphere.)""";
        } HollowSphere;
        // Symbol: drake::multibody::UnitInertia::PointMass
        struct /* PointMass */ {
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc =
R"""(Construct a unit inertia for a point mass of unit mass located at
point Q, whose location in a frame F is given by the position vector
``p_FQ`` (that is, p_FoQ_F). The unit inertia ``G_QFo_F`` of point
mass Q about the origin ``Fo`` of frame F and expressed in F for this
unit mass point equals the square of the cross product matrix of
``p_FQ``. In coordinate-free form:

.. math:: G^{Q/F_o} = (^Fp^Q_\times)^2 = (^Fp^Q_\times)^T \, ^Fp^Q_\times =
              -^Fp^Q_\times \, ^Fp^Q_\times

where :math:`^Fp^Q_\times` is the cross product matrix of vector
:math:`^Fp^Q`. In source code the above expression is written as:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    G_QFo_F = px_FQ² = px_FQᵀ * px_FQ = -px_FQ * px_FQ

.. raw:: html

    </details>

where ``px_FQ`` denotes the cross product matrix of the position
vector ``p_FQ`` (expressed in F) such that the cross product with
another vector ``a`` can be obtained as ``px.cross(a) = px * a``. The
cross product matrix ``px`` is skew-symmetric. The square of the cross
product matrix is a symmetric matrix with non-negative diagonals and
obeys the triangle inequality. Matrix ``px²`` can be used to compute
the triple vector product as ``-p x (p x a) = -p.cross(p.cross(a)) =
px² * a``.)""";
        } PointMass;
        // Symbol: drake::multibody::UnitInertia::ReExpress
        struct /* ReExpress */ {
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc =
R"""(Given ``this`` unit inertia ``G_BP_E`` of a body B about a point P and
expressed in frame E, this method computes the same unit inertia
re-expressed in another frame A as ``G_BP_A = R_AE * G_BP_E *
(R_AE)ᵀ``.

Parameter ``R_AE``:
    RotationMatrix relating frames A and E.

Returns ``G_BP_A``:
    The same unit inertia for body B about point P but now
    re-expressed in frame A.)""";
        } ReExpress;
        // Symbol: drake::multibody::UnitInertia::ReExpressInPlace
        struct /* ReExpressInPlace */ {
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc =
R"""(Re-express a unit inertia in a different frame, performing the
operation in place and modifying the original object.

See also:
    ReExpress() for details.)""";
        } ReExpressInPlace;
        // Symbol: drake::multibody::UnitInertia::SetFromRotationalInertia
        struct /* SetFromRotationalInertia */ {
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc =
R"""(Sets ``this`` unit inertia from a generally non-unit inertia I
corresponding to a body with a given ``mass``.

Raises:
    RuntimeError if the provided ``mass`` is not strictly positive.)""";
        } SetFromRotationalInertia;
        // Symbol: drake::multibody::UnitInertia::ShiftFromCenterOfMass
        struct /* ShiftFromCenterOfMass */ {
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc =
R"""(Shifts this central unit inertia to a different point, and returns the
result. See ShiftFromCenterOfMassInPlace() for details.

Parameter ``p_BcmQ_E``:
    A vector from the body's centroid ``Bcm`` to point Q expressed in
    the same frame E in which ``this`` inertia is expressed.

Returns ``G_BQ_E``:
    This same unit inertia taken about a point Q instead of the
    centroid ``Bcm``.)""";
        } ShiftFromCenterOfMass;
        // Symbol: drake::multibody::UnitInertia::ShiftFromCenterOfMassInPlace
        struct /* ShiftFromCenterOfMassInPlace */ {
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc =
R"""(For a central unit inertia ``G_Bcm_E`` computed about a body's center
of mass (or centroid) ``Bcm`` and expressed in a frame E, this method
shifts this inertia using the parallel axis theorem to be computed
about a point Q. This operation is performed in place, modifying the
original object which is no longer a central inertia. On return,
``this`` is modified to be taken about point Q so can be written as
``G_BQ_E``.

Parameter ``p_BcmQ_E``:
    A vector from the body's centroid ``Bcm`` to point Q expressed in
    the same frame E in which ``this`` inertia is expressed.)""";
        } ShiftFromCenterOfMassInPlace;
        // Symbol: drake::multibody::UnitInertia::ShiftToCenterOfMass
        struct /* ShiftToCenterOfMass */ {
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc =
R"""(For the unit inertia ``G_BQ_E`` of a body or composite body B computed
about a point Q and expressed in a frame E, this method shifts this
inertia using the parallel axis theorem to be computed about the
center of mass ``Bcm`` of B. See ShiftToCenterOfMassInPlace() for
details.

Parameter ``p_QBcm_E``:
    A position vector from the about point Q to the body's centroid
    ``Bcm`` expressed in the same frame E in which ``this`` inertia is
    expressed.

Returns ``G_Bcm_E``:
    This same unit which has now been taken about point ``Bcm`` so
    that it can be written as ``G_BBcm_E``, or ``G_Bcm_E``.

Warning:
    This operation could result in a non-physical rotational inertia.
    Use with care. See ShiftToCenterOfMassInPlace() for details.)""";
        } ShiftToCenterOfMass;
        // Symbol: drake::multibody::UnitInertia::ShiftToCenterOfMassInPlace
        struct /* ShiftToCenterOfMassInPlace */ {
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc = R"""()""";
        } ShiftToCenterOfMassInPlace;
        // Symbol: drake::multibody::UnitInertia::SolidBox
        struct /* SolidBox */ {
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc =
R"""(Computes the unit inertia for a unit-mass solid box of uniform density
taken about its geometric center. If one length is zero the inertia
corresponds to that of a thin rectangular sheet. If two lengths are
zero the inertia corresponds to that of a thin rod in the remaining
direction.

Parameter ``Lx``:
    The length of the box edge in the principal x-axis.

Parameter ``Ly``:
    The length of the box edge in the principal y-axis.

Parameter ``Lz``:
    The length of the box edge in the principal z-axis.

Raises:
    RuntimeError if any of Lx, Ly, Lz are negative.)""";
        } SolidBox;
        // Symbol: drake::multibody::UnitInertia::SolidCapsule
        struct /* SolidCapsule */ {
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc =
R"""(Creates a unit inertia for a uniform density solid capsule B about its
center of mass Bcm (which is coincident with B's geometric center Bo).

Parameter ``radius``:
    radius of the cylinder/half-sphere parts of the capsule.

Parameter ``length``:
    length of cylindrical part of the capsule.

Parameter ``unit_vector``:
    unit vector defining the axial direction of the cylindrical part
    of the capsule, expressed in a frame E.

Returns ``G_BBcm_E``:
    B's unit inertia about Bcm expressed in the same frame E as the
    unit_vector is expressed.

Note:
    B's unit inertia about Bcm is axially symmetric, meaning B has an
    equal moment of inertia about any line that both passes through
    Bcm and is perpendicular to unit_vector.

Raises:
    RuntimeError if radius or length is negative or if ‖unit_vector‖
    is not within 1.0E-14 of 1.0.)""";
        } SolidCapsule;
        // Symbol: drake::multibody::UnitInertia::SolidCube
        struct /* SolidCube */ {
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc =
R"""(Computes the unit inertia for a unit-mass solid cube (a box with
equal-sized sides) of uniform density taken about its geometric
center.

Parameter ``L``:
    The length of each of the cube's sides.)""";
        } SolidCube;
        // Symbol: drake::multibody::UnitInertia::SolidCylinder
        struct /* SolidCylinder */ {
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc =
R"""(Creates a unit inertia for a uniform density solid cylinder B about
its center of mass Bcm (which is coincident with B's geometric center
Bo).

Parameter ``radius``:
    radius of the cylinder (meters).

Parameter ``length``:
    length of cylinder in unit_vector direction (meters).

Parameter ``unit_vector``:
    unit vector defining the axial direction of the cylinder,
    expressed in a frame E.

Returns ``G_BBcm_E``:
    B's unit inertia about Bcm expressed in the same frame E as the
    unit_vector is expressed.

Note:
    B's unit inertia about Bcm is axially symmetric, meaning B has an
    equal moment of inertia about any line that both passes through
    Bcm and is perpendicular to unit_vector.

Raises:
    RuntimeError if radius or length is negative or if ‖unit_vector‖
    is not within 1.0E-14 of 1.0.

See also:
    SolidCylinderAboutEnd() to calculate G_BBp_E, B's unit inertia
    about point Bp (Bp is at the center of one of the cylinder's
    circular ends).)""";
        } SolidCylinder;
        // Symbol: drake::multibody::UnitInertia::SolidCylinderAboutEnd
        struct /* SolidCylinderAboutEnd */ {
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc =
R"""(Creates a unit inertia for a uniform-density solid cylinder B about an
end-point Bp of the cylinder's axis (see below for more about Bp).

Parameter ``radius``:
    radius of cylinder (meters).

Parameter ``length``:
    length of cylinder in unit_vector direction (meters).

Parameter ``unit_vector``:
    unit vector parallel to the axis of the cylinder and directed from
    Bp to Bcm (B's center of mass), expressed in a frame E.

Returns ``G_BBp_E``:
    B's unit inertia about Bp expressed in the same frame E as the
    unit_vector is expressed.

Note:
    The position from Bp to Bcm is p_BpBcm = length / 2 * unit_vector.

Note:
    B's unit inertia about Bp is axially symmetric, meaning B has an
    equal moment of inertia about any line that both passes through Bp
    and is perpendicular to unit_vector.

Raises:
    RuntimeError if radius or length is negative or if ‖unit_vector‖
    is not within 1.0E-14 of 1.0.)""";
        } SolidCylinderAboutEnd;
        // Symbol: drake::multibody::UnitInertia::SolidEllipsoid
        struct /* SolidEllipsoid */ {
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc =
R"""(Computes the unit inertia for a unit-mass solid ellipsoid of uniform
density taken about its center. The lengths of the semi-axes of the
ellipsoid in the principal x,y,z-axes are ``a``, `b`, and ``c``
respectively.)""";
        } SolidEllipsoid;
        // Symbol: drake::multibody::UnitInertia::SolidSphere
        struct /* SolidSphere */ {
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc =
R"""(Computes the unit inertia for a unit-mass solid sphere of uniform
density and radius ``r`` taken about its center.)""";
        } SolidSphere;
        // Symbol: drake::multibody::UnitInertia::SolidTetrahedronAboutPoint
        struct /* SolidTetrahedronAboutPoint */ {
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc =
R"""(Creates a unit inertia for a unit-mass uniform density solid
tetrahedron B about a point A, from which position vectors to B's 4
vertices B0, B1, B2, B3 are measured (position vectors are all
expressed in a common frame E).

Parameter ``p0``:
    position vector p_AB0_E from point A to B0, expressed in E.

Parameter ``p1``:
    position vector p_AB1_E from point A to B1, expressed in E.

Parameter ``p2``:
    position vector p_AB2_E from point A to B2, expressed in E.

Parameter ``p3``:
    position vector p_AB3_E from point A to B3, expressed in E.

Returns ``G_BA_E``:
    B's unit inertia about point A, expressed in E.

See also:
    SolidTetrahedronAboutVertex() to efficiently calculate a unit
    inertia about a vertex of B.)""";
        } SolidTetrahedronAboutPoint;
        // Symbol: drake::multibody::UnitInertia::SolidTetrahedronAboutVertex
        struct /* SolidTetrahedronAboutVertex */ {
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc =
R"""((Advanced) Creates a unit inertia for a unit-mass uniform density
solid tetrahedron B about its vertex B0, from which position vectors
to B's other 3 vertices B1, B2, B3 are measured (vectors are all
expressed in a common frame E).

Parameter ``p1``:
    position vector p_B0B1_E from B0 to B1, expressed in E.

Parameter ``p2``:
    position vector p_B0B2_E from B0 to B2, expressed in E.

Parameter ``p3``:
    position vector p_B0B3_E from B0 to B3, expressed in E.

Returns ``G_BB0_E``:
    B's unit inertia about its vertex B0, expressed in E.

See also:
    SolidTetrahedronAboutPoint() to calculate a unit inertia about an
    arbitrary point.)""";
        } SolidTetrahedronAboutVertex;
        // Symbol: drake::multibody::UnitInertia::StraightLine
        struct /* StraightLine */ {
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc =
R"""(Creates a unit inertia for a straight line segment B about a point Bp
on the line segment.

Parameter ``moment_perpendicular``:
    Unit moment of inertia about any axis that passes through Bp and
    is perpendicular to the line segment.

Parameter ``unit_vector``:
    unit vector defining the line segment's direction, expressed in a
    frame E.

Returns ``G_BBp_E``:
    B's unit inertia about Bp, expressed in the same frame E that the
    unit_vector is expressed.

Note:
    B's unit inertia about Bp is axially symmetric, meaning B has an
    equal moment of inertia about any line that both passes through Bp
    and is perpendicular to unit_vector.

Raises:
    RuntimeError if moment_perpendicular is not positive or if
    ‖unit_vector‖ is not within 1.0E-14 of 1.0.

Note:
    B's axial moment of inertia (along the line segment) is zero.

See also:
    ThinRod() is an example of an object that is axially symmetric and
    that has a zero moment of inertia about Bp in the unit_vector
    direction.)""";
        } StraightLine;
        // Symbol: drake::multibody::UnitInertia::ThinRod
        struct /* ThinRod */ {
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc =
R"""(Creates a unit inertia for a uniform density thin rod B about its
center of mass Bcm (which is coincident with B's geometric center Bo).

Parameter ``length``:
    length of rod in unit_vector direction (meters).

Parameter ``unit_vector``:
    unit vector defining the rod's axial direction, expressed in a
    frame E.

Returns ``G_BBcm_E``:
    B's unit inertia about Bcm expressed in the same frame E that the
    unit_vector is expressed.

Note:
    B's unit inertia about Bcm is axially symmetric, meaning B has an
    equal moment of inertia about any line that both passes through
    Bcm and is perpendicular to unit_vector.

Raises:
    RuntimeError if length is not positive or if ‖unit_vector‖ is not
    within 1.0E-14 of 1.0.

Note:
    B's axial moment of inertia (along the rod) is zero..)""";
        } ThinRod;
        // Symbol: drake::multibody::UnitInertia::TriaxiallySymmetric
        struct /* TriaxiallySymmetric */ {
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc = R"""()""";
        } TriaxiallySymmetric;
        // Symbol: drake::multibody::UnitInertia::UnitInertia<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc_0args =
R"""(Default UnitInertia constructor sets all entries to NaN for quick
detection of uninitialized values.)""";
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc_3args =
R"""(Creates a unit inertia with moments of inertia ``Ixx``, `Iyy`,
``Izz``, and with each product of inertia set to zero. In debug
builds, throws RuntimeError if unit inertia constructed from these
arguments violates RotationalInertia::CouldBePhysicallyValid().)""";
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc_6args =
R"""(Creates a unit inertia with moments of inertia ``Ixx``, `Iyy`,
``Izz``, and with products of inertia ``Ixy``, `Ixz`, ``Iyz``. In
debug builds, throws RuntimeError if unit inertia constructed from
these arguments violates RotationalInertia::CouldBePhysicallyValid().)""";
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc_1args =
R"""(Constructs a UnitInertia from a RotationalInertia. This constructor
has no way to verify that the input rotational inertia actually is a
unit inertia. But the construction will nevertheless succeed, and the
values of the input rotational inertia will henceforth be considered a
valid unit inertia.

Precondition:
    The user is responsible for passing a valid rotational inertia.)""";
        } ctor;
        // Symbol: drake::multibody::UnitInertia::cast
        struct /* cast */ {
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc =
R"""(Returns a new UnitInertia object templated on ``Scalar`` initialized
from the value of ``this`` unit inertia.

Template parameter ``Scalar``:
    The scalar type on which the new unit inertia will be templated.

Note:
    ``UnitInertia<From>::cast<To>()`` creates a new
    ``UnitInertia<To>`` from a ``UnitInertia<From>`` but only if type
    ``To`` is constructible from type ``From``. As an example of this,
    ``UnitInertia<double>::cast<AutoDiffXd>()`` is valid since
    ``AutoDiffXd a(1.0)`` is valid. However,
    ``UnitInertia<AutoDiffXd>::cast<double>()`` is not.)""";
        } cast;
        // Symbol: drake::multibody::UnitInertia::operator*=
        struct /* operator_imul */ {
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc = R"""()""";
        } operator_imul;
        // Symbol: drake::multibody::UnitInertia::operator+=
        struct /* operator_iadd */ {
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc = R"""()""";
        } operator_iadd;
        // Symbol: drake::multibody::UnitInertia::operator-=
        struct /* operator_isub */ {
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc = R"""()""";
        } operator_isub;
        // Symbol: drake::multibody::UnitInertia::operator/=
        struct /* operator_idiv */ {
          // Source: drake/multibody/tree/unit_inertia.h
          const char* doc = R"""()""";
        } operator_idiv;
      } UnitInertia;
      // Symbol: drake::multibody::UniversalJoint
      struct /* UniversalJoint */ {
        // Source: drake/multibody/tree/universal_joint.h
        const char* doc =
R"""(This joint models a universal joint allowing two bodies to rotate
relative to one another with two degrees of freedom. A universal joint
can be thought of as a mechanism consisting of three bodies; the
parent body P, an intermediate cross-shaped body I, and the child body
B. In a physical universal joint, body I has a significantly smaller
mass than P or B. This universal joint model corresponds to the
mathematical limit of having a body I of negligible mass. Given a
frame F attached to the parent body P and a frame M attached to the
child body B (see the Joint class's documentation), the orientation of
M in F can then naturally be defined as follows using a body fixed
rotation sequence. A first rotation of θ₁ about Fx defines the
orientation R_FI of the intermediate frame I attached to body I
(notice that by definition Ix = Fx at all times). A second rotation of
θ₂ about Iy defines the orientation R_IM of frame M (notice that by
definition My = Iy at all times). Mathematically, the orientation of
frame M in F is given by


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    R_FM(q) = R_FI(θ₁) * R_IM(θ₂)

.. raw:: html

    </details>

No translational motion of M in F is allowed and the origins, ``Mo``
and ``Fo``, of frames M and F respectively remain coincident. The
angles of rotation about F's x-axis and M's y-axis, along with their
rates, specify the state of the joint. Zero θ₁, θ₂ angles corresponds
to frames F, I, and M being coincident. Angles (θ₁, θ₂) are defined to
be positive according to the right-hand-rule with the thumb aligned in
the direction of their respective axes.)""";
        // Symbol: drake::multibody::UniversalJoint::DoAddInDamping
        struct /* DoAddInDamping */ {
          // Source: drake/multibody/tree/universal_joint.h
          const char* doc =
R"""(Joint<T> override called through public NVI, Joint::AddInDamping().
Therefore arguments were already checked to be valid. This method adds
into ``forces`` a dissipative torque according to the viscous law ``τ
= -d⋅ω``, with d the damping coefficient (see default_damping()).)""";
        } DoAddInDamping;
        // Symbol: drake::multibody::UniversalJoint::DoAddInOneForce
        struct /* DoAddInOneForce */ {
          // Source: drake/multibody/tree/universal_joint.h
          const char* doc =
R"""(Joint<T> override called through public NVI, Joint::AddInForce().
Therefore arguments were already checked to be valid. For a
UniversalJoint, we must always have ``joint_dof < 2`` since there are
two degrees of freedom (num_velocities() == 2). ``joint_tau`` is the
torque applied about the axis specified by ``joint_dof``, the x-axis
of the parent frame F if ``joint_dof = 0`` or the y-axis of the child
frame M if ``joint_dof = 1``. The torque is applied to the body
declared as child (according to the universal joint's constructor) at
the origin of the child frame M (which is coincident with the origin
of the parent frame F at all times). The torque is defined to be
positive according to the right-hand-rule with the thumb aligned in
the direction of the selected axis. That is, a positive torque causes
a positive rotational acceleration (of the child body frame).)""";
        } DoAddInOneForce;
        // Symbol: drake::multibody::UniversalJoint::UniversalJoint<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/universal_joint.h
          const char* doc =
R"""(Constructor to create a universal joint between two bodies so that
frame F attached to the parent body P and frame M attached to the
child body B rotate as described in the class's documentation. See
class documentation for details on the angles defining orientation.
This constructor signature creates a joint with no joint limits, i.e.
the joint position, velocity and acceleration limits are the pair
``(-∞, ∞)``. These can be set using the Joint methods
set_position_limits(), set_velocity_limits() and
set_acceleration_limits(). The first three arguments to this
constructor are those of the Joint class constructor. See the Joint
class's documentation for details. The additional parameters are:

Parameter ``damping``:
    Viscous damping coefficient, in N⋅m⋅s, used to model losses within
    the joint. See documentation of default_damping() for details on
    modelling of the damping torque.

Raises:
    RuntimeError if damping is negative.)""";
        } ctor;
        // Symbol: drake::multibody::UniversalJoint::default_damping
        struct /* default_damping */ {
          // Source: drake/multibody/tree/universal_joint.h
          const char* doc =
R"""(Returns ``this`` joint's default damping constant in N⋅m⋅s. The
damping torque (in N⋅m) is modeled as ``τᵢ = -damping⋅ωᵢ, i = 1, 2``
i.e. opposing motion, with ωᵢ the angular rates about the i-th axis
for ``this`` joint (see get_angular_rates())and τᵢ the torque on child
body B about the same i-th axis.)""";
        } default_damping;
        // Symbol: drake::multibody::UniversalJoint::get_angles
        struct /* get_angles */ {
          // Source: drake/multibody/tree/universal_joint.h
          const char* doc =
R"""(Gets the rotation angles of ``this`` joint from ``context``. See class
documentation for the definition of these angles.

Parameter ``context``:
    The context of the model this joint belongs to.

Returns:
    The angle coordinates of ``this`` joint stored in the ``context``
    ordered as (θ₁, θ₂).)""";
        } get_angles;
        // Symbol: drake::multibody::UniversalJoint::get_angular_rates
        struct /* get_angular_rates */ {
          // Source: drake/multibody/tree/universal_joint.h
          const char* doc =
R"""(Gets the rates of change, in radians per second, of ``this`` joint's
angles (see class documentation) from ``context``.

Parameter ``context``:
    The context of the model this joint belongs to.

Returns:
    The rates of change of ``this`` joint's angles as stored in the
    ``context``.)""";
        } get_angular_rates;
        // Symbol: drake::multibody::UniversalJoint::get_default_angles
        struct /* get_default_angles */ {
          // Source: drake/multibody/tree/universal_joint.h
          const char* doc =
R"""(Gets the default angles for ``this`` joint. Wrapper for the more
general ``Joint::default_positions()``.

Returns:
    The default angles of ``this`` stored in ``default_positions_``)""";
        } get_default_angles;
        // Symbol: drake::multibody::UniversalJoint::set_angles
        struct /* set_angles */ {
          // Source: drake/multibody/tree/universal_joint.h
          const char* doc =
R"""(Sets the ``context`` so that the generalized coordinates corresponding
to the rotation angles of ``this`` joint equals ``angles``.

Parameter ``context``:
    The context of the model this joint belongs to.

Parameter ``angles``:
    The desired angles in radians to be stored in ``context`` ordered
    as (θ₁, θ₂). See class documentation for details.

Returns:
    a constant reference to ``this`` joint.)""";
        } set_angles;
        // Symbol: drake::multibody::UniversalJoint::set_angular_rates
        struct /* set_angular_rates */ {
          // Source: drake/multibody/tree/universal_joint.h
          const char* doc =
R"""(Sets the rates of change, in radians per second, of this ``this``
joint's angles (see class documentation) to ``theta_dot``. The new
rates of change get stored in ``context``.

Parameter ``context``:
    The context of the model this joint belongs to.

Parameter ``theta_dot``:
    The desired rates of change of ``this`` joints's angles in radians
    per second.

Returns:
    a constant reference to ``this`` joint.)""";
        } set_angular_rates;
        // Symbol: drake::multibody::UniversalJoint::set_default_angles
        struct /* set_default_angles */ {
          // Source: drake/multibody/tree/universal_joint.h
          const char* doc =
R"""(Sets the default angles of this joint.

Parameter ``angles``:
    The desired default angles of the joint)""";
        } set_default_angles;
        // Symbol: drake::multibody::UniversalJoint::set_random_angles_distribution
        struct /* set_random_angles_distribution */ {
          // Source: drake/multibody/tree/universal_joint.h
          const char* doc =
R"""(Sets the random distribution that angles of this joint will be
randomly sampled from. See class documentation for details on the
definition of the angles.)""";
        } set_random_angles_distribution;
        // Symbol: drake::multibody::UniversalJoint::type_name
        struct /* type_name */ {
          // Source: drake/multibody/tree/universal_joint.h
          const char* doc = R"""()""";
        } type_name;
      } UniversalJoint;
      // Symbol: drake::multibody::WeldJoint
      struct /* WeldJoint */ {
        // Source: drake/multibody/tree/weld_joint.h
        const char* doc =
R"""(This Joint fixes the relative pose between two frames as if "welding"
them together.)""";
        // Symbol: drake::multibody::WeldJoint::DoAddInOneForce
        struct /* DoAddInOneForce */ {
          // Source: drake/multibody/tree/weld_joint.h
          const char* doc =
R"""(Joint<T> override called through public NVI, Joint::AddInForce().
Since frame F and M are welded together, it is physically not possible
to apply forces between them. Therefore this method throws an
exception if invoked.)""";
        } DoAddInOneForce;
        // Symbol: drake::multibody::WeldJoint::WeldJoint<T>
        struct /* ctor */ {
          // Source: drake/multibody/tree/weld_joint.h
          const char* doc =
R"""(Constructor for a WeldJoint between a ``frame_on_parent_F`` and a
``frame_on_child_M`` so that their relative pose ``X_FM`` is fixed as
if they were "welded" together.)""";
        } ctor;
        // Symbol: drake::multibody::WeldJoint::X_FM
        struct /* X_FM */ {
          // Source: drake/multibody/tree/weld_joint.h
          const char* doc = R"""(Returns the pose X_FM of frame M in F.)""";
        } X_FM;
        // Symbol: drake::multibody::WeldJoint::type_name
        struct /* type_name */ {
          // Source: drake/multibody/tree/weld_joint.h
          const char* doc = R"""()""";
        } type_name;
      } WeldJoint;
      // Symbol: drake::multibody::default_model_instance
      struct /* default_model_instance */ {
        // Source: drake/multibody/tree/multibody_tree_indexes.h
        const char* doc =
R"""(Returns the model instance which contains all tree elements with no
explicit model instance specified.)""";
      } default_model_instance;
      // Symbol: drake::multibody::to_string
      struct /* to_string */ {
        // Source: drake/multibody/tree/rotational_inertia.h
        const char* doc_1args_constRotationalInertia =
R"""(Returns the string representation of a RotationalInertia object.)""";
        // Source: drake/multibody/tree/spatial_inertia.h
        const char* doc_1args_constSpatialInertia =
R"""(Returns the string representation of a SpatialInertia object.)""";
      } to_string;
      // Symbol: drake::multibody::world_frame_index
      struct /* world_frame_index */ {
        // Source: drake/multibody/tree/multibody_tree_indexes.h
        const char* doc =
R"""(For every MultibodyPlant the **world** frame *always* has this unique
index and it is always zero.)""";
      } world_frame_index;
      // Symbol: drake::multibody::world_index
      struct /* world_index */ {
        // Source: drake/multibody/tree/multibody_tree_indexes.h
        const char* doc =
R"""(For every MultibodyPlant the **world** body *always* has this unique
index and it is always zero.)""";
      } world_index;
      // Symbol: drake::multibody::world_model_instance
      struct /* world_model_instance */ {
        // Source: drake/multibody/tree/multibody_tree_indexes.h
        const char* doc =
R"""(Returns the model instance containing the *world* body. For every
MultibodyPlant the **world** body *always* has this unique model
instance and it is always zero (as described in #3088).)""";
      } world_model_instance;
    } multibody;
  } drake;
} pydrake_doc_multibody_tree;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
