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

// #include "drake/multibody/math/spatial_acceleration.h"
// #include "drake/multibody/math/spatial_algebra.h"
// #include "drake/multibody/math/spatial_force.h"
// #include "drake/multibody/math/spatial_momentum.h"
// #include "drake/multibody/math/spatial_vector.h"
// #include "drake/multibody/math/spatial_velocity.h"

// Symbol: pydrake_doc_multibody_math
constexpr struct /* pydrake_doc_multibody_math */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::multibody
    struct /* multibody */ {
      // Symbol: drake::multibody::SpatialAcceleration
      struct /* SpatialAcceleration */ {
        // Source: drake/multibody/math/spatial_acceleration.h
        const char* doc =
R"""(This class represents a *spatial acceleration* A and has 6 elements
with an angular (rotational) acceleration α (3-element vector) on top
of a translational (linear) acceleration 𝐚 (3-element vector). Spatial
acceleration represents the rotational and translational acceleration
of a frame B with respect to a *measured-in* frame M. This class
assumes that both the angular acceleration α and translational
acceleration 𝐚 are expressed in the same *expressed-in* frame E. This
class only stores 6 elements (namely α and 𝐚) and does not store the
underlying frames B, M, E. The user is responsible for explicitly
tracking the underlying frames with multibody_quantities "monogram
notation". For example, A_MB_E denotes frame B's spatial acceleration
measured in frame M, expressed in frame E and contains alpha_MB_E (B's
angular acceleration measured in M, expressed in E) and a_MBo_E (Bo's
translational acceleration measured in M, expressed in E), where Bo is
frame B's origin point. For an multibody_frames_and_bodies "offset
frame" Bp, the monogram notation A_MBp_E denotes frame Bp's spatial
acceleration measured in M, expressed in E. Details on spatial vectors
and monogram notation are in sections multibody_spatial_vectors and
multibody_quantities.

The typeset for A_MB is :math:`\,{^MA^B}` and its definition is
:math:`^MA^B = \frac{^Md}{dt}\,{^MV^B}\,`, where :math:`{^MV^B}` is
frame B's spatial velocity in frame M and :math:`\frac{^Md}{dt}`
denotes the time derivative taken in frame M. To differentiate a
vector, we need to specify in what frame the time derivative is taken,
see [Mitiguy 2022, §7.2] for an in-depth discussion. Time derivatives
in different frames are related by the "Transport Theorem", which in
Drake is implemented in
drake∷math∷ConvertTimeDerivativeToOtherFrame(). In source code
(monogram) notation, we write A_MB = DtM(V_MB), where DtM() denotes
the time derivative in frame M. Details on vector differentiation is
in section Dt_multibody_quantities.

[Mitiguy 2022] Mitiguy, P., 2022. Advanced Dynamics & Motion
Simulation.)""";
        // Symbol: drake::multibody::SpatialAcceleration::ComposeWithMovingFrameAcceleration
        struct /* ComposeWithMovingFrameAcceleration */ {
          // Source: drake/multibody/math/spatial_acceleration.h
          const char* doc =
R"""(Compose ``this`` spatial acceleration (measured in some frame M) with
the spatial acceleration of another frame to form the 𝐨𝐭𝐡𝐞𝐫 frame's
spatial acceleration in frame M. Herein, ``this`` is the spatial
acceleration of a frame (designated B) in frame M and the 𝐨𝐭𝐡𝐞𝐫 frame
is designated C.

Parameter ``position_of_moving_frame``:
    which is the position vector p_BoCo_E (from frame B's origin Bo to
    frame C's origin Co), expressed in frame E. p_BoCo_E must have the
    same expressed-in frame E as ``this``, where ``this`` is A_MB_E
    (frame B's spatial acceleration measured in M, expressed in E).

Parameter ``angular_velocity_of_this_frame``:
    which is ω_MB_E, frame B's angular velocity measured in frame W
    and expressed in frame E.

Parameter ``velocity_of_moving_frame``:
    which is V_BC_E, frame C's spatial velocity measured in frame B,
    expressed in frame E.

Parameter ``acceleration_of_moving_frame``:
    which is A_BC_E, frame C's spatial acceleration measured in frame
    B, expressed in frame E.

Returns ``A_MC_E``:
    frame C's spatial acceleration measured in frame M, expressed in
    frame E.

See also:
    SpatialVelocity∷ComposeWithMovingFrameVelocity(). Use Shift() if
    frames B and C are both fixed to the same frame or body, i.e.,
    velocity_of_moving_frame = 0 and acceleration_of_moving_frame = 0.

Note:
    The returned spatial acceleration A_MC_E contains an angular
    acceleration α_MC_E and translational acceleration a_MCo_E that
    are calculated as:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    α_MC_E  = α_MB_E + α_BC_E + ω_MB_E x ω_BC_E
     a_MCo_E = a_BCo_E + α_MB_E x p_BoCo_E + ω_MB_E x (ω_MB_E x p_BoCo_E)
             + 2 ω_MB_E x v_BCo_E + a_BCo_E

.. raw:: html

    </details>

If frame C is rigidly fixed to frame B, A_BC_E = 0 and V_BC_E = 0 and
this method produces a Shift() operation (albeit inefficiently). The
previous equations show composing spatial acceleration is not simply
adding A_MB + A_BC and these equations differ significantly from their
spatial velocity counterparts. For example, angular velocities simply
add as


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ω_MC = ω_MB + ω_BC,   but 3D angular acceleration is more complicated as
      α_MC = α_MB + α_BC + ω_MB x ω_BC

.. raw:: html

    </details>

** Derivation **

* Rotational acceleration component *

ω_MC (frame C's angular velocity in frame M) can be calculated with
the angular velocity addition theorem as


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ω_MC = ω_MB + ω_BC

.. raw:: html

    </details>

α_MC (frame C's angular acceleration measured in frame M) is defined
as the time-derivative in frame M of ω_MC, and can be calculated using
the "Transport Theorem" (Golden rule for vector differentation) which
converts the time-derivative of a vector in frame M to frame B, e.g.,
as DtM(ω_BC) = DtB(ω_BC) + ω_MB x ω_BC, as


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    α_MC = DtM(ω_MC) = DtM(ω_MB) + DtM(ω_BC)
                       =     α_MB  + DtB(ω_BC) + ω_MB x ω_BC
                       =     α_MB  +     α_BC  + ω_MB x ω_BC   (End of proof).

.. raw:: html

    </details>

* Translational acceleration component *

v_MCo (frame C's translational velocity in frame M) is calculated in
SpatialVelocity∷ComposeWithMovingFrameVelocity) as


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    v_MCo = v_MBo + ω_MB x p_BoCo + v_BCo

.. raw:: html

    </details>

a_MCo (frame C's translational acceleration measured in frame M) is
defined as the time-derivative in frame M of v_MCo, calculated as


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    a_MCo = DtM(v_MCo)                             Definition.
           = DtM(v_MBo + ω_MB x p_BoCo + v_BCo)     Substitution.
           = DtM(v_MBo) + DtM(ω_MB) x p_BoCo + ω_MB x DtM(p_BoCo) + DtM(v_BCo)
           =     a_MBo  +     α_MB  x p_BoCo + ω_MB x DtM(p_BoCo) + DtM(v_BCo)

.. raw:: html

    </details>

The last two terms are modified using the "Transport Theorem" (Golden
rule for vector differentation) which converts time-derivatives of
vectors in frame M to frame B via DtM(vec) = DtB(vec) + ω_MB x vec.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    DtM(p_BoCo) = DtB(p_BoCo) + ω_MB x p_BoCo
                 =     v_BCo   + ω_MB x p_BoCo
     DtM(v_BCo)  = DtB(v_BCo)  + ω_MB x v_BCo
                 =     a_BCo   + ω_MB x v_BCo

.. raw:: html

    </details>

Combining the last few equations proves the formula for a_MCo as:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    a_MCo = a_MBo + α_MB x p_BoCo + ω_MB x (ω_MB x p_BoCo)
            + 2 ω_MB x v_BCo + a_BCo                           (End of proof).

.. raw:: html

    </details>

Some terms in the previous equation have names, e.g.,


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    centripetal acceleration   ω_MB x (ω_MB x p_BoCo)
      Coriolis acceleration    2 ω_MB x v_BCo
      Coincident point acceleration, i.e., acceleration of the point of frame
      B coincident with Co      a_MBo + α_MB x p_BoCo + ω_MB x (ω_MB x p_BoCo)

.. raw:: html

    </details>

Note: The coincident point acceleration can be calculated with a
Shift().

Note: The three cross products appearing in the previous calculation
of a_MCo can be reduced to one, possibly improving efficiency via


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ω_MB x (ω_MB x p_BoCo) + 2 ω_MB x v_BCo = ω_MB x (v_MCo - v_MBo + v_BCo)

.. raw:: html

    </details>

To show this, we rearrange and substitute our expression for v_MCo.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    v_MCo = v_MBo + ω_MB x p_BoCo + v_BCo        which rearranges to
      ω_MB x p_BoCo = v_MCo - v_MBo - v_BCo.             Substitution produces
      ω_MB x (ω_MB x p_BoCo) = ω_MB x (v_MCo - v_MBo - v_BCo)           Hence,
      ω_MB x (ω_MB x p_BoCo) + 2 ω_MB x v_BCo = ω_MB x (v_MCo - v_MBo + v_BCo)

.. raw:: html

    </details>)""";
        } ComposeWithMovingFrameAcceleration;
        // Symbol: drake::multibody::SpatialAcceleration::Shift
        struct /* Shift */ {
          // Source: drake/multibody/math/spatial_acceleration.h
          const char* doc =
R"""(Shifts a SpatialAcceleration from a frame B to a frame C, where both B
and C are fixed to the same frame or rigid body.

Parameter ``offset``:
    which is the position vector p_BoCo_E from Bo (frame B's origin)
    to Co (frame C's origin), expressed in frame E. p_BoCo_E must have
    the same expressed-in frame E as ``this`` spatial acceleration,
    where ``this`` is A_MB_E (frame B's spatial acceleration measured
    in M, expressed in E).

Parameter ``angular_velocity_of_this_frame``:
    which is ω_MB_E, frame B's angular velocity measured in frame M
    and expressed in frame E.

Returns ``A_MC_E``:
    which is frame C's spatial acceleration measured in frame M,
    expressed in frame E.

Note:
    Shift() differs from ShiftInPlace() in that Shift() does not
    modify ``this`` whereas ShiftInPlace() does modify ``this``.

See also:
    ShiftInPlace() for more information and how A_MC_E is calculated.
    Use ComposeWithMovingFrameAcceleration() if frame C is moving on
    frame B.)""";
        } Shift;
        // Symbol: drake::multibody::SpatialAcceleration::ShiftInPlace
        struct /* ShiftInPlace */ {
          // Source: drake/multibody/math/spatial_acceleration.h
          const char* doc =
R"""(In-place shift of a SpatialAcceleration from a frame B to a frame C,
where both B and C are fixed to the same frame or rigid body. On
entry, ``this`` is A_MB_E (frame B's spatial acceleration measured in
a frame M and expressed in a frame E). On return ``this`` is modified
to A_MC_E (frame C's spatial acceleration measured in frame M and
expressed in frame E). The components of A_MC_E are calculated as:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    α_MC_E = α_MB_E           (angular acceleration of ``this`` is unchanged).
     a_MCo_E = a_MBo_E + α_MB_E x p_BoCo_E + ω_MB_E x (ω_MB_E x p_BoCo_E)

.. raw:: html

    </details>

Parameter ``offset``:
    which is the position vector p_BoCo_E from Bo (frame B's origin)
    to Co (frame C's origin), expressed in frame E. p_BoCo_E must have
    the same expressed-in frame E as ``this`` spatial acceleration.

Parameter ``angular_velocity_of_this_frame``:
    which is ω_MB_E, frame B's angular velocity measured in frame W
    and expressed in frame E.

See also:
    Shift() to shift spatial acceleration without modifying ``this``.
    Use ComposeWithMovingFrameAcceleration() if frame C is moving on
    frame B.

** Derivation **

* Rotational acceleration component *

Frame B and frame C are fixed (e.g., welded) to the same rigid object.
Hence frames B and C always rotate together at the same rate and:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ω_MC_E = ω_MB_E
      α_MC_E = α_MB_E

.. raw:: html

    </details>

* Translational acceleration component *

Since frames B and C are fixed to the same rigid object, the
translational velocity of Co (frame C's origin) measured in frame M
can be calculated as


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    v_MCo = v_MBo + ω_MB x p_BoCo

.. raw:: html

    </details>

Point Co's translational acceleration in frame M is:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    a_MCo = DtM(v_MCo)                      (definition of acceleration).
            = DtM(v_MBo  + ω_MB x p_BoCo)     (substitution)
            = DtM(v_MBo) + DtM(ω_MB) x p_BoCo + ω_MB x DtM(p_BoCo)
            =     a_MBo  +     α_MB  x p_BoCo + ω_MB x DtM(p_BoCo)

.. raw:: html

    </details>

The "Transport Theorem" converts the time-derivative of the last term
from DtM() to DtB() -- see math∷ConvertTimeDerivativeToOtherFrame(),
as


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    DtM(p_BoCo) = DtB(p_BoCo) + ω_MB x p_BoCo
                  =          0  + ω_MB x p_BoCo

.. raw:: html

    </details>)""";
        } ShiftInPlace;
        // Symbol: drake::multibody::SpatialAcceleration::ShiftWithZeroAngularVelocity
        struct /* ShiftWithZeroAngularVelocity */ {
          // Source: drake/multibody/math/spatial_acceleration.h
          const char* doc =
R"""((Advanced) Given ``this`` spatial acceleration A_MB of a frame B
measured in a frame M, shifts SpatialAcceleration from frame B to a
frame C (i.e., A_MB to A_MC), where both B and C are fixed to the same
frame or rigid body and where ω_MB = 0 (frame B's angular velocity in
frame M is zero).

Parameter ``offset``:
    which is the position vector p_BoCo_E from Bo (frame B's origin)
    to Co (frame C's origin), expressed in frame E. p_BoCo_E must have
    the same expressed-in frame E as ``this`` spatial acceleration,
    where ``this`` is A_MB_E (frame B's spatial acceleration measured
    in M, expressed in E).

Returns ``A_MC_E``:
    which is frame C's spatial acceleration measured in frame M,
    expressed in frame E.

See also:
    ShiftInPlace() for more information and how A_MC_E is calculated.

Note:
    ShiftWithZeroAngularVelocity() speeds the Shift() computation when
    ω_MB = 0, even if α_MB ≠ 0 (α_MB is stored in ``this``).)""";
        } ShiftWithZeroAngularVelocity;
        // Symbol: drake::multibody::SpatialAcceleration::SpatialAcceleration<T>
        struct /* ctor */ {
          // Source: drake/multibody/math/spatial_acceleration.h
          const char* doc_0args =
R"""(Default constructor. In Release builds, all 6 elements of a newly
constructed spatial acceleration are uninitialized (for speed). In
Debug builds, the 6 elements are set to NaN so that invalid operations
on an uninitialized spatial acceleration fail fast (fast bug
detection).)""";
          // Source: drake/multibody/math/spatial_acceleration.h
          const char* doc_2args =
R"""(Constructs a spatial acceleration A from an angular acceleration α
(alpha) and a translational acceleration 𝐚.)""";
          // Source: drake/multibody/math/spatial_acceleration.h
          const char* doc_1args =
R"""(Constructs a spatial acceleration A from an Eigen expression that
represents a 6-element vector, i.e., a 3-element angular acceleration
α and a 3-element translational acceleration 𝐚. This constructor will
assert the size of A is six (6) either at compile-time for fixed sized
Eigen expressions or at run-time for dynamic sized Eigen expressions.)""";
        } ctor;
      } SpatialAcceleration;
      // Symbol: drake::multibody::SpatialForce
      struct /* SpatialForce */ {
        // Source: drake/multibody/math/spatial_force.h
        const char* doc =
R"""(This class represents a *spatial force* F (also called a *wrench*) and
has 6 elements with a torque 𝛕 (3-element vector) on top of a force 𝐟
(3-element vector). Frequently, a spatial force represents the
replacement of a set S of forces on a frame B with an equivalent set
consisting of a torque 𝛕 applied to frame B which is equal to the
moment of the set S about a point Bp of B together with a force 𝐟
applied to Bp, where 𝐟 is equal to set S's resultant force. This class
assumes that both the torque 𝛕 and force 𝐟 have the same
*expressed-in* frame E. This class only stores 6 elements (namely 𝛕
and 𝐟) and does not store the underlying frame B, application point
Bp, or expressed-in frame E. The user is responsible for explicitly
tracking these underlying quantities with multibody_quantities
"monogram notation". For example, F_B_E denotes a spatial force on
frame B with application point Bo (frame B's origin), expressed in
frame E and contains tau_B_E (torque 𝛕 applied to frame B, expressed
in frame E) and f_Bo_E (force 𝐟 applied to Bo, expressed in frame E).

The monogram notation F_Bp has a typeset equivalent :math:`{F^{Bp}}`
which denotes the spatial force applied to point Bp of frame B. F_Bp
contains a torque tau_B (:math:`{\tau^B}`) applied to frame B and a
force f_Bp (:math:`{f^{Bp}}`) applied to point Bp of frame B. Details
on spatial vectors and monogram notation are in sections
multibody_spatial_vectors and multibody_quantities.)""";
        // Symbol: drake::multibody::SpatialForce::Shift
        struct /* Shift */ {
          // Source: drake/multibody/math/spatial_force.h
          const char* doc_1args =
R"""(Shifts a SpatialForce from one point fixed on frame B to another point
fixed on frame B.

Parameter ``offset``:
    which is the position vector p_BpBq_E from point Bp (fixed on
    frame B) to point Bq (fixed on frame B), expressed in frame E.
    p_BpBq_E must have the same expressed-in frame E as ``this``
    spatial force, where ``this`` is F_Bp_E (spatial force on Bp,
    expressed in frame E).

Returns ``F_Bq_E``:
    which is the spatial force on Bq, expressed in frame E.

See also:
    Member function ShiftInPlace() to shift one spatial force
    (modifying ``this``) and static functions ShiftInPlace() and
    Shift() to shift multiple spatial forces (with or without
    modifying the input parameter spatial_forces).)""";
          // Source: drake/multibody/math/spatial_force.h
          const char* doc_3args =
R"""(Shifts a matrix of spatial forces from one point fixed on frame B to
another point fixed on frame B.

Parameter ``spatial_forces``:
    which is the 6 x n matrix F_Bp_E_all, where each of the n columns
    is a spatial force applied to a point Bp of frame B, and where
    each spatial forces is expressed in a frame E.

Parameter ``offset``:
    which is the position vector p_BpBq_E from point Bp (fixed on
    frame B) to a point Bq (fixed on frame B), expressed in frame E.
    p_BpBq_E must have the same expressed-in frame E as in
    spatial_forces.

Parameter ``shifted_forces``:
    which is the 6 x n matrix F_Bq_E_all, where each of the n columns
    is a spatial force which was shifted from point Bp (fixed on B) to
    point Bq (fixed on B). On output, each spatial force contained in
    shifted_forces is expressed in frame E.

Precondition:
    shifted_forces must be non-null and must point to a 6 x n matrix
    (i.e., it must be the same size as the input matrix
    spatial_forces).

See also:
    Static function ShiftInPlace() to shift multiple spatial forces
    with modification to the input parameter spatial_forces and member
    functions ShiftInPlace() and Shift() to shift one spatial force
    (with or without modifying ``this``).

Note:
    Although this Shift() function will work properly if the input and
    output matrices are the same (i.e., spatial_forces =
    shifted_forces), it is faster and more efficient (avoids copying)
    to use ShiftInPlace().)""";
        } Shift;
        // Symbol: drake::multibody::SpatialForce::ShiftInPlace
        struct /* ShiftInPlace */ {
          // Source: drake/multibody/math/spatial_force.h
          const char* doc_1args =
R"""(In-place shift of a SpatialForce from one point fixed on frame B to
another fixed on frame B. On entry, ``this`` is F_Bp_E (spatial force
on point Bp of frame B, expressed in a frame E). On return ``this`` is
modified to F_Bq_E (spatial force on point Bq of frame B, expressed in
frame E). The components of F_Bq_E are calculated as:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    τ_B  = τ_B -  p_BpBq x f_Bp    (the torque 𝛕 stored in ``this`` changes).
     f_Bq_E = f_Bp_E              (the force 𝐟 stored in ``this`` is unchanged).

.. raw:: html

    </details>

Parameter ``offset``:
    which is the position vector p_BpBq_E from point Bp (fixed on
    frame B) to point Bq (fixed on frame B), expressed in frame E.
    p_BpBq_E must have the same expressed-in frame E as ``this``
    spatial force.

Note:
    There are related functions that shift spatial momentum and
    spatial velocity (see SpatialMomentum∷Shift() and
    SpatialVelocity:Shift()).

See also:
    Member function Shift() to shift one spatial force without
    modifying ``this`` and static functions ShiftInPlace() and Shift()
    to shift multiple spatial forces (with or without modifying the
    input parameter spatial_forces).)""";
          // Source: drake/multibody/math/spatial_force.h
          const char* doc_2args =
R"""(Shifts a matrix of spatial forces from one point fixed on frame B to
another point fixed on frame B.

Parameter ``spatial_forces``:
    which is the 6 x n matrix F_Bp_E_all, where each of the n columns
    is a spatial force expressed in a frame E. On input, each spatial
    force is applied to a point Bp of frame B. On output, each spatial
    force has been shifted to a point Bq of frame B. In other words,
    on output, spatial_forces = F_Bq_E_all.

Parameter ``offset``:
    which is the position vector p_BpBq_E from point Bp (fixed on
    frame B) to a point Bq (fixed on frame B), expressed in frame E.
    p_BpBq_E must have the same expressed-in frame E as in
    spatial_forces.

See also:
    Static function Shift() to shift multiple spatial forces without
    modifying the input parameter spatial_forces and member functions
    ShiftInPlace() and Shift() to shift one spatial force (with or
    without modifying ``this``).)""";
        } ShiftInPlace;
        // Symbol: drake::multibody::SpatialForce::SpatialForce<T>
        struct /* ctor */ {
          // Source: drake/multibody/math/spatial_force.h
          const char* doc_0args =
R"""(Default constructor. In Release builds, all 6 elements of a newly
constructed spatial force are uninitialized (for speed). In Debug
builds, the 6 elements are set to NaN so that invalid operations on an
uninitialized spatial force fail fast (fast bug detection).)""";
          // Source: drake/multibody/math/spatial_force.h
          const char* doc_2args =
R"""(Constructs a spatial force F from a torque 𝛕 (tau) and a force 𝐟.)""";
          // Source: drake/multibody/math/spatial_force.h
          const char* doc_1args =
R"""(Constructs a spatial force F from an Eigen expression that represents
a 6-element vector, i.e., a 3-element torque 𝛕 and a 3-element force
𝐟. This constructor will assert the size of F is six (6) either at
compile-time for fixed sized Eigen expressions or at run-time for
dynamic sized Eigen expressions.)""";
        } ctor;
        // Symbol: drake::multibody::SpatialForce::dot
        struct /* dot */ {
          // Source: drake/multibody/math/spatial_force.h
          const char* doc =
R"""(Calculates the power generated by a spatial force. For an arbitrary
frame B, calculates the dot-product of ``this`` = F_B_E (frame B's
spatial force, expressed in frame E) with V_MB_E (frame B's spatial
velocity measured in a frame M, expressed in a frame E).

Parameter ``velocity``:
    which is V_MB_E, frame B's spatial velocity measured in frame M,
    expressed in the same frame E as ``this`` = F_B_E.

Returns:
    Power of spatial force F_B_E in frame M, i.e., F_B_E ⋅ V_MB_E.

Note:
    Just as equating force 𝐅 to mass * acceleration as 𝐅 = m𝐚 relies
    on acceleration 𝐚 being measured in a world frame (also called an
    inertial or Newtonian frame), equating power = dK/dt (where K is
    kinetic energy) relies on K being measured in a world frame.
    Hence, it is unusual to use this method unless frame M is the
    world frame W.

Note:
    Although the spatial vectors F_B_E and V_MB_E must have the same
    expressed-in frame E, the returned scalar is independent of frame
    E.)""";
        } dot;
      } SpatialForce;
      // Symbol: drake::multibody::SpatialMomentum
      struct /* SpatialMomentum */ {
        // Source: drake/multibody/math/spatial_momentum.h
        const char* doc =
R"""(This class represents a *spatial momentum* L and has 6 elements with
an angular (rotational) momentum 𝐡 (3-element vector) on top of a
translational (linear) momentum 𝐥 (3-element vector). A spatial
momentum L stores the angular momentum 𝐡 and translational momentum 𝐥
of a system S about a point P, measured in a frame M, and expressed in
a frame E. The system S may be a particle, a rigid or deformable body,
or a set of particles and/or bodies. This class assumes that both the
angular momentum 𝐡 and translational momentum 𝐥 are expressed in the
same *expressed-in* frame E. This class only stores 6 elements (namely
𝐡 and 𝐥) and does not store the underlying system S, about-point P,
measured-in frame M, or expressed-in frame E. The user is responsible
for explicitly tracking the underlying system, about-point, and frames
with multibody_quantities "monogram notation". For example, L_MSP_E
denotes a system S's spatial momentum about point P, measured in frame
M, and expressed in frame E. L_MSP_E contains h_MSP_E (S's angular
momentum about point P, measured in M, expressed in E) and l_MS_E (S's
translational momentum measured in M, expressed in E). A body B's
spatial momentum about point Bo (B's origin), measured in frame M,
expressed in frame E has explicit monogram notation L_MBBo_E which can
be abbreviated L_MBo_E. Similarly L_MSScm_E is abbreviated L_MScm_E
(Scm is S's center of mass). Details on spatial vectors and monogram
notation are in sections multibody_spatial_vectors and
multibody_quantities.

The typeset for L_MSP_E is :math:`[^ML^{S/P}]_E`. For a set S of
particles Qᵢ, L_MSP contains S's angular momentum 𝐡 about-point P,
measured in frame M and S's translational momentum 𝐥 measured in frame
M, defined as


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    h_MSP = ∑ h_MQᵢP = ∑ p_PQᵢ x l_MQᵢ  where l_MQᵢ = mᵢ v_MQᵢ.
      l_MS  = ∑ l_MQᵢ  = ∑ mᵢ v_MQᵢ

.. raw:: html

    </details>

where mᵢ is the mass of particle Qᵢ, v_MQᵢ is Qᵢ's translational
velocity measured in frame M, l_MQᵢ = mᵢ v_MQQᵢ is Qᵢ's translational
momentum measured in frame M, h_MQᵢP is Qᵢ's angular momentum about
point P measured in frame M, and p_PQᵢ is the position vector from
point P to Qᵢ. These definitions extend to a body (continuum of
particles) by using the density ρ(r) of the body at each material
location r as:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    h_MSP = ∫p_PQ(r) x v_MQ(r) ρ(r) d³r
      l_MS  = ∫v_MQ(r) ρ(r) d³r

.. raw:: html

    </details>)""";
        // Symbol: drake::multibody::SpatialMomentum::Shift
        struct /* Shift */ {
          // Source: drake/multibody/math/spatial_momentum.h
          const char* doc =
R"""(Shifts a SpatialMomentum from an about-point P to an about-point Q.

Parameter ``offset``:
    which is the position vector p_PQ_E from point P to point Q,
    expressed in frame E. p_PQ_E must have the same expressed-in frame
    E as ``this`` spatial momentum, where ``this`` is L_MSP_E (system
    S's spatial momentum about P, measured in frame M, expressed in
    frame E).

Returns ``L_MSQ_E``:
    which is system S's spatial momentum about point Q, measured in
    frame M, expressed in frame E.

Note:
    Shift() differs from ShiftInPlace() in that Shift() does not
    modify ``this`` whereas ShiftInPlace() does modify ``this``.

See also:
    ShiftInPlace() for more information and how L_MSQ_E is calculated.)""";
        } Shift;
        // Symbol: drake::multibody::SpatialMomentum::ShiftInPlace
        struct /* ShiftInPlace */ {
          // Source: drake/multibody/math/spatial_momentum.h
          const char* doc =
R"""(In-place shift of a SpatialMomentum from an about-point P to an
about-point Q. On entry, ``this`` is L_MSP_E (system S's spatial
momentum about point P, measured in a frame M and expressed in a frame
E). On return ``this`` is modified to L_MSQ_E (S's spatial momentum
about point Q, measured in frame M and expressed in frame E). The
components of L_MSQ_E are:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    l_MS_E = l_MS_E         (translational momentum of ``this`` is unchanged).
     h_MSQ_E = h_MSP_E + p_QP_E x l_MS_E
             = h_MSP_E - p_PQ_E x l_MS_E

.. raw:: html

    </details>

Parameter ``offset``:
    which is the position vector p_PQ_E from point P to point Q, with
    the same expressed-in frame E as ``this`` spatial momentum.

Note:
    Spatial momenta shift similar to spatial force (see SpatialForce)
    and in a related/different way for spatial velocity (see
    SpatialVelocity).

See also:
    Shift() to shift spatial momentum without modifying ``this``.)""";
        } ShiftInPlace;
        // Symbol: drake::multibody::SpatialMomentum::SpatialMomentum<T>
        struct /* ctor */ {
          // Source: drake/multibody/math/spatial_momentum.h
          const char* doc_0args =
R"""(Default constructor. In Release builds, all 6 elements of a newly
constructed spatial momentum are uninitialized (for speed). In Debug
builds, the 6 elements are set to NaN so that invalid operations on an
uninitialized spatial momentum fail fast (fast bug detection).)""";
          // Source: drake/multibody/math/spatial_momentum.h
          const char* doc_2args =
R"""(Constructs a spatial momentum L from an angular momentum 𝐡 and a
translational momentum 𝐥.)""";
          // Source: drake/multibody/math/spatial_momentum.h
          const char* doc_1args =
R"""(Constructs a spatial momentum L from an Eigen expression that
represents a 6-element vector, i.e., a 3-element angular momentum 𝐡
and a 3-element translational momentum 𝐥. This constructor will assert
the size of L is six (6) either at compile-time for fixed sized Eigen
expressions or at run-time for dynamic sized Eigen expressions.)""";
        } ctor;
        // Symbol: drake::multibody::SpatialMomentum::dot
        struct /* dot */ {
          // Source: drake/multibody/math/spatial_momentum.h
          const char* doc =
R"""(Calculates twice (2x) a body B's kinetic energy measured in a frame M.
For any frame (e.g., an multibody_frames_and_bodies "offset frame") Bp
that is fixed to a rigid body B, calculates the dot-product of
``this`` = L_MBp_E (body B's spatial momentum measured in frame M,
about Bp's origin, expressed in frame E) with V_MBp_E (frame Bp's
spatial velocity measured in frame M, expressed in frame E).

Parameter ``velocity``:
    which is V_MBp_E, frame Bp's spatial velocity measured in frame M,
    and expressed in the same frame E as ``this`` = L_MBp_E.

Returns:
    2*K_MB, twice (2x) body B's kinetic energy measured in frame M.

Note:
    In general, kinetic energy calculations are only useful when frame
    M is a world frame (also called a Newtonian or inertial frame).
    Hence, it is unusual to use this method unless frame M is the
    world frame W.

Note:
    Although the spatial vectors V_MBp_E and L_MBp_E must have the
    same expressed-in frame E, the resulting scalar K_MB is
    independent of frame E.

Note:
    As shown below, K_MB can be calculated from any frame Bp fixed on
    B, including body B's center of mass frame Bcm. This is due to how
    spatial momentum and spatial velocity shift from Bcm to Bp. For
    more information, see SpatialMomentum∷Shift() and
    SpatialVelocity∷Shift().


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    K_MB = 1/2 (L_MBp · V_MBp) = 1/2 (L_MBcm · V_MBcm)

.. raw:: html

    </details>)""";
        } dot;
      } SpatialMomentum;
      // Symbol: drake::multibody::SpatialVector
      struct /* SpatialVector */ {
        // Source: drake/multibody/math/spatial_vector.h
        const char* doc =
R"""(This class represents a *spatial vector* and has 6 elements, with a
3-element rotational vector on top of a 3-element translational
vector. Important subclasses of SpatialVector include SpatialVelocity,
SpatialAcceleration, SpatialForce, and SpatialMomentum. Each of the
3-element vectors is assumed to be expressed in the same
*expressed-in* frame E. This class only stores 6 elements and does not
store the underlying expressed-in frame E or other information. The
user is responsible for explicitly tracking the underlying frames with
multibody_quantities "monogram notation". For example, Foo_E denotes
an arbitrary spatial vector Foo expressed in a frame E. Details on
spatial vectors and monogram notation are in sections
multibody_spatial_vectors and multibody_quantities "monogram
notation".

Template parameter ``SV``:
    The type of the more specialized spatial vector class. It must be
    a template on the scalar type T.)""";
        // Symbol: drake::multibody::SpatialVector::CoeffsEigenType
        struct /* CoeffsEigenType */ {
          // Source: drake/multibody/math/spatial_vector.h
          const char* doc =
R"""(The type of the underlying in-memory representation using an Eigen
vector.)""";
        } CoeffsEigenType;
        // Symbol: drake::multibody::SpatialVector::GetMaximumAbsoluteDifferences
        struct /* GetMaximumAbsoluteDifferences */ {
          // Source: drake/multibody/math/spatial_vector.h
          const char* doc =
R"""(Returns the maximum absolute values of the differences in the
rotational and translational components of ``this`` and ``other``
(i.e., the infinity norms of the difference in rotational and
translational components).

Parameter ``other``:
    spatial vector to subtract from ``this`` spatial vector.

Returns:
    The following quantities in a tuple, in the order below. std∷tuple
    | Description
    -----------------|-------------------------------------------------
    w_max_difference | Maximum absolute difference in rotation
    components v_max_difference | Maximum absolute difference in
    translation components)""";
        } GetMaximumAbsoluteDifferences;
        // Symbol: drake::multibody::SpatialVector::IsApprox
        struct /* IsApprox */ {
          // Source: drake/multibody/math/spatial_vector.h
          const char* doc =
R"""(Determines whether all six corresponding elements of two spatial
vectors are equal to each other to within a specified tolerance
epsilon.

Parameter ``other``:
    spatial vector to compare to ``this`` spatial vector.

Parameter ``tolerance``:
    specified tolerance for this test.

Returns:
    true if ‖this - other‖∞ < epsilon, otherwise returns false. Note:
    the infinity norm ‖this - other‖∞ is simply the maximum of the six
    absolute values in (this - other).)""";
        } IsApprox;
        // Symbol: drake::multibody::SpatialVector::IsNearlyEqualWithinAbsoluteTolerance
        struct /* IsNearlyEqualWithinAbsoluteTolerance */ {
          // Source: drake/multibody/math/spatial_vector.h
          const char* doc =
R"""(Compares the rotational and translational parts of ``this`` and
``other`` to check if they are the same to within specified absolute
differences.

Parameter ``other``:
    spatial vector to compare to ``this`` spatial vector.

Parameter ``rotational_tolerance``:
    maximum allowable absolute difference between the rotational parts
    of ``this`` and ``other``. The units depend on the underlying
    class. For example, spatial velocity, acceleration, and force have
    units of rad/sec, rad/sec^2, and N*m, respectively.

Parameter ``translational_tolerance``:
    maximum allowable absolute difference between the translational
    parts of ``this`` and ``other``. The units depend on the
    underlying class. For example, spatial velocity, acceleration, and
    force have units of meter/sec, meter/sec^2, and Newton,
    respectively.

Returns:
    true if all three rotational elements of ``this`` and ``other``
    are equal within rotational_tolerance and all three translational
    elements of ``this`` and ``other`` are equal within
    translational_tolerance.)""";
        } IsNearlyEqualWithinAbsoluteTolerance;
        // Symbol: drake::multibody::SpatialVector::SetNaN
        struct /* SetNaN */ {
          // Source: drake/multibody/math/spatial_vector.h
          const char* doc =
R"""(Sets all the elements in ``this`` SpatialVector to NaN. This is
typically used to quickly detect uninitialized values since NaN will
trigger a chain of invalid computations that can be tracked back to
their source.)""";
        } SetNaN;
        // Symbol: drake::multibody::SpatialVector::SetZero
        struct /* SetZero */ {
          // Source: drake/multibody/math/spatial_vector.h
          const char* doc =
R"""(Sets both the rotational and translational components of ``this``
SpatialVector to zero.)""";
        } SetZero;
        // Symbol: drake::multibody::SpatialVector::SpatialQuantity
        struct /* SpatialQuantity */ {
          // Source: drake/multibody/math/spatial_vector.h
          const char* doc =
R"""(The more specialized spatial vector class templated on the scalar type
T.)""";
        } SpatialQuantity;
        // Symbol: drake::multibody::SpatialVector::SpatialVector<SV, T>
        struct /* ctor */ {
          // Source: drake/multibody/math/spatial_vector.h
          const char* doc_0args =
R"""(Default constructor. In Release builds, all 6 elements of a newly
constructed spatial vector are uninitialized (for speed). In Debug
builds, the 6 elements are set to NaN so that invalid operations on an
uninitialized spatial vector fail fast (fast bug detection).)""";
          // Source: drake/multibody/math/spatial_vector.h
          const char* doc_2args =
R"""(Constructs a spatial vector from a rotational component w and a
translational component v.)""";
          // Source: drake/multibody/math/spatial_vector.h
          const char* doc_1args =
R"""(Constructs a spatial vector V from an Eigen expression that represents
a 6-element vector (3-element rotational vector on top of a 3-element
translational vector). This constructor asserts the size of V is six
(6) either at compile-time for fixed sized Eigen expressions or at
run-time for dynamic sized Eigen expressions.)""";
        } ctor;
        // Symbol: drake::multibody::SpatialVector::Zero
        struct /* Zero */ {
          // Source: drake/multibody/math/spatial_vector.h
          const char* doc =
R"""(Factory to create a *zero* spatial vector, i.e., a SpatialVector whose
rotational and translational components are both zero.)""";
        } Zero;
        // Symbol: drake::multibody::SpatialVector::data
        struct /* data */ {
          // Source: drake/multibody/math/spatial_vector.h
          const char* doc =
R"""(Returns a (const) bare pointer to the underlying data. It is
guaranteed that there will be six (6) T's densely packed at data[0],
data[1], etc.)""";
        } data;
        // Symbol: drake::multibody::SpatialVector::get_coeffs
        struct /* get_coeffs */ {
          // Source: drake/multibody/math/spatial_vector.h
          const char* doc_0args_nonconst =
R"""(Returns a mutable reference to the underlying storage.)""";
          // Source: drake/multibody/math/spatial_vector.h
          const char* doc_0args_const =
R"""(Returns a constant reference to the underlying storage.)""";
        } get_coeffs;
        // Symbol: drake::multibody::SpatialVector::kRotationSize
        struct /* kRotationSize */ {
          // Source: drake/multibody/math/spatial_vector.h
          const char* doc = R"""()""";
        } kRotationSize;
        // Symbol: drake::multibody::SpatialVector::kSpatialVectorSize
        struct /* kSpatialVectorSize */ {
          // Source: drake/multibody/math/spatial_vector.h
          const char* doc = R"""()""";
        } kSpatialVectorSize;
        // Symbol: drake::multibody::SpatialVector::kTranslationSize
        struct /* kTranslationSize */ {
          // Source: drake/multibody/math/spatial_vector.h
          const char* doc = R"""()""";
        } kTranslationSize;
        // Symbol: drake::multibody::SpatialVector::mutable_data
        struct /* mutable_data */ {
          // Source: drake/multibody/math/spatial_vector.h
          const char* doc =
R"""(Returns a (mutable) bare pointer to the underlying data. It is
guaranteed that there will be six (6) T's densely packed at data[0],
data[1], etc.)""";
        } mutable_data;
        // Symbol: drake::multibody::SpatialVector::operator*=
        struct /* operator_imul */ {
          // Source: drake/multibody/math/spatial_vector.h
          const char* doc = R"""(Multiplication assignment operator.)""";
        } operator_imul;
        // Symbol: drake::multibody::SpatialVector::operator+=
        struct /* operator_iadd */ {
          // Source: drake/multibody/math/spatial_vector.h
          const char* doc = R"""(Addition assignment operator.)""";
        } operator_iadd;
        // Symbol: drake::multibody::SpatialVector::operator-
        struct /* operator_sub */ {
          // Source: drake/multibody/math/spatial_vector.h
          const char* doc = R"""(Unary minus operator.)""";
        } operator_sub;
        // Symbol: drake::multibody::SpatialVector::operator-=
        struct /* operator_isub */ {
          // Source: drake/multibody/math/spatial_vector.h
          const char* doc = R"""(Subtraction assignment operator.)""";
        } operator_isub;
        // Symbol: drake::multibody::SpatialVector::operator[]
        struct /* operator_array */ {
          // Source: drake/multibody/math/spatial_vector.h
          const char* doc_1args_i_const =
R"""(Const access to the i-th element of this spatial vector. In Debug
builds, this function asserts that i is in bounds whereas for release
builds, no bounds-check on i is performed (for speed).)""";
          // Source: drake/multibody/math/spatial_vector.h
          const char* doc_1args_i_nonconst =
R"""(Mutable access to the i-th element of this spatial vector. In Debug
builds, this function asserts that i is in bounds whereas for release
builds, no bounds-check on i is performed (for speed).)""";
        } operator_array;
        // Symbol: drake::multibody::SpatialVector::rotational
        struct /* rotational */ {
          // Source: drake/multibody/math/spatial_vector.h
          const char* doc_0args_const =
R"""(Const access to the rotational component of this spatial vector.)""";
          // Source: drake/multibody/math/spatial_vector.h
          const char* doc_0args_nonconst =
R"""(Mutable access to the rotational component of this spatial vector.)""";
        } rotational;
        // Symbol: drake::multibody::SpatialVector::size
        struct /* size */ {
          // Source: drake/multibody/math/spatial_vector.h
          const char* doc =
R"""(For 3D (three-dimensional) analysis, the total size of the
concatenated rotational vector (3 elements) and translational vector
(3 elements) is six (6), which is known at compile time.)""";
        } size;
        // Symbol: drake::multibody::SpatialVector::translational
        struct /* translational */ {
          // Source: drake/multibody/math/spatial_vector.h
          const char* doc_0args_const =
R"""(Const access to the translational component of this spatial vector.)""";
          // Source: drake/multibody/math/spatial_vector.h
          const char* doc_0args_nonconst =
R"""(Mutable access to the translational component of this spatial vector.)""";
        } translational;
      } SpatialVector;
      // Symbol: drake::multibody::SpatialVelocity
      struct /* SpatialVelocity */ {
        // Source: drake/multibody/math/spatial_velocity.h
        const char* doc =
R"""(This class represents a *spatial velocity* V (also called a *twist*)
and has 6 elements with an angular (rotational) velocity ω (3-element
vector) on top of a translational (linear) velocity v (3-element
vector). Spatial velocity represents the rotational and translational
motion of a frame B with respect to a *measured-in* frame M. This
class assumes that both the angular velocity ω and translational
velocity v are expressed in the same *expressed-in* frame E. This
class only stores 6 elements (namely ω and v) and does not store the
underlying frames B, M, E. The user is responsible for explicitly
tracking the underlying frames with multibody_quantities "monogram
notation". For example, V_MB_E denotes frame B's spatial velocity
measured in frame M, expressed in frame E and contains ω_MB_E (B's
angular velocity measured in M, expressed in E) and v_MBo_E (Bo's
translational velocity measured in M, expressed in E), where Bo is
frame B's origin point. For an multibody_frames_and_bodies "offset
frame" Bp, the monogram notation V_MBp_E denotes the spatial velocity
of frame Bp measured in M, expressed in E. Details on spatial vectors
and monogram notation are in sections multibody_spatial_vectors and
multibody_quantities.)""";
        // Symbol: drake::multibody::SpatialVelocity::ComposeWithMovingFrameVelocity
        struct /* ComposeWithMovingFrameVelocity */ {
          // Source: drake/multibody/math/spatial_velocity.h
          const char* doc =
R"""(Compose ``this`` spatial velocity (measured in some frame M) with the
spatial velocity of another frame to form the 𝐨𝐭𝐡𝐞𝐫 frame's spatial
velocity measured in frame M. Herein, ``this`` is the spatial velocity
of a frame (designated B) in frame M and the 𝐨𝐭𝐡𝐞𝐫 frame is designated
C.

Parameter ``position_of_moving_frame``:
    which is the position vector p_BoCo_E (from frame B's origin Bo to
    frame C's origin Co), expressed in a frame E. p_BoCo_E must have
    the same expressed-in frame E as ``this``, where ``this`` is
    V_MB_E (frame B's spatial velocity measured in M, expressed in E).

Parameter ``velocity_of_moving_frame``:
    which is V_BC_E, frame C's spatial velocity measured in frame B,
    expressed in frame E.

Returns ``V_MC_E``:
    frame C's spatial velocity measured in frame M, expressed in frame
    E.

Note:
    The returned spatial velocity V_MC_E contains an angular velocity
    ω_MC_E and translational velocity v_MCo_E that are calculated as:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ω_MC_E  = ω_MB_E + ω_BC_E
     v_MCo_E = v_MBo_E + ω_MB_E x p_BoCo_E + v_BCo_E

.. raw:: html

    </details>

If frame C is rigidly fixed to frame B, V_BC_E = 0 and this method
produces a Shift() operation (albeit inefficiently). In other words,
use Shift() if velocity_of_moving_frame = 0.

See also:
    SpatialAcceleration∷ComposeWithMovingFrameAcceleration().)""";
        } ComposeWithMovingFrameVelocity;
        // Symbol: drake::multibody::SpatialVelocity::Shift
        struct /* Shift */ {
          // Source: drake/multibody/math/spatial_velocity.h
          const char* doc =
R"""(Shifts a SpatialVelocity from a frame B to a frame C, where both B and
C are fixed to the same frame or rigid body.

Parameter ``offset``:
    which is the position vector p_BoCo_E from frame B's origin to
    frame C's origin, expressed in frame E. p_BoCo_E must have the
    same expressed-in frame E as ``this`` spatial velocity (``this`` =
    V_MB_E).

Returns ``V_MC_E``:
    which is frame C's spatial velocity measured in frame M, expressed
    in frame E.

Note:
    Shift() differs from ShiftInPlace() in that Shift() does not
    modify ``this`` whereas ShiftInPlace() does modify ``this``.

See also:
    ShiftInPlace() for more information and how V_MC_E is calculated.)""";
        } Shift;
        // Symbol: drake::multibody::SpatialVelocity::ShiftInPlace
        struct /* ShiftInPlace */ {
          // Source: drake/multibody/math/spatial_velocity.h
          const char* doc =
R"""(In-place shift of a SpatialVelocity from a frame B to a frame C, where
both B and C are fixed to the same frame or rigid body. On entry,
``this`` is V_MB_E (frame B's spatial velocity measured in a frame M
and expressed in a frame E). On return ``this`` is modified to V_MC_E
(frame C's spatial velocity measured in frame M and expressed in frame
E). The components of V_MC_E are calculated as:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ω_MC_E = ω_MB_E                (angular velocity of ``this`` is unchanged).
     v_MC_E = v_MB_E + ω_MB_E x p_BoCo_E     (translational velocity changes).

.. raw:: html

    </details>

Parameter ``offset``:
    which is the position vector p_BoCo_E from frame B's origin to
    frame C's origin, expressed in frame E. p_BoCo_E must have the
    same expressed-in frame E as ``this`` spatial velocity.

See also:
    Shift() to shift spatial velocity without modifying ``this``.)""";
        } ShiftInPlace;
        // Symbol: drake::multibody::SpatialVelocity::SpatialVelocity<T>
        struct /* ctor */ {
          // Source: drake/multibody/math/spatial_velocity.h
          const char* doc_0args =
R"""(Default constructor. In Release builds, all 6 elements of a newly
constructed spatial velocity are uninitialized (for speed). In Debug
builds, the 6 elements are set to NaN so that invalid operations on an
uninitialized spatial velocity fail fast (fast bug detection).)""";
          // Source: drake/multibody/math/spatial_velocity.h
          const char* doc_2args =
R"""(Constructs a spatial velocity V from an angular velocity ω and a
translational velocity v.)""";
          // Source: drake/multibody/math/spatial_velocity.h
          const char* doc_1args =
R"""(Constructs a spatial velocity V from an Eigen expression that
represents a 6-element vector, i.e., two 3-element vectors, namely an
angular velocity ω and a translational velocity v. This constructor
will assert the size of V is six (6) either at compile-time for fixed
sized Eigen expressions or at run-time for dynamic sized Eigen
expressions.)""";
        } ctor;
        // Symbol: drake::multibody::SpatialVelocity::dot
        struct /* dot */ {
          // Source: drake/multibody/math/spatial_velocity.h
          const char* doc_1args_force =
R"""(Calculates the power generated by a spatial force. For an arbitrary
frame B, calculates the dot-product of ``this`` = V_MB_E (frame B's
spatial velocity measured in a frame M, expressed in a frame E) with
F_B_E (frame B's spatial force, expressed in frame E).

Parameter ``force``:
    which is F_B_E, frame B's spatial force, expressed in the same
    frame E as ``this`` = V_MB_E.

Returns:
    Power of spatial force F_B_E in frame M, i.e., F_B_E ⋅ V_MB_E.

Note:
    Just as equating force 𝐅 to mass * acceleration as 𝐅 = m𝐚 relies
    on acceleration 𝐚 being measured in a world frame (also called an
    inertial or Newtonian frame), equating power = dK/dt (where K is
    kinetic energy) relies on K being measured in a world frame.
    Hence, it is unusual to use this method unless frame M is the
    world frame W.

Note:
    Although the spatial vectors F_B_E and V_MB_E must have the same
    expressed-in frame E, the returned scalar is independent of frame
    E.)""";
          // Source: drake/multibody/math/spatial_velocity.h
          const char* doc_1args_momentum =
R"""(Calculates twice (2x) a body B's kinetic energy measured in a frame M.
For any frame (e.g., an multibody_frames_and_bodies "offset frame") Bp
that is fixed to a rigid body B, calculates the dot-product of
``this`` = V_MBp_E (frame Bp's spatial velocity measured in frame M,
expressed in frame E) with L_MBp_E (body B's spatial momentum measured
in frame M, about Bp's origin, expressed in frame E).

Parameter ``momentum``:
    which is L_MBp_E, body B's spatial momentum measured in frame M,
    about frame Bp's origin, expressed in the same frame E as ``this``
    = V_MBp_E.

Returns:
    2*K_MB, twice (2x) body B's kinetic energy measured in frame M.

Note:
    In general, kinetic energy calculations are only useful when frame
    M is a world frame (also called a Newtonian or inertial frame).
    Hence, it is unusual to use this method unless frame M is the
    world frame W.

Note:
    Although the spatial vectors V_MBp_E and L_MBp_E must have the
    same expressed-in frame E, the resulting scalar K_MB is
    independent of frame E.

Note:
    As shown below, K_MB can be calculated from any frame Bp fixed on
    B, including body B's center of mass frame Bcm. This is due to how
    spatial momentum and spatial velocity shift from Bcm to Bp. For
    more information, see SpatialMomentum∷Shift() and
    SpatialVelocity∷Shift().


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    K_MB = 1/2 (L_MBp · V_MBp) = 1/2 (L_MBcm · V_MBcm)

.. raw:: html

    </details>)""";
        } dot;
      } SpatialVelocity;
      // Symbol: drake::multibody::operator+
      struct /* operator_add */ {
        // Source: drake/multibody/math/spatial_acceleration.h
        const char* doc_2args_constSpatialAcceleration_constSpatialAcceleration =
R"""(Adds two spatial accelerations by simply adding their 6 underlying
elements.

Parameter ``A1_E``:
    spatial acceleration expressed in the same frame E as A2_E.

Parameter ``A2_E``:
    spatial acceleration expressed in the same frame E as A1_E.

Note:
    The general utility of this operator+() function is questionable
    and it should only be used if you are sure it makes sense.

See also:
    Shift(), ShiftInPlace(), and ComposeWithMovingFrameAcceleration().)""";
        // Source: drake/multibody/math/spatial_force.h
        const char* doc_2args_constSpatialForce_constSpatialForce =
R"""(Adds two spatial forces by simply adding their 6 underlying elements.

Parameter ``F1_E``:
    spatial force expressed in the same frame E as F2_E.

Parameter ``F2_E``:
    spatial force expressed in the same frame E as F1_E.

Note:
    The general utility of this operator+() function seems limited to
    situations when F1 and F2 are associated with different sets of
    forces, but are applied to the same frame B, with same application
    point Bp, and have the same expressed-in frame E.)""";
        // Source: drake/multibody/math/spatial_momentum.h
        const char* doc_2args_constSpatialMomentum_constSpatialMomentum =
R"""(Adds two spatial momenta by simply adding their 6 underlying elements.

Parameter ``L1_E``:
    spatial momentum expressed in the same frame E as L2_E.

Parameter ``L2_E``:
    spatial momentum expressed in the same frame E as L1_E.

Note:
    The general utility of this operator+() function seems limited to
    situations when L1 and L2 are associated with different systems
    (S1 and S2), but have the same about-point P, same measured-in
    frame M, and same expressed-in frame E.)""";
        // Source: drake/multibody/math/spatial_velocity.h
        const char* doc_2args_constSpatialVelocity_constSpatialVelocity =
R"""(Adds two spatial velocities by simply adding their 6 underlying
elements.

Parameter ``V1_E``:
    spatial velocity expressed in the same frame E as V2_E.

Parameter ``V2_E``:
    spatial velocity expressed in the same frame E as V1_E.

Note:
    The general utility of this operator+() function is questionable
    and it should only be used if you are sure it makes sense. One use
    case is for calculating the spatial velocity V_MC of a frame C
    measured in a frame M when frame C is moving on a frame B and one
    has pre-calculated V_MBc (frame Bc's spatial velocity measured in
    frame M, where frame Bc is instantaneously coincident with frame
    C). For this use case, the operator+ function returns V_MC_E =
    V_MBc_E + V_BC_E, where the precalculated V_MBc_E is equal to
    V_MBo_E.Shift(p_BoCo_E).

See also:
    Shift(), ShiftInPlace(), and ComposeWithMovingFrameVelocity().)""";
      } operator_add;
      // Symbol: drake::multibody::operator-
      struct /* operator_sub */ {
        // Source: drake/multibody/math/spatial_acceleration.h
        const char* doc_2args_constSpatialAcceleration_constSpatialAcceleration =
R"""(Subtracts spatial accelerations by simply subtracting their 6
underlying elements.

Parameter ``A1_E``:
    spatial acceleration expressed in the same frame E as A2_E.

Parameter ``A2_E``:
    spatial acceleration expressed in the same frame E as A1_E.

Note:
    The general utility of this operator-() function is questionable
    and it should only be used if you are sure it makes sense.

See also:
    Shift(), ShiftInPlace(), and ComposeWithMovingFrameAcceleration().)""";
        // Source: drake/multibody/math/spatial_force.h
        const char* doc_2args_constSpatialForce_constSpatialForce =
R"""(Subtracts spatial forces by simply subtracting their 6 underlying
elements.

Parameter ``F1_E``:
    spatial force expressed in the same frame E as F2_E.

Parameter ``F2_E``:
    spatial force expressed in the same frame E as F1_E.

Note:
    The general utility of this operator-() function seems limited to
    situations when F1 and F2 are associated with different sets of
    forces, but are applied to the same frame B, with same application
    point Bp, and have the same expressed-in frame E.)""";
        // Source: drake/multibody/math/spatial_momentum.h
        const char* doc_2args_constSpatialMomentum_constSpatialMomentum =
R"""(Subtracts spatial momenta by simply subtracting their 6 underlying
elements.

Parameter ``L1_E``:
    spatial momentum expressed in the same frame E as L2_E.

Parameter ``L2_E``:
    spatial momentum expressed in the same frame E as L1_E.

Note:
    The general utility of this operator-() function is questionable
    and it should only be used if you are sure it makes sense.)""";
        // Source: drake/multibody/math/spatial_velocity.h
        const char* doc_2args_constSpatialVelocity_constSpatialVelocity =
R"""(Subtracts spatial velocities by simply subtracting their 6 underlying
elements.

Parameter ``V1_E``:
    spatial velocity expressed in the same frame E as V2_E.

Parameter ``V2_E``:
    spatial velocity expressed in the same frame E as V1_E.

Note:
    The general utility of this operator-() function is questionable
    and it should only be used if you are sure it makes sense. One use
    case is for calculating relative spatial velocity, e.g., a frame
    C's spatial velocity relative to a frame B, measure in a frame M.
    This use case calculates V_M_BC_E = V_MC_E - V_MB_E, which
    contains ω_BC (C's angular velocity measured in B) and v_M_BoCo
    (Co's velocity relative to Bo, measured in M), where both ω_BC and
    v_M_BoCo are expressed in frame E.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ω_BC  = ω_MC - ω_MB
     v_M_BoCo = v_MCo - v_MBo = DtM(p_BoCo)

.. raw:: html

    </details>

where DtM(p_BoCo) is the time-derivative in frame M of p_BoCo
(position vector from Bo to Co). A second use case has to do with a
frame C that is moving on a frame B and calculates frame C's spatial
velocity measured in frame B. It assumes you have pre-calculated V_MBc
(frame Bc's spatial velocity measured in frame M, where frame Bc is
fixed to B and instantaneously coincident with frame C. This use case
returns V_BC_E = V_MC_E - V_MBc_E, where the precalculated V_MBc_E is
equal to V_MBo_E.Shift(p_BoBc_E).

See also:
    Shift(), ShiftInPlace(), and ComposeWithMovingFrameVelocity().)""";
      } operator_sub;
    } multibody;
  } drake;
} pydrake_doc_multibody_math;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
