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

// #include "drake/multibody/rational/rational_forward_kinematics.h"
// #include "drake/multibody/rational/rational_forward_kinematics_internal.h"

// Symbol: pydrake_doc_multibody_rational
constexpr struct /* pydrake_doc_multibody_rational */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::multibody
    struct /* multibody */ {
      // Symbol: drake::multibody::RationalForwardKinematics
      struct /* RationalForwardKinematics */ {
        // Source: drake/multibody/rational/rational_forward_kinematics.h
        const char* doc =
R"""(For certain robots (whose joint transforms are algebraic functions of
joint variables, for example revolute/prismatic/floating-base joints),
we can represent the pose (position, orientation) of each body, as
rational functions, namely n(s) / d(s) where both the numerator n(s)
and denominator d(s) are polynomials of s, and s is some variable
related to the generalized position.

One example is that for a rotation matrix with angle θ and axis a, the
rotation matrix can be written as I + sinθ A + (1-cosθ) A², where A is
the skew-symmetric matrix from axis a. We can use the stereographic
projection to substitute the trigonometric function sinθ and cosθ as
cosθ = cos(θ*+Δθ) = cos(θ*)•cos(Δθ) - sin(θ*)•sin(Δθ) = (1-s²)/(1+s²)
cos(θ*)- 2s/(1+s²) sin(θ*) (1) sinθ = sin(θ*+Δθ) = sin(θ*)•cos(Δθ) -
cos(θ*)•sin(Δθ) = (1-s²)/(1+s²) sin(θ*)- 2s/(1+s²) cos(θ*) (2) where
θ* is the angle around which we take the stereographic projection. θ =
θ*+Δθ, and s = tan(Δθ/2). With (1) and (2), both sinθ and cosθ are
written as a rational function of s. Thus the rotation matrix can be
written as rational functions of s.

We will use q* to denote the nominal posture. For a revolute joint,
the q* is the variable θ*. Note that the stereographic projection has
a singularity at θ = θ* ± π, as s = tan(Δθ/2)=tan(±π/2) is ±infinity.

For prismatic joint, we define s = d - d*, where d is the displacement
of the joint, and d* is the joint value in q*.

Currently we only support robots with revolute, prismatic and weld
joints.

Throughout this file, we use the following convention 1. q denotes the
generalized position of the entire robot. It includes the angle of a
revolute joint, the displacement of a prismatic joint, etc. 2. θ
denotes the revolute joint angle. 3. t=tan((θ-θ*)/2) 4. s is the
vector of variables, such that the robot body pose is written as an
algebraic function (rational function) of s.)""";
        // Symbol: drake::multibody::RationalForwardKinematics::CalcBodyPoseAsMultilinearPolynomial
        struct /* CalcBodyPoseAsMultilinearPolynomial */ {
          // Source: drake/multibody/rational/rational_forward_kinematics.h
          const char* doc =
R"""(Computes the pose X_AB as a multilinear polynomial function. The
indeterminates of the polynomials are cos_delta() and sin_delta(). To
compute the pose as a rational function of indeterminates s(), we
recommend to first call this function to compute the pose as a
multilinear polynomial functions, and then call
ConvertMultilinearPolynomialToRationalFunction() to convert these
polynomials to rational function with indeterminates s().

Parameter ``q_star``:
    The nominal posture q*.

Parameter ``body_index``:
    Frame B, the body whose pose is computed.

Parameter ``expressed_body_index``:
    Frame A, the body in whose frame the pose is expressed.

Returns:
    X_AB The pose of body ``body_index`` expressed in
    ``expressed_body_index``.)""";
        } CalcBodyPoseAsMultilinearPolynomial;
        // Symbol: drake::multibody::RationalForwardKinematics::ComputeQValue
        struct /* ComputeQValue */ {
          // Source: drake/multibody/rational/rational_forward_kinematics.h
          const char* doc =
R"""(Computes values of q from s_val and q_star_val, while handling the
index matching between q and s (we don't guarantee that s(i) is
computed from q(i)).)""";
        } ComputeQValue;
        // Symbol: drake::multibody::RationalForwardKinematics::ComputeSValue
        struct /* ComputeSValue */ {
          // Source: drake/multibody/rational/rational_forward_kinematics.h
          const char* doc =
R"""(Computes values of s from q_val and q_star_val, while handling the
index matching between q and s (we don't guarantee that s(i) is
computed from q(i)).

Parameter ``angles_wrap_to_inf``:
    If set to True, then for a revolute joint whose (θ −θ*) >=π/2 (or
    <= −π/2), we set the corresponding s to ∞ (or −∞), $*Default:* is
    false.)""";
        } ComputeSValue;
        // Symbol: drake::multibody::RationalForwardKinematics::ConvertMultilinearPolynomialToRationalFunction
        struct /* ConvertMultilinearPolynomialToRationalFunction */ {
          // Source: drake/multibody/rational/rational_forward_kinematics.h
          const char* doc =
R"""(Given a polynomial whose indeterminates are cos_delta() and
sin_delta() (typically this polynomial is obtained from calling
CalcBodyPoseAsMultilinearPolynomial()), converts this polynomial to a
rational function with indeterminates s().)""";
        } ConvertMultilinearPolynomialToRationalFunction;
        // Symbol: drake::multibody::RationalForwardKinematics::Pose
        struct /* Pose */ {
          // Source: drake/multibody/rational/rational_forward_kinematics.h
          const char* doc =
R"""(This is a proxy for math∷RigidTransform. It captures the rigid pose of
one frame w.r.t another.

Template parameter ``T``:
    The scalar type, which must be one of double, symbolic∷Expression,
    symbolic∷Polynomial or symbolic∷RationalFunction.)""";
          // Symbol: drake::multibody::RationalForwardKinematics::Pose::position
          struct /* position */ {
            // Source: drake/multibody/rational/rational_forward_kinematics.h
            const char* doc = R"""()""";
          } position;
          // Symbol: drake::multibody::RationalForwardKinematics::Pose::rotation
          struct /* rotation */ {
            // Source: drake/multibody/rational/rational_forward_kinematics.h
            const char* doc = R"""()""";
          } rotation;
        } Pose;
        // Symbol: drake::multibody::RationalForwardKinematics::RationalForwardKinematics
        struct /* ctor */ {
          // Source: drake/multibody/rational/rational_forward_kinematics.h
          const char* doc =
R"""(Parameter ``plant``:
    The plant for which we compute forward kinematics. ``plant``
    should outlive this RationalForwardKinematics object.)""";
        } ctor;
        // Symbol: drake::multibody::RationalForwardKinematics::map_mobilizer_to_s_index
        struct /* map_mobilizer_to_s_index */ {
          // Source: drake/multibody/rational/rational_forward_kinematics.h
          const char* doc =
R"""(map_mobilizer_to_s_index_[mobilizer_index] returns the starting index
of the mobilizer's variable in s_ (the variable will be contiguous in
s for the same mobilizer). If this mobilizer doesn't have a variable
in s_ (like the weld joint), then the s index is set to -1.)""";
        } map_mobilizer_to_s_index;
        // Symbol: drake::multibody::RationalForwardKinematics::plant
        struct /* plant */ {
          // Source: drake/multibody/rational/rational_forward_kinematics.h
          const char* doc = R"""()""";
        } plant;
        // Symbol: drake::multibody::RationalForwardKinematics::s
        struct /* s */ {
          // Source: drake/multibody/rational/rational_forward_kinematics.h
          const char* doc = R"""()""";
        } s;
      } RationalForwardKinematics;
    } multibody;
  } drake;
} pydrake_doc_multibody_rational;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
