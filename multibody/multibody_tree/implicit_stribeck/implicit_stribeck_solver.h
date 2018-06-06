#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace implicit_stribeck {

namespace internal {

template <typename T>
struct LimitDirectionChange {
// Limit the angle change between vₜᵏ⁺¹ and vₜᵏ for all contact points.
// The angle change θ is defined by the dot product between vₜᵏ⁺¹ and vₜᵏ
// as: cos(θ) = vₜᵏ⁺¹⋅vₜᵏ/(‖vₜᵏ⁺¹‖‖vₜᵏ‖).
// We'll do so by computing a coefficient 0 < α < 1 so that if the
// generalized velocities are updated as vᵏ⁺¹ = vᵏ + α Δvᵏ then θ < θₘₐₓ
// for all contact points.

// TODO:
//  Define what the "Stribeck circle" is so that it helps understand comments.
//  Define "strong" vs. "weak" gradients.
//  Define v_alpha, also to help understand comments.
//  Mention that cases with norm(v_alpha) < epsilon_v are not allowed when v
//  is outside the Stribeck circle, to avoid falling within a region of
//  "weak gradints"

static T run(const Eigen::Ref<const Vector2<T>>& v,
             const Eigen::Ref<const Vector2<T>>& dv,
             double cos_min, double v_stribeck, double tolerance);
};
}  // namespace internal

}  // namespace implicit_stribeck
}  // namespace multibody
}  // namespace drake
