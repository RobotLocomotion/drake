#pragma once

#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

// We want to compute X_AB such that B is posed relative to A as documented in
// BoxesOverlapTest::AllCases. We can do so by generating the rotation
// component, R_AB, such that Bq has a minimum value along the chosen axis, and
// we can solve for the translation component, p_AoBo_A = p_AoAf_A + p_AfBq_A +
// p_BqBo_A.  The boxes are represented by triples of their half sizes
// (measured in each box's canonical frame).
math::RigidTransformd CalcCornerTransform(const Eigen::Vector3d& a_half,
                                          const Eigen::Vector3d& b_half,
                                          const int axis,
                                          const bool expect_overlap);

// We want to compute X_AB such that B is posed relative to A as documented in
// BoxesOverlapTest::AllCases. We can do so by generating the rotation
// component, R_AB, such that Bq lies on the minimum edge along the chosen
// axis, and we can solve for the translation component, p_AoBo_A = p_AoAf_A +
// p_AfBq_A + p_BqBo_A.
math::RigidTransformd CalcEdgeTransform(const Eigen::Vector3d& a_half,
                                        const Eigen::Vector3d& b_half,
                                        const int a_axis, const int b_axis,
                                        const bool expect_overlap);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
