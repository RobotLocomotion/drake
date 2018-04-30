#include "drake/solvers/mixed_integer_rotation_constraint.h"

#include "drake/solvers/mixed_integer_rotation_constraint_internal.h"

namespace drake {
namespace solvers {
template <MixedIntegerRotationConstraintType ConstraintType>
MixedIntegerRotationConstraintGenerator<ConstraintType>::
    MixedIntegerRotationConstraintGenerator(int num_intervals_per_half_axis,
                                            IntervalBinning interval_binning)
    : num_intervals_per_half_axis_(num_intervals_per_half_axis),
      interval_binning_(interval_binning),
      phi_nonnegative_{
          Eigen::VectorXd::LinSpaced(0, 1, num_intervals_per_half_axis_ + 1)} {
  phi_.resize(2 * num_intervals_per_half_axis_ + 1);
  phi_(num_intervals_per_half_axis_) = 0;
  for (int i = 0; i < num_intervals_per_half_axis_; ++i) {
    phi_(num_intervals_per_half_axis_ - i - 1) = -phi_nonnegative_(i);
    phi_(num_intervals_per_half_axis_ + i) = phi_nonnegative_(i);
  }

  // If we consider the box-sphere intersection, then we need to compute the
  // halfspace nᵀx≥ d, as the tightest halfspace for each intersection region.
  if (ConstraintType ==
          MixedIntegerRotationConstraintType::kBoxSphereIntersection ||
      ConstraintType == MixedIntegerRotationConstraintType::kBoth) {
    const double kEpsilon = std::numeric_limits<double>::epsilon();

    box_sphere_intersection_vertices_.resize(num_intervals_per_half_axis_);
    box_sphere_intersection_halfspace_.resize(num_intervals_per_half_axis_);
    for (int xi = 0; xi < num_intervals_per_half_axis_; ++xi) {
      box_sphere_intersection_vertices_[xi].resize(
          num_intervals_per_half_axis_);
      box_sphere_intersection_halfspace_[xi].resize(
          num_intervals_per_half_axis_);
      for (int yi = 0; yi < num_intervals_per_half_axis_; ++yi) {
        box_sphere_intersection_vertices_[xi][yi].resize(
            num_intervals_per_half_axis_);
        box_sphere_intersection_halfspace_[xi][yi].resize(
            num_intervals_per_half_axis_);
        for (int zi = 0; zi < num_intervals_per_half_axis_; ++zi) {
          const Eigen::Vector3d box_min(
              phi_nonnegative_(xi), phi_nonnegative_(yi), phi_nonnegative_(zi));
          const Eigen::Vector3d box_max(phi_nonnegative_(xi + 1),
                                        phi_nonnegative_(yi + 1),
                                        phi_nonnegative_(zi + 1));
          const double box_min_norm = box_min.lpNorm<2>();
          const double box_max_norm = box_max.lpNorm<2>();
          if (box_min_norm <= 1.0 - 2 * kEpsilon &&
              box_max_norm >= 1.0 + 2 * kEpsilon) {
            // box_min is strictly inside the sphere, box_max is strictly
            // outside of the sphere.
            box_sphere_intersection_vertices_[xi][yi][zi] =
                internal::ComputeBoxEdgesAndSphereIntersection(box_min,
                                                               box_max);
            DRAKE_DEMAND(box_sphere_intersection_vertices_[xi][yi][zi].size() >=
                         3);

            Eigen::Vector3d normal{};
            internal::ComputeHalfSpaceRelaxationForBoxSphereIntersection(
                box_sphere_intersection_vertices_[xi][yi][zi],
                &(box_sphere_intersection_halfspace_[xi][yi][zi].first),
                &(box_sphere_intersection_vertices_[xi][yi][zi].second));
          } else if (std::abs(box_min_norm - 1) < 2 * kEpsilon) {
            // box_min is on the surface. This is the unique intersection point
            // between the sphere surface and the box.
            box_sphere_intersection_vertices_[xi][yi][zi].push_back(
                box_min / box_min_norm);
          } else if (std::abs(box_max_norm - 1) < 2 * kEpsilon) {
            // box_max is on the surface. This is the unique intersection point
            // between the sphere surface and the box.
            box_sphere_intersection_vertices_[xi][yi][zi].push_back(
                box_max / box_max_norm);
          }
        }
      }
    }
  }
}

template <MixedIntegerRotationConstraintType ConstraintType>
typename AddMixedIntegerRotationConstraintReturn<ConstraintType>::Type
MixedIntegerRotationConstraintGenerator<ConstraintType>::AddToProgram(
    MathematicalProgram* prog,
    const Eigen::Ref<const MatrixDecisionVariable<3, 3>>& R) const {}

template <>
AddMixedIntegerRotationConstraintReturn<
    MixedIntegerRotationConstraintType::kBilinearMcCormick>::Type
MixedIntegerRotationConstraintGenerator<
    MixedIntegerRotationConstraintType::kBilinearMcCormick>::
    AddToProgram(
        MathematicalProgram* prog,
        const Eigen::Ref<const MatrixDecisionVariable<3, 3>>& R) const {
  std::array<std::array<VectorXDecisionVariable, 3>, 3> lambda;
  AddMixedIntegerRotationConstraintReturn<
      MixedIntegerRotationConstraintType::kBilinearMcCormick>::Type B;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      const std::string lambda_name =
          "lambda[" + std::to_string(i) + "][" + std::to_string(j) + "]";
      lambda[i][j] = prog->NewContinuousVariables(
          2 * num_intervals_per_half_axis_ + 1, lambda_name);
      // R(i, j) = φᵀ * λ[i][j]
      prog->AddLinearConstraint(
          R(i, j) -
              phi_.dot(lambda[i][j].template cast<symbolic::Expression>()) ==
          0);
      switch (interval_binning_) {
        case IntervalBinning::kLogarithmic: {
          B[i][j] = AddLogarithmicSos2Constraint(
              prog, lambda[i][j].cast<symbolic::Expression>());
          break;
        }
        case IntervalBinning::kLinear: {
          B[i][j] = prog->NewBinaryVariables(
              2 * num_intervals_per_half_axis_,
              "B[" + std::to_string(i) + "][" + std::to_string(j) + "]");
          AddSos2Constraint(prog, lambda[i][j].cast<symbolic::Expression>(),
                            B[i][j].cast<symbolic::Expression>());
          break;
        }
      }
    }
  }
  return B;
}
}  // namespace solvers
}  // namespace drake
