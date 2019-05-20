#include "drake/multibody/inverse_kinematics/minimum_distance_constraint.h"

#include <limits>
#include <vector>

#include <Eigen/Dense>

#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

namespace drake {
namespace multibody {
using internal::RefFromPtrOrThrow;

namespace {
/** Computes log(∑exp(xᵢ)). Exploits the shift invariance of log-sum-exp to
avoid overflow. */
template <typename T>
T LogSumExp(const std::vector<T>& x) {
  using std::exp;
  using std::log;
  const T x_max = *std::max_element(x.begin(), x.end());
  T sum_exp{0.0};
  for (const T& xi : x) {
    sum_exp += exp(xi - x_max);
  }
  return x_max + log(sum_exp);
}

/** Computes a smooth approximation of max(x). */
template <typename T>
T SmoothMax(const std::vector<T>& x) {
  // We compute the soft-max of x as softmax(x) = log(∑ᵢ exp(αxᵢ)) / α.
  // This soft-max approaches max(x) as α increases. We choose α = 100, as that
  // gives a qualitatively good fit for xᵢ ∈ [0, 1], which is the range of
  // potential penalty values when the MinimumDistanceConstraint is feasible.
  const double alpha{100};
  std::vector<T> x_scaled{x};
  for (T& xi_scaled : x_scaled) {
    xi_scaled *= alpha;
  }
  return LogSumExp(x_scaled) / alpha;
}

template <typename T>
T ScaleDistance(T distance, double minimum_distance,
                double influence_distance) {
  return (distance - influence_distance) /
         (influence_distance - minimum_distance);
}

}  // namespace

void ExponentiallySmoothedHingeLoss(double x, double* penalty,
                                    double* dpenalty_dx) {
  if (x >= 0) {
    *penalty = 0;
    if (dpenalty_dx) {
      *dpenalty_dx = 0;
    }
  } else {
    const double exp_one_over_x = std::exp(1.0 / x);
    *penalty = -x * exp_one_over_x;
    if (dpenalty_dx) {
      *dpenalty_dx = -exp_one_over_x + exp_one_over_x / x;
    }
  }
}

void QuadraticallySmoothedHingeLoss(double x, double* penalty,
                                    double* dpenalty_dx) {
  if (x >= 0) {
    *penalty = 0;
    if (dpenalty_dx) {
      *dpenalty_dx = 0;
    }
  } else {
    if (x > -1) {
      *penalty = x * x / 2;
      if (dpenalty_dx) {
        *dpenalty_dx = x;
      }
    } else {
      *penalty = -0.5 - x;
      if (dpenalty_dx) {
        *dpenalty_dx = -1;
      }
    }
  }
}

MinimumDistanceConstraint::MinimumDistanceConstraint(
    const multibody::MultibodyPlant<double>* const plant,
    double minimum_distance, systems::Context<double>* plant_context,
    MinimumDistancePenaltyFunction penalty_function,
    double influence_distance_offset)
    : solvers::Constraint(1, RefFromPtrOrThrow(plant).num_positions(),
                          Vector1d(0), Vector1d(1)),
      plant_{RefFromPtrOrThrow(plant)},
      minimum_distance_{minimum_distance},
      influence_distance_{minimum_distance + influence_distance_offset},
      // Since penalty_function uses output parameters rather than a return
      // value, we construct a lambda and then call it immediately to compute
      // the output scaling factor.
      penalty_output_scaling_{
          1.0 /
          [&penalty_function](double x) {
            double upper_bound{};
            double dummy{};
            penalty_function(x, &upper_bound, &dummy);
            return upper_bound;
          }(ScaleDistance(minimum_distance, minimum_distance,
                          minimum_distance + influence_distance_offset))},
      plant_context_{plant_context},
      penalty_function_{penalty_function} {
  if (!plant_.geometry_source_is_registered()) {
    throw std::invalid_argument(
        "MinimumDistanceConstraint: MultibodyPlant has not registered its "
        "geometry source with SceneGraph yet. Please refer to "
        "AddMultibodyPlantSceneGraph on how to connect MultibodyPlant to "
        "SceneGraph.");
  }
  if (!std::isfinite(influence_distance_offset)) {
    throw std::invalid_argument(
        "MinimumDistanceConstraint: influence_distance_offset must be finite.");
  }
  if (influence_distance_offset <= 0) {
    throw std::invalid_argument(
        "MinimumDistanceConstraint: influence_distance_offset must be "
        "positive.");
  }
  const auto& query_port = plant_.get_geometry_query_input_port();
  if (!query_port.HasValue(*plant_context_)) {
    throw std::invalid_argument(
        "MinimumDistanceConstraint: Cannot get a valid geometry::QueryObject. "
        "Either the plant geometry_query_input_port() is not properly "
        "connected to the SceneGraph's output port, or the plant_context_ is "
        "incorrect. Please refer to AddMultibodyPlantSceneGraph on connecting "
        "MultibodyPlant to SceneGraph.");
  }
  // Maximum number of SignedDistancePairs returned by calls to
  // ComputeSignedDistancePairwiseClosestPoints().
  num_collision_candidates_ =
      query_port.Eval<geometry::QueryObject<double>>(*plant_context_)
          .inspector()
          .GetCollisionCandidates()
          .size();
}

namespace {
void InitializeY(const Eigen::Ref<const Eigen::VectorXd>&, Eigen::VectorXd* y,
                 double y_value) {
  (*y)(0) = y_value;
}

void InitializeY(const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y,
                 double y_value) {
  (*y) = math::initializeAutoDiffGivenGradientMatrix(
      Vector1d(y_value), Eigen::RowVectorXd::Zero(x(0).derivatives().size()));
}

void Penalty(const MultibodyPlant<double>&, const systems::Context<double>&,
             const Frame<double>&, const Frame<double>&, const Eigen::Vector3d&,
             double distance, double minimum_distance,
             double influence_distance,
             MinimumDistancePenaltyFunction penalty_function,
             const Eigen::Vector3d&, const Eigen::Ref<const Eigen::VectorXd>&,
             double* y) {
  double penalty;
  const double x =
      ScaleDistance(distance, minimum_distance, influence_distance);
  penalty_function(x, &penalty, nullptr);
  *y = penalty;
}

void Penalty(const MultibodyPlant<double>& plant,
             const systems::Context<double>& context,
             const Frame<double>& frameA, const Frame<double>& frameB,
             const Eigen::Vector3d& p_ACa, double distance,
             double minimum_distance, double influence_distance,
             MinimumDistancePenaltyFunction penalty_function,
             const Eigen::Vector3d& nhat_BA_W,
             const Eigen::Ref<const AutoDiffVecXd>& q, AutoDiffXd* y) {
  // The distance is d = sign * |p_CbCa_B|, where the
  // closest points are Ca on object A, and Cb on object B.
  // So the gradient ∂d/∂q = p_CbCa_W * ∂p_BCa_B/∂q / d (Note that
  // ∂p_BCa_B/∂q = ∂p_CbCa_B/∂q).
  //
  // Since dividing by d is undefined when d = 0, and p_CbCa_W / d =
  // nhat_BA_W whenever d ≠ 0, we use ∂d/∂q = nhat_BA_W′ * ∂p_BCa_B/∂q. This
  // allows us to compute a gradient at d = 0 in certain cases (See
  // geometry::SignedDistancePair for details).
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jq_v_BCa_W(3, plant.num_positions());
  plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable::kQDot,
                                          frameA, p_ACa, frameB,
                                          plant.world_frame(), &Jq_v_BCa_W);
  const Eigen::RowVectorXd ddistance_dq = nhat_BA_W.transpose() * Jq_v_BCa_W;
  AutoDiffXd distance_autodiff{
      distance, ddistance_dq * math::autoDiffToGradientMatrix(q)};
  const AutoDiffXd scaled_distance_autodiff =
      ScaleDistance(distance_autodiff, minimum_distance, influence_distance);
  double penalty, dpenalty_dscaled_distance;
  penalty_function(scaled_distance_autodiff.value(), &penalty,
                   &dpenalty_dscaled_distance);

  const Vector1<AutoDiffXd> penalty_autodiff =
      math::initializeAutoDiffGivenGradientMatrix(
          Vector1d(penalty),
          dpenalty_dscaled_distance *
              math::autoDiffToGradientMatrix(
                  Vector1<AutoDiffXd>{scaled_distance_autodiff}));
  *y = penalty_autodiff(0);
}
}  // namespace

template <typename T>
void MinimumDistanceConstraint::DoEvalGeneric(
    const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const {
  y->resize(1);

  internal::UpdateContextConfiguration(plant_context_, plant_, x);
  const auto& query_port = plant_.get_geometry_query_input_port();
  if (!query_port.HasValue(*plant_context_)) {
    throw std::invalid_argument(
        "MinimumDistanceConstraint: Cannot get a valid geometry::QueryObject. "
        "Either the plant geometry_query_input_port() is not properly "
        "connected to the SceneGraph's output port, or the plant_context_ is "
        "incorrect. Please refer to AddMultibodyPlantSceneGraph on connecting "
        "MultibodyPlant to SceneGraph.");
  }
  const auto& query_object =
      query_port.Eval<geometry::QueryObject<double>>(*plant_context_);

  const std::vector<geometry::SignedDistancePair<double>>
      signed_distance_pairs =
          query_object.ComputeSignedDistancePairwiseClosestPoints(
              influence_distance_);

  // Initialize y to SmoothMax([0, 0, ..., 0]).
  InitializeY(x, y,
              SmoothMax(std::vector<double>(num_collision_candidates_, 0.0)));

  std::vector<T> penalties;
  for (const auto& signed_distance_pair : signed_distance_pairs) {
    const double distance = signed_distance_pair.distance;
    if (distance < influence_distance_) {
      const geometry::SceneGraphInspector<double>& inspector =
          query_object.inspector();
      const geometry::FrameId frame_A_id =
          inspector.GetFrameId(signed_distance_pair.id_A);
      const geometry::FrameId frame_B_id =
          inspector.GetFrameId(signed_distance_pair.id_B);
      const Frame<double>& frameA =
          plant_.GetBodyFromFrameId(frame_A_id)->body_frame();
      const Frame<double>& frameB =
          plant_.GetBodyFromFrameId(frame_B_id)->body_frame();
      penalties.emplace_back();
      Penalty(plant_, *plant_context_, frameA, frameB,
              inspector.X_FG(signed_distance_pair.id_A) *
                  signed_distance_pair.p_ACa,
              distance, minimum_distance_, influence_distance_,
              penalty_function_, signed_distance_pair.nhat_BA_W, x,
              &penalties.back());
      penalties.back() *= penalty_output_scaling_;
    }
  }
  if (!penalties.empty()) {
    // Pad penalties up to num_collision_candidates_ so that the constraint
    // function is actually smooth.
    penalties.resize(num_collision_candidates_, T{0.0});
    (*y)(0) = SmoothMax(penalties);
  }
}

void MinimumDistanceConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  DoEvalGeneric(x, y);
}

void MinimumDistanceConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                       AutoDiffVecXd* y) const {
  DoEvalGeneric(x, y);
}
}  // namespace multibody
}  // namespace drake
