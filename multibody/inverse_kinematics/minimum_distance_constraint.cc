#include "drake/multibody/inverse_kinematics/minimum_distance_constraint.h"

#include <limits>
#include <vector>

#include <Eigen/Dense>

#include "drake/multibody/inverse_kinematics/distance_constraint_utilities.h"
#include "drake/multibody/inverse_kinematics/kinematic_evaluator_utilities.h"

namespace drake {
namespace multibody {
using internal::RefFromPtrOrThrow;

namespace {
const double kInf = std::numeric_limits<double>::infinity();

int NumConstraints(double minimum_distance_lower,
                   double minimum_distance_upper) {
  return static_cast<int>(std::isfinite(minimum_distance_lower)) +
         static_cast<int>(std::isfinite(minimum_distance_upper));
}
}  // namespace

template <typename T, typename S>
VectorX<S> Distances(const MultibodyPlant<T>& plant,
                     systems::Context<T>* context,
                     const Eigen::Ref<const VectorX<S>>& q,
                     double influence_distance) {
  internal::UpdateContextConfiguration(context, plant, q);
  const auto& query_port = plant.get_geometry_query_input_port();
  if (!query_port.HasValue(*context)) {
    throw std::invalid_argument(
        "MinimumDistanceConstraint: Cannot get a valid geometry::QueryObject. "
        "Either the plant geometry_query_input_port() is not properly "
        "connected to the SceneGraph's output port, or the plant_context_ is "
        "incorrect. Please refer to AddMultibodyPlantSceneGraph on connecting "
        "MultibodyPlant to SceneGraph.");
  }
  const auto& query_object =
      query_port.template Eval<geometry::QueryObject<T>>(*context);

  const std::vector<geometry::SignedDistancePair<T>> signed_distance_pairs =
      query_object.ComputeSignedDistancePairwiseClosestPoints(
          influence_distance);
  VectorX<S> distances(signed_distance_pairs.size());
  for (int i = 0; i < static_cast<int>(signed_distance_pairs.size()); ++i) {
    const geometry::SceneGraphInspector<T>& inspector =
        query_object.inspector();
    const geometry::FrameId frame_A_id =
        inspector.GetFrameId(signed_distance_pairs[i].id_A);
    const geometry::FrameId frame_B_id =
        inspector.GetFrameId(signed_distance_pairs[i].id_B);
    const Frame<T>& frameA = plant.GetBodyFromFrameId(frame_A_id)->body_frame();
    const Frame<T>& frameB = plant.GetBodyFromFrameId(frame_B_id)->body_frame();
    internal::CalcDistanceDerivatives(
        plant, *context, frameA, frameB,
        // GetPoseInFrame() returns RigidTransform<double> -- we can't
        // multiply across heterogeneous scalar types; so we cast the double
        // to T.
        inspector.GetPoseInFrame(signed_distance_pairs[i].id_A)
                .template cast<T>() *
            signed_distance_pairs[i].p_ACa,
        signed_distance_pairs[i].distance, signed_distance_pairs[i].nhat_BA_W,
        q, &distances(i));
  }
  return distances;
}

Eigen::VectorXd Distances(
    const planning::CollisionChecker& collision_checker,
    planning::CollisionCheckerContext* collision_checker_context,
    const Eigen::Ref<const Eigen::VectorXd>& x, double influence_distance_val) {
  return collision_checker
      .CalcContextRobotClearance(collision_checker_context, x,
                                 influence_distance_val)
      .distances();
}

AutoDiffVecXd Distances(
    const planning::CollisionChecker& collision_checker,
    planning::CollisionCheckerContext* collision_checker_context,
    const Eigen::Ref<const AutoDiffVecXd>& x, double influence_distance_val) {
  const planning::RobotClearance robot_clearance =
      collision_checker.CalcContextRobotClearance(collision_checker_context,
                                                  math::ExtractValue(x),
                                                  influence_distance_val);
  return math::InitializeAutoDiff(
      robot_clearance.distances(),
      robot_clearance.jacobians() * math::ExtractGradient(x));
}

void MinimumDistanceConstraint::CheckMinimumDistanceBounds(
    double minimum_distance_lower, double minimum_distance_upper,
    double influence_distance) const {
  if (!std::isfinite(influence_distance)) {
    throw std::invalid_argument(
        "MinimumDistanceConstraint: influence_distance must be finite.");
  }
  if (std::isnan(minimum_distance_lower) ||
      influence_distance <= minimum_distance_lower) {
    throw std::invalid_argument(fmt::format(
        "MinimumDistanceConstraint: influence_distance={}, must be "
        "larger than minimum_distance_lower={}; equivalently, "
        "influence_distance_offset={}, but it needs to be positive.",
        influence_distance, minimum_distance_lower,
        influence_distance - minimum_distance_lower));
  }
  if (std::isfinite(minimum_distance_upper) &&
      influence_distance <= minimum_distance_upper) {
    throw std::invalid_argument(fmt::format(
        "MinimumDistanceConstraint: influence_distance={}, must be larger than "
        "minimum_distance_upper={}.",
        influence_distance, minimum_distance_upper));
  }
}

template <typename T>
void MinimumDistanceConstraint::Initialize(
    const MultibodyPlant<T>& plant, systems::Context<T>* plant_context,
    double minimum_distance_lower, double minimum_distance_upper,
    double influence_distance,
    MinimumDistancePenaltyFunction penalty_function) {
  CheckPlantIsConnectedToSceneGraph(plant, *plant_context);
  CheckMinimumDistanceBounds(minimum_distance_lower, minimum_distance_upper,
                             influence_distance);
  const auto& query_port = plant.get_geometry_query_input_port();
  // Maximum number of SignedDistancePairs returned by calls to
  // ComputeSignedDistancePairwiseClosestPoints().
  const int num_collision_candidates =
      query_port.template Eval<geometry::QueryObject<T>>(*plant_context)
          .inspector()
          .GetCollisionCandidates()
          .size();
  minimum_value_constraint_ = std::make_unique<solvers::MinimumValueConstraint>(
      this->num_vars(), minimum_distance_lower, minimum_distance_upper,
      influence_distance, num_collision_candidates,
      [&plant, plant_context](const auto& x, double influence_distance_val) {
        return Distances<T, AutoDiffXd>(plant, plant_context, x,
                                        influence_distance_val);
      },
      [&plant, plant_context](const auto& x, double influence_distance_val) {
        return Distances<T, double>(plant, plant_context, x,
                                    influence_distance_val);
      });
  this->set_bounds(minimum_value_constraint_->lower_bound(),
                   minimum_value_constraint_->upper_bound());
  if (penalty_function) {
    minimum_value_constraint_->set_penalty_function(penalty_function);
  }
}

void MinimumDistanceConstraint::Initialize(
    const planning::CollisionChecker& collision_checker,
    planning::CollisionCheckerContext* collision_checker_context,
    double minimum_distance_lower, double minimum_distance_upper,
    double influence_distance,
    MinimumDistancePenaltyFunction penalty_function) {
  CheckMinimumDistanceBounds(minimum_distance_lower, minimum_distance_upper,
                             influence_distance);
  minimum_value_constraint_ = std::make_unique<solvers::MinimumValueConstraint>(
      collision_checker.plant().num_positions(), minimum_distance_lower,
      minimum_distance_upper, influence_distance,
      collision_checker.MaxContextNumDistances(*collision_checker_context),
      [this](const Eigen::Ref<const AutoDiffVecXd>& x,
             double influence_distance_val) {
        return Distances(*(this->collision_checker_),
                         this->collision_checker_context_, x,
                         influence_distance_val);
      },
      [this](const Eigen::Ref<const Eigen::VectorXd>& x,
             double influence_distance_val) {
        return Distances(*(this->collision_checker_),
                         this->collision_checker_context_, x,
                         influence_distance_val);
      });
  this->set_bounds(minimum_value_constraint_->lower_bound(),
                   minimum_value_constraint_->upper_bound());
  if (penalty_function) {
    minimum_value_constraint_->set_penalty_function(penalty_function);
  }
}

MinimumDistanceConstraint::MinimumDistanceConstraint(
    const multibody::MultibodyPlant<double>* const plant,
    double minimum_distance, systems::Context<double>* plant_context,
    MinimumDistancePenaltyFunction penalty_function,
    double influence_distance_offset)
    : MinimumDistanceConstraint(plant, minimum_distance,
                                kInf /* minimum_distance_upper */,
                                plant_context, penalty_function,
                                influence_distance_offset + minimum_distance) {}

MinimumDistanceConstraint::MinimumDistanceConstraint(
    const multibody::MultibodyPlant<double>* const plant,
    double minimum_distance_lower, double minimum_distance_upper,
    systems::Context<double>* plant_context,
    MinimumDistancePenaltyFunction penalty_function,
    double influence_distance_offset)
    : solvers::Constraint(
          NumConstraints(minimum_distance_lower, minimum_distance_upper),
          RefFromPtrOrThrow(plant).num_positions(),
          Eigen::VectorXd::Zero(
              NumConstraints(minimum_distance_lower, minimum_distance_upper)),
          Eigen::VectorXd::Zero(
              NumConstraints(minimum_distance_lower, minimum_distance_upper))),
      /* The lower and upper bounds will be set to correct value later in
         Initialize() function */
      plant_double_{plant},
      plant_context_double_{plant_context},
      plant_autodiff_{nullptr},
      plant_context_autodiff_{nullptr},
      collision_checker_{nullptr} {
  Initialize(*plant_double_, plant_context_double_, minimum_distance_lower,
             minimum_distance_upper, influence_distance_offset,
             penalty_function);
}

MinimumDistanceConstraint::MinimumDistanceConstraint(
    const multibody::MultibodyPlant<AutoDiffXd>* const plant,
    double minimum_distance, systems::Context<AutoDiffXd>* plant_context,
    MinimumDistancePenaltyFunction penalty_function,
    double influence_distance_offset)
    : MinimumDistanceConstraint(plant, minimum_distance,
                                kInf /* minimum_distance_upper */,
                                plant_context, penalty_function,
                                influence_distance_offset + minimum_distance) {}

MinimumDistanceConstraint::MinimumDistanceConstraint(
    const multibody::MultibodyPlant<AutoDiffXd>* const plant,
    double minimum_distance_lower, double minimum_distance_upper,
    systems::Context<AutoDiffXd>* plant_context,
    MinimumDistancePenaltyFunction penalty_function, double influence_distance)
    : solvers::Constraint(
          NumConstraints(minimum_distance_lower, minimum_distance_upper),
          RefFromPtrOrThrow(plant).num_positions(),
          Eigen::VectorXd::Zero(
              NumConstraints(minimum_distance_lower, minimum_distance_upper)),
          Eigen::VectorXd::Zero(
              NumConstraints(minimum_distance_lower, minimum_distance_upper))),
      plant_double_{nullptr},
      plant_context_double_{nullptr},
      plant_autodiff_{plant},
      plant_context_autodiff_{plant_context},
      collision_checker_{nullptr} {
  Initialize(*plant_autodiff_, plant_context_autodiff_, minimum_distance_lower,
             minimum_distance_upper, influence_distance, penalty_function);
}

MinimumDistanceConstraint::MinimumDistanceConstraint(
    const planning::CollisionChecker* collision_checker,
    double minimum_distance_lower,
    planning::CollisionCheckerContext* collision_checker_context,
    MinimumDistancePenaltyFunction penalty_function,
    double influence_distance_offset)
    : MinimumDistanceConstraint(
          collision_checker, minimum_distance_lower,
          kInf /* minimum_distance_upper */, collision_checker_context,
          penalty_function,
          minimum_distance_lower + influence_distance_offset) {}

MinimumDistanceConstraint::MinimumDistanceConstraint(
    const planning::CollisionChecker* collision_checker,
    double minimum_distance_lower, double minimum_distance_upper,
    planning::CollisionCheckerContext* collision_checker_context,
    MinimumDistancePenaltyFunction penalty_function, double influence_distance)
    : solvers::Constraint(
          NumConstraints(minimum_distance_lower, minimum_distance_upper),
          internal::PtrOrThrow(
              collision_checker,
              "MinimumDistanceConstraint: collision_checker is nullptr")
              ->plant()
              .num_positions(),
          Eigen::VectorXd::Zero(
              NumConstraints(minimum_distance_lower, minimum_distance_upper)),
          Eigen::VectorXd::Zero(
              NumConstraints(minimum_distance_lower, minimum_distance_upper))),
      plant_double_{nullptr},
      plant_context_double_{nullptr},
      plant_autodiff_{nullptr},
      plant_context_autodiff_{nullptr},
      collision_checker_{collision_checker},
      collision_checker_context_{collision_checker_context} {
  Initialize(*collision_checker_, collision_checker_context_,
             minimum_distance_lower, minimum_distance_upper, influence_distance,
             penalty_function);
}

template <typename T>
void MinimumDistanceConstraint::DoEvalGeneric(
    const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const {
  minimum_value_constraint_->Eval(x, y);
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
