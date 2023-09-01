#include "drake/multibody/inverse_kinematics/in_collision_constraint.h"

#include <limits>
#include <vector>

#include <Eigen/Dense>

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/distance_constraint_utilities.h"
#include "drake/multibody/inverse_kinematics/kinematic_evaluator_utilities.h"
namespace drake {
namespace multibody {
namespace {
const double kInf = std::numeric_limits<double>::infinity();
}  // namespace

InCollisionConstraint::InCollisionConstraint(
    const multibody::MultibodyPlant<double>* const plant,
    double minimum_distance_upper, double normalizer,
    systems::Context<double>* plant_context)
    : solvers::Constraint(1, internal::RefFromPtrOrThrow(plant).num_positions(),
                          Vector1d(0), Vector1d(0)),
      plant_double_{plant},
      plant_context_double_{plant_context},
      plant_autodiff_{nullptr},
      plant_context_autodiff_{nullptr},
      collision_checker_{nullptr},
      collision_checker_context_{nullptr} {
  Initialize<double>(*plant_double_, plant_context_double_,
                     minimum_distance_upper, normalizer);
}

InCollisionConstraint::InCollisionConstraint(
    const multibody::MultibodyPlant<AutoDiffXd>* const plant,
    double minimum_distance_upper, double normalizer,
    systems::Context<AutoDiffXd>* plant_context)
    : solvers::Constraint(1, RefFromPtrOrThrow(plant).num_positions(),
                          Vector1d(0), Vector1d(0)),
      plant_double_{nullptr},
      plant_context_double_{nullptr},
      plant_autodiff_{plant},
      plant_context_autodiff_{plant_context},
      collision_checker_{nullptr},
      collision_checker_context_{nullptr} {
  Initialize<AutoDiffXd>(*plant_autodiff_, plant_context_autodiff_,
                         minimum_distance_upper, normalizer);
}

InCollisionConstraint::InCollisionConstraint(
    const planning::CollisionChecker* collision_checker,
    double minimum_distance_upper, double normalizer,
    planning::CollisionCheckerContext* collision_checker_context)
    : solvers::Constraint(1, collision_checker->plant().num_positions(),
                          Vector1d(0), Vector1d(0)),
      plant_double_{nullptr},
      plant_context_double_{nullptr},
      plant_autodiff_{nullptr},
      plant_context_autodiff_{nullptr},
      collision_checker_{collision_checker},
      collision_checker_context_{collision_checker_context} {
  Initialize(*collision_checker, minimum_distance_upper, normalizer);
}

template <typename T>
void InCollisionConstraint::Initialize(const MultibodyPlant<T>& plant,
                                       systems::Context<T>* plant_context,
                                       double minimum_distance_upper,
                                       double normalizer) {
  internal::CheckPlantIsConnectedToSceneGraph(plant, *plant_context);
  minimum_distance_upper_ = minimum_distance_upper;
  normalizer_ = normalizer;
  DRAKE_DEMAND(normalizer > 0);
  minimum_value_constraint_ =
      std::make_unique<solvers::MinimumValueUpperBoundConstraint>(
          plant.num_positions(), minimum_distance_upper, normalizer,
          [&plant, plant_context](const auto& x) {
            return internal::Distances<T, AutoDiffXd>(plant, plant_context, x,
                                                      kInf);
          },
          [&plant, plant_context](const auto& x) {
            return internal::Distances<T, double>(plant, plant_context, x,
                                                  kInf);
          });
  this->set_bounds(minimum_value_constraint_->lower_bound(),
                   minimum_value_constraint_->upper_bound());
}

void InCollisionConstraint::Initialize(
    const planning::CollisionChecker& collision_checker,
    double minimum_distance_upper, double normalizer) {
  minimum_distance_upper_ = minimum_distance_upper;
  normalizer_ = normalizer;
  // TODO(hongkai.dai): I want to use infinity as the influence_distance, but
  // CollisionChecker doesn't support it yet. Hence I use a very large
  // influence_distance.
  const double influence_distance = minimum_distance_upper_ + 10 * normalizer;
  minimum_value_constraint_ =
      std::make_unique<solvers::MinimumValueUpperBoundConstraint>(
          collision_checker.plant().num_positions(), minimum_distance_upper_,
          normalizer_,
          [this, influence_distance](const Eigen::Ref<const AutoDiffVecXd>& x) {
            return internal::Distances(*(this->collision_checker_),
                                       this->collision_checker_context_, x,
                                       influence_distance);
          },
          [this,
           influence_distance](const Eigen::Ref<const Eigen::VectorXd>& x) {
            return internal::Distances(*(this->collision_checker_),
                                       this->collision_checker_context_, x,
                                       influence_distance);
          });
  this->set_bounds(minimum_value_constraint_->lower_bound(),
                   minimum_value_constraint_->upper_bound());
}

template <typename T>
void InCollisionConstraint::DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                                          VectorX<T>* y) const {
  minimum_value_constraint_->Eval(x, y);
}

void InCollisionConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                   Eigen::VectorXd* y) const {
  DoEvalGeneric<double>(x, y);
}

void InCollisionConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                   AutoDiffVecXd* y) const {
  DoEvalGeneric<AutoDiffXd>(x, y);
}
}  // namespace multibody
}  // namespace drake
