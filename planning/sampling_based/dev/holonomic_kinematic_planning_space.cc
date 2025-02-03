#include "planning/holonomic_kinematic_planning_space.h"

#include <utility>

namespace anzu {
namespace planning {

HolonomicKinematicPlanningSpace::HolonomicKinematicPlanningSpace(
    std::unique_ptr<drake::planning::CollisionChecker> collision_checker,
    const JointLimits& joint_limits, const double propagation_step_size,
    const uint64_t seed)
    : SymmetricCollisionCheckerPlanningSpace<Eigen::VectorXd>(
        std::move(collision_checker), joint_limits, seed) {
  SetPropagationStepSize(propagation_step_size);
}

HolonomicKinematicPlanningSpace::~HolonomicKinematicPlanningSpace() = default;

HolonomicKinematicPlanningSpace::HolonomicKinematicPlanningSpace(
    const HolonomicKinematicPlanningSpace& other) = default;

std::unique_ptr<PlanningSpace<Eigen::VectorXd>>
HolonomicKinematicPlanningSpace::DoClone() const {
  return std::unique_ptr<HolonomicKinematicPlanningSpace>(
      new HolonomicKinematicPlanningSpace(*this));
}

bool HolonomicKinematicPlanningSpace::DoCheckStateValidity(
    const Eigen::VectorXd& state, const int thread_number) const {
  // TODO(calderpg) Incorporate joint limits check into state validity.
  return collision_checker().CheckConfigCollisionFree(state, thread_number);
}

bool HolonomicKinematicPlanningSpace::DoCheckEdgeValidity(
    const Eigen::VectorXd& from, const Eigen::VectorXd& to,
    const int thread_number) const {
  // TODO(calderpg) Incorporate joint limits check into edge validity.
  return collision_checker().CheckEdgeCollisionFree(from, to, thread_number);
}

Eigen::VectorXd HolonomicKinematicPlanningSpace::DoSampleState(
    const int thread_number) {
  const JointLimits& limits = joint_limits();
  Eigen::VectorXd sample = Eigen::VectorXd::Zero(limits.num_positions());
  for (int index = 0; index < sample.size(); ++index) {
    const double lower = limits.position_lower()(index);
    const double upper = limits.position_upper()(index);
    const double ratio = random_source().DrawUniformUnitReal(thread_number);
    sample(index) = std::lerp(lower, upper, ratio);
  }
  return sample;
}

double HolonomicKinematicPlanningSpace::DoStateDistance(
    const Eigen::VectorXd& from, const Eigen::VectorXd& to) const {
  return collision_checker().ComputeConfigurationDistance(from, to);
}

Eigen::VectorXd HolonomicKinematicPlanningSpace::DoInterpolate(
    const Eigen::VectorXd& from, const Eigen::VectorXd& to, const double ratio)
    const {
  return collision_checker().InterpolateBetweenConfigurations(from, to, ratio);
}

std::vector<Eigen::VectorXd> HolonomicKinematicPlanningSpace::DoPropagate(
    const Eigen::VectorXd& from, const Eigen::VectorXd& to,
    std::map<std::string, double>* propagation_statistics,
    const int thread_number) {
  // Ensure that propagation_statistics contains the right keys. If additional
  // tracking statistics are added below, they must be added here as well. Note
  // that try_emplace only adds a new element if it does not already exist.
  propagation_statistics->try_emplace("edges_considered", 0.0);
  propagation_statistics->try_emplace("valid_edges", 0.0);
  propagation_statistics->try_emplace("edges_in_collision", 0.0);
  propagation_statistics->try_emplace("complete_propagation_successful", 0.0);

  std::vector<Eigen::VectorXd> propagated_states;

  // Compute a maximum number of steps to take.
  const double total_distance = StateDistance(from, to);
  const int total_steps =
      static_cast<int>(std::ceil(total_distance / propagation_step_size()));

  Eigen::VectorXd current = from;
  for (int step = 1; step <= total_steps; ++step) {
    (*propagation_statistics)["edges_considered"] += 1.0;
    const double ratio =
        static_cast<double>(step) / static_cast<double>(total_steps);
    const Eigen::VectorXd intermediate =
        collision_checker().InterpolateBetweenConfigurations(from, to, ratio);
    if (CheckEdgeValidity(current, intermediate, thread_number)) {
      (*propagation_statistics)["valid_edges"] += 1.0;
      if (step == total_steps) {
        (*propagation_statistics)["complete_propagation_successful"] += 1.0;
      }
      propagated_states.emplace_back(intermediate);
      current = intermediate;
    } else {
      (*propagation_statistics)["edges_in_collision"] += 1.0;
      break;
    }
  }

  return propagated_states;
}

double HolonomicKinematicPlanningSpace::DoMotionCost(
    const Eigen::VectorXd& from, const Eigen::VectorXd& to) const {
  return StateDistance(from, to);
}

}  // namespace planning
}  // namespace anzu
