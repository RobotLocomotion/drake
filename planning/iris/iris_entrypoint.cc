#include "drake/planning/iris/iris_entrypoint.h"

#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/iris.h"
#include "drake/planning/collision_checker.h"
#include "drake/planning/scene_graph_collision_checker.h"

namespace drake {
namespace planning {

using geometry::optimization::HPolyhedron;

HPolyhedron GrowIrisRegion(const CollisionChecker& checker,
                           const IrisOptions& options,
                           const Eigen::VectorXd seed) {
  DRAKE_THROW_UNLESS(options.algorithm != IrisAlgorithm::Unset);
  DRAKE_THROW_UNLESS(options.region_space != IrisRegionSpace::Unset);

  if (options.algorithm == IrisAlgorithm::Convex) {
    throw std::runtime_error("IrisAlgorithm::Convex is not supported yet.");
  } else if (options.algorithm == IrisAlgorithm::NP2) {
    throw std::runtime_error("IrisAlgorithm::NP2 is not supported yet.");
  } else if (options.algorithm == IrisAlgorithm::ZO) {
    throw std::runtime_error("IrisAlgorithm::ZO is not supported yet.");
  } else if (options.algorithm == IrisAlgorithm::Certified) {
    throw std::runtime_error("IrisAlgorithm::Certified is not supported yet.");
  } else if (options.algorithm == IrisAlgorithm::NP) {
    if (options.region_space == IrisRegionSpace::TaskSpace2d ||
        options.region_space == IrisRegionSpace::TaskSpace3d ||
        options.region_space == IrisRegionSpace::AbstractSpaceNd ||
        options.region_space == IrisRegionSpace::RationalConfigurationSpace) {
      throw std::runtime_error(
          "IrisAlgorithm::NP only supports options.region_space == "
          "IrisRegionSpace::ConfigurationSpace.");
    }
    const SceneGraphCollisionChecker* maybe_scene_graph_collision_checker =
        dynamic_cast<const SceneGraphCollisionChecker*>(&checker);
    if (!maybe_scene_graph_collision_checker) {
      throw std::runtime_error(
          "IrisAlgorithm::NP only supports SceneGraphCollisionChecker.");
    }
    // Check the dimension of the seed point.
    const int num_positions =
        maybe_scene_graph_collision_checker->plant().num_positions();
    if (seed.size() != num_positions) {
      throw std::runtime_error(fmt::format(
          "Seed point has dimension {}, but the collision checker's underlying "
          "plant has a configuration space of dimension {}",
          seed.size(), num_positions));
    }

    // Copy over the settings to the old IRIS-NP options struct.
    geometry::optimization::IrisOptions iris_np_options;
    iris_np_options.require_sample_point_is_contained =
        options.require_sample_point_is_contained;
    iris_np_options.iteration_limit = options.iteration_limit;
    iris_np_options.termination_threshold =
        options.absolute_termination_threshold;
    iris_np_options.relative_termination_threshold =
        options.relative_termination_threshold;
    iris_np_options.configuration_space_margin =
        options.configuration_space_margin;
    iris_np_options.num_collision_infeasible_samples =
        options.num_collision_infeasible_samples;
    iris_np_options.configuration_obstacles = options.configuration_obstacles;
    iris_np_options.starting_ellipse = options.starting_ellipse;
    iris_np_options.bounding_region = options.bounding_region;
    iris_np_options.prog_with_additional_constraints =
        options.prog_with_additional_constraints;
    iris_np_options.num_additional_constraint_infeasible_samples =
        options.num_additional_constraint_infeasible_samples;
    iris_np_options.random_seed = options.random_seed;
    iris_np_options.meshcat = options.meshcat;
    iris_np_options.termination_func = options.termination_function;
    iris_np_options.mixing_steps = options.mixing_steps;
    iris_np_options.solver_options = options.solver_options;

    // Get the plant and context for IrisInConfigurationSpace.
    const RobotDiagram<double>& diagram =
        maybe_scene_graph_collision_checker->model();
    const multibody::MultibodyPlant<double>& plant =
        maybe_scene_graph_collision_checker->plant();
    std::unique_ptr<systems::Context<double>> diagram_context_ptr =
        diagram.CreateDefaultContext();
    systems::Context<double>& plant_context =
        plant.GetMyMutableContextFromRoot(diagram_context_ptr.get());
    plant.SetPositions(&plant_context, seed);
    return geometry::optimization::IrisInConfigurationSpace(
        plant, plant_context, iris_np_options);
  }

  // Code should never reach this point, since we throw if the algorithm is
  // unset, and every algorithm will return an HPolyhedron or throw.
  return HPolyhedron::MakeUnitBox(seed.size());
}

}  // namespace planning
}  // namespace drake
