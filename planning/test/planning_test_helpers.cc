#include "planning/test/planning_test_helpers.h"

#include <memory>
#include <vector>

#include "drake/multibody/tree/revolute_joint.h"
#include "common/anzu_model_directives.h"
#include "planning/robot_diagram_builder.h"

namespace anzu {
namespace planning {
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::multibody::CoulombFriction;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::RevoluteJoint;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::planning::ConfigurationDistanceFunction;

std::unique_ptr<RobotDiagram<double>> MakePlanningTestModel(
    const drake::multibody::parsing::ModelDirectives& directives) {
  auto builder = common::MakeRobotDiagramBuilderFromAnzuModelDirectives(
      directives);
  return builder->BuildDiagram();
}

ConfigurationDistanceFunction MakeWeightedIiwaConfigurationDistanceFunction() {
  Eigen::VectorXd weights = Eigen::VectorXd::Zero(7);
  weights << 2.0, 2.0, 2.0, 2.0, 1.0, 1.0, 1.0;

  const ConfigurationDistanceFunction weighted_cspace_distance_fn = [weights] (
      const Eigen::VectorXd& q1, const Eigen::VectorXd& q2) {
    const Eigen::VectorXd delta = q2 - q1;
    const Eigen::VectorXd weighted_delta =
        delta.cwiseProduct(weights);
    return weighted_delta.norm();
  };

  return weighted_cspace_distance_fn;
}

ModelInstanceIndex AddChain(MultibodyPlant<double>* plant, int n, int num_geo) {
  ModelInstanceIndex instance = plant->AddModelInstance(fmt::format("m{}", n));
  std::vector<const RigidBody<double>*> bodies;
  for (int k = 0; k < n; ++k) {
    bodies.push_back(
        &plant->AddRigidBody(fmt::format("b{}", k), instance,
                             SpatialInertia<double>::MakeUnitary()));
    if (plant->geometry_source_is_registered()) {
      for (int i = 0; i < num_geo; ++i) {
        plant->RegisterCollisionGeometry(
            *bodies.back(), RigidTransformd::Identity(), Sphere(0.01),
            fmt::format("g{}", i), CoulombFriction<double>());
      }
    }
  }
  for (int k = 1; k < n; ++k) {
    plant->AddJoint<RevoluteJoint>(fmt::format("j{}", k), *bodies[k - 1], {},
                                   *bodies[k], {}, Eigen::Vector3d::UnitY());
  }
  return instance;
}
}  // namespace planning
}  // namespace anzu
