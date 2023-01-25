#include "drake/planning/test/planning_test_helpers.h"

#include <memory>
#include <vector>

#include "drake/multibody/parsing/process_model_directives.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/planning/robot_diagram_builder.h"

namespace drake {
namespace planning {
namespace test {

using geometry::Sphere;
using math::RigidTransformd;
using multibody::CoulombFriction;
using multibody::ModelInstanceIndex;
using multibody::MultibodyPlant;
using multibody::RevoluteJoint;
using multibody::RigidBody;
using multibody::SpatialInertia;
using multibody::parsing::ModelDirectives;
using multibody::parsing::ProcessModelDirectives;

std::unique_ptr<RobotDiagram<double>> MakePlanningTestModel(
    const ModelDirectives& directives) {
  auto builder = std::make_unique<RobotDiagramBuilder<double>>();
  auto& parser = builder->mutable_parser();
  ProcessModelDirectives(directives, &parser);
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
    DRAKE_DEMAND(plant->geometry_source_is_registered());
    for (int i = 0; i < num_geo; ++i) {
      plant->RegisterCollisionGeometry(
          *bodies.back(), RigidTransformd::Identity(), Sphere(0.01),
          fmt::format("g{}", i), CoulombFriction<double>());
    }
  }
  for (int k = 1; k < n; ++k) {
    plant->AddJoint<RevoluteJoint>(fmt::format("j{}", k), *bodies[k - 1], {},
                                   *bodies[k], {}, Eigen::Vector3d::UnitY());
  }
  return instance;
}

}  // namespace test
}  // namespace planning
}  // namespace drake
