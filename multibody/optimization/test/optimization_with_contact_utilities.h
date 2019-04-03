#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace multibody {
namespace test {
// Create the test on free spheres/blocks and a ground
struct SphereSpecification {
  SphereSpecification(double radius_in, double density,
                      CoulombFriction<double> friction_in)
      : radius{radius_in}, friction{std::move(friction_in)} {
    const double mass = 4.0 / 3 * M_PI * std::pow(radius, 3) * density;
    const double I = 2.0 / 5.0 * mass * std::pow(radius, 2);  // inertia
    inertia = SpatialInertia<double>(mass, Eigen::Vector3d::Zero(),
                                     UnitInertia<double>(I, I, I));
  }
  double radius;
  SpatialInertia<double> inertia;
  CoulombFriction<double> friction;
};

struct BoxSpecification {
  // Full dimensions of a box (not the half dimensions).
  Eigen::Vector3d size;
  SpatialInertia<double> inertia;
  CoulombFriction<double> friction;
};

template <typename T>
class FreeSpheresAndBoxes {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FreeSpheresAndBoxes)

  FreeSpheresAndBoxes(std::vector<SphereSpecification> spheres,
                      std::vector<BoxSpecification> boxes,
                      CoulombFriction<double> ground_friction);

  const systems::Diagram<T>& diagram() const { return *diagram_; }

  const MultibodyPlant<T>& plant() const { return *plant_; }

  const geometry::SceneGraph<T>& scene_graph() const { return *scene_graph_; }

  const systems::Context<T>& diagram_context() const {
    return *diagram_context_;
  }

  const systems::Context<T>& plant_context() const { return *plant_context_; }

  systems::Context<T>* get_mutable_plant_context() const {
    return plant_context_;
  }

  const std::vector<geometry::GeometryId>& sphere_geometry_ids() const {
    return sphere_geometry_ids_;
  }

  const std::vector<geometry::GeometryId>& box_geometry_ids() const {
    return box_geometry_ids_;
  }

  geometry::GeometryId ground_geometry_id() const {
    return ground_geometry_id_;
  }

  const std::vector<SphereSpecification>& spheres() const { return spheres_; }

  const std::vector<BoxSpecification>& boxes() const { return boxes_; }

  const CoulombFriction<double>& ground_friction() const {
    return ground_friction_;
  }

 private:
  const std::vector<SphereSpecification> spheres_;
  const std::vector<BoxSpecification> boxes_;
  const CoulombFriction<double> ground_friction_;
  std::vector<geometry::GeometryId> sphere_geometry_ids_;
  std::vector<geometry::GeometryId> box_geometry_ids_;
  geometry::GeometryId ground_geometry_id_;
  std::unique_ptr<systems::Diagram<T>> diagram_;
  MultibodyPlant<T>* plant_{nullptr};
  geometry::SceneGraph<T>* scene_graph_{nullptr};
  std::unique_ptr<systems::Context<T>> diagram_context_{nullptr};
  systems::Context<T>* plant_context_{nullptr};
};
}  // namespace test
}  // namespace multibody
}  // namespace drake
