#pragma once

#include <memory>
#include <optional>
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
  BoxSpecification(Eigen::Vector3d size_in, double density,
                   CoulombFriction<double> friction_in,
                   std::optional<math::RigidTransformd> X_WB_in)
      : size(std::move(size_in)),
        friction{std::move(friction_in)},
        X_WB(std::move(X_WB_in)) {
    const double mass = size(0) * size(1) * size(2) * density;
    const UnitInertia<double> I(
        1.0 / 12 * (size(1) * size(1) + size(2) * size(2)),
        1.0 / 12 * (size(0) * size(0) + size(2) * size(2)),
        1.0 / 12 * (size(0) * size(0) + size(1) * size(1)));
    inertia = SpatialInertia<double>(mass, Eigen::Vector3d::Zero(), I);
  }
  // Full dimensions of a box (not the half dimensions).
  Eigen::Vector3d size;
  SpatialInertia<double> inertia;
  CoulombFriction<double> friction;
  std::optional<math::RigidTransform<double>> X_WB;
};

template <typename T>
class FreeSpheresAndBoxes {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FreeSpheresAndBoxes)

  FreeSpheresAndBoxes(std::vector<SphereSpecification> spheres,
                      std::vector<BoxSpecification> boxes,
                      CoulombFriction<double> ground_friction);

  const systems::Diagram<T>& diagram() const { return *diagram_; }

  systems::Diagram<T>* get_mutable_diagram() { return diagram_.get(); }

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

  const std::vector<BodyIndex>& sphere_body_indices() const {
    return sphere_body_indices_;
  }

  const std::vector<BodyIndex>& box_body_indices() const {
    return box_body_indices_;
  }

 private:
  const std::vector<SphereSpecification> spheres_;
  const std::vector<BoxSpecification> boxes_;
  const CoulombFriction<double> ground_friction_;
  std::vector<geometry::GeometryId> sphere_geometry_ids_;
  std::vector<geometry::GeometryId> box_geometry_ids_;
  geometry::GeometryId ground_geometry_id_;
  std::vector<multibody::BodyIndex> sphere_body_indices_;
  std::vector<multibody::BodyIndex> box_body_indices_;
  std::unique_ptr<systems::Diagram<T>> diagram_;
  MultibodyPlant<T>* plant_{nullptr};
  geometry::SceneGraph<T>* scene_graph_{nullptr};
  std::unique_ptr<systems::Context<T>> diagram_context_{nullptr};
  systems::Context<T>* plant_context_{nullptr};
};
}  // namespace test
}  // namespace multibody
}  // namespace drake
