#include "drake/multibody/optimization/test/optimization_with_contact_utilities.h"

#include <utility>

#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace multibody {
namespace test {

template <typename T>
void AddDrakeVisualizer(systems::DiagramBuilder<T>*,
                        const geometry::SceneGraph<T>&) {
  // Disabling visualization for non-double scalar type T.
}

template <>
void AddDrakeVisualizer<double>(
    systems::DiagramBuilder<double>* builder,
    const geometry::SceneGraph<double>& scene_graph) {
  geometry::ConnectDrakeVisualizer(builder, scene_graph);
}

template <typename T>
void InitializeDiagramSimulator(const systems::Diagram<T>&) {}

template <>
void InitializeDiagramSimulator<double>(
    const systems::Diagram<double>& diagram) {
  systems::Simulator<double>(diagram).Initialize();
}

template <typename T>
FreeSpheresAndBoxes<T>::FreeSpheresAndBoxes(
    std::vector<SphereSpecification> spheres,
    std::vector<BoxSpecification> boxes,
    CoulombFriction<double> ground_friction)
    : spheres_{std::move(spheres)},
      boxes_{std::move(boxes)},
      ground_friction_{std::move(ground_friction)} {
  const int num_spheres = static_cast<int>(spheres_.size());
  const int num_boxes = static_cast<int>(boxes_.size());
  systems::DiagramBuilder<T> builder;
  std::tie(plant_, scene_graph_) = AddMultibodyPlantSceneGraph(&builder);
  // Add spheres and register collision geometry.
  for (int i = 0; i < num_spheres; ++i) {
    const auto& sphere =
        plant_->AddRigidBody("sphere" + std::to_string(i), spheres_[i].inertia);
    sphere_geometry_ids_.push_back(plant_->RegisterCollisionGeometry(
        sphere, math::RigidTransformd::Identity(),
        geometry::Sphere(spheres_[i].radius), "sphere" + std::to_string(i),
        spheres_[i].friction, scene_graph_));
    plant_->RegisterVisualGeometry(
        sphere, math::RigidTransformd::Identity(),
        geometry::Sphere(spheres_[i].radius), "sphere" + std::to_string(i),
        geometry::IllustrationProperties(), scene_graph_);
  }
  // Add boxes and register collision geometry.
  for (int i = 0; i < num_boxes; ++i) {
    const Body<T>* box_body;
    math::RigidTransformd X_BBox;
    if (boxes_[i].X_WB.has_value()) {
      // register this box to the world frame.
      box_body = &(plant_->world_body());
      X_BBox = boxes_[i].X_WB.value();
    } else {
      box_body =
          &(plant_->AddRigidBody("box" + std::to_string(i), boxes_[i].inertia));
      X_BBox = math::RigidTransformd::Identity();
    }
    box_geometry_ids_.push_back(plant_->RegisterCollisionGeometry(
        *box_body, X_BBox,
        geometry::Box(boxes_[i].size(0), boxes_[i].size(1), boxes_[i].size(2)),
        "box" + std::to_string(i), boxes_[i].friction, scene_graph_));
    plant_->RegisterVisualGeometry(
        *box_body, X_BBox,
        geometry::Box(boxes_[i].size(0), boxes_[i].size(1), boxes_[i].size(2)),
        "box" + std::to_string(i), geometry::IllustrationProperties(),
        scene_graph_);
  }
  // Add the ground, register collision geometry.
  // The mass and inertia of the ground don't matter. Set them to arbitrary
  // values.
  const Eigen::Vector3d ground_box_size(100, 100, 100);
  math::RigidTransformd X_WG = math::RigidTransformd::Identity();
  X_WG.set_translation(Eigen::Vector3d(0, 0, -ground_box_size(2) / 2));
  ground_geometry_id_ = plant_->RegisterCollisionGeometry(
      plant_->world_body(), X_WG,
      geometry::Box(ground_box_size(0), ground_box_size(1), ground_box_size(2)),
      "ground", ground_friction_, scene_graph_);

  // Add gravity.
  plant_->template AddForceElement<UniformGravityFieldElement>();

  plant_->Finalize();

  AddDrakeVisualizer<T>(&builder, *scene_graph_);

  diagram_ = builder.Build();

  // Create context.
  diagram_context_ = diagram_->CreateDefaultContext();
  plant_context_ =
      &(diagram_->GetMutableSubsystemContext(*plant_, diagram_context_.get()));

  // Initialize a simulator for the diagram. This initialization step is
  // necessary for visualizing the posture.
  InitializeDiagramSimulator(*diagram_);
}

template class FreeSpheresAndBoxes<double>;
template class FreeSpheresAndBoxes<AutoDiffXd>;
}  // namespace test
}  // namespace multibody
}  // namespace drake
