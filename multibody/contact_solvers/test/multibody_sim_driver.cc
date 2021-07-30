#include "drake/multibody/contact_solvers/test/multibody_sim_driver.h"

#include "drake/geometry/drake_visualizer.h"

namespace drake {
namespace multibody {
namespace test {

void MultibodySimDriver::BuildModel(double dt, const std::string& model_file) {
  auto pair = AddMultibodyPlantSceneGraph(&builder_, dt);
  plant_ = &pair.plant;
  scene_graph_ = &pair.scene_graph;
  const std::string full_name = FindResourceOrThrow(model_file);
  multibody::Parser(plant_).AddModelFromFile(full_name);

  // We set gravity to a simpler number for tests.
  plant_->mutable_gravity_field().set_gravity_vector(
      Vector3<double>(0.0, 0.0, -10.0));
}

void MultibodySimDriver::Initialize() {
  DRAKE_DEMAND(!initialized_);
  plant_->Finalize();

  // Add visualization.
  // Note: the call to Initialize() below will cause the DrakeVisualizer
  // periodic publish event to be processed. This will broadcast a load message
  // *and* a draw message. The draw message requires SceneGraph to pull on its
  // input ports (evaluating all up-stream dependencies). If this isn't
  // desirable, and we only want to load the geometry without the system
  // evaluation, we need some mechanism that will send a load message based on
  // SceneGraph's geometry data.
  geometry::DrakeVisualizerd::AddToBuilder(&builder_, *scene_graph_);
  diagram_ = builder_.Build();

  simulator_ = std::make_unique<systems::Simulator<double>>(*diagram_);
  diagram_context_ = &simulator_->get_mutable_context();
  plant_context_ = &plant_->GetMyMutableContextFromRoot(diagram_context_);
  scene_graph_context_ =
      &scene_graph_->GetMyMutableContextFromRoot(diagram_context_);
  simulator_->Initialize();

  initialized_ = true;
}

void MultibodySimDriver::AddGround(double stiffness, double damping,
                                   double dynamic_friction) {
  DRAKE_DEMAND(!initialized_);
  const Vector4<double> green(0.5, 1.0, 0.5, 1.0);
  plant_->RegisterVisualGeometry(plant_->world_body(), math::RigidTransformd(),
                                 geometry::HalfSpace(), "GroundVisualGeometry",
                                 green);
  // For a time-stepping model only static friction is used.
  const multibody::CoulombFriction<double> ground_friction(dynamic_friction,
                                                           dynamic_friction);
  plant_->RegisterCollisionGeometry(
      plant_->world_body(), math::RigidTransformd(), geometry::HalfSpace(),
      "GroundCollisionGeometry", ground_friction);

  SetPointContactParameters(plant_->world_body(), stiffness, damping);
}

void MultibodySimDriver::SetPointContactParameters(const Body<double>& body,
                                                   double stiffness,
                                                   double damping) {
  const std::vector<geometry::GeometryId>& geometries =
      plant_->GetCollisionGeometriesForBody(body);

  const auto& inspector = GetInspector();
  for (const auto& id : geometries) {
    const geometry::ProximityProperties* old_props =
        inspector.GetProximityProperties(id);
    DRAKE_DEMAND(old_props != nullptr);
    geometry::ProximityProperties new_props(*old_props);
    // Update to a new property. If not existent, UpdateProperty() behaves as
    // AddProperty.
    new_props.UpdateProperty(geometry::internal::kMaterialGroup,
                             geometry::internal::kPointStiffness, stiffness);
    new_props.UpdateProperty(geometry::internal::kMaterialGroup,
                             geometry::internal::kHcDissipation, damping);
    if (initialized_) {
      scene_graph_->AssignRole(scene_graph_context_, *plant_->get_source_id(),
                               id, new_props, geometry::RoleAssign::kReplace);
    } else {
      scene_graph_->AssignRole(*plant_->get_source_id(), id, new_props,
                               geometry::RoleAssign::kReplace);
    }
  }
}

std::vector<double> MultibodySimDriver::GetDynamicFrictionCoefficients(
    const Body<double>& body) const {
  const std::vector<geometry::GeometryId>& geometries =
      plant_->GetCollisionGeometriesForBody(body);
  const auto& inspector = GetInspector();
  std::vector<double> params;
  for (const auto& id : geometries) {
    const geometry::ProximityProperties* props =
        inspector.GetProximityProperties(id);
    DRAKE_DEMAND(props != nullptr);
    const auto& friction = props->GetProperty<CoulombFriction<double>>(
        geometry::internal::kMaterialGroup, geometry::internal::kFriction);
    params.push_back(friction.dynamic_friction());
  }
  return params;
}

const geometry::SceneGraphInspector<double>& MultibodySimDriver::GetInspector()
    const {
  const geometry::SceneGraphInspector<double>* inspector{nullptr};
  if (initialized_) {
    const auto& query_object =
        plant_->get_geometry_query_input_port()
            .Eval<geometry::QueryObject<double>>(*plant_context_);
    inspector = &query_object.inspector();
  } else {
    inspector = &scene_graph_->model_inspector();
  }
  return *inspector;
}

}  // namespace test
}  // namespace multibody
}  // namespace drake
