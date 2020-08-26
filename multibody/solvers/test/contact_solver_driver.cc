#include "drake/multibody/solvers/test/contact_solver_driver.h"

namespace drake {
namespace multibody {
namespace test {

void ContactSolverDriver::BuildModel(double dt, const std::string& model_file) {
  systems::DiagramBuilder<double> builder;
  auto lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();
  auto pair = AddMultibodyPlantSceneGraph(&builder, dt);
  plant_ = &pair.plant;
  scene_graph_ = &pair.scene_graph;

  const std::string full_name = FindResourceOrThrow(model_file);
  multibody::Parser(plant_).AddModelFromFile(full_name);

  // We set gravity to a simpler number for tests.
  plant_->mutable_gravity_field().set_gravity_vector(
      Vector3<double>(0.0, 0.0, -10.0));

  // We make the ground very stiff so that the stiffness of the bodies in
  // contact with it dominates.
  const double kGroundDynamicFriction = 1.0;
  const double kGroundStiffness = 1.0e40;
  const double kGroundDamping = 0.0;
  AddGround(kGroundStiffness, kGroundDamping, kGroundDynamicFriction);

  plant_->Finalize();

  // External input.
  //plant.get_actuation_input_port().FixValue(&context, -10.0);

  // Add visualization.
  geometry::DispatchLoadMessage(*scene_graph_, lcm);
  geometry::ConnectDrakeVisualizer(&builder, pair.scene_graph);
  diagram_ = builder.Build();

  // Temporary workspace to store discrete updates.
  //discrete_values_ = diagram_->AllocateDiscreteVariables();

  simulator_ = std::make_unique<systems::Simulator<double>>(*diagram_);
  diagram_context_ = &simulator_->get_mutable_context();
  plant_context_ = &plant_->GetMyMutableContextFromRoot(diagram_context_);
  scene_graph_context_ =
      &scene_graph_->GetMyMutableContextFromRoot(diagram_context_);
  simulator_->Initialize();
}

void ContactSolverDriver::AddGround(double stiffness, double damping,
                                    double dynamic_friction) {
  // We demand all geometry registration happens pre- context creation so that
  // it is all well defined within the context post- context creation.
  DRAKE_DEMAND(diagram_context_ == nullptr);
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

void ContactSolverDriver::SetPointContactParameters(const Body<double>& body,
                                                    double stiffness,
                                                    double damping) {
  // We demand all geometry registration happens pre- context creation so that
  // it is all well defined within the context post- context creation.
  DRAKE_DEMAND(diagram_context_ == nullptr);
  const std::vector<geometry::GeometryId>& geometries =
      plant_->GetCollisionGeometriesForBody(body);

  PRINT_VAR(body.name());

  for (const auto id : geometries) {
    PRINT_VAR(id);
    const geometry::ProximityProperties* old_props =
        scene_graph_->model_inspector().GetProximityProperties(id);
    DRAKE_DEMAND(old_props);
    geometry::ProximityProperties new_props(*old_props);
    // Add a new property.
    PRINT_VAR(geometry::internal::kMaterialGroup);
    PRINT_VAR(geometry::internal::kHcDissipation);
    new_props.AddProperty(geometry::internal::kMaterialGroup,
                          geometry::internal::kPointStiffness, stiffness);
    new_props.AddProperty(geometry::internal::kMaterialGroup,
                          geometry::internal::kHcDissipation, damping);

    // Remove a property previously assigned.
    // new_props.RemoveProperty("old_group", "old_name_1");
    // Update the *value* of an existing property (but enforce same type).
    // new_props.UpdateProperty("old_group", "old_name_2", new_value);

    scene_graph_->AssignRole(*plant_->get_source_id(), id, new_props,
                             geometry::RoleAssign::kReplace);
  }
}

void ContactSolverDriver::SetDynamicFrictionCoefficient(
    const Body<double>& body, double mu_d) {
  // We demand this method to be called post- contex creation.
  DRAKE_DEMAND(diagram_context_ != nullptr);
  const std::vector<geometry::GeometryId>& geometries =
      plant_->GetCollisionGeometriesForBody(body);

  PRINT_VAR(body.name());
  PRINT_VAR(mu_d);

  const auto& query_object =
      plant_->get_geometry_query_input_port()
          .Eval<geometry::QueryObject<double>>(*plant_context_);
  const auto& inspector = query_object.inspector();

  for (const auto id : geometries) {
    PRINT_VAR(id);
    const geometry::ProximityProperties* old_props =
        inspector.GetProximityProperties(id);
    DRAKE_DEMAND(old_props);
    geometry::ProximityProperties new_props(*old_props);
    // Add a new property.
    CoulombFriction<double> friction(mu_d, mu_d);
    new_props.UpdateProperty(geometry::internal::kMaterialGroup,
                             geometry::internal::kFriction, friction);
    scene_graph_->AssignRole(scene_graph_context_, *plant_->get_source_id(),
                             id, new_props, geometry::RoleAssign::kReplace);
  }
}

std::vector<std::pair<double, double>>
ContactSolverDriver::GetPointContactComplianceParameters(
    const Body<double>& body) {
  DRAKE_DEMAND(diagram_context_ != nullptr);
  const std::vector<geometry::GeometryId>& geometries =
      plant_->GetCollisionGeometriesForBody(body);

  PRINT_VAR(body.name());

  std::vector<std::pair<double, double>> params;

  for (const auto id : geometries) {
    PRINT_VAR(id);
    const geometry::ProximityProperties* props =
        scene_graph_->model_inspector().GetProximityProperties(id);
    DRAKE_DEMAND(props);

    const double k =
        props->GetProperty<double>(geometry::internal::kMaterialGroup,
                                   geometry::internal::kPointStiffness);
    const double d = props->GetProperty<double>(
        geometry::internal::kMaterialGroup, geometry::internal::kHcDissipation);

    params.push_back({k, d});
  }
  return params;
}

std::vector<double> ContactSolverDriver::GetDynamicFrictionCoefficients(
    const Body<double>& body) const {
  DRAKE_DEMAND(diagram_context_ != nullptr);
  const std::vector<geometry::GeometryId>& geometries =
      plant_->GetCollisionGeometriesForBody(body);

  PRINT_VAR(body.name());

  const auto& query_object =
      plant_->get_geometry_query_input_port()
          .Eval<geometry::QueryObject<double>>(*plant_context_);
  const auto& inspector = query_object.inspector();

  std::vector<double> params;
  for (const auto id : geometries) {
    PRINT_VAR(id);
    const geometry::ProximityProperties* props =
        inspector.GetProximityProperties(id);
    DRAKE_DEMAND(props);

    const auto& friction = props->GetProperty<CoulombFriction<double>>(
        geometry::internal::kMaterialGroup, geometry::internal::kFriction);
    params.push_back(friction.dynamic_friction());
  }
  return params;    
}

}  // namespace test
}  // namespace multibody
}  // namespace drake
