#include "drake/multibody/test_utilities/robot_model.h"

#include "drake/common/test_utilities/maybe_pause_for_user.h"

namespace drake {
namespace multibody {
namespace test {

std::ostream& operator<<(std::ostream& out, const RobotModelConfig& c) {
  switch (c.contact_approximation) {
    case DiscreteContactApproximation::kLagged: {
      out << "Lagged";
      break;
    }
    case DiscreteContactApproximation::kSimilar: {
      out << "Similar";
      break;
    }
    case DiscreteContactApproximation::kSap: {
      out << "Sap";
      break;
    }
// Remove on 2026-09-01 per TAMSI deprecation.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    case DiscreteContactApproximation::kTamsi: {
      out << "Tamsi";
      break;
    }
#pragma GCC diagnostic pop
  }

  switch (c.contact_model) {
    case ContactModel::kHydroelastic: {
      out << "Hydro";
      break;
    }
    case ContactModel::kHydroelasticWithFallback: {
      out << "HydroWithFallback";
      break;
    }
    case ContactModel::kPoint: {
      out << "Point";
      break;
    }
  }

  switch (c.contact_configuration) {
    case RobotModelConfig::ContactConfig::kNoGeometry: {
      out << "NoGeometry";
      break;
    }
    case RobotModelConfig::ContactConfig::kNoContactState: {
      out << "NoContactState";
      break;
    }
    case RobotModelConfig::ContactConfig::kInContactState: {
      out << "InContactState";
      break;
    }
  }

  return out;
}

template <typename T>
RobotModel<T>::RobotModel(const RobotModelConfig& config,
                          bool add_visualization)
  requires std::is_same_v<T, double>
{  // NOLINT(whitespace/braces)
  systems::DiagramBuilder<double> builder;
  auto items = AddMultibodyPlantSceneGraph(&builder, kTimeStep);
  plant_ = &items.plant;

  Parser parser(&builder);
  robot_model_instance_ = parser.AddModelsFromUrl(
      "package://drake_models/iiwa_description/sdf/iiwa7_no_collision.sdf")[0];
  parser.AddModelsFromUrl("package://drake_models/dishes/plate_8in.sdf");

  // Weld the robot's base to the world.
  plant_->WeldFrames(plant_->world_frame(),
                     plant_->GetBodyByName("iiwa_link_0").body_frame(),
                     math::RigidTransformd::Identity());

  // Ground geometry.
  geometry::ProximityProperties proximity_properties;
  geometry::AddContactMaterial(kHcDissipation, kStiffness,
                               CoulombFriction<double>(kMu, kMu),
                               &proximity_properties);
  proximity_properties.AddProperty(geometry::internal::kMaterialGroup,
                                   geometry::internal::kRelaxationTime,
                                   kRelaxationTime / 2);
  plant_->RegisterCollisionGeometry(
      plant_->world_body(),
      math::RigidTransformd(Eigen::Vector3d(0.0, 0.0, -0.05)),
      geometry::Box(2.0, 2.0, 0.1), "ground_collision", proximity_properties);

  // Add simple contact geometry at the end effector.
  plant_->RegisterCollisionGeometry(
      plant_->GetBodyByName("iiwa_link_7"),
      math::RigidTransformd(Eigen::Vector3d(0.0, 0.0, 0.07)),
      geometry::Sphere(0.05), "iiwa_link_7_collision", proximity_properties);

  plant_->set_contact_model(config.contact_model);
  plant_->set_discrete_contact_approximation(config.contact_approximation);
  plant_->Finalize();

  // Remove proximity roles if geometry is not requested.
  if (config.contact_configuration ==
      RobotModelConfig::ContactConfig::kNoGeometry) {
    const auto& inspector = items.scene_graph.model_inspector();
    for (const auto id :
         inspector.GetAllGeometryIds(geometry::Role::kProximity)) {
      items.scene_graph.RemoveRole(*plant_->get_source_id(), id,
                                   geometry::Role::kProximity);
    }
  }

  // Make and add a manager so that we have access to it and its driver.
  if (plant_->get_discrete_contact_solver() == DiscreteContactSolver::kSap) {
    auto owned_contact_manager = std::make_unique<
        multibody::internal::CompliantContactManager<double>>();
    manager_ = owned_contact_manager.get();
    plant_->SetDiscreteUpdateManager(std::move(owned_contact_manager));
    driver_ = &multibody::internal::CompliantContactManagerTester::sap_driver(
        *manager_);
  }

  // We add a visualizer for visual inspection of the model in this test.
  if (add_visualization) {
    visualization::AddDefaultVisualization(&builder);
  }

  diagram_ = builder.Build();

  // Create context.
  context_ = diagram_->CreateDefaultContext();
  plant_context_ =
      &diagram_->GetMutableSubsystemContext(*plant_, context_.get());
  SetPlateInitialState();

  switch (config.contact_configuration) {
    case RobotModelConfig::ContactConfig::kNoGeometry: {
      SetRobotState(RobotZeroState());
      break;
    }
    case RobotModelConfig::ContactConfig::kNoContactState: {
      SetRobotState(RobotZeroState());
      break;
    }
    case RobotModelConfig::ContactConfig::kInContactState: {
      SetRobotState(RobotStateWithOneContactStiction());
      break;
    }
  }

  // Fix input ports.
  const VectorX<double> tau =
      VectorX<double>::Zero(plant_->num_actuated_dofs());
  plant_->get_actuation_input_port().FixValue(plant_context_, tau);
}

template <typename T>
template <typename U>
std::unique_ptr<RobotModel<U>> RobotModel<T>::ToScalarType() const {
  auto converted_model = std::make_unique<RobotModel<U>>();

  // Scalar-convert the model.
  converted_model->diagram_ = dynamic_pointer_cast<systems::Diagram<U>>(
      diagram_->template ToScalarType<U>());
  converted_model->plant_ = const_cast<MultibodyPlant<U>*>(
      &converted_model->diagram_
           ->template GetDowncastSubsystemByName<MultibodyPlant<U>>(
               plant_->get_name()));

  // Make and add a manager so that we have access to it and its driver.
  if constexpr (!std::is_same_v<U, symbolic::Expression>) {
    if (plant_->get_discrete_contact_solver() == DiscreteContactSolver::kSap) {
      auto owned_contact_manager_ad =
          std::make_unique<multibody::internal::CompliantContactManager<U>>();
      converted_model->manager_ = owned_contact_manager_ad.get();
      converted_model->plant_->SetDiscreteUpdateManager(
          std::move(owned_contact_manager_ad));
      converted_model->driver_ =
          &multibody::internal::CompliantContactManagerTester::sap_driver(
              *converted_model->manager_);
    }
  }

  // Create context.
  converted_model->context_ = converted_model->diagram_->CreateDefaultContext();
  converted_model->context_->SetTimeStateAndParametersFrom(*context_);
  converted_model->plant_context_ =
      &converted_model->diagram_->GetMutableSubsystemContext(
          *converted_model->plant_, converted_model->context_.get());

  // Fix input ports.
  const VectorX<U> tau_ad = VectorX<U>::Zero(plant_->num_actuated_dofs());
  converted_model->plant_->get_actuation_input_port().FixValue(
      converted_model->plant_context_, tau_ad);

  return converted_model;
}

template <typename T>
std::unique_ptr<RobotModel<AutoDiffXd>> RobotModel<T>::ToAutoDiffXd() const {
  return ToScalarType<AutoDiffXd>();
}

template <typename T>
void RobotModel<T>::ForcedPublish() {
  diagram_->ForcedPublish(*context_);
}

template <typename T>
void RobotModel<T>::SetState(const VectorX<T>& x) {
  plant_->SetPositionsAndVelocities(plant_context_, x);
}

template <typename T>
void RobotModel<T>::SetRobotState(const VectorX<T>& x) {
  plant_->SetPositionsAndVelocities(plant_context_, robot_model_instance_, x);
}

template <typename T>
VectorX<T> RobotModel<T>::GetState() const {
  return plant_->GetPositionsAndVelocities(*plant_context_);
}

template <typename T>
const multibody::contact_solvers::internal::SapContactProblem<T>&
RobotModel<T>::EvalContactProblem(const VectorX<T>& x0)
  requires(!std::is_same_v<T, symbolic::Expression>)
{  // NOLINT(whitespace/braces)
  DRAKE_DEMAND(plant_->get_discrete_contact_solver() ==
               DiscreteContactSolver::kSap);
  SetState(x0);
  const auto& problem_cache = driver_->EvalContactProblemCache(*plant_context_);
  // There are no locked dofs. Sanity check.
  DRAKE_DEMAND(problem_cache.sap_problem_locked == nullptr);
  return *problem_cache.sap_problem;
}

template <typename T>
Eigen::VectorXd RobotModel<T>::RobotStateWithOneContactStiction() {
  return (VectorX<double>(14) << 0, 1.17, 0, -1.33, 0, 0.58, 0,  // q
          0, 0, 0, 0, 0, 0, 0                                    // v
          )
      .finished();
}

template <typename T>
Eigen::VectorXd RobotModel<T>::RobotStateWithOneOneContactSliding() {
  return (VectorX<double>(14) << 0, 1.17, 0, -1.33, 0, 0.58, 0,  // q
          0, -0.1, 0, -0.2, 0, 0, 0                              // v
          )
      .finished();
}

template <typename T>
Eigen::VectorXd RobotModel<T>::RobotZeroState() {
  return VectorX<double>::Zero(14);
}

template <typename T>
void RobotModel<T>::SetPlateInitialState() {
  plant_->SetFreeBodyPose(plant_context_, plant_->GetBodyByName("plate_8in"),
                          math::RigidTransform<T>{Vector3<T>(0.5, 0.5, 0.5)});
}

void VisualizeRobotModel() {
  // Visualize the contact configuration.
  RobotModelConfig config{
      .contact_configuration = RobotModelConfig::ContactConfig::kInContactState,
  };
  RobotModel<double> model(config, true /* add viz */);

  model.ForcedPublish();
  common::MaybePauseForUser();
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    (&RobotModel<T>::template ToScalarType<U>));

}  // namespace test
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::test::RobotModel);
