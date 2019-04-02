#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

#include <algorithm>
#include <utility>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram_builder.h"

using drake::geometry::SceneGraph;

// TODO(eric.cousineau): Replace `TwoFreeBodies*` methods with
// `two_bodies.sdf`; consider using geometry queries to get sphere radii, etc.

namespace drake {
namespace multibody {

using math::RigidTransformd;

IiwaKinematicConstraintTest::IiwaKinematicConstraintTest() {
  const std::string iiwa_path = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/sdf/"
      "iiwa14_no_collision.sdf");
  systems::DiagramBuilder<double> builder{};
  plant_ = builder.AddSystem<MultibodyPlant<double>>();
  plant_->RegisterAsSourceForSceneGraph(
      builder.AddSystem<SceneGraph<double>>());
  multibody::Parser parser{plant_};
  parser.AddModelFromFile(iiwa_path, "iiwa");
  plant_->WeldFrames(plant_->world_frame(),
                     plant_->GetFrameByName("iiwa_link_0"));
  plant_->Finalize();

  drake::log()->info("plant_->num_positions = {}", plant_->num_positions());

  diagram_ = builder.Build();
  diagram_context_ = diagram_->CreateDefaultContext();
  plant_context_ =
      &diagram_->GetMutableSubsystemContext(*plant_, diagram_context_.get());

  plant_autodiff_ =
      systems::System<double>::ToAutoDiffXd(*ConstructIiwaPlant(iiwa_path, 0.));
  plant_context_autodiff_ = plant_autodiff_->CreateDefaultContext();
}

TwoFreeBodiesConstraintTest::TwoFreeBodiesConstraintTest() {
  systems::DiagramBuilder<double> builder{};
  plant_ = builder.AddSystem(ConstructTwoFreeBodiesPlant<double>());
  diagram_ = builder.Build();
  diagram_context_ = diagram_->CreateDefaultContext();
  plant_context_ =
      &diagram_->GetMutableSubsystemContext(*plant_, diagram_context_.get());
  body1_index_ = plant_->GetBodyByName("body1").body_frame().index();
  body2_index_ = plant_->GetBodyByName("body2").body_frame().index();

  plant_autodiff_ = ConstructTwoFreeBodiesPlant<AutoDiffXd>();
  plant_context_autodiff_ = plant_autodiff_->CreateDefaultContext();
}

template <typename T>
void AddTwoFreeBodiesToPlant(MultibodyPlant<T>* model) {
  const double mass{1};
  const Eigen::Vector3d p_AoAcm_A(0, 0, 0);
  const RotationalInertia<double> I_AAcm_A{0.001, 0.001, 0.001};
  const SpatialInertia<double> M_AAo_A =
      SpatialInertia<double>::MakeFromCentralInertia(mass, p_AoAcm_A, I_AAcm_A);

  model->AddRigidBody("body1", M_AAo_A);
  model->AddRigidBody("body2", M_AAo_A);
}

template <typename T>
std::unique_ptr<MultibodyPlant<T>> ConstructTwoFreeBodiesPlant() {
  auto model = std::make_unique<MultibodyPlant<T>>();
  AddTwoFreeBodiesToPlant(model.get());
  model->Finalize();
  return model;
}

std::unique_ptr<MultibodyPlant<double>> ConstructIiwaPlant(
    const std::string& file_path, double time_step) {
  auto plant = std::make_unique<MultibodyPlant<double>>(time_step);
  Parser(plant.get()).AddModelFromFile(file_path);
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("iiwa_link_0"));
  plant->Finalize();
  return plant;
}

Eigen::Vector4d QuaternionToVectorWxyz(const Eigen::Quaterniond& q) {
  return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
}

namespace {
template <typename T>
std::unique_ptr<systems::Diagram<T>> BuildTwoFreeSpheresDiagram(
    double radius1, double radius2, const Eigen::Isometry3d& X_B1S1,
    const Eigen::Isometry3d& X_B2S2, MultibodyPlant<T>** plant,
    geometry::SceneGraph<T>** scene_graph, FrameIndex* sphere1_index,
    FrameIndex* sphere2_index) {
  systems::DiagramBuilder<T> builder;
  std::tie(*plant, *scene_graph) = AddMultibodyPlantSceneGraph(&builder);
  AddTwoFreeBodiesToPlant(*plant);
  const auto& sphere1 = (*plant)->GetBodyByName("body1");
  const auto& sphere2 = (*plant)->GetBodyByName("body2");
  (*plant)->RegisterCollisionGeometry(
      sphere1, RigidTransformd(X_B1S1), geometry::Sphere(radius1),
      "sphere1_collision", multibody::CoulombFriction<double>(), *scene_graph);
  (*plant)->RegisterCollisionGeometry(
      sphere2, RigidTransformd(X_B2S2), geometry::Sphere(radius2),
      "sphere2_collision", multibody::CoulombFriction<double>(), *scene_graph);
  *sphere1_index = sphere1.body_frame().index();
  *sphere2_index = sphere2.body_frame().index();
  (*plant)->Finalize(*scene_graph);

  return builder.Build();
}
}  // namespace

TwoFreeSpheresTest::TwoFreeSpheresTest() {
  X_B1S1_.setIdentity();
  X_B1S1_.translation() << 0.01, 0.02, 0.03;
  X_B2S2_.setIdentity();
  X_B2S2_.translation() << 0.02, -0.01, -0.02;
  diagram_double_ = BuildTwoFreeSpheresDiagram(
      radius1_, radius2_, X_B1S1_, X_B2S2_, &plant_double_,
      &scene_graph_double_, &sphere1_index_, &sphere2_index_);
  diagram_context_double_ = diagram_double_->CreateDefaultContext();
  plant_context_double_ = &(diagram_double_->GetMutableSubsystemContext(
      *plant_double_, diagram_context_double_.get()));

  diagram_autodiff_ = BuildTwoFreeSpheresDiagram(
      radius1_, radius2_, X_B1S1_, X_B2S2_, &plant_autodiff_,
      &scene_graph_autodiff_, &sphere1_index_, &sphere2_index_);

  diagram_context_autodiff_ = diagram_autodiff_->CreateDefaultContext();
  plant_context_autodiff_ = &(diagram_autodiff_->GetMutableSubsystemContext(
      *plant_autodiff_, diagram_context_autodiff_.get()));
}

template <typename T>
std::unique_ptr<systems::Diagram<T>> ConstructBoxSphereDiagram(
    const Eigen::Vector3d& box_size, double radius, MultibodyPlant<T>** plant,
    geometry::SceneGraph<T>** scene_graph) {
  systems::DiagramBuilder<T> builder;
  std::tie(*plant, *scene_graph) = AddMultibodyPlantSceneGraph(&builder);
  const auto& box = (*plant)->AddRigidBody(
      "box", SpatialInertia<double>(1, Eigen::Vector3d(0, 0, 0),
                                    UnitInertia<double>(1, 1, 1)));
  (*plant)->RegisterCollisionGeometry(
      box, RigidTransformd::Identity(),
      geometry::Box(box_size(0), box_size(1), box_size(2)), "box",
      CoulombFriction<double>(0.9, 0.8), *scene_graph);

  const auto& sphere = (*plant)->AddRigidBody(
      "sphere", SpatialInertia<double>(1, Eigen::Vector3d::Zero(),
                                       UnitInertia<double>(1, 1, 1)));
  (*plant)->RegisterCollisionGeometry(
      sphere, RigidTransformd::Identity(), geometry::Sphere(radius), "sphere",
      CoulombFriction<double>(0.9, 0.8), *scene_graph);

  (*plant)->Finalize();

  return builder.Build();
}

BoxSphereTest::BoxSphereTest() : box_size_(10, 10, 10), radius_(1) {
  diagram_double_ = ConstructBoxSphereDiagram(
      box_size_, radius_, &plant_double_, &scene_graph_double_);
  diagram_context_double_ = diagram_double_->CreateDefaultContext();
  plant_context_double_ = &(diagram_double_->GetMutableSubsystemContext(
      *plant_double_, diagram_context_double_.get()));

  diagram_autodiff_ = ConstructBoxSphereDiagram(
      box_size_, radius_, &plant_autodiff_, &scene_graph_autodiff_);
  diagram_context_autodiff_ = diagram_autodiff_->CreateDefaultContext();
  plant_context_autodiff_ = &(diagram_autodiff_->GetMutableSubsystemContext(
      *plant_autodiff_, diagram_context_autodiff_.get()));
}

template <typename T>
T BoxSphereSignedDistance(const Eigen::Ref<const Eigen::Vector3d>& box_size,
                          double radius, const math::RigidTransform<T>& X_WB,
                          const math::RigidTransform<T>& X_WS) {
  const math::RigidTransform<T> X_BS = X_WB.inverse() * X_WS;
  const Vector3<T>& p_BS = X_BS.translation();
  // Check if the sphere center is within the box.
  const Eigen::Array3d half_size = box_size.array() / 2;
  const bool is_sphere_center_inside_box =
      (p_BS.array().abs() <= half_size).all();
  using std::max;
  if (is_sphere_center_inside_box) {
    // Find the distance from the sphere center to the closest face, add the
    // radius, and negate to indicate penetration.
    return -(half_size - p_BS.array().abs()).minCoeff() - radius;
  } else {
    T signed_distance = 0;
    using std::pow;
    using std::abs;
    for (int i = 0; i < 3; ++i) {
      // Compute the distance from the sphere center box face along the i'th
      // dimension.
      T distance_i = max(T(0), abs(p_BS(i)) - T(half_size(i)));
      signed_distance += pow(distance_i, 2);
    }
    using std::sqrt;
    signed_distance = sqrt(signed_distance) - radius;
    return signed_distance;
  }
}

template std::unique_ptr<MultibodyPlant<double>>
ConstructTwoFreeBodiesPlant<double>();
template std::unique_ptr<MultibodyPlant<AutoDiffXd>>
ConstructTwoFreeBodiesPlant<AutoDiffXd>();

template double BoxSphereSignedDistance<double>(
    const Eigen::Ref<const Eigen::Vector3d>& box_size, double radius,
    const math::RigidTransform<double>& X_WB,
    const math::RigidTransform<double>& X_WS);
template AutoDiffXd BoxSphereSignedDistance<AutoDiffXd>(
    const Eigen::Ref<const Eigen::Vector3d>& box_size, double radius,
    const math::RigidTransform<AutoDiffXd>& X_WB,
    const math::RigidTransform<AutoDiffXd>& X_WS);

}  // namespace multibody
}  // namespace drake
