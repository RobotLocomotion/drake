#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/multibody/contact_solvers/icf/icf_builder.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"

using drake::math::RigidTransformd;
using Eigen::Vector3d;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

class TwoSpheres : public testing::TestWithParam<ContactModel> {
 public:
  TwoSpheres() {
    multibody::MultibodyPlantConfig plant_config{.time_step = 0.0};

    geometry::SceneGraphConfig scene_graph_config{
        .default_proximity_properties = {.compliance_type = "compliant"}};

    std::tie(plant_, scene_graph_) = multibody::AddMultibodyPlant(
        plant_config, scene_graph_config, &builder_);

    // Add two spheres with arbitrary mass/inertia.
    std::string xml = fmt::format(R"""(
    <mujoco model="twospheres">
      <worldbody>
         <body name="sphere1"> <geom type="sphere" size="{}"/><freejoint name="freejoint1"/></body>
         <body name="sphere2"> <geom type="sphere" size="{}"/><freejoint name="freejoint2"/></body>
      </worldbody>
    </mujoco>
    )""",
                                  kRadius1, kRadius2);
    multibody::Parser parser(plant_);
    parser.AddModelsFromString(xml, "xml");
    sphere1_ = &plant_->GetBodyByName("sphere1");
    sphere2_ = &plant_->GetBodyByName("sphere2");
    sphere1_index_ = sphere1_->index();
    sphere2_index_ = sphere2_->index();

    plant_->set_contact_model(GetParam());
  }

  void FinalizeAndBuild() {
    plant_->Finalize();

    diagram_ = builder_.Build();

    diagram_context_ = diagram_->CreateDefaultContext();
    plant_context_ =
        &plant_->GetMyMutableContextFromRoot(diagram_context_.get());
  }

  void SetInContact(double penetration) {
    plant_->SetFloatingBaseBodyPoseInWorldFrame(plant_context_, *sphere1_,
                                                RigidTransformd::Identity());
    plant_->SetFloatingBaseBodyPoseInWorldFrame(
        plant_context_, *sphere2_,
        RigidTransformd(Vector3d(kRadius1 + kRadius2 - penetration, 0, 0)));
  }

 protected:
  // Sphere parameters.
  static constexpr double kRadius1{0.1};
  static constexpr double kRadius2{0.2};

  systems::DiagramBuilder<double> builder_;
  std::unique_ptr<systems::Diagram<double>> diagram_;
  multibody::MultibodyPlant<double>* plant_{};
  geometry::SceneGraph<double>* scene_graph_{};
  const multibody::RigidBody<double>* sphere1_{};
  const multibody::RigidBody<double>* sphere2_{};
  multibody::BodyIndex sphere1_index_;
  multibody::BodyIndex sphere2_index_;
  std::unique_ptr<systems::Context<double>> diagram_context_;
  systems::Context<double>* plant_context_{};
};

TEST_P(TwoSpheres, MakeData) {
  FinalizeAndBuild();
  const double penetration = 0.002;
  const double time_step = 0.01;
  SetInContact(penetration);
  const int nv = plant_->num_velocities();

  const int num_pairs =
      plant_->get_contact_model() == ContactModel::kPoint ? 1 : 4;

  IcfBuilder<double> builder(plant_);
  IcfModel<double> model;
  builder.UpdateModel(*plant_context_, time_step, nullptr, nullptr, &model);
  EXPECT_EQ(model.num_cliques(), 2);
  EXPECT_EQ(model.num_velocities(), nv);
  EXPECT_EQ(model.num_patch_constraints(), 1);
  EXPECT_EQ(model.clique_size(0), 6);
  EXPECT_EQ(model.clique_size(1), 6);

  PatchConstraintsPool<double>& patch_constraints =
      model.patch_constraints_pool();
  EXPECT_EQ(patch_constraints.num_patches(), 1);
  EXPECT_EQ(patch_constraints.total_num_pairs(), num_pairs);
  EXPECT_THAT(patch_constraints.patch_sizes(), testing::ElementsAre(num_pairs));

  IcfData<double> data;
  model.ResizeData(&data);
  EXPECT_EQ(data.num_velocities(), model.num_velocities());
  EXPECT_EQ(data.patch_constraints_data().num_constraints(), 1);

  // Clear patch constraints and verify resizing data does not allocate.
  patch_constraints.Resize({});
  EXPECT_EQ(model.num_velocities(), nv);
  EXPECT_EQ(model.num_patch_constraints(), 0);
  {
    drake::test::LimitMalloc guard;
    model.ResizeData(&data);
  }
  EXPECT_EQ(model.num_velocities(), nv);
  EXPECT_EQ(data.patch_constraints_data().num_constraints(), 0);

  // Update problem. There should be no allocations for the same problem size.
  {
    // TODO(amcastro-tri): Fix this function to not allocate. You'll need a
    // pre-allocated workspace for this function.
    drake::test::LimitMalloc guard({
        .max_num_allocations = 352,
        .min_num_allocations = 0,
        .ignore_realloc_noops = true,
    });
    builder.UpdateModel(*plant_context_, time_step, nullptr, nullptr, &model);
  }
  {
    drake::test::LimitMalloc guard;
    model.ResizeData(&data);
  }
  EXPECT_EQ(model.num_velocities(), nv);
  EXPECT_EQ(data.patch_constraints_data().num_constraints(), 1);
  // TODO(rpoyner-tri): test more results.
}

TEST_P(TwoSpheres, ZeroMass) {
  FinalizeAndBuild();
  const double penetration = 0.002;
  const double time_step = 0.01;
  SetInContact(penetration);
  sphere1_->SetMass(plant_context_, 0.0);

  IcfBuilder<double> builder(plant_);
  IcfModel<double> model;
  DRAKE_EXPECT_THROWS_MESSAGE(
      builder.UpdateModel(*plant_context_, time_step, nullptr, nullptr, &model),
      ".*sphere1.*zero.*mass.*");
}

TEST_P(TwoSpheres, SwappedAnchorage) {
  // Weld the second sphere to force a reverse-ordered contact.
  plant_->WeldFrames(plant_->get_body(BodyIndex(0)).body_frame(),
                     sphere2_->body_frame());
  FinalizeAndBuild();
  const double penetration = 0.002;
  const double time_step = 0.01;
  // Construct contact pose by hand.
  plant_->SetFloatingBaseBodyPoseInWorldFrame(
      plant_context_, *sphere1_,
      RigidTransformd(Vector3d(kRadius1 + kRadius2 - penetration, 0, 0)));

  IcfBuilder<double> builder(plant_);
  IcfModel<double> model;
  builder.UpdateModel(*plant_context_, time_step, nullptr, nullptr, &model);

  PatchConstraintsPool<double>& patch_constraints =
      model.patch_constraints_pool();
  ASSERT_EQ(patch_constraints.num_patches(), 1);
  // In the patch constraints pairs, the dynamic body appears first.
  EXPECT_EQ(patch_constraints.bodies()[0].first, sphere1_->index());
}

INSTANTIATE_TEST_SUITE_P(
    TestContactModels, TwoSpheres,
    testing::Values(ContactModel::kHydroelastic, ContactModel::kPoint,
                    ContactModel::kHydroelasticWithFallback),
    [](const testing::TestParamInfo<TwoSpheres::ParamType>& stuff) {
      return multibody::internal::GetStringFromContactModel(stuff.param);
    });

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
