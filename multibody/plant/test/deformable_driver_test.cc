#include "drake/multibody/plant/deformable_driver.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/systems/framework/diagram_builder.h"

using drake::geometry::GeometryId;
using drake::geometry::GeometryInstance;
using drake::geometry::SceneGraph;
using drake::geometry::Sphere;
using drake::geometry::VolumeMesh;
using drake::math::RigidTransformd;
using drake::multibody::fem::FemState;
using drake::systems::Context;
using std::make_unique;
using std::move;

namespace drake {
namespace multibody {
namespace internal {

// Friend class used to provide access to a selection of private functions in
// CompliantContactManager for testing purposes.
class CompliantContactManagerTest {
 public:
  static const DeformableDriver<double>* deformable_driver(
      const CompliantContactManager<double>& manager) {
    return manager.deformable_driver_.get();
  }
};

class DeformableDriverTest : public ::testing::Test {
 protected:
  void SetUp() override {
    constexpr double kDt = 0.01;
    std::tie(plant_, scene_graph_) =
        AddMultibodyPlantSceneGraph(&builder_, kDt);
    auto deformable_model = make_unique<DeformableModel<double>>(plant_);
    constexpr double kRezHint = 0.5;
    body_id_ = RegisterSphere(deformable_model.get(), kRezHint);
    model_ = deformable_model.get();
    plant_->AddPhysicalModel(move(deformable_model));
    plant_->Finalize();
    auto contact_manager = make_unique<CompliantContactManager<double>>();
    manager_ = contact_manager.get();
    plant_->SetDiscreteUpdateManager(move(contact_manager));
    driver_ = CompliantContactManagerTest::deformable_driver(*manager_);
    context_ = plant_->CreateDefaultContext();
  }

  /* Calls private member function in DeformableDriver with the same name. */
  const fem::FemState<double>& EvalFemState(
      const systems::Context<double>& context,
      DeformableBodyIndex index) const {
    return driver_->EvalFemState(context, index);
  }

  systems::DiagramBuilder<double> builder_;
  MultibodyPlant<double>* plant_{nullptr};
  SceneGraph<double>* scene_graph_{nullptr};
  const DeformableModel<double>* model_{nullptr};
  const CompliantContactManager<double>* manager_{nullptr};
  const DeformableDriver<double>* driver_{nullptr};
  std::unique_ptr<Context<double>> context_{nullptr};
  DeformableBodyId body_id_;

 private:
  DeformableBodyId RegisterSphere(DeformableModel<double>* model,
                                  double resolution_hint) {
    auto geometry = make_unique<GeometryInstance>(
        RigidTransformd(), make_unique<Sphere>(1), "sphere");
    const fem::DeformableBodyConfig<double> default_body_config;
    DeformableBodyId body_id = model->RegisterDeformableBody(
        move(geometry), default_body_config, resolution_hint);
    return body_id;
  }
};

namespace {

/* Verifies that a DeformableDriver has been successfully created. */
TEST_F(DeformableDriverTest, Constructor) { ASSERT_NE(driver_, nullptr); }

TEST_F(DeformableDriverTest, ScalarConversion) {
  EXPECT_FALSE(driver_->is_cloneable_to_double());
  EXPECT_FALSE(driver_->is_cloneable_to_autodiff());
  EXPECT_FALSE(driver_->is_cloneable_to_symbolic());
}

TEST_F(DeformableDriverTest, FemState) {
  const int num_dofs = model_->GetFemModel(body_id_).num_dofs();
  const auto q = 2.0 * VectorX<double>::Ones(num_dofs);
  const auto v = VectorX<double>::Ones(num_dofs);
  const auto a = VectorX<double>::Zero(num_dofs);
  VectorX<double> state_value(3 * num_dofs);
  state_value << q, v, a;
  const systems::DiscreteStateIndex state_index =
      model_->GetDiscreteStateIndex(body_id_);
  context_->SetDiscreteState(state_index, state_value);
  const FemState<double>& fem_state =
      EvalFemState(*context_, DeformableBodyIndex(0));
  EXPECT_EQ(fem_state.GetPositions(), q);
  EXPECT_EQ(fem_state.GetVelocities(), v);
  EXPECT_EQ(fem_state.GetAccelerations(), a);
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
