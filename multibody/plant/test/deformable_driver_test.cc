#include "drake/multibody/plant/deformable_driver.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/systems/framework/diagram_builder.h"

using drake::geometry::GeometryId;
using drake::geometry::GeometryInstance;
using drake::geometry::SceneGraph;
using drake::geometry::SceneGraphInspector;
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

// Friend class used to provide access to a selection of private functions in
// DeformableDriver for testing purposes.
class DeformableDriverTester {
 public:
  static const FemState<double>& EvalFemState(
      const DeformableDriver<double>& driver, const Context<double>& context,
      DeformableBodyIndex index) {
    return driver.EvalFemState(context, index);
  }
};

namespace {

class DeformableDriverTest : public ::testing::Test {
 protected:
  void SetUp() override {
    constexpr double kDt = 0.01;
    std::tie(plant_, scene_graph_) =
        AddMultibodyPlantSceneGraph(&builder_, kDt);
    auto deformable_model = make_unique<DeformableModel<double>>(plant_);
    constexpr double kRezHint = 0.5;
    DeformableBodyId body_id = RegisterSphere(deformable_model.get(), kRezHint);
    geometry_id_ = deformable_model->GetGeometryIdOrThrow(body_id);
    plant_->AddPhysicalModel(move(deformable_model));
    plant_->Finalize();
    auto contact_manager = make_unique<CompliantContactManager<double>>();
    manager_ = contact_manager.get();
    plant_->SetDiscreteUpdateManager(move(contact_manager));
    driver_ = CompliantContactManagerTest::deformable_driver(*manager_);
    context_ = plant_->CreateDefaultContext();
  }

  systems::DiagramBuilder<double> builder_;
  MultibodyPlant<double>* plant_{nullptr};
  SceneGraph<double>* scene_graph_{nullptr};
  const CompliantContactManager<double>* manager_{nullptr};
  const DeformableDriver<double>* driver_{nullptr};
  std::unique_ptr<Context<double>> context_{nullptr};
  GeometryId geometry_id_;

 private:
  DeformableBodyId RegisterSphere(DeformableModel<double>* model,
                                  double resolution_hint) {
    auto geometry = make_unique<GeometryInstance>(
        RigidTransformd(), make_unique<Sphere>(1), "sphere");
    const fem::DeformableBodyConfig<double> default_body_config;
    DeformableBodyId body_id = model->RegisterDeformableBody(
        std::move(geometry), default_body_config, resolution_hint);
    return body_id;
  }
};

/* Verifies that a DeformableDriver has been successfully created. */
TEST_F(DeformableDriverTest, Constructor) { ASSERT_NE(driver_, nullptr); }

TEST_F(DeformableDriverTest, ScalarConversion) {
  EXPECT_FALSE(driver_->is_cloneable_to_double());
  EXPECT_FALSE(driver_->is_cloneable_to_autodiff());
  EXPECT_FALSE(driver_->is_cloneable_to_symbolic());
}

TEST_F(DeformableDriverTest, FemState) {
  const SceneGraphInspector<double>& inspector =
      scene_graph_->model_inspector();
  const VolumeMesh<double>* mesh_ptr = inspector.GetReferenceMesh(geometry_id_);
  ASSERT_NE(mesh_ptr, nullptr);
  const VolumeMesh<double>& mesh = *mesh_ptr;
  const int num_vertices = mesh.num_vertices();
  VectorX<double> q(3 * num_vertices);
  for (int v = 0; v < num_vertices; ++v) {
    q.segment<3>(3 * v) = mesh.vertex(v);
  }
  const auto v = VectorX<double>::Zero(3 * num_vertices);
  const auto a = VectorX<double>::Zero(3 * num_vertices);
  const FemState<double>& fem_state = DeformableDriverTester::EvalFemState(
      *driver_, *context_, DeformableBodyIndex(0));
  EXPECT_EQ(fem_state.GetPositions(), q);
  EXPECT_EQ(fem_state.GetVelocities(), v);
  EXPECT_EQ(fem_state.GetAccelerations(), a);
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
