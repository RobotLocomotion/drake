#include <gtest/gtest.h>

#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/examples/multibody/rolling_sphere/make_rolling_sphere_plant.h"
#include "drake/multibody/plant/multibody_plant.h"

using Eigen::Vector3d;

namespace drake {
namespace multibody {
namespace {

class HydroelasticContactOutputTester : public ::testing::Test {
 protected:
  void SetUp() {
    const double radius = 1.0;                         // sphere radius (m).

    // The vertical location of the sphere. If this value is smaller than the
    // sphere radius, the sphere will intersect the half-space described by
    // z <= 0.
    const double z0 = 0.95 * radius;

    // Set some reasonable, but arbitrary, parameters: none of these will be
    // affect the test results.
    const double mass = 2.0;                           // kg.
    const double elastic_modulus = 1e7;                // Pascals.
    const double dissipation = 1.0;                    // s/m.
    const CoulombFriction<double> friction(1.0, 1.0);  // Static/dynamic.
    const Vector3<double> gravity_W(0, 0, -9.8);       // m/s^2.

    // Create the plant.
    systems::DiagramBuilder<double> builder;
    geometry::SceneGraph<double>& scene_graph =
        *builder.AddSystem<geometry::SceneGraph<double>>();
    scene_graph.set_name("scene_graph");
    plant_ = builder.AddSystem(
        examples::multibody::bouncing_ball::MakeBouncingBallPlant(
            radius, mass, elastic_modulus, dissipation, friction, gravity_W,
            &scene_graph));
    plant_->set_contact_model(ContactModel::kHydroelasticsOnly);
    plant_->Finalize();

    diagram_ = builder.Build();

    // Create a context for this system:
    diagram_context_ = diagram_->CreateDefaultContext();
    diagram_->SetDefaultContext(diagram_context_.get());
    plant_context_ =
        &diagram_->GetMutableSubsystemContext(*plant_, diagram_context_.get());

    // Set the sphere's initial pose.
    math::RigidTransformd X_WB(Vector3d(0.0, 0.0, z0));
    plant_->SetFreeBodyPose(plant_context_, plant_->GetBodyByName("Ball"),
                            X_WB);
  }

  MultibodyPlant<double>* plant_{};
  systems::Context<double>* plant_context_{};

 private:
  std::unique_ptr<systems::Diagram<double>> diagram_{};
  std::unique_ptr<systems::Context<double>> diagram_context_{};
};

TEST_F(HydroelasticContactOutputTester, ContactSurface) {
  // Get the contact results from the plant.
  const ContactResults<double>& contact_results =
      plant_->get_contact_results_output_port().Eval<ContactResults<double>>(
          *plant_context_);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
