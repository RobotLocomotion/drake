#include "drake/multibody/plant/tamsi_driver.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/test/compliant_contact_manager_tester.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

using drake::math::RigidTransformd;
using drake::systems::Context;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

constexpr double kEps = std::numeric_limits<double>::epsilon();

namespace drake {
namespace multibody {
namespace internal {

struct TestConfig {
  // This is a gtest test suffix; no underscores or spaces.
  std::string description;
  // Model with point contact if `true` or with hydroelastic contact if `false`.
  bool point_contact{};
};

// This provides the suffix for each test parameter: the test config
// description.
std::ostream& operator<<(std::ostream& out, const TestConfig& c) {
  out << c.description;
  return out;
}

// The purpose of this fixture is to verify the implementation of TamsiDriver's
// CalcContactSolverResults(). In this regard this is more of an integration
// test where the correctness of the results rely on the ability of the driver
// to properly setup a contact problem for TAMSI using MultibodyPlant (for
// kinematics and dynamics) and CompliantContactManager's services (for
// contact), and solve it with TAMSI. MultibodyPlant, CompliantContactManager
// and TamsiSolver are tested elsewhere.
class RigidBodyOnCompliantGround : public ::testing::TestWithParam<TestConfig> {
 public:
  // This fixture sets up a problem where a rigid body is set on top of a
  // compliant ground. The position of the body is set so that the compliant
  // contact force with the ground balances that of gravity.
  //
  // Depending on the TestConfig parameter of this fixture, the geometry model
  // of the rigid body will consist of either a point contact sphere or a rigid
  // hydroelastic mesh of a square plate.
  void SetUp() override {
    DiagramBuilder<double> builder;
    auto items = AddMultibodyPlantSceneGraph(&builder, 0.001 /* time_step */);
    plant_ = &items.plant;
    plant_->set_discrete_contact_solver(DiscreteContactSolver::kTamsi);

    // We change the default gravity magnitude so that numbers are simpler to
    // work with.
    plant_->mutable_gravity_field().set_gravity_vector(
        Vector3d(0.0, 0.0, -kGravity_));

    // Arbitrary inertia values.
    const double radius = 0.1;
    const SpatialInertia<double> M_Bo =
        SpatialInertia<double>::MakeFromCentralInertia(
            kMass_, Vector3d::Zero(),
            UnitInertia<double>::SolidSphere(radius) * kMass_);

    body_ = &plant_->AddRigidBody("body", M_Bo);

    geometry::ProximityProperties body_props;
    geometry::AddContactMaterial(0.0, 1.0e40, CoulombFriction<double>(),
                                 &body_props);

    const TestConfig& config = GetParam();
    if (config.point_contact) {
      plant_->RegisterCollisionGeometry(
          *body_, RigidTransformd::Identity(),
          geometry::Sphere(kPointContactSphereRadius_),
          "point_contact_geometry", body_props);
    } else {
      const std::string mesh_file =
          FindResourceOrThrow("drake/multibody/plant/test/square_surface.obj");
      geometry::AddRigidHydroelasticProperties(&body_props);
      plant_->RegisterCollisionGeometry(
          *body_, RigidTransformd::Identity(), geometry::Mesh(mesh_file),
          "hydroelastic_contact_geometry", body_props);
    }

    // Ground geometry.
    geometry::ProximityProperties ground_props;
    geometry::AddContactMaterial(kHcDissipation_, kStiffness_,
                                 CoulombFriction<double>(), &ground_props);
    geometry::AddCompliantHydroelasticPropertiesForHalfSpace(
        kGroundThickness_, kHydroelasticModulus_, &ground_props);
    plant_->RegisterCollisionGeometry(
        plant_->world_body(),
        geometry::HalfSpace::MakePose(Vector3d::UnitZ(), Vector3d::Zero()),
        geometry::HalfSpace(), "ground_collision", ground_props);

    plant_->Finalize();

    diagram_ = builder.Build();

    // Make and add a manager so that we have access to it.
    auto owned_contact_manager =
        std::make_unique<CompliantContactManager<double>>();
    manager_ = owned_contact_manager.get();
    plant_->SetDiscreteUpdateManager(std::move(owned_contact_manager));
    tamsi_driver_ = std::make_unique<TamsiDriver<double>>(manager_);

    // Create context.
    context_ = diagram_->CreateDefaultContext();
    plant_context_ =
        &diagram_->GetMutableSubsystemContext(*plant_, context_.get());

    // Set a known configuration.
    const Vector3d p_WB(0.0, 0.0, CalcEquilibriumZPosition());
    const RigidTransformd X_WB(p_WB);
    plant_->SetFreeBodyPose(plant_context_, *body_, X_WB);
  }

 protected:
  // Weight in Newtons of the rigid body.
  double CalcBodyWeight() const {
    return kMass_ * plant_->gravity_field().gravity_vector().norm();
  }

  // Computes the z position of the body in this test at which the compliant
  // contact force balances its weight.
  double CalcEquilibriumZPosition() const {
    const TestConfig& config = GetParam();
    const double weight = CalcBodyWeight();
    // Either point contact stiffness or the effective hydroelastic stiffness.
    if (config.point_contact) {
      return kPointContactSphereRadius_ - weight / kStiffness_;
    } else {
      const double stiffness =
          kArea_ * kHydroelasticModulus_ / kGroundThickness_;
      return -weight / stiffness;
    }
  }

  std::unique_ptr<Diagram<double>> diagram_;
  MultibodyPlant<double>* plant_{nullptr};
  const RigidBody<double>* body_{nullptr};
  CompliantContactManager<double>* manager_{nullptr};
  std::unique_ptr<Context<double>> context_;
  Context<double>* plant_context_{nullptr};
  std::unique_ptr<TamsiDriver<double>> tamsi_driver_;

  // Parameters of the problem.
  const double kGravity_{10.0};  // Acceleration of gravity, in m/s².
  const double kMass_{10.0};     // Mass of the rigid body, in kg.
  const double kPointContactSphereRadius_{0.02};  // In m.
  const double kStiffness_{1.0e4};                // In N/m.
  const double kHydroelasticModulus_{250.0};      // In Pa.
  const double kHcDissipation_{0.2};              // In s/m.
  const double kGroundThickness_{0.1};            // In m.
  // Number of triangles and area of the plate must be kept in sync with the
  // mesh file square_surface.obj.
  const int kNumberOfTriangles_{2};  // Number of triangles in the hydro mesh.
  const double kArea_{4.0};          // Area of the rigid hydroelastic mesh.
};

// This test verifies contact results in the equilibrium configuration.
TEST_P(RigidBodyOnCompliantGround, VerifyEquilibriumConfiguration) {
  const TestConfig& config = GetParam();
  EXPECT_EQ(plant_->num_velocities(), 6);
  contact_solvers::internal::ContactSolverResults<double> results;
  tamsi_driver_->CalcContactSolverResults(*plant_context_, &results);

  EXPECT_EQ(results.v_next.size(), plant_->num_velocities());
  const int num_contacts = config.point_contact ? 1 : kNumberOfTriangles_;
  EXPECT_EQ(results.fn.size(), num_contacts);

  const double normal_force_expected = CalcBodyWeight();
  const double normal_force = results.fn.sum();
  EXPECT_NEAR(normal_force, normal_force_expected, kEps);
}

// Setup test cases using point and hydroelastic contact.
std::vector<TestConfig> MakeTestCases() {
  return std::vector<TestConfig>{
      {.description = "HydroelasticContact", .point_contact = false},
      {.description = "PointContact", .point_contact = true},
  };
}

INSTANTIATE_TEST_SUITE_P(TamsiDriverTests, RigidBodyOnCompliantGround,
                         testing::ValuesIn(MakeTestCases()),
                         testing::PrintToStringParamName());

}  // namespace internal
}  // namespace multibody
}  // namespace drake
