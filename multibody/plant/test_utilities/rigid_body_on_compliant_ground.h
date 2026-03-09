#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/sap_driver.h"
#include "drake/multibody/plant/tamsi_driver.h"
#include "drake/multibody/plant/test/compliant_contact_manager_tester.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace internal {

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using math::RigidTransformd;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::Simulator;

struct ContactTestConfig {
  // This is a gtest test suffix; no underscores or spaces.
  std::string description;
  // Contact is modeled with point contact if `true` or with hydroelastic
  // contact if `false`.
  bool point_contact{};
  // Option to allow changing the default contact model. This allows unit
  // testing of cases using the hydroelastic contact model, whether point
  // contact is used or not.
  ContactModel contact_model{ContactModel::kHydroelasticWithFallback};
// Remove on 2026-09-01 per TAMSI deprecation.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  // Option that allows to exercise the TAMSI and SAP solver code paths
  DiscreteContactSolver contact_solver{DiscreteContactSolver::kTamsi};
#pragma GCC diagnostic pop
};

// This provides the suffix for each test parameter: the test config
// description.
std::ostream& operator<<(std::ostream& out, const ContactTestConfig& c) {
  out << c.description;
  return out;
}

// The purpose of this fixture is to unit test the implementation of TamsiDriver
// and SapDriver's contact computations. In this regard this is more of an
// integration test where the correctness of the results rely on the ability of
// the driver to properly setup a contact problem using MultibodyPlant (for
// kinematics and dynamics) and CompliantContactManager's services (for
// contact), and solve it with the corresponding solver.
class RigidBodyOnCompliantGround
    : public ::testing::TestWithParam<ContactTestConfig> {
 public:
  // This fixture sets up a problem where a rigid body is set on top of a
  // compliant ground. The position of the body is set so that the compliant
  // contact force with the ground balances that of gravity.
  //
  // Depending on the ContactTestConfig parameter of this fixture, the geometry
  // model of the rigid body will consist of either a point contact sphere or a
  // rigid hydroelastic mesh of a square plate.
  void SetUp() override {
    const ContactTestConfig& config = GetParam();
    DiagramBuilder<double> builder;
    auto items = AddMultibodyPlantSceneGraph(&builder, kTimeStep_);
    plant_ = &items.plant;
// Remove on 2026-09-01 per TAMSI deprecation.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    // N.B. We want to exercise the TAMSI and SAP code paths. Therefore we
    // arbitrarily choose two model approximations to accomplish this.
    switch (config.contact_solver) {
      case DiscreteContactSolver::kTamsi:
        plant_->set_discrete_contact_approximation(
            DiscreteContactApproximation::kTamsi);
        break;
      case DiscreteContactSolver::kSap:
        plant_->set_discrete_contact_approximation(
            DiscreteContactApproximation::kSap);
        break;
    }
#pragma GCC diagnostic pop

    // We change the default gravity magnitude so that numbers are simpler to
    // work with.
    plant_->mutable_gravity_field().set_gravity_vector(
        Vector3d(0.0, 0.0, -kGravity_));

    // Arbitrary inertia values.
    const double radius = 0.1;
    const SpatialInertia<double> M_BBcm =
        SpatialInertia<double>::SolidSphereWithMass(kMass_, radius);

    body_ = &plant_->AddRigidBody("body", M_BBcm);

    // N.B. We add an intermediate zero mass body so that we can use separate
    // prismatic joints for the x and z directions. Even though the intermediate
    // body has zero mass, a non-zero velocity in z leads to the motion of the
    // non-zero mass of `body_`. Therefore kinetic energy still is positive and
    // the mass matrix is positive-definite.
    const auto& intermediate_body = plant_->AddRigidBody(
        "intermediate_body",
        SpatialInertia<double>(0.0, Vector3d::Zero(),
                               UnitInertia<double>(1.0, 1.0, 1.0)));

    z_axis_ = &plant_->AddJoint<PrismaticJoint>("z_axis", plant_->world_body(),
                                                {}, intermediate_body, {},
                                                Vector3d::UnitZ());
    x_axis_ = &plant_->AddJoint<PrismaticJoint>("x_axis", intermediate_body, {},
                                                *body_, {}, Vector3d::UnitX());

    // Make body_ very stiff (1e40). The effective stiffness of the contact
    // between body_ and the ground will be kStiffness_, by the combination rule
    // in GetCombinedPointContactStiffness().
    geometry::ProximityProperties body_props;
    geometry::AddContactMaterial(0.0, 1e40, CoulombFriction<double>(kMu_, kMu_),
                                 &body_props);

    if (config.point_contact) {
      plant_->RegisterCollisionGeometry(
          *body_, RigidTransformd::Identity(),
          geometry::Sphere(kPointContactSphereRadius_),
          "point_contact_geometry", body_props);
    } else {
      const std::string mesh_file = FindResourceOrThrow(
          "drake/multibody/plant/test_utilities/square_surface.obj");
      geometry::AddRigidHydroelasticProperties(&body_props);
      plant_->RegisterCollisionGeometry(
          *body_, RigidTransformd::Identity(), geometry::Mesh(mesh_file),
          "hydroelastic_contact_geometry", body_props);
    }

    // Ground geometry.
    geometry::ProximityProperties ground_props;
    geometry::AddContactMaterial(kHcDissipation_, kStiffness_,
                                 CoulombFriction<double>(kMu_, kMu_),
                                 &ground_props);
    geometry::AddCompliantHydroelasticPropertiesForHalfSpace(
        kGroundThickness_, kHydroelasticModulus_, &ground_props);
    plant_->RegisterCollisionGeometry(
        plant_->world_body(),
        geometry::HalfSpace::MakePose(Vector3d::UnitZ(), Vector3d::Zero()),
        geometry::HalfSpace(), "ground_collision", ground_props);

    plant_->set_contact_model(config.contact_model);

    plant_->Finalize();

    diagram_ = builder.Build();

    // Make and add a manager so that we have access to it.
    auto owned_contact_manager =
        std::make_unique<CompliantContactManager<double>>();
    manager_ = owned_contact_manager.get();
    plant_->SetDiscreteUpdateManager(std::move(owned_contact_manager));
    tamsi_driver_ = std::make_unique<TamsiDriver<double>>(manager_);
    sap_driver_ = std::make_unique<SapDriver<double>>(manager_);

    // Create context.
    context_ = diagram_->CreateDefaultContext();
    plant_context_ =
        &diagram_->GetMutableSubsystemContext(*plant_, context_.get());

    // Set a known configuration.
    z_axis_->set_translation(plant_context_, CalcEquilibriumZPosition());
  }

 protected:
  // Weight in Newtons of the rigid body.
  double CalcBodyWeight() const {
    return kMass_ * plant_->gravity_field().gravity_vector().norm();
  }

  // Computes the z position of the body in this test at which the compliant
  // contact force balances its weight.
  double CalcEquilibriumZPosition() const {
    const ContactTestConfig& config = GetParam();
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

  // Normal force exactly opposes weight for the body in equilibrium.
  // Applying a force fₜ, perpendicular to fₙ with |fₜ| < kMu_ * |fₙ|
  // should keep the body in stiction. We assume that `plant_context_`
  // stores the static equilibrium condition and therefore:
  //  fₙ = (0, 0, kMass_ * kGravity_)
  // so we set:
  //  f_Bq_W = kScale_ * kMu_ * kMass_ * kGravity * (1, 0, 0)
  // where kScale < 1.
  void ApplyTangentialForceForBodyInStiction() const {
    std::vector<ExternallyAppliedSpatialForce<double>> forces(1);
    forces[0].body_index = body_->index();
    forces[0].p_BoBq_B = body_->CalcCenterOfMassInBodyFrame(*plant_context_);
    forces[0].F_Bq_W =
        SpatialForce<double>(Vector3d::Zero(), kTangentialStictionForce_);
    plant_->get_applied_spatial_force_input_port().FixValue(plant_context_,
                                                            forces);
  }

  // Normal force exactly opposes weight for the body in equilibrium.
  // Applying a force fₜ, perpendicular to fₙ with |fₜ| >= kMu_ * |fₙ|
  // should result in a body slipping with a friction force of magnitude:
  //   |fₛ| = kMu_ * |fₙ|
  // We assume that `plant_context_` stores the static equilibrium condition and
  // therefore:
  //  fₙ = (0, 0, kMass_ * kGravity_)
  // so we set:
  //  f_Bq_W = (kMu_ + 0.05) * kMass_ * kGravity * (1, 0, 0)
  // such that the net tangential force on the body produces an acceleration
  // that is 5% of that due to gravity on the body.
  void ApplyTangentialForceForBodyInSlip() const {
    std::vector<ExternallyAppliedSpatialForce<double>> forces(1);
    forces[0].body_index = body_->index();
    forces[0].p_BoBq_B = body_->CalcCenterOfMassInBodyFrame(*plant_context_);
    forces[0].F_Bq_W = SpatialForce<double>(
        Vector3d::Zero(), kTangentialSlipForce_ + kTangentialExtraForce_);
    plant_->get_applied_spatial_force_input_port().FixValue(plant_context_,
                                                            forces);
  }

  void Simulate(int num_time_steps) {
    simulator_.reset(new Simulator<double>(*diagram_, std::move(context_)));
    simulator_->Initialize();
    simulator_->AdvanceTo(num_time_steps * kTimeStep_);
  }

  std::unique_ptr<Diagram<double>> diagram_;
  MultibodyPlant<double>* plant_{nullptr};
  const RigidBody<double>* body_{nullptr};
  const PrismaticJoint<double>* z_axis_{nullptr};
  const PrismaticJoint<double>* x_axis_{nullptr};
  CompliantContactManager<double>* manager_{nullptr};
  std::unique_ptr<Context<double>> context_;
  Context<double>* plant_context_{nullptr};
  std::unique_ptr<TamsiDriver<double>> tamsi_driver_;
  std::unique_ptr<SapDriver<double>> sap_driver_;
  std::unique_ptr<Simulator<double>> simulator_;

  // Parameters of the problem.
  const double kTimeStep_{0.001};  // Discrete time step of the plant.
  const double kGravity_{10.0};    // Acceleration of gravity, in m/s².
  const double kMass_{10.0};       // Mass of the rigid body, in kg.
  const double kPointContactSphereRadius_{0.02};  // In m.
  const double kStiffness_{1.0e4};                // In N/m.
  const double kHydroelasticModulus_{250.0};      // In Pa.
  const double kHcDissipation_{0.2};              // In s/m.
  const double kGroundThickness_{0.1};            // In m.
  const double kMu_{0.5};                         // Coefficient of friction.
  const double kScale_{0.1};  // Scale factor for tangential force.
  // Horizontal force on the body that should be completely opposed by stiction.
  const Vector3d kTangentialStictionForce_{kScale_ * kMu_ * kMass_ * kGravity_,
                                           0.0, 0.0};
  // Expected horizontal friction force on the body when in slip.
  const Vector3d kTangentialSlipForce_{kMu_ * kMass_ * kGravity_, 0.0, 0.0};
  // Applied force added such that the resulting net tangential force on the
  // body produces an acceleration that is 5% of that due to gravity on the
  // body.
  const Vector3d kTangentialExtraForce_{0.05 * kMass_ * kGravity_, 0.0, 0.0};
  // Number of triangles and area of the plate must be kept in sync with the
  // mesh file square_surface.obj.
  const int kNumberOfTriangles_{2};  // Number of triangles in the hydro mesh.
  const double kArea_{4.0};          // Area of the rigid hydroelastic mesh.
};
}  // namespace internal
}  // namespace multibody
}  // namespace drake
