#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/value.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/bus_value.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {

// Exposes ComputeSurfaceVelocity for testing.
class MultibodyPlantTester {
 public:
  MultibodyPlantTester() = delete;
  static Vector3<double> ComputeSurfaceVelocity(
      const MultibodyPlant<double>& plant, BodyIndex body_index,
      const systems::Context<double>& context, const Eigen::Vector3d& n_W) {
    return plant.ComputeSurfaceVelocity(body_index, context, n_W);
  }
};

namespace {

using Eigen::Vector3d;
using geometry::AddCompliantHydroelasticProperties;
using geometry::AddContactMaterial;
using geometry::AddRigidHydroelasticProperties;
using geometry::Box;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::ProximityProperties;
using geometry::Sphere;
using math::RigidTransformd;
using math::RollPitchYawd;

GTEST_TEST(MultibodyPlantTest, RegisterSurfaceVelocityErrors) {
  MultibodyPlant<double> plant(0.0);
  const RigidBody<double>& belt =
      plant.AddRigidBody("belt", SpatialInertia<double>::MakeUnitary());

  // World body is not allowed.
  EXPECT_THROW(plant.RegisterSurfaceVelocity(plant.world_body(), {1, 0, 0}),
               std::exception);

  // Registering the same body twice is an error.
  plant.RegisterSurfaceVelocity(belt, {1, 0, 0});
  EXPECT_THROW(plant.RegisterSurfaceVelocity(belt, {1, 0, 0}), std::exception);

  // Registration after Finalize() is not allowed.
  plant.Finalize();
  EXPECT_THROW(plant.RegisterSurfaceVelocity(belt, {1, 0, 0}), std::exception);
}

// Fixture: finalized standalone plant, standalone context
//
// "belt" is registered with non-unit normal (2,0,0) to exercise normalization.
// "other" is intentionally left unregistered.  Tests that need a speed wired
// to the surface_speeds port call FixValue().
class SurfaceVelocityTest : public ::testing::Test {
 protected:
  void SetUp() override {
    belt_ = &plant_.AddRigidBody("belt", SpatialInertia<double>::MakeUnitary());
    other_ =
        &plant_.AddRigidBody("other", SpatialInertia<double>::MakeUnitary());
    // Intentionally non-unit to exercise normalization.
    plant_.RegisterSurfaceVelocity(*belt_, Vector3d(2, 0, 0));
    plant_.Finalize();
    context_ = plant_.CreateDefaultContext();
  }

  // Fixes a constant speed for "belt" on the surface_speeds port.
  void FixBeltSpeed(double speed) {
    systems::BusValue bus;
    bus.Set(belt_->scoped_name().to_string(), Value<double>(speed));
    plant_.get_surface_speeds_input_port().FixValue(context_.get(), bus);
  }

  MultibodyPlant<double> plant_{0.0};
  const RigidBody<double>* belt_{nullptr};
  const RigidBody<double>* other_{nullptr};
  std::unique_ptr<systems::Context<double>> context_;
};

// RegisterSurfaceVelocity normalizes the stored direction: (2,0,0) → (1,0,0).
TEST_F(SurfaceVelocityTest, RegisterNormalizesInput) {
  EXPECT_TRUE(CompareMatrices(
      plant_.GetSurfaceVelocityNormal(*context_, *belt_), Vector3d(1, 0, 0)));
}

// SetSurfaceVelocityNormal normalizes its input and the result is visible via
// GetSurfaceVelocityNormal.
TEST_F(SurfaceVelocityTest, SetNormalNormalizesAndPersists) {
  plant_.SetSurfaceVelocityNormal(context_.get(), *belt_, Vector3d(0, 3, 0));
  EXPECT_TRUE(CompareMatrices(
      plant_.GetSurfaceVelocityNormal(*context_, *belt_), Vector3d(0, 1, 0)));
}

TEST_F(SurfaceVelocityTest, AccessorsThrowForUnregisteredBody) {
  EXPECT_THROW(plant_.GetSurfaceVelocityNormal(*context_, *other_),
               std::exception);
  EXPECT_THROW(
      plant_.SetSurfaceVelocityNormal(context_.get(), *other_, {1, 0, 0}),
      std::exception);
}

// Unconnected port (no FixValue called) → speed treated as zero.
TEST_F(SurfaceVelocityTest, ZeroVelocityWhenPortUnconnected) {
  const Vector3d v = MultibodyPlantTester::ComputeSurfaceVelocity(
      plant_, belt_->index(), *context_, Vector3d(0, 0, 1));
  EXPECT_TRUE(CompareMatrices(v, Vector3d::Zero()));
}

// Port connected but carrying no signal for this body → zero.
TEST_F(SurfaceVelocityTest, ZeroVelocityWhenSignalAbsent) {
  plant_.get_surface_speeds_input_port().FixValue(context_.get(),
                                                  systems::BusValue{});
  const Vector3d v = MultibodyPlantTester::ComputeSurfaceVelocity(
      plant_, belt_->index(), *context_, Vector3d(0, 0, 1));
  EXPECT_TRUE(CompareMatrices(v, Vector3d::Zero()));
}

// Body never registered → zero regardless of port state.
TEST_F(SurfaceVelocityTest, ZeroVelocityForUnregisteredBody) {
  FixBeltSpeed(1.0);
  const Vector3d v = MultibodyPlantTester::ComputeSurfaceVelocity(
      plant_, other_->index(), *context_, Vector3d(0, 0, 1));
  EXPECT_TRUE(CompareMatrices(v, Vector3d::Zero()));
}

// The speed can be positive or negative, flipping the velocity direction.
// n_ss_B = (1,0,0), n_W = (0,0,1) (R_WB = I, so n_B = n_W).
// speed = s: v = s*(1,0,0)×(0,0,1) = s*(0, -1, 0) = (0, -s, 0).
TEST_F(SurfaceVelocityTest, WorksWithFiniteSpeeds) {
  for (double speed : {0.25, 0.0, -0.75}) {
    SCOPED_TRACE(fmt::format("with speed = {}", speed));
    FixBeltSpeed(speed);
    const Vector3d v = MultibodyPlantTester::ComputeSurfaceVelocity(
        plant_, belt_->index(), *context_, Vector3d(0, 0, 1));
    EXPECT_TRUE(CompareMatrices(v, Vector3d(0, -speed, 0)));
  }
}

// SetSurfaceVelocityNormal changes the velocity direction in subsequent calls.
TEST_F(SurfaceVelocityTest, SetNormalAffectsComputation) {
  constexpr double tol = 1e-12;
  FixBeltSpeed(1.0);

  // Initial n_ss_B = (1,0,0): v = (1,0,0)×(0,0,1) = (0,-1,0).
  Vector3d v = MultibodyPlantTester::ComputeSurfaceVelocity(
      plant_, belt_->index(), *context_, Vector3d(0, 0, 1));
  EXPECT_TRUE(CompareMatrices(v, Vector3d(0, -1, 0), tol));

  // After update to (0,1,0): v = (0,1,0)×(0,0,1) = (1,0,0).
  plant_.SetSurfaceVelocityNormal(context_.get(), *belt_, Vector3d(0, 1, 0));
  v = MultibodyPlantTester::ComputeSurfaceVelocity(
      plant_, belt_->index(), *context_, Vector3d(0, 0, 1));
  EXPECT_TRUE(CompareMatrices(v, Vector3d(1, 0, 0), tol));
}

TEST_F(SurfaceVelocityTest, SurfaceSpeedsPortName) {
  EXPECT_EQ(plant_.get_surface_speeds_input_port().get_name(),
            "surface_speeds");
}

// Confirm that the surface velocity follows the body pose in the world; put the
// body in an arbitrary, non-trival pose.
TEST_F(SurfaceVelocityTest, SurfaceVelocityPosedInWorld) {
  const RigidTransformd X_WB(RollPitchYawd(0.1, 0.2, 0.3),
                             Vector3d(0.5, -0.4, 0.3));
  plant_.SetFloatingBaseBodyPoseInWorldFrame(context_.get(), *belt_, X_WB);

  // For contact normal n_C_B = (0, 0, 1), the expected surface velocity,
  // v_ss_B is simply (0, -s, 0). So, given X_WB, we pass in n_C_W and should
  // get the expected v_ss_B.
  const Vector3d n_C_B(0, 0, 1);
  double speed = 0.75;
  const Vector3d v_ss_B_expected(0, -speed, 0);
  FixBeltSpeed(speed);
  const Vector3d n_C_W = X_WB.rotation() * n_C_B;

  const Vector3d v_ss_B = MultibodyPlantTester::ComputeSurfaceVelocity(
      plant_, belt_->index(), *context_, n_C_W);
  EXPECT_TRUE(CompareMatrices(v_ss_B, v_ss_B_expected, 1e-15));
}

// Generalized physics test harness.
//
// We've established that the surface velocity reports the right thing based
// on port values and body poses and the like. What remains is some positive
// indication that observed contact introduces the expected effect. This test
// is an attempt to do that in a *general* way. We want to make sure the
// contact models do the following:
//
//  a. Consider both bodies in contact for surface velocity.
//  b. Account for the bodies' poses in mapping surface velocity to the world.
//  c. Combine the surface velocity contributions in a consistent way.
//
// If it does all that, we should get appropriate forces (due to friction).
// Generally, examining the forces isn't trivial across contact models and
// time steppers. So, we'll adopt a more indirect, but universal, approach.
//
//  - Take a small time step and observe the contact forces (as reported
//    in the plant's output port). Does it have a tangential component we can
//    clearly attribute to the surface velocity?
//  - With no other source of motion, do bodies move relative to each other
//    which, again, can be clearly attributed to the surface velocity?
//
// To that end, we will place a box on a ground plane. The configuration will
// help us achieve the testing goals:
//
//  (a) Both bodies will be given surface velocities in orthogonal directions.
//  (b) Both bodies will be rotated 90° around Wz.
//  (c) Given the surface velocities, we'll predict force and movement
//      directions.
//
// Details
//
// Surface velocities *at contact* for both bodies:
//
//   ground belt : speed kGroundSpeed in +Wx.
//   box surface : speed kBoxSpeed    in +Wy.
//
// Those surface velocities arise from the following (both bodies rotated
// R_z(90°)). Contact pairs pass the outward normal to each body: +Ẑ to A
// (ground), −Ẑ to B (box):
//
//   ground (A): n_ss_B=(1,0,0): R_WB*(1,0,0)=(0,1,0); (0,1,0)×(+Ẑ)=(1,0,0)
//   box    (B): n_ss_B=(0,-1,0): R_WB*(0,-1,0)=(1,0,0); (1,0,0)×(-Ẑ)=(0,1,0)
//
// With both bodies kinematically at rest:
//
//   slip = v_ss_box - v_ss_ground:
//   slip = (0, kBoxSpeed, 0) - (kGroundSpeed, 0, 0)
//        = (-kGroundSpeed, +kBoxSpeed, 0)
//   friction on box ∝ (kGroundSpeed, -kBoxSpeed, 0)
//
// For kGroundSpeed == kBoxSpeed the direction is (1,-1,0)/√2.
//
// The suite verifies this for point and hydroelastic contact models in both
// continuous and discrete time stepping, plus a deformable-sphere variant that
// exercises the deformable contact path (5 configurations total). The contact
// force check (one step) tests the reporter; the displacement check (long
// integration) tests the dynamics independently.

struct OrthogonalContactTestConfig {
  std::string description;
  MultibodyPlantConfig plant_config;
  bool use_deformable{false};
};

class OrthogonalSurfaceVelocityTest
    : public ::testing::TestWithParam<OrthogonalContactTestConfig> {
 protected:
  static constexpr double kTheta = M_PI / 2;    // 90° rotation around world Z
  static constexpr double kHalfSize = 0.1;      // box is 0.2 m cube
  static constexpr double kPenetration = 1e-3;  // initial overlap, m
  static constexpr double kStiffness = 1e5;     // N/m, per body (point contact)
  static constexpr double kHydroModulus = 1e6;  // Pa (hydroelastic compliance)
  static constexpr double kMu = 1.0;
  static constexpr double kGroundSpeed = 1.0;  // m/s
  static constexpr double kBoxSpeed = 1.0;     // m/s

  void SetUp() override {
    const auto& config = GetParam().plant_config;

    systems::DiagramBuilder<double> builder;
    auto [plant_ref, scene_graph_ref] =
        AddMultibodyPlantSceneGraph(&builder, config.time_step);
    ApplyMultibodyPlantConfig(config, &plant_ref);
    plant_ = &plant_ref;

    ProximityProperties material;
    AddContactMaterial(0.0, kStiffness, CoulombFriction<double>(kMu, kMu),
                       &material);
    ProximityProperties rigid(material);
    AddRigidHydroelasticProperties(&rigid);
    ProximityProperties compliant(material);
    AddCompliantHydroelasticProperties(kHalfSize, kHydroModulus, &compliant);

    const RollPitchYawd Rz_90(0.0, 0.0, kTheta);
    ;

    // Ground: large Box welded to world (top face at z=0), rotated kTheta around Z.
    // n_ss_B = (1,0,0) → world surface velocity = kGroundSpeed * (+X).
    ground_ =
        &plant_->AddRigidBody("ground", SpatialInertia<double>::MakeUnitary());
    plant_->WeldFrames(plant_->world_frame(), ground_->body_frame(),
                       RigidTransformd(Rz_90, Vector3d::Zero()));
    plant_->RegisterCollisionGeometry(
        *ground_, RigidTransformd(Vector3d(0, 0, -0.5)),
        Box(10.0, 10.0, 1.0), "ground", rigid);
    plant_->RegisterSurfaceVelocity(*ground_, Vector3d(1, 0, 0));

    if (!GetParam().use_deformable) {
      // Box: free floating, 0.2 m cube.
      // n_ss_B = (0,-1,0) → world surface velocity = kBoxSpeed * (+Y).
      // (Box is body B; contact pairs pass −Ẑ to B, so the cross product flips.)
      box_ = &plant_->AddRigidBody(
          "box", SpatialInertia<double>::SolidBoxWithMass(
                     1.0, 2 * kHalfSize, 2 * kHalfSize, 2 * kHalfSize));
      plant_->RegisterCollisionGeometry(
          *box_, RigidTransformd::Identity(),
          Box(2 * kHalfSize, 2 * kHalfSize, 2 * kHalfSize), "box", compliant);
      plant_->RegisterSurfaceVelocity(*box_, Vector3d(0, -1, 0));
    } else {
      // Deformable sphere: no surface velocity; contacts the ground belt only.
      auto sphere_instance = std::make_unique<GeometryInstance>(
          RigidTransformd(Vector3d(0, 0, kHalfSize - kPenetration)),
          std::make_unique<Sphere>(kHalfSize), "deformable_sphere");
      ProximityProperties deformable_props(material);
      sphere_instance->set_proximity_properties(std::move(deformable_props));
      fem::DeformableBodyConfig<double> body_config;
      deformable_id_ =
          plant_->mutable_deformable_model().RegisterDeformableBody(
              std::move(sphere_instance), body_config, kHalfSize);
    }

    plant_->Finalize();
    auto diagram = builder.Build();
    sim_ = std::make_unique<systems::Simulator<double>>(std::move(diagram));
    auto& plant_context =
        plant_->GetMyMutableContextFromRoot(&sim_->get_mutable_context());

    // Place box rotated kTheta around Z with bottom face at z = -kPenetration.
    if (!GetParam().use_deformable) {
      plant_->SetFloatingBaseBodyPoseInWorldFrame(
          &plant_context, *box_,
          RigidTransformd(Rz_90, Vector3d(0, 0, kHalfSize - kPenetration)));
    }

    systems::BusValue bus;
    bus.Set(ground_->scoped_name().to_string(), Value<double>(kGroundSpeed));
    if (!GetParam().use_deformable) {
      bus.Set(box_->scoped_name().to_string(), Value<double>(kBoxSpeed));
    }
    plant_->get_surface_speeds_input_port().FixValue(&plant_context, bus);
    sim_->Initialize();
  }

  // Returns the total contact force on the contacting body (rigid box or
  // deformable sphere) from ContactResults.
  Vector3d ContactForceOnContactingBody(
      const systems::Context<double>& plant_context) const {
    const auto& results =
        plant_->get_contact_results_output_port().Eval<ContactResults<double>>(
            plant_context);

    Vector3d f = Vector3d::Zero();

    if (box_ != nullptr) {
      // Point contacts: contact_force() is the force on body B.
      for (int i = 0; i < results.num_point_pair_contacts(); ++i) {
        const auto& info = results.point_pair_contact_info(i);
        if (info.bodyB_index() == box_->index()) {
          f += info.contact_force();
        } else if (info.bodyA_index() == box_->index()) {
          f -= info.contact_force();
        }
      }

      // Hydroelastic contacts: F_Ac_W() is the force on body A (id_M geometry).
      const auto& box_geom_ids = plant_->GetCollisionGeometriesForBody(*box_);
      for (int i = 0; i < results.num_hydroelastic_contacts(); ++i) {
        const auto& hinfo = results.hydroelastic_contact_info(i);
        const GeometryId id_m = hinfo.contact_surface().id_M();
        const bool box_is_body_m =
            std::find(box_geom_ids.begin(), box_geom_ids.end(), id_m) !=
            box_geom_ids.end();
        if (box_is_body_m) {
          f += hinfo.F_Ac_W().translational();
        } else {
          f -= hinfo.F_Ac_W().translational();
        }
      }
    } else {
      // Deformable contacts: F_Ac_W() is the force on the deformable body A.
      for (int i = 0; i < results.num_deformable_contacts(); ++i) {
        f += results.deformable_contact_info(i).F_Ac_W().translational();
      }
    }

    return f;
  }

  // Returns the centroid of the contacting body (rigid box or deformable sphere).
  Vector3d ContactingBodyCentroid(
      const systems::Context<double>& plant_context) const {
    if (box_ != nullptr) {
      return plant_->EvalBodyPoseInWorld(plant_context, *box_).translation();
    }
    return plant_->deformable_model()
        .GetPositions(plant_context, deformable_id_.value())
        .rowwise()
        .mean();
  }

  MultibodyPlant<double>* plant_{nullptr};
  const RigidBody<double>* ground_{nullptr};
  const RigidBody<double>* box_{nullptr};
  std::optional<DeformableBodyId> deformable_id_;
  std::unique_ptr<systems::Simulator<double>> sim_;
};

// After one contact step, the tangential force on the box should lie in the
// (+X, -Y) direction: +X from the ground belt, -Y from the box's own surface.
// With kGroundSpeed == kBoxSpeed the two components are equal in magnitude.
TEST_P(OrthogonalSurfaceVelocityTest, ContactForceTangentialDirection) {
  const double time_step = GetParam().plant_config.time_step;
  // Discrete: advance one step to populate DiscreteStepMemory.
  // Continuous: contact results are available on demand at t = 0.
  if (time_step > 0.0) sim_->AdvanceTo(time_step);
  const Vector3d f =
      ContactForceOnContactingBody(
          plant_->GetMyContextFromRoot(sim_->get_context()));

  EXPECT_GT(f.z(), 0.0);  // normal force pushes body up
  EXPECT_GT(f.x(), 0.0)
      << "ground belt (kGroundSpeed in +X) should push body in +X";
  if (!GetParam().use_deformable) {
    EXPECT_LT(f.y(), 0.0) << "box surface (kBoxSpeed in +Y) should react in -Y";
    // Equal speeds → equal-magnitude tangential components, within 10%.
    EXPECT_NEAR(f.x(), -f.y(), 0.1 * f.x());
  }
}

// After integrating for 0.3 s the box should have displaced in (+X, -Y). This
// tests the dynamics path independently: if surface velocity only entered the
// contact-results reporter and not the constraint/force computation, the box
// would stay at its initial position.
TEST_P(OrthogonalSurfaceVelocityTest, BoxDisplacementDirection) {
  sim_->AdvanceTo(0.3);
  const auto& final_ctx = plant_->GetMyContextFromRoot(sim_->get_context());
  const Vector3d p_WBody = ContactingBodyCentroid(final_ctx);

  constexpr double kMinDisplacement = 0.01;  // 1 cm
  EXPECT_GT(p_WBody.x(), kMinDisplacement)
      << "body should have moved in +X due to ground belt";
  if (!GetParam().use_deformable) {
    EXPECT_LT(p_WBody.y(), -kMinDisplacement)
        << "box should have moved in -Y due to box surface velocity";
  }
}

INSTANTIATE_TEST_SUITE_P(
    AllContactRegimes, OrthogonalSurfaceVelocityTest, testing::ValuesIn([] {
      MultibodyPlantConfig continuous_point;
      continuous_point.time_step = 0.0;
      continuous_point.contact_model = "point";

      MultibodyPlantConfig discrete_point;
      discrete_point.time_step = 1e-3;
      discrete_point.contact_model = "point";
      discrete_point.discrete_contact_approximation = "sap";

      MultibodyPlantConfig continuous_hydro;
      continuous_hydro.time_step = 0.0;
      continuous_hydro.contact_model = "hydroelastic_with_fallback";

      MultibodyPlantConfig discrete_hydro;
      discrete_hydro.time_step = 1e-3;
      discrete_hydro.contact_model = "hydroelastic_with_fallback";
      discrete_hydro.discrete_contact_approximation = "sap";

      return std::vector<OrthogonalContactTestConfig>{
          {"continuous_point", continuous_point},
          {"discrete_point_sap", discrete_point},
          {"continuous_hydro", continuous_hydro},
          {"discrete_hydro_sap", discrete_hydro},
          {"discrete_deformable_sap", discrete_hydro, true},
      };
    }()),
    [](const testing::TestParamInfo<OrthogonalContactTestConfig>& param_info) {
      return param_info.param.description;
    });

}  // namespace
}  // namespace multibody
}  // namespace drake
