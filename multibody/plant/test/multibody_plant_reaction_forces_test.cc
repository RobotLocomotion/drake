#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/proximity/deformable_contact_geometries.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/contact_solvers/sap/sap_solver.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/quaternion_floating_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

using drake::geometry::Box;
using drake::geometry::GeometryInstance;
using drake::geometry::SceneGraph;
using drake::geometry::SceneGraphInspector;
using drake::geometry::Sphere;
using drake::geometry::VolumeMesh;
using drake::geometry::internal::HydroelasticType;
using drake::geometry::internal::kComplianceType;
using drake::geometry::internal::kElastic;
using drake::geometry::internal::kHcDissipation;
using drake::geometry::internal::kHydroGroup;
using drake::geometry::internal::kMaterialGroup;
using drake::geometry::internal::kRezHint;
using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::multibody::ConnectContactResultsToDrakeVisualizer;
using drake::multibody::DeformableBodyId;
using drake::multibody::Parser;
using drake::multibody::RevoluteJoint;
using drake::multibody::contact_solvers::internal::SapSolverParameters;
using drake::multibody::internal::CompliantContactManager;
using drake::systems::Context;
using drake::systems::Diagram;
using drake::systems::Simulator;
using Eigen::Vector3d;

namespace drake {
namespace multibody {

class MultibodyPlantTester {
 public:
  // Returns the manager for the given plant.
  // @pre The plant must be discrete time and already finalized.
  static CompliantContactManager<double>& manager(
      const MultibodyPlant<double>& plant) {
    auto* manager = dynamic_cast<CompliantContactManager<double>*>(
        plant.discrete_update_manager_.get());
    DRAKE_DEMAND(manager != nullptr);
    return *manager;
  }
};

namespace {

struct LadderTestConfig {
  // This is a gtest test suffix; no underscores or spaces.
  std::string description;
  double time_step{0.0};
  double simulation_time{5.0};
  // Contact is modeled with hydroelastic contact if `true` or with
  // point contact if `false`.
  bool hydro_geometry{true};
  // Discrete solver used in the update.
  DiscreteContactSolver contact_solver{DiscreteContactSolver::kTamsi};
  // The ladder is split into two pieces. We join them together using different
  // methods to verify the correctness of reaction forces on a variety of
  // constrained configurations.
  enum class WeldMethod {
    kWeldJoint,
    kRevoluteJointWithLimits,  // Revolute joint with lower limit.
    // TODO(amcastro-tri): consider a weld constraint case.
  } weld_method{WeldMethod::kWeldJoint};
};

// This provides the suffix for each test parameter: the test config
// description.
std::ostream& operator<<(std::ostream& out, const LadderTestConfig& c) {
  out << c.description;
  return out;
}

// This test simulates a "ladder" leaning against a wall, under the action of
// gravity pulling in the -z axis direction. The bottom of the ladder is pinned
// to the ground by a revolute joint with its axis of revolution aligned with
// the y axis. We apply an external torque to this joint.
// Please run this unit test along with Drake's visualizer to obtain a
// visualization of the setup.
// We run a simulation to reach the steady state in which contact forces balance
// the action of gravity and external actuation. This problem essentially is a
// two dimensional problem in the x-z plane, with the joint axis into this
// plane. We define the length kProblemWidth to be the width into the
// x-z plane along the y axis. However, we make the problem of computing
// reaction forces a bit more interesting by defining a contact geometry that
// is not symmetric along the y-axis leading to an additional reaction torque
// along the z axis. We emulate a single point of contact between the ladder and
// the wall by placing a sphere on the top corner (y positive, z positive in the
// body frame) of the ladder. Therefore, since the point of contact is not at
// y = 0 but offset to the size at y = kProblemWidth / 4, there is an
// additional reaction torque along the z axis.
// In addition, we split the ladder in two and join them together. How the two
// pieces are joined is dictated by LadderTestConfig.  This allow us
// to test the computation of reaction forces at different joint types with and
// without constraints.
//
// Summarizing, this problem setup computes the (static) reaction force at the
// bottom pin joint holding the ladder to the ground, in the presence of contact
// and actuation.
//
// Passing true as the value of the `hydro_geometry` argument sets up the above
// problem with hydroelastic contact. The point contact sphere geometry is
// replaced by a compliant hydroelastic sphere and the wall's box geometry is
// replaced by a rigid hydroelastic box. Verification is done using hydroelastic
// contact results.
//
// We perform this test with point contact for both continuous and discrete
// models.
//
class LadderTest : public ::testing::TestWithParam<LadderTestConfig> {
 protected:
  void BuildLadderModel() {
    const LadderTestConfig& config = GetParam();
    systems::DiagramBuilder<double> builder;
    std::tie(plant_, scene_graph_) = AddMultibodyPlantSceneGraph(
        &builder, std::make_unique<MultibodyPlant<double>>(config.time_step));

    AddWall(config.hydro_geometry);
    AddPinnedLadder(config);
    plant_->mutable_gravity_field().set_gravity_vector(
        Vector3d(0.0, 0.0, -kGravity));

    if (plant_->is_discrete()) {
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
    }

    plant_->Finalize();

    if (plant_->is_discrete() &&
        config.contact_solver == DiscreteContactSolver::kSap) {
      // When using the SAP solver, the solver convergence tolerance must be set
      // accordingly to the level of high accuracy used in these tests, dictated
      // by the fixture's parameter kTolerance.
      auto& manager = MultibodyPlantTester::manager(*plant_);
      SapSolverParameters sap_parameters;
      sap_parameters.rel_tolerance = kTolerance / 100;
      manager.set_sap_solver_parameters(sap_parameters);
    }

    // Add visualization for verification of the results when we have the
    // visualizer running.
    geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph_, &lcm_);
    ConnectContactResultsToDrakeVisualizer(&builder, *plant_, *scene_graph_,
                                           &lcm_);

    diagram_ = builder.Build();
  }

  // Adds the model for a wall anchored to the wall.
  void AddWall(bool hydro_geometry = false) {
    const Vector3d size(kWallWidth, kProblemWidth, kWallHeight);
    const RigidTransformd X_WB = Eigen::Translation3d(
        kDistanceToWall + kWallWidth / 2.0, 0.0, kWallHeight / 2.0);
    const Vector4<double> green(0.5, 1.0, 0.5, 1.0);
    const auto shape = geometry::Box(size(0), size(1), size(2));
    plant_->RegisterVisualGeometry(plant_->world_body(), X_WB, shape,
                                   "wall_visual", green);

    geometry::ProximityProperties properties;
    properties.AddProperty(
        geometry::internal::kMaterialGroup, geometry::internal::kFriction,
        CoulombFriction<double>(kFrictionCoefficient, kFrictionCoefficient));

    if (hydro_geometry) {
      properties.AddProperty(kHydroGroup, kComplianceType,
                             HydroelasticType::kRigid);
      properties.AddProperty(kHydroGroup, kRezHint, kProblemWidth);
      properties.AddProperty(kMaterialGroup, kHcDissipation, kDissipation);
    }

    plant_->RegisterCollisionGeometry(plant_->world_body(), X_WB, shape,
                                      "wall_collision", properties);
  }

  // Adds the model for the ladder pinned to the ground at the origin.
  void AddPinnedLadder(const LadderTestConfig& config) {
    // We split the ladder into two halves and join them with a weld joint so
    // we can evaluate the reaction force right at the middle.
    // We define body frame Bl and Bu for the lower and upper portions of the
    // ladder respectively. Both of these frame origins are located at the lower
    // end of each half. In particular, the lower frame Bl attaches to the
    // ground with the pin joint.
    const double half_ladder_length = kLadderLength / 2.0;
    const double half_ladder_mass = kLadderMass / 2.0;
    const SpatialInertia<double> M_BBo_B =
        SpatialInertia<double>::ThinRodWithMassAboutEnd(
            half_ladder_mass, half_ladder_length, Vector3d::UnitZ());

    // Create a rigid body for the ladder.
    ladder_lower_ = &plant_->AddRigidBody("ladder_lower", M_BBo_B);
    ladder_upper_ = &plant_->AddRigidBody("ladder_upper", M_BBo_B);

    // Both lower and upper sections have a box geometry for visualization.
    auto shape = Box(kLadderWidth, kProblemWidth, kLadderLength / 2.0);
    // We want the side of the box to rest between the pin joint and the contact
    // point at the wall. Therefore we place the geometry frame origin Go with a
    // shift -kLadderWidth / 2.0 in the body frame's x-axis.
    const RigidTransformd X_BV(
        Vector3d(-kLadderWidth / 2.0, 0.0, kLadderLength / 4.0));
    const Vector4<double> light_blue(0.5, 0.8, 1.0, 1.0);
    const Vector4<double> dark_blue(0.0, 0.0, 0.8, 1.0);
    plant_->RegisterVisualGeometry(*ladder_lower_, X_BV, shape,
                                   "LadderLowerVisualGeometry", light_blue);
    plant_->RegisterVisualGeometry(*ladder_upper_, X_BV, shape,
                                   "LadderUpperVisualGeometry", dark_blue);
    // We'll add a sphere to emulate a single point of contact against the wall.
    // We place it at the y+ corner of the ladder geometry so that the contact
    // force causes a non-zero torque at the pin joint for a more interesting
    // case.
    const RigidTransformd X_BC(
        Vector3d(0.0, kProblemWidth / 4.0, kLadderLength / 2.0));

    geometry::ProximityProperties properties;
    properties.AddProperty(
        geometry::internal::kMaterialGroup, geometry::internal::kFriction,
        CoulombFriction<double>(kFrictionCoefficient, kFrictionCoefficient));

    if (config.hydro_geometry) {
      properties.AddProperty(kHydroGroup, kComplianceType,
                             HydroelasticType::kSoft);
      properties.AddProperty(kHydroGroup, kRezHint, kPointContactRadius);
      properties.AddProperty(kHydroGroup, kElastic, kElasticModulus);
      properties.AddProperty(kMaterialGroup, kHcDissipation, kDissipation);
    }

    ladder_upper_geometry_id_ = plant_->RegisterCollisionGeometry(
        *ladder_upper_, X_BC, Sphere(kPointContactRadius),
        "LadderUpperCollisionGeometry", properties);
    plant_->RegisterVisualGeometry(*ladder_upper_, X_BC,
                                   Sphere(kPointContactRadius),
                                   "LadderUpperCollisionVisualGeometry",
                                   Vector4<double>(0.0, 0.0, 0.8, 0.3));

    // Pin to the floor with a revolute joint.
    pin_ = &plant_->AddJoint<RevoluteJoint>("PinToGround", plant_->world_body(),
                                            {}, *ladder_lower_, {},
                                            Vector3d::UnitY(), kPinDamping);

    // Join the two halves using a method specified in the LadderTestConfig.
    const RigidTransformd X_BlBu(Vector3d(0.0, 0.0, kLadderLength / 2.0));
    switch (config.weld_method) {
      case LadderTestConfig::WeldMethod::kWeldJoint: {
        joint_ = &plant_->WeldFrames(ladder_lower_->body_frame(),
                                     ladder_upper_->body_frame(), X_BlBu);
        break;
      }
      case LadderTestConfig::WeldMethod::kRevoluteJointWithLimits: {
        const double kInf = std::numeric_limits<double>::infinity();
        joint_ = &plant_->AddJoint<RevoluteJoint>("MiddleJoint", *ladder_lower_,
                                                  X_BlBu, *ladder_upper_, {},
                                                  Vector3d::UnitY(), 0, kInf);
        break;
      }
    }

    // Add actuation.
    plant_->AddJointActuator("PinActuator", *pin_);
  }

  // Build and run a simulator, and return it after simulating.
  std::unique_ptr<Simulator<double>> Simulate(
      std::unique_ptr<Context<double>> diagram_context) {
    Context<double>* plant_context =
        &diagram_->GetMutableSubsystemContext(*plant_, diagram_context.get());

    // Set initial condition with the ladder leaning against the wall.
    // We compute the angle in the pin joint for this condition.
    // The contact sphere of radius `kPointContactRadius` is placed at the
    // upper end of the ladder.
    // We want the contact sphere to initially be tangent to the wall
    // so we compute the necessary offset using similar triangles.
    // We define the following in the xz plane:
    //   O: origin and placement point of the pin joint at the lower end.
    //   A: Bottom of the wall offset from O by (+kDistanceToWall, 0).
    //   E: Upper endpoint of the ladder |E - O| = kLadderLength.
    //   R: Point at which the contact sphere is tangent to the wall.
    //      |R - E| = kPointContactRadius
    //   U: Projection point along the line OE touching the wall.
    //
    // In this configuration there are two similar right triangles:
    //   ΔAOU and ΔREU
    // Therefore the following ratios are equal:
    //   |EU| / |RE| = |OU| / |AO| = (|OE| + |EU|) / |AO|
    // Substituting known quantities:
    //   |EU| / kPointContactRadius == (kLadderLength + |EU|) / kDistanceToWall
    // Solving for |EU| gives:
    //   |EU| =   (kLadderLength * kPointContactRadius)
    //          / (kDistanceToWall - kPointContactRadius)
    const double norm_EU = (kLadderLength * kPointContactRadius /
                            (kDistanceToWall - kPointContactRadius));
    const double theta = std::asin(kDistanceToWall / (kLadderLength + norm_EU));
    pin_->set_angle(plant_context, theta);

    // Fix the actuation.
    const Vector1d tau_actuation = kActuationTorque * Vector1d::Ones();
    plant_->get_actuation_input_port().FixValue(plant_context, tau_actuation);

    // Sanity check model size.
    auto sanity_check = [this]() {
      const LadderTestConfig& config = GetParam();
      ASSERT_EQ(plant_->num_bodies(), 3);
      if (config.weld_method == LadderTestConfig::WeldMethod::kWeldJoint) {
        ASSERT_EQ(plant_->num_velocities(), 1);
      } else {
        ASSERT_EQ(plant_->num_velocities(), 2);
      }
      ASSERT_EQ(plant_->num_actuated_dofs(), 1);
    };
    sanity_check();

    // We run a simulation to steady state so that contact forces balance
    // gravity and actuation.
    const LadderTestConfig& config = GetParam();
    auto simulator = std::make_unique<Simulator<double>>(
        *diagram_, std::move(diagram_context));
    // The default RK3 integrator requires specifying a very high accuracy to
    // reach steady state within kTolerance and therefore it is very costly.
    // However implicit Euler does a much better job with larger time steps.
    simulator->reset_integrator<systems::ImplicitEulerIntegrator<double>>();
    simulator->get_mutable_integrator().set_maximum_step_size(5e-3);
    simulator->get_mutable_integrator().set_target_accuracy(1e-6);
    simulator->Initialize();
    simulator->AdvanceTo(config.simulation_time);
    return simulator;
  }

  void VerifyJointReactionForces(Context<double>* diagram_context,
                                 bool hydro_geometry = false) {
    Context<double>* plant_context =
        &diagram_->GetMutableSubsystemContext(*plant_, diagram_context);
    // Evaluate the reaction forces output port to get the reaction force at the
    // pin joint. Re-express in the world frame W.
    const auto& reaction_forces =
        plant_->get_reaction_forces_output_port()
            .Eval<std::vector<SpatialForce<double>>>(*plant_context);
    ASSERT_EQ(reaction_forces.size(), 2u);
    const SpatialForce<double>& F_Bl_Bl = reaction_forces[pin_->ordinal()];
    const RigidTransformd X_WBl =
        ladder_lower_->EvalPoseInWorld(*plant_context);
    const SpatialForce<double> F_Bl_W = X_WBl.rotation() * F_Bl_Bl;

    // We evaluate the contact forces so that we can perform the balance of
    // forces by hand and compare with the results obtained by evaluating the
    // reaction forces port.
    const ContactResults<double>& contact_results =
        plant_->get_contact_results_output_port().Eval<ContactResults<double>>(
            *plant_context);

    // Contact force on the ladder at the contact point, or center of pressure
    // P, expressed in the world frame.
    Vector3d f_Bp_W(0, 0, 0);
    // The contact point.
    Vector3d p_WP(0, 0, 0);
    if (hydro_geometry) {
      // There should be a single contact surface.
      ASSERT_EQ(contact_results.num_hydroelastic_contacts(), 1);
      const HydroelasticContactInfo<double>& hydroelastic_contact_info =
          contact_results.hydroelastic_contact_info(0);
      const double direction =
          hydroelastic_contact_info.contact_surface().id_M() ==
                  ladder_upper_geometry_id_
              ? 1.0
              : -1.0;
      // There is a slight non-zero moment on the y-axis because the centroid in
      // general does not coincide with the center of pressure, P. Shift the
      // contact force back to a point with zero torque for easier analysis.

      // Spatial force on the ladder, applied at the centroid of the contact
      // patch, expressed in the world frame.
      const SpatialForce<double>& F_Bc_W = hydroelastic_contact_info.F_Ac_W();
      const Vector3d p_CP_W(
          0.0, 0.0, F_Bc_W.rotational().y() / F_Bc_W.translational().x());

      f_Bp_W = direction * F_Bc_W.Shift(p_CP_W).translational();
      p_WP = hydroelastic_contact_info.contact_surface().centroid() + p_CP_W;

    } else {
      // There should be a single contact pair.
      ASSERT_EQ(contact_results.num_point_pair_contacts(), 1);
      const PointPairContactInfo<double>& point_pair_contact_info =
          contact_results.point_pair_contact_info(0);

      const double direction =
          point_pair_contact_info.bodyB_index() == ladder_upper_->index()
              ? 1.0
              : -1.0;

      f_Bp_W = direction * point_pair_contact_info.contact_force();
      p_WP = point_pair_contact_info.contact_point();
    }

    // Ladder's weight.
    const double weight = kGravity * kLadderMass;

    // Position of the ladder's center of gravity.
    const Vector3d p_WBcm =
        plant_->CalcCenterOfMassPositionInWorld(*plant_context);

    // Using a free-body diagram of the entire ladder and known quantities,
    // we use the balance of moments to verify the pin joint's reaction force
    // and torque.

    // The x component of the contact force must counteract the torque due to
    // gravity plus the actuation torque.
    const double tau_g = p_WBcm.x() * weight;  // gravity torque about Bo.
    const double fc_x = (tau_g + kActuationTorque) / p_WP.z();
    const Vector3d f_Bl_W_expected(fc_x, 0.0, weight);
    EXPECT_TRUE(CompareMatrices(F_Bl_W.translational(), f_Bl_W_expected,
                                kTolerance, MatrixCompareType::relative));

    // Expected contact force.
    const Vector3d f_C_W_expected(-fc_x, 0.0, 0.0);
    EXPECT_TRUE(CompareMatrices(f_Bp_W, f_C_W_expected, kTolerance,
                                MatrixCompareType::relative));

    // Since the contact point was purposely located at
    // y = (kProblemWidth / 2.0) - kPointContactRadius, the contact force
    // causes a reaction torque at the pin joint oriented along the z-axis.
    const Vector3d t_Bl_W_expected(0.0, kActuationTorque, -fc_x * p_WP.y());
    EXPECT_TRUE(CompareMatrices(F_Bl_W.rotational(), t_Bl_W_expected,
                                kTolerance, MatrixCompareType::relative));

    // Verify reaction forces at the weld joint. We use a free body diagram of
    // the upper half of the ladder and balance of moments at the upper half's
    // body frame.

    // Upper half's origin.
    const RigidTransformd X_WBu =
        ladder_upper_->EvalPoseInWorld(*plant_context);
    const Vector3d p_WBu = X_WBu.translation();
    // Upper half's COM in W.
    const Vector3d p_BuBucm =
        ladder_upper_->CalcCenterOfMassInBodyFrame(*plant_context);
    const Vector3d p_WBucm = X_WBu * p_BuBucm;
    // Reaction forces at X_WBu in W.
    const SpatialForce<double>& F_Bu_W =
        X_WBu.rotation() * reaction_forces[joint_->ordinal()];
    // Apart from reaction forces, two forces act on the upper half:
    // Contact force fc_x and gravity.
    const Vector3d f_Bu_expected(fc_x, 0.0, weight / 2.0);
    // Compute the y component of the expected torque due to gravity applied at
    // Bcm and torque due to the contact force applied at P.
    const double t_Bu_y = -(weight / 2.0) * (p_WBucm.x() - p_WBu.x()) +
                          fc_x * (p_WP.z() - p_WBu.z());
    // Contact point offset causes a torque at the weld joint oriented along
    // the z-axis.
    const Vector3d t_Bu_expected(0.0, t_Bu_y, -fc_x * (p_WP.y() - p_WBu.y()));
    EXPECT_TRUE(CompareMatrices(F_Bu_W.rotational(), t_Bu_expected, kTolerance,
                                MatrixCompareType::relative));
    EXPECT_TRUE(CompareMatrices(F_Bu_W.translational(), f_Bu_expected,
                                kTolerance, MatrixCompareType::relative));
  }

  void TestWithThreads() {
    const LadderTestConfig& config = GetParam();
    SCOPED_TRACE(fmt::format("time_step = {}", config.time_step));
    BuildLadderModel();
    ASSERT_EQ(plant_->is_discrete(), (config.time_step != 0.));

    // Create the threads' contexts by cloning a prototype. This will help
    // ensure the context deep copy is properly working.
    auto context_prototype = diagram_->CreateDefaultContext();
    auto simulator_prototype = Simulate(std::move(context_prototype));
    VerifyJointReactionForces(&simulator_prototype->get_mutable_context(),
                              config.hydro_geometry);

    // TODO(#17720): As articulated in the issue, baking a query object when
    // cloning scene graph context is thread-unsafe. In particular, updating the
    // proxy cache entry for pose/configuration (instead of simply grabbing the
    // up-to-date cache value) is a race condition. Therefore, we call
    // `ComputeDeformableContact` below to explicitly bring the
    // pose/configuration proxy cache entries up-to-date to circumvent the race
    // condition when we clone the contexts over multiple threads later on. This
    // is a hack by taking advantage of the side effects of the caching
    // mechanism in SceneGraph and should be removed when the referenced issue
    // is fixed.
    const Context<double>& sg_context = diagram_->GetSubsystemContext(
        *scene_graph_, simulator_prototype->get_context());
    const auto& query_object =
        scene_graph_->get_query_output_port()
            .Eval<geometry::QueryObject<double>>(sg_context);
    geometry::internal::DeformableContact<double> deformable_contact;
    query_object.ComputeDeformableContact(&deformable_contact);

    // Running the simulation in multiple threads here gives us a chance to
    // check readiness for context-per-thread usage. Even though all threads
    // are doing the same thing, ThreadSanitizer will be able to detect
    // potential data races.
    static constexpr int kThreads = 2;
    std::vector<std::thread> threads;
    for (int k = 0; k < kThreads; k++) {
      threads.push_back(std::thread([this, &simulator_prototype]() {
        auto context = simulator_prototype->get_context().Clone();
        Simulate(std::move(context));
        // We skip verifying forces here because system evolution
        // invalidates the expected values used above.
      }));
    }
    for (auto& thread : threads) {
      thread.join();
    }
  }

  // This problem essentially is two-dimensional.
  // This is the length in the direction normal to the x-z plane, along the
  // y-axis.
  const double kProblemWidth{1.0};  // [m]

  // Ladder parameters.
  const double kLadderLength{2.0};           // [m]
  const double kLadderMass{7.0};             // [kg]
  const double kLadderWidth{0.15};           // [m]
  const double kFrictionCoefficient{0.0};    // Frictionless contact, [-]
  const double kPointContactRadius{5.0e-3};  // [m]

  // Wall parameters.
  const double kWallWidth{0.3};   // [m]
  const double kWallHeight{3.0};  // [m]

  // Hydroelastic parameters.
  const double kElasticModulus{5e6};  // [Pa]
  const double kDissipation{100.0};   // [s/m]

  // Pin joint parameters.
  const double kDistanceToWall{1.0};
  const double kPinDamping{0.0};        // [N⋅m⋅s]
  const double kActuationTorque{20.0};  // [N⋅m]

  // We round off gravity for simpler numbers.
  const double kGravity{10.0};  // [m/s²]

  // Integration accuracy used for these tests.
  const double kIntegratorTargetAccuracy{1.0e-6};

  // We validate the numerical results to be within this tolerance value, which
  // is chosen consistently with the time the system is left to reach steady
  // state and the (continuous) integration accuracy (for the continuous model).
  const double kTolerance{1.0e-9};

  lcm::DrakeLcm lcm_;  // For visualization.
  MultibodyPlant<double>* plant_{nullptr};
  SceneGraph<double>* scene_graph_{nullptr};
  const RigidBody<double>* ladder_lower_{nullptr};
  const RigidBody<double>* ladder_upper_{nullptr};
  geometry::GeometryId ladder_upper_geometry_id_;
  const RevoluteJoint<double>* pin_{nullptr};

  // Weld joint joining the two halves of the ladder.
  const Joint<double>* joint_{nullptr};

  std::unique_ptr<Diagram<double>> diagram_;
};

TEST_P(LadderTest, TestWithThreads) {
  TestWithThreads();
}

// Set up test cases using point and hydroelastic contact.
std::vector<LadderTestConfig> MakeTestCases() {
  return std::vector<LadderTestConfig>{
      // Continuous integration tests.
      {.description = "ContinuousPoint",
       .time_step = 0.0,
       .hydro_geometry = false},
      {.description = "ContinuousHydroelastic",
       .time_step = 0.0,
       .hydro_geometry = true},

      // Discrete TAMSI solver tests.
      {.description = "WeldJointDiscretePointTamsi",
       .time_step = 2.0e-2,
       .hydro_geometry = false,
       .contact_solver = DiscreteContactSolver::kTamsi,
       .weld_method = LadderTestConfig::WeldMethod::kWeldJoint},
      {.description = "RevoluteJointWithLimitsDiscretePointTamsi",
       .time_step = 1.0e-2,  // N.B. TAMSI goes unstable if using a larger step.
       .hydro_geometry = false,
       .contact_solver = DiscreteContactSolver::kTamsi,
       .weld_method = LadderTestConfig::WeldMethod::kRevoluteJointWithLimits},
      {.description = "WeldJointDiscreteHydroelasticTamsi",
       .time_step = 2.0e-2,
       .hydro_geometry = true,
       .contact_solver = DiscreteContactSolver::kTamsi,
       .weld_method = LadderTestConfig::WeldMethod::kWeldJoint},
      {.description = "RevoluteJointWithLimitsDiscreteHydroelasticTamsi",
       .time_step = 1.0e-2,  // N.B. TAMSI goes unstable if using a larger step.
       .hydro_geometry = true,
       .contact_solver = DiscreteContactSolver::kTamsi,
       .weld_method = LadderTestConfig::WeldMethod::kRevoluteJointWithLimits},

      // Discrete SAP solver tests.
      {.description = "WeldJointDiscretePointSap",
       .time_step = 2.0e-2,
       .hydro_geometry = false,
       .contact_solver = DiscreteContactSolver::kSap,
       .weld_method = LadderTestConfig::WeldMethod::kWeldJoint},
      {.description = "RevoluteJointWithLimitsDiscretePointSap",
       .time_step = 2.0e-2,
       .hydro_geometry = false,
       .contact_solver = DiscreteContactSolver::kSap,
       .weld_method = LadderTestConfig::WeldMethod::kRevoluteJointWithLimits},
      {.description = "WeldJointDiscreteHydroelasticSap",
       .time_step = 2.0e-2,
       .hydro_geometry = true,
       .contact_solver = DiscreteContactSolver::kSap,
       .weld_method = LadderTestConfig::WeldMethod::kWeldJoint},
      {.description = "RevoluteJointWithLimitsDiscreteHydroelasticSap",
       .time_step = 2.0e-2,
       .hydro_geometry = true,
       .contact_solver = DiscreteContactSolver::kSap,
       .weld_method = LadderTestConfig::WeldMethod::kRevoluteJointWithLimits},
  };
}

INSTANTIATE_TEST_SUITE_P(ReactionForcesTests, LadderTest,
                         testing::ValuesIn(MakeTestCases()),
                         testing::PrintToStringParamName());

// This test verifies the computation of joint reaction forces for a case in
// which centrifugal terms cannot be neglected.
// The setup consists of a rod subject to spin about the world's z axis around
// a pin joint. Gravity is aligned with the world's z axis, perpendicular to the
// plane of rotation.
// We thus compute the reaction forces and verify that both centrifugal and
// gravitational terms are correct.
class SpinningRodTest : public ::testing::Test {
 protected:
  void BuildModel(double discrete_update_period) {
    plant_ = std::make_unique<MultibodyPlant<double>>(discrete_update_period);
    plant_->SetUseSampledOutputPorts(false);  // We're not stepping time.

    // We define rod B's origin Bo to be located at the rod's center of mass.
    const SpatialInertia<double> M_BBo_B =
        SpatialInertia<double>::ThinRodWithMass(kMass, kLength,
                                                Vector3d::UnitZ());
    rod_ = &plant_->AddRigidBody("rod", M_BBo_B);

    // Notice that axis Bz is aligned with the rod. We want to define frame Jb
    // on body B at the attachment point to have its z axis aligned with the
    // world's z axis. Therefore we must rotate it 90 degrees about the B's x
    // axis.
    const RigidTransformd X_BJb(RollPitchYawd(M_PI_2, 0.0, 0.0),
                                Vector3d(0.0, 0.0, -kLength / 2.0));
    pin_ = &plant_->AddJoint<RevoluteJoint>("pin", plant_->world_body(), {},
                                            *rod_, X_BJb, Vector3d::UnitZ());

    plant_->mutable_gravity_field().set_gravity_vector(
        Vector3d(0.0, 0.0, -kGravity));

    // Done defining the model.
    plant_->Finalize();

    // Create and set context.
    context_ = plant_->CreateDefaultContext();
    pin_->set_angle(context_.get(), M_PI / 4.0);
    pin_->set_angular_rate(context_.get(), kOmega);
  }

  void VerifyJointReactionForces() {
    const double kTolerance = 40 * std::numeric_limits<double>::epsilon();

    // Evaluate the spatial acceleration of the rod.
    const auto& A_WB_all =
        plant_->get_body_spatial_accelerations_output_port()
            .Eval<std::vector<SpatialAcceleration<double>>>(*context_);
    const SpatialAcceleration<double>& A_WRod = A_WB_all[rod_->index()];

    // Bz is the unit vector along the rod's length.
    const Vector3d Bz_W =
        rod_->EvalPoseInWorld(*context_).rotation().matrix().col(2);

    // There is no damping, therefore we expect the angular acceleration to be
    // zero.
    const Vector3d alpha_WB = Vector3d::Zero();

    // And the translational acceleration to contain the centrifugal
    // acceleration.
    const Vector3d a_WB = -Bz_W * kLength / 2.0 * kOmega * kOmega;

    // Verify the result.
    EXPECT_TRUE(CompareMatrices(A_WRod.rotational(), alpha_WB, kTolerance));
    EXPECT_TRUE(CompareMatrices(A_WRod.translational(), a_WB, kTolerance));

    // Evaluate reaction force at the pin.
    const auto& reaction_forces =
        plant_->get_reaction_forces_output_port()
            .Eval<std::vector<SpatialForce<double>>>(*context_);
    ASSERT_EQ(reaction_forces.size(), 1u);
    const SpatialForce<double>& F_BJb_Jb = reaction_forces[pin_->ordinal()];

    // Verify that the value of the reaction force includes the centripetal
    // component (along Jb's y axis) and the weight component (along Jb's z
    // axis).
    const Vector3d f_BJb_Jb_expected(
        0.0, -kMass * kLength / 2.0 * kOmega * kOmega, kMass * kGravity);
    EXPECT_TRUE(CompareMatrices(F_BJb_Jb.translational(), f_BJb_Jb_expected,
                                kTolerance));

    // The reaction torque must counteract gravity.
    const Vector3d t_BJb_Jb_expected(kMass * kGravity * kLength / 2.0, 0.0,
                                     0.0);
    EXPECT_TRUE(
        CompareMatrices(F_BJb_Jb.rotational(), t_BJb_Jb_expected, kTolerance));
  }

  const double kMass{1.5};      // [kg]
  const double kLength{2.0};    // [m]
  const double kGravity{10.0};  // [m/s²]
  const double kOmega{5.0};     // [rad/s]

  std::unique_ptr<MultibodyPlant<double>> plant_;
  const RigidBody<double>* rod_{nullptr};
  const RevoluteJoint<double>* pin_{nullptr};
  std::unique_ptr<Context<double>> context_;
};

TEST_F(SpinningRodTest, PinReactionForcesContinuous) {
  BuildModel(0);
  ASSERT_FALSE(plant_->is_discrete());
  VerifyJointReactionForces();
}

TEST_F(SpinningRodTest, PinReactionForcesDiscrete) {
  BuildModel(1.0e-3);
  ASSERT_TRUE(plant_->is_discrete());
  VerifyJointReactionForces();
}

// We verify the computation of joint reaction forces in a model where all
// bodies are anchored to the world using weld joints, and therefore the model
// has a zero sized state.
// In this case a body A is welded to the world. A second body B is welded to A
// with a fixed offset along the x axis. Therefore we expect the a non-zero
// moment at the weld joint between the two bodies to counteract the weight of
// body B. The force on A should match the total weight of A plus B.
class WeldedBoxesTest : public ::testing::Test {
 protected:
  void BuildModel(double discrete_update_period) {
    plant_ = std::make_unique<MultibodyPlant<double>>(discrete_update_period);
    plant_->SetUseSampledOutputPorts(false);  // We're not stepping time.
    AddBoxes();
    plant_->mutable_gravity_field().set_gravity_vector(
        Vector3d(0.0, 0.0, -kGravity));
    plant_->Finalize();
    plant_context_ = plant_->CreateDefaultContext();
  }

  void AddBoxes() {
    const SpatialInertia<double> M_BBo_B =
        SpatialInertia<double>::SolidCubeWithMass(kBoxMass, kCubeSize);

    // Create two rigid bodies.
    boxA_ = &plant_->AddRigidBody("boxA", M_BBo_B);
    boxB_ = &plant_->AddRigidBody("boxB", M_BBo_B);

    // Desired transformation for the boxes in the world.
    const RigidTransformd X_WA(Vector3d::Zero());
    const RigidTransformd X_WB(Vector3d(kCubeSize, 0, 0));
    const RigidTransformd X_AB = X_WA.inverse() * X_WB;

    // Pin boxA to the world and boxB to boxA with weld joints.
    weld1_ = &plant_->WeldFrames(plant_->world_body().body_frame(),
                                 boxA_->body_frame(), X_WA);
    weld2_ =
        &plant_->WeldFrames(boxA_->body_frame(), boxB_->body_frame(), X_AB);
  }

  void VerifyBodyReactionForces() {
    const auto& reaction_forces =
        plant_->get_reaction_forces_output_port()
            .Eval<std::vector<SpatialForce<double>>>(*plant_context_);

    ASSERT_EQ(reaction_forces.size(), 2u);  // we have two joints.

    // Particulars for this setup:
    //   1. A is weld1's child body and its frame corresponds to the joint's
    //      child frame Jc.
    //   2. Moreover, A is coincident with the world and its origin is located
    //      at A's center of mass Acm.
    // Therefore the reaction at weld1 corresponds to F_Acm_W.
    const SpatialForce<double>& F_Acm_W = reaction_forces[weld1_->ordinal()];

    // Verify the reaction force balances the weight of the two boxes.
    const double box_weight = kBoxMass * kGravity;
    const Vector3d f_Acm_W_expected(0.0, 0.0, 2.0 * box_weight);
    // Box B hangs from box A, and therefore the reaction on weld1 must balance
    // the torque due to gravity on box B applied on box A.
    const Vector3d t_Acm_W_expected =
        -kCubeSize * box_weight * Vector3d::UnitY();
    EXPECT_EQ(F_Acm_W.translational(), f_Acm_W_expected);
    EXPECT_EQ(F_Acm_W.rotational(), t_Acm_W_expected);

    // Particulars for this setup:
    //   1. Body B is weld2's child body and its frame corresponds to the
    //      joint's child frame Jc.
    //   2. There is no rotation between B and the world frame.
    //   3. Frame B's origin is located at B's center of mass Bcm.
    // Therefore the reaction at weld2 corresponds to F_Bcm_W.
    const SpatialForce<double>& F_Bcm_W = reaction_forces[weld2_->ordinal()];
    const Vector3d f_Bcm_W_expected = box_weight * Vector3d::UnitZ();
    const Vector3d t_Bcm_W_expected = Vector3d::Zero();
    EXPECT_EQ(F_Bcm_W.translational(), f_Bcm_W_expected);
    EXPECT_EQ(F_Bcm_W.rotational(), t_Bcm_W_expected);
  }

  const RigidBody<double>* boxA_{nullptr};
  const RigidBody<double>* boxB_{nullptr};
  std::unique_ptr<MultibodyPlant<double>> plant_{nullptr};
  const WeldJoint<double>* weld1_{nullptr};
  const WeldJoint<double>* weld2_{nullptr};
  std::unique_ptr<Context<double>> plant_context_;
  const double kCubeSize{1.5};  // Size of the box, in meters.
  const double kBoxMass{2.0};   // Mass of each box, in Kg.
  // We round off gravity for simpler numbers.
  const double kGravity{10.0};  // [m/s²]
};

TEST_F(WeldedBoxesTest, ReactionForcesDiscrete) {
  BuildModel(1.0e-3);
  ASSERT_TRUE(plant_->is_discrete());
  VerifyBodyReactionForces();
}

TEST_F(WeldedBoxesTest, ReactionForcesContinuous) {
  BuildModel(0.0);
  ASSERT_FALSE(plant_->is_discrete());
  VerifyBodyReactionForces();
}

class WeldedAndFloatingTest : public ::testing::TestWithParam<bool> {
 public:
  void SetUp() {
    const bool replace_joints = GetParam();

    plant_ = std::make_unique<MultibodyPlant<double>>(0.01);
    plant_->SetUseSampledOutputPorts(false);  // We're not stepping time.

    // Create four rigid bodies.
    const RigidBody<double>& sphere0 = plant_->AddRigidBody(
        "sphere0",
        SpatialInertia<double>::SolidSphereWithMass(kBodyMasses[0], 1.0));
    const RigidBody<double>& sphere1 = plant_->AddRigidBody(
        "sphere1",
        SpatialInertia<double>::SolidSphereWithMass(kBodyMasses[1], 1.0));
    const RigidBody<double>& sphere2 = plant_->AddRigidBody(
        "sphere2",
        SpatialInertia<double>::SolidSphereWithMass(kBodyMasses[2], 1.0));
    const RigidBody<double>& sphere3 = plant_->AddRigidBody(
        "sphere3",
        SpatialInertia<double>::SolidSphereWithMass(kBodyMasses[3], 1.0));

    // Make sphere0 and sphere1 floating and weld sphere2 and sphere3 to world.
    floating0_ = &plant_->AddJoint<QuaternionFloatingJoint>(
        "floating0", plant_->world_body(), {}, sphere0, {});
    floating1_ = &plant_->AddJoint<QuaternionFloatingJoint>(
        "floating1", plant_->world_body(), {}, sphere1, {});
    weld2_ =
        &plant_->AddJoint<WeldJoint>("weld2", plant_->world_body(), {}, sphere2,
                                     {}, RigidTransformd::Identity());
    weld3_ =
        &plant_->AddJoint<WeldJoint>("weld3", plant_->world_body(), {}, sphere3,
                                     {}, RigidTransformd::Identity());

    // If specified, replace the floating joints with welds.
    if (replace_joints) {
      plant_->RemoveJoint(*floating0_);
      plant_->RemoveJoint(*floating1_);
      floating0_ = nullptr;
      floating1_ = nullptr;
      weld0_ = &plant_->AddJoint<WeldJoint>("weld0", plant_->world_body(), {},
                                            sphere0, {},
                                            RigidTransformd::Identity());
      weld1_ = &plant_->AddJoint<WeldJoint>("weld1", plant_->world_body(), {},
                                            sphere1, {},
                                            RigidTransformd::Identity());
    }
    plant_->mutable_gravity_field().set_gravity_vector(
        Vector3d(0.0, 0.0, -kGravity));
    plant_->Finalize();
    plant_context_ = plant_->CreateDefaultContext();
  }

 protected:
  std::unique_ptr<MultibodyPlant<double>> plant_;
  std::unique_ptr<Context<double>> plant_context_;
  const QuaternionFloatingJoint<double>* floating0_{nullptr};
  const QuaternionFloatingJoint<double>* floating1_{nullptr};
  const WeldJoint<double>* weld0_{nullptr};
  const WeldJoint<double>* weld1_{nullptr};
  const WeldJoint<double>* weld2_{nullptr};
  const WeldJoint<double>* weld3_{nullptr};
  // Mass of each body in Kg.
  // N.B. for the first two bodies, it is important that masses have an exact
  // floating point representation of their root square so that Cholesky
  // factorizations within the forward dynamics used by SAP do not introduce
  // machine epsilon round-off errors. With that consideration, the force
  // comparisons below can be performed exactly.
  const std::array<double, 4> kBodyMasses{1.0, 4.0, 3.0, 4.0};
  // We round off gravity for simpler numbers.
  const double kGravity{10.0};  // [m/s²]
};

TEST_P(WeldedAndFloatingTest, ReactionForcesOrdinalIndexing) {
  const bool replace_joints = GetParam();
  const auto& reaction_forces =
      plant_->get_reaction_forces_output_port()
          .Eval<std::vector<SpatialForce<double>>>(*plant_context_);

  ASSERT_EQ(reaction_forces.size(), 4);
  ASSERT_EQ(plant_->num_joints(), 4);

  if (replace_joints) {
    // Replace the floating joints with welds. This should shift indices around.
    // We confirm that reaction forces are using the correct indexing from the
    // joints.

    // Floating joints were replaced with weld joints.
    EXPECT_FALSE(plant_->HasJointNamed("floating0"));
    EXPECT_FALSE(plant_->HasJointNamed("floating1"));
    EXPECT_FALSE(plant_->has_joint(JointIndex(0)));
    EXPECT_FALSE(plant_->has_joint(JointIndex(1)));
    EXPECT_TRUE(plant_->HasJointNamed("weld0"));
    EXPECT_TRUE(plant_->HasJointNamed("weld1"));

    // Because we removed and replaced the floating joints after all four
    // joints were originally added, they receive joint indices 4 and 5. The
    // ordinals should have been updated during the removal, so the joints
    // should be ordered as: ["weld2", "weld3", "weld0", "weld1"] and reaction
    // forces should correspond to that order.
    const std::vector<const Joint<double>*> joints{weld0_, weld1_, weld2_,
                                                   weld3_};
    const std::vector<JointIndex> expected_indices{
        JointIndex(4), JointIndex(5), JointIndex(2), JointIndex(3)};
    const std::vector<JointOrdinal> expected_ordinals{
        JointOrdinal(2), JointOrdinal(3), JointOrdinal(0), JointOrdinal(1)};
    for (int i = 0; i < 4; ++i) {
      EXPECT_EQ(joints[i]->index(), expected_indices[i]);
      EXPECT_EQ(joints[i]->ordinal(), expected_ordinals[i]);

      // All joints are welded, so we expect the reaction forces to
      // oppose gravity on the bodies.
      const SpatialForce<double>& F_Bcm_W =
          reaction_forces[joints[i]->ordinal()];
      EXPECT_EQ(F_Bcm_W.translational(),
                kBodyMasses[i] * kGravity * Vector3d::UnitZ());

      EXPECT_EQ(F_Bcm_W.rotational(), Vector3d::Zero());
    }

  } else {
    // Do not replace the floating joints.

    // Joints will have assigned contiguous indices and ordinals in the order
    // they were created.
    const std::vector<const Joint<double>*> joints{floating0_, floating1_,
                                                   weld2_, weld3_};
    const std::vector<JointIndex> expected_indices{
        JointIndex(0), JointIndex(1), JointIndex(2), JointIndex(3)};
    const std::vector<JointOrdinal> expected_ordinals{
        JointOrdinal(0), JointOrdinal(1), JointOrdinal(2), JointOrdinal(3)};
    for (int i = 0; i < 4; ++i) {
      EXPECT_EQ(joints[i]->index(), expected_indices[i]);
      EXPECT_EQ(joints[i]->ordinal(), expected_ordinals[i]);

      const SpatialForce<double>& F_Bcm_W =
          reaction_forces[joints[i]->ordinal()];
      if (i < 2) {
        // Joints for bodies 0 and 1 are floating, so we expect no reaction
        // forces.
        EXPECT_EQ(F_Bcm_W.translational(), Vector3d::Zero());
        EXPECT_EQ(F_Bcm_W.rotational(), Vector3d::Zero());
      } else {
        // Joints for bodies 2 and 3 are welded, so we expect the reaction
        // forces to oppose gravity on the bodies.
        EXPECT_EQ(F_Bcm_W.translational(),
                  kBodyMasses[i] * kGravity * Vector3d::UnitZ());
        EXPECT_EQ(F_Bcm_W.rotational(), Vector3d::Zero());
      }
    }
  }
}

INSTANTIATE_TEST_SUITE_P(ReactionForcesTests, WeldedAndFloatingTest,
                         testing::ValuesIn({false, true}));

// This test verifies the computation of joint reaction forces in a model with a
// deformable object sitting on a rigid floor. The deformable is a floating
// body. The floor is welded to the world frame. The position of the weld joint
// is such that its aligned with the center of mass of the rigid floor and the
// deformable box that sits atop it. The reaction force for the weld joint
// connecting the floor to the world should equal the combined weight of the
// deformable object and the floor. Since the weld joint is aligned with the
// center of mass of the rigid floor and deformable box, there should be zero
// reaction torque.
class DeformableReactionForcesTest : public ::testing::Test {
 protected:
  void SetUp() override {
    systems::DiagramBuilder<double> builder;
    std::tie(plant_, scene_graph_) =
        AddMultibodyPlantSceneGraph(&builder, kTimeStep);

    // Add a rigid floor welded to the world
    AddFloor();

    // Add a deformable model to the plant
    Parser parser(&builder);
    const std::string absolute_path =
        FindResourceOrThrow("drake/multibody/plant/test/deformable_box.sdf");
    const auto model_instances =
        parser.AddModelsFromUrl("file://" + absolute_path);
    DeformableModel<double>& deformable_model =
        plant_->mutable_deformable_model();
    deformable_model_ptr_ = &deformable_model;
    const std::vector<DeformableBodyId> deformable_body_ids =
        deformable_model.GetBodyIds(model_instances[0]);
    deformable_body_id_ = deformable_body_ids[0];

    // Set gravity
    plant_->mutable_gravity_field().set_gravity_vector(
        Vector3d(0.0, 0.0, -kGravity));

    // Finalize plant
    plant_->Finalize();

    // Connect visualizer for debugging
    geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph_);

    diagram_ = builder.Build();
  }

  // Adds a rigid floor welded to the world
  void AddFloor() {
    // Create a rigid body for the floor
    const SpatialInertia<double> M_FloorCm =
        SpatialInertia<double>::SolidBoxWithMass(kFloorMass, kFloorWidth,
                                                 kFloorWidth, kFloorHeight);
    floor_ = &plant_->AddRigidBody("floor", M_FloorCm);

    // Position the floor such that the top of the floor makes contact with
    // the bottom of the deformable box when the sim starts
    const auto floor_z_pos = -kFloorHeight / 2.0 - kBoxSize / 2.0;
    const RigidTransformd X_WF(Vector3d(0.0, 0.0, floor_z_pos));

    // Weld the floor to the world
    weld_ = &plant_->WeldFrames(plant_->world_body().body_frame(),
                                floor_->body_frame(), X_WF);

    // Add collision geometry for the floor
    geometry::ProximityProperties proximity_props;
    geometry::AddContactMaterial({}, {}, kFriction, &proximity_props);
    const Box floor_shape(kFloorWidth, kFloorWidth, kFloorHeight);
    plant_->RegisterCollisionGeometry(*floor_, RigidTransformd::Identity(),
                                      floor_shape, "floor_collision",
                                      proximity_props);

    // Add visual geometry for the floor
    const Vector4<double> floor_color(0.5, 0.5, 0.5, 1.0);  // Gray
    plant_->RegisterVisualGeometry(*floor_, RigidTransformd::Identity(),
                                   floor_shape, "floor_visual", floor_color);
  }

  // Computes the reference volume of the registered deformable body
  double CalcDeformableReferenceVolume() const {
    const SceneGraphInspector<double>& inspector =
        scene_graph_->model_inspector();
    const geometry::GeometryId g_id =
        deformable_model_ptr_->GetGeometryId(deformable_body_id_);
    const VolumeMesh<double>* reference_mesh = inspector.GetReferenceMesh(g_id);
    DRAKE_DEMAND(reference_mesh != nullptr);
    return reference_mesh->CalcVolume();
  }

  // Run a simulation
  std::unique_ptr<Simulator<double>> Simulate() {
    auto simulator = std::make_unique<Simulator<double>>(*diagram_);
    simulator->set_target_realtime_rate(kTargetRealtimeRate);
    simulator->AdvanceTo(kSimulationTime);
    return simulator;
  }

  // Verify that joint reaction forces match the expected values
  void VerifyJointReactionForces(const Simulator<double>& simulator) {
    const auto& diagram_context = simulator.get_context();

    const Context<double>& plant_context =
        plant_->GetMyContextFromRoot(diagram_context);

    // Get the reaction forces at the weld joint
    const auto& reaction_forces =
        plant_->get_reaction_forces_output_port()
            .Eval<std::vector<SpatialForce<double>>>(plant_context);

    // There should be one joint (the weld joint)
    ASSERT_EQ(reaction_forces.size(), 1u)
        << "Expected 1 joint in reaction forces, got "
        << reaction_forces.size();

    // Get the reaction force at the weld joint
    const SpatialForce<double>& F_Weld = reaction_forces[weld_->ordinal()];

    // Calculate the expected reaction force (equal to the weight of the
    // deformable object plus the weight of the rigid floor)
    const double volume = CalcDeformableReferenceVolume();
    const double deformable_mass = volume * kMassDensity;
    const double deformable_weight = deformable_mass * kGravity;
    const double floor_weight = kFloorMass * kGravity;

    // The reaction force should be equal to the combined weight of the
    // deformable object and floor
    const Vector3d expected_force(0.0, 0.0, deformable_weight + floor_weight);

    // Verify the reaction force
    EXPECT_TRUE(CompareMatrices(F_Weld.translational(), expected_force,
                                kTolerance, MatrixCompareType::relative));

    // There should be no reaction torque since the weld joint is aligned
    // (in world frame x and y) with the center of mass of both the
    // rigid floor and deformable object.
    const Vector3d expected_torque = Vector3d::Zero();
    EXPECT_TRUE(CompareMatrices(F_Weld.rotational(), expected_torque,
                                kTolerance, MatrixCompareType::relative));
  }

  // Deformable object parameters (from the SDFormat file).
  const double kMassDensity{1000.0};  // Mass density in kg/m³
  const double kBoxSize{0.1};         // Size of the box edge in m

  // Floor parameters
  const double kFloorWidth{10.0};  // Width of the floor in m
  const double kFloorHeight{1.0};  // Height of the floor in m
  const double kFloorMass{10.0};   // Mass of the floor in kg

  // Simulation parameters
  const double kTimeStep{0.01};         // Time step in s
  const double kSimulationTime{1.0};    // Maximum simulation time in s
  const double kTargetRealtimeRate{0};  // Target realtime rate
  const double kGravity{9.81};          // Gravity in m/s²
  const CoulombFriction<double> kFriction{0.4, 0.4};  // Friction
  const double kTolerance{1e-8};  // Tolerance for comparisons

  // System components
  MultibodyPlant<double>* plant_{nullptr};
  SceneGraph<double>* scene_graph_{nullptr};
  DeformableModel<double>* deformable_model_ptr_{nullptr};
  std::unique_ptr<systems::Diagram<double>> diagram_;

  // Bodies and joints
  const RigidBody<double>* floor_{nullptr};
  const WeldJoint<double>* weld_{nullptr};
  DeformableBodyId deformable_body_id_;
};

TEST_F(DeformableReactionForcesTest, ReactionForcesMatchCombinedWeight) {
  auto simulator = Simulate();
  VerifyJointReactionForces(*simulator);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
