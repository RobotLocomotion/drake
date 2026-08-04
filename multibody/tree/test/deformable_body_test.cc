#include "drake/multibody/tree/deformable_body.h"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/deformable_model.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/multibody/tree/force_density_field.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using Eigen::Matrix3X;
using Eigen::Vector3d;
using Eigen::VectorXd;
using geometry::GeometryInstance;
using geometry::SceneGraph;
using geometry::Sphere;
using math::RigidTransformd;
using systems::Context;
using systems::DiagramBuilder;

/* Builds a plant with exactly one deformable body, then finalizes and
 retrieves the DeformableBody for direct testing. Most of the DeformableBody
 APIs are already tested through the DeformableModel tests (e.g. adding
 constraints, boundary conditions, enabling/disabling a body, etc), and here we
 only test the APIs not fully covered in DeformableModel tests. */
class DeformableBodyTest : public ::testing::Test {
 protected:
  void SetUp() override {
    MultibodyPlantConfig plant_config;
    plant_config.time_step = 0.01;
    std::tie(plant_, scene_graph_) = AddMultibodyPlant(plant_config, &builder_);

    DeformableModel<double>& deformable_model =
        plant_->mutable_deformable_model();

    ModelInstanceIndex model_instance = plant_->AddModelInstance("deformable");

    const math::RigidTransformd X_WG(
        math::RotationMatrixd::MakeZRotation(M_PI / 2),
        Vector3d(3.0, 2.0, 1.0));
    auto sphere = std::make_unique<GeometryInstance>(
        X_WG, std::make_unique<Sphere>(1.0), "test_sphere");
    sphere->set_proximity_properties({});
    /* Register with the coarsest resolution hint for the sphere so that we know
     the mesh is an octahedron. */
    body_id_ = deformable_model.RegisterDeformableBody(
        std::move(sphere), model_instance, default_body_config_,
        /*resolution_hint=*/2.0);

    plant_->Finalize();
    diagram_ = builder_.Build();
    diagram_context_ = diagram_->CreateDefaultContext();
    plant_context_ =
        &plant_->GetMyMutableContextFromRoot(diagram_context_.get());

    body_index_ = deformable_model.GetBodyIndex(body_id_);
    body_ = &deformable_model.GetBody(body_id_);
    mutable_body_ = &deformable_model.GetMutableBody(body_id_);
  }

  DiagramBuilder<double> builder_;
  MultibodyPlant<double>* plant_{nullptr};
  SceneGraph<double>* scene_graph_{nullptr};
  std::unique_ptr<systems::Diagram<double>> diagram_;
  std::unique_ptr<systems::Context<double>> diagram_context_;
  systems::Context<double>* plant_context_{nullptr};

  const fem::DeformableBodyConfig<double> default_body_config_{};
  DeformableBodyId body_id_;
  DeformableBodyIndex body_index_;
  const DeformableBody<double>* body_{nullptr};
  DeformableBody<double>* mutable_body_{nullptr};
};

TEST_F(DeformableBodyTest, Accessors) {
  /* index(), body_id(), name(), scoped_name(), geometry_id() */
  EXPECT_EQ(body_->index(), body_index_);
  EXPECT_EQ(body_->body_id(), body_id_);
  EXPECT_EQ(body_->name(), "test_sphere");
  EXPECT_EQ(body_->scoped_name().get_full(), "deformable::test_sphere");
  EXPECT_TRUE(scene_graph_->model_inspector().IsDeformableGeometry(
      body_->geometry_id()));
  /* config() matches what we registered */
  EXPECT_EQ(body_->config().youngs_modulus(),
            default_body_config_.youngs_modulus());
  EXPECT_EQ(body_->config().mass_density(),
            default_body_config_.mass_density());
  /* FEM model accessor. */
  const fem::FemModel<double>& fem_model = body_->fem_model();
  EXPECT_EQ(fem_model.num_dofs(), body_->num_dofs());
  /* External forces. */
  const std::vector<const ForceDensityFieldBase<double>*>& external_forces =
      body_->external_forces();
  ASSERT_EQ(external_forces.size(), 1);  // gravity is the only one.
  const ForceDensityFieldBase<double>* gravity_force = external_forces[0];
  const GravityForceField<double>* gravity_force_field =
      dynamic_cast<const GravityForceField<double>*>(gravity_force);
  ASSERT_NE(gravity_force_field, nullptr);
  EXPECT_EQ(gravity_force_field->EvaluateAt(*plant_context_, Vector3d(1, 2, 3)),
            Vector3d(0, 0, -9.81) * default_body_config_.mass_density());

  /* Discrete state index. */
  const systems::DiscreteStateIndex discrete_state_index =
      body_->discrete_state_index();
  EXPECT_EQ(plant_context_->get_discrete_state(discrete_state_index).size(),
            3 * body_->num_dofs());  // positions + velocities + accelerations.
  /* is_enabled_parameter_index() */
  const systems::AbstractParameterIndex is_enabled_index =
      body_->is_enabled_parameter_index();
  EXPECT_EQ(
      plant_context_->get_parameters().template get_abstract_parameter<bool>(
          is_enabled_index),
      true);
}

TEST_F(DeformableBodyTest, FemStateCache) {
  /* fem_state_cache_index() */
  const systems::CacheIndex fem_state_cache_index =
      body_->fem_state_cache_index();
  EXPECT_TRUE(fem_state_cache_index.is_valid());
  /* Verify we can evaluate the cache entry. */
  EXPECT_NO_THROW(plant_->get_cache_entry(fem_state_cache_index)
                      .template Eval<fem::FemState<double>>(*plant_context_));
}

TEST_F(DeformableBodyTest, SetGetPositions) {
  const int n = body_->num_dofs();
  Matrix3X<double> q(3, n / 3);
  for (int i = 0; i < q.cols(); ++i) {
    q.col(i) = Vector3d(0.1 * i, -0.2 * i, 0.3 * i);
  }
  body_->SetPositions(plant_context_, q);
  EXPECT_TRUE(CompareMatrices(body_->GetPositions(*plant_context_), q));
}

TEST_F(DeformableBodyTest, SetGetVelocities) {
  const int n = body_->num_dofs();
  Matrix3X<double> v(3, n / 3);
  for (int i = 0; i < v.cols(); ++i) {
    v.col(i) = Vector3d(0.4 * i, -0.5 * i, 0.6 * i);
  }
  body_->SetVelocities(plant_context_, v);
  EXPECT_TRUE(CompareMatrices(body_->GetVelocities(*plant_context_), v));
}

TEST_F(DeformableBodyTest, SetGetPositionsAndVelocities) {
  const int n = body_->num_dofs();
  const int num_nodes = n / 3;
  Matrix3X<double> q(3, num_nodes);
  for (int i = 0; i < q.cols(); ++i) {
    q.col(i) = Vector3d(0.1 * i, -0.2 * i, 0.3 * i);
  }
  Matrix3X<double> v(3, num_nodes);
  for (int i = 0; i < v.cols(); ++i) {
    v.col(i) = Vector3d(0.4 * i, -0.5 * i, 0.6 * i);
  }
  body_->SetPositionsAndVelocities(plant_context_, q, v);
  EXPECT_TRUE(CompareMatrices(body_->GetPositions(*plant_context_), q));
  EXPECT_TRUE(CompareMatrices(body_->GetVelocities(*plant_context_), v));
  const Matrix3X<double> qv = body_->GetPositionsAndVelocities(*plant_context_);
  EXPECT_EQ(qv.cols(), 2 * num_nodes);
  EXPECT_TRUE(CompareMatrices(qv.leftCols(num_nodes), q));
  EXPECT_TRUE(CompareMatrices(qv.rightCols(num_nodes), v));
}

TEST_F(DeformableBodyTest, Parallelism) {
  mutable_body_->set_parallelism(Parallelism(4));
  EXPECT_EQ(body_->fem_model().parallelism().num_threads(), 4);
}

TEST_F(DeformableBodyTest, DefaultPose) {
  const double kTolerance = 4.0 * std::numeric_limits<double>::epsilon();
  /* The registered sphere looks like this in its geometry frame, G.
                  +Gz   -Gx
                   |   /
                   v5 v3
                   | /
                   |/
   -Gy---v4------v0+------v2---+ Gy
                  /| Fo
                 / |
               v1  v6
               /   |
             +Gx   |
                  -Gz  */
  VectorXd p_GV(7 * 3);
  p_GV.segment<3>(0) = Vector3d(0, 0, 0);    // v0
  p_GV.segment<3>(3) = Vector3d(1, 0, 0);    // v1
  p_GV.segment<3>(6) = Vector3d(0, 1, 0);    // v2
  p_GV.segment<3>(9) = Vector3d(-1, 0, 0);   // v3
  p_GV.segment<3>(12) = Vector3d(0, -1, 0);  // v4
  p_GV.segment<3>(15) = Vector3d(0, 0, 1);   // v5
  p_GV.segment<3>(18) = Vector3d(0, 0, -1);  // v6

  const VectorXd p_WVg =
      plant_context_->get_discrete_state(body_->discrete_state_index()).value();
  const math::RigidTransformd X_WG = body_->get_default_pose();
  for (int v = 0; v < 7; ++v) {
    EXPECT_TRUE(CompareMatrices(p_WVg.segment<3>(3 * v),
                                X_WG * p_GV.segment<3>(3 * v), kTolerance));
  }

  /* Create a new pose different from the initial one. */
  const math::RigidTransformd X_WD_expected(
      math::RotationMatrixd::MakeXRotation(M_PI / 3), Vector3d(1.0, 2.0, 3.0));
  /* Set the new pose. */
  mutable_body_->set_default_pose(X_WD_expected);
  /* Verify the new pose is correctly set. */
  const math::RigidTransformd X_WD = body_->get_default_pose();
  EXPECT_TRUE(X_WD.IsExactlyEqualTo(X_WD_expected));
  EXPECT_FALSE(X_WD.IsNearlyEqualTo(X_WG, 0.1));
  /* Verify the default state is updated. */
  plant_->SetDefaultState(*plant_context_,
                          &plant_context_->get_mutable_state());
  const VectorXd p_WVd =
      plant_context_->get_discrete_state(body_->discrete_state_index()).value();
  const VectorXd& p_DV = p_GV;
  for (int v = 0; v < 7; ++v) {
    EXPECT_TRUE(CompareMatrices(p_WVd.segment<3>(3 * v),
                                X_WD * p_DV.segment<3>(3 * v), kTolerance));
  }
}

TEST_F(DeformableBodyTest, CalcCenterOfMassPositionInWorld) {
  constexpr double kEpsilon = 1e-14;
  /* Set the pose of the ball in the world frame to identity. */
  mutable_body_->set_default_pose(math::RigidTransformd::Identity());
  plant_->SetDefaultState(*plant_context_,
                          &plant_context_->get_mutable_state());
  /* For the undeformed sphere centered at the origin, the CoM should be very
   close to the origin. */
  const Vector3d com = body_->CalcCenterOfMassPositionInWorld(*plant_context_);
  EXPECT_TRUE(CompareMatrices(com, Vector3d::Zero(), kEpsilon));

  /* Move the sphere center So to a new location and check CoM. */
  Matrix3X<double> q = body_->GetPositions(*plant_context_);
  const Vector3d p_WoSo_W(1.0, 2.0, 3.0);
  for (int i = 0; i < q.cols(); ++i) {
    q.col(i) += p_WoSo_W;
  }
  body_->SetPositions(plant_context_, q);
  /* Re-evaluating the CoM requires the FemState cache to be recomputed.
   DeformableBody::SetPositions() is expected to invalidate the FemState cache
   entry. Therefore, simply evaluating the cache entry again should trigger a
   recomputation using the new positions. */
  plant_->get_cache_entry(body_->fem_state_cache_index())
      .template Eval<fem::FemState<double>>(*plant_context_);

  const Vector3d com_translated =
      body_->CalcCenterOfMassPositionInWorld(*plant_context_);
  EXPECT_TRUE(CompareMatrices(com_translated, p_WoSo_W, kEpsilon));
}

TEST_F(DeformableBodyTest, NumDofsAndReferencePositions) {
  const double kTolerance = 4.0 * std::numeric_limits<double>::epsilon();
  const int num_dofs = body_->num_dofs();
  EXPECT_GT(num_dofs, 0);
  const VectorX<double>& q_ref = body_->reference_positions();
  EXPECT_EQ(q_ref.size(), num_dofs);
  /* Ensure that GetPositions() initially returns the same reference positions.
   */
  Matrix3X<double> q = body_->GetPositions(*plant_context_);
  Eigen::Map<const Eigen::VectorXd> q_flat(q.data(), q.size());
  EXPECT_TRUE(CompareMatrices(q_flat, q_ref, kTolerance));
}

TEST_F(DeformableBodyTest, CalcCenterOfMassTranslationalVelocityInWorld) {
  constexpr double kEpsilon = 1e-14;
  /* Set a uniform translational velocity. */
  VectorX<double> discrete_state =
      plant_context_->get_discrete_state(body_->discrete_state_index()).value();
  const int num_dofs = body_->num_dofs();
  /* Set v to (1,2,3) for all nodes. */
  for (int i = 0; i < num_dofs / 3; ++i) {
    discrete_state.segment<3>(num_dofs + i * 3) << 1.0, 2.0, 3.0;
  }
  plant_context_->SetDiscreteState(body_->discrete_state_index(),
                                   discrete_state);
  const Vector3d v_WCcm =
      body_->CalcCenterOfMassTranslationalVelocityInWorld(*plant_context_);
  EXPECT_TRUE(CompareMatrices(v_WCcm, Vector3d(1.0, 2.0, 3.0), kEpsilon));
}

TEST_F(DeformableBodyTest, CalcEffectiveAngularVelocity) {
  constexpr double kEpsilon = 1e-14;
  /* Set the pose of the ball in the world frame to identity. */
  mutable_body_->set_default_pose(math::RigidTransformd::Identity());
  plant_->SetDefaultState(*plant_context_,
                          &plant_context_->get_mutable_state());
  /* Let q be the 3N vector concatenation of all vertex positions, measured and
   expressed in the world frame. */
  VectorX<double> discrete_state =
      plant_context_->get_discrete_state(body_->discrete_state_index()).value();
  const int num_dofs = body_->num_dofs();
  VectorX<double> q = discrete_state.head(num_dofs);
  /* The following sets up translational velocities of points of a ball S such
   that they are consistently associated with a rigid ball's angular velocity.
  */
  {
    /* Set the position of the ball's geometric center So in the world frame.
     */
    const Vector3d p_WSo = Vector3d(1.0, 0.0, 0.0);
    /* Translate all vertices of the ball by (1, 0, 0). The resulting effect
     is that now the ball is centered at (1, 0, 0) in the world frame. */
    for (int i = 0; i < num_dofs / 3; ++i) {
      auto p_WV = q.segment<3>(i * 3);
      p_WV += p_WSo;
    }
    /* Reshape q to a 3xN matrix to be compatible with
     DeformableBody::SetPositions. */
    Matrix3X<double> q_matrix(3, num_dofs / 3);
    for (int i = 0; i < num_dofs / 3; ++i) {
      q_matrix.col(i) = q.segment<3>(i * 3);
    }
    body_->SetPositions(plant_context_, q_matrix);
    /* Set a velocity field associated with a uniform angular velocity about
     axis through (1, 0, 0) and parallel to the z-axis. */
    const double omega_z = 1.23;
    auto v = discrete_state.segment(num_dofs, num_dofs);
    for (int i = 0; i < num_dofs / 3; ++i) {
      const Vector3d p_WP = q.segment<3>(i * 3);
      auto v_WP = v.template segment<3>(i * 3);
      const Vector3d p_WSoP_W = p_WP - p_WSo;
      v_WP << -omega_z * p_WSoP_W.y(), omega_z * p_WSoP_W.x(), 0.0;
    }
    plant_context_->SetDiscreteState(body_->discrete_state_index(),
                                     discrete_state);
    /* The body S is a single FEM system, and w_WScm_W is its effective
     angular velocity for Scm, measured and expressed in the world frame W. */
    Vector3d w_WScm_W = body_->CalcEffectiveAngularVelocity(*plant_context_);
    EXPECT_TRUE(
        CompareMatrices(w_WScm_W, Vector3d(0.0, 0.0, omega_z), kEpsilon));
  }

  /* Now, modify the velocity field to be radially outward from the center of
   the ball. The resulting effective angular velocity should be zero. */
  {
    auto v = discrete_state.segment(num_dofs, num_dofs);
    /* Set the position of the ball's geometric center So in the world frame.
     */
    const Vector3d p_WSo = Vector3d(1.0, 0.0, 0.0);
    const double dilation = 2.34;  // unit 1/s.
    for (int i = 0; i < num_dofs / 3; ++i) {
      const Vector3d p_WP = q.segment<3>(i * 3);
      auto v_WP = v.template segment<3>(i * 3);
      const Vector3d p_SoP_W = p_WP - p_WSo;
      v_WP = dilation * p_SoP_W;
    }
    plant_context_->SetDiscreteState(body_->discrete_state_index(),
                                     discrete_state);
    const Vector3d w_WScm_W =
        body_->CalcEffectiveAngularVelocity(*plant_context_);
    EXPECT_TRUE(CompareMatrices(w_WScm_W, Vector3d::Zero(), kEpsilon));
  }
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
