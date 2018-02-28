#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/mobilizer_impl.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/space_xyz_mobilizer.h"
#include "drake/multibody/multibody_tree/spatial_inertia.h"
#include "drake/multibody/multibody_tree/unit_inertia.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::Vector3d;
using Eigen::VectorXd;
using systems::Context;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

// A cylindrical-like mobilizer that permits rotation about the x-axis and
// translation along the y-axis.
template <typename T>
class FeatherstoneMobilizer final : public MobilizerImpl<T, 2, 2> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FeatherstoneMobilizer)

  FeatherstoneMobilizer(const Frame<T>& inboard_frame_F,
                        const Frame<T>& outboard_frame_M) :
      MobilizerImpl<T, 2, 2>(inboard_frame_F, outboard_frame_M) {
    H_FM_ << 1, 0,
             0, 0,
             0, 0,
             0, 0,
             0, 1,
             0, 0;
  }

  void set_zero_state(
      const systems::Context<T>& context,
      systems::State<T>* state) const override {
    this->set_default_zero_state(context, state);
  }

  const FeatherstoneMobilizer<T>& set_angles(
      systems::Context<T>* context,
      const Vector2<T>& angles) const {
    MultibodyTreeContext<T>& mbt_context =
        this->GetMutableMultibodyTreeContextOrThrow(context);
    auto q = this->get_mutable_positions(&mbt_context);
    DRAKE_ASSERT(q.size() == kNq);
    q = angles;
    return *this;
  }

  Isometry3<T> CalcAcrossMobilizerTransform(
      const MultibodyTreeContext<T>& context) const override {
    Isometry3<T> X_FM = Isometry3<T>::Identity();

    const Vector3<T> axis_rotation_F = rotation_axis();
    const T rotation = get_rotation(context);
    X_FM.linear() = Eigen::AngleAxis<T>(
        rotation, axis_rotation_F).toRotationMatrix();

    const Vector3<T> axis_translation_F = translation_axis();
    const T translation = get_translation(context);
    X_FM.translation() = translation * axis_translation_F;

    return X_FM;
  }

  SpatialVelocity<T> CalcAcrossMobilizerSpatialVelocity(
      const MultibodyTreeContext<T>& context,
      const Eigen::Ref<const VectorX<T>>& v) const override {
    DRAKE_ASSERT(v.size() == kNv);
    return SpatialVelocity<T>(H_FM_ * v);
  }

  SpatialAcceleration<T> CalcAcrossMobilizerSpatialAcceleration(
      const MultibodyTreeContext<T>& context,
      const Eigen::Ref<const VectorX<T>>& vdot) const override {
    DRAKE_ASSERT(vdot.size() == kNv);
    // Note that Hdot * v = 0 for this mobilizer.
    return SpatialAcceleration<T>(H_FM_ * vdot);
  }

  void ProjectSpatialForce(
      const MultibodyTreeContext<T>& context,
      const SpatialForce<T>& F_Mo_F,
      Eigen::Ref<VectorX<T>> tau) const override {
    DRAKE_ASSERT(tau.size() == kNv);
    tau = H_FM_.transpose() * F_Mo_F.get_coeffs();
  }

  void MapVelocityToQDot(
      const MultibodyTreeContext<T>& context,
      const Eigen::Ref<const VectorX<T>>& v,
      EigenPtr<VectorX<T>> qdot) const override {
    DRAKE_ASSERT(v.size() == kNv);
    DRAKE_ASSERT(qdot != nullptr);
    DRAKE_ASSERT(qdot->size() == kNq);
    *qdot = v;
  }

  void MapQDotToVelocity(
      const MultibodyTreeContext<T>& context,
      const Eigen::Ref<const VectorX<T>>& qdot,
      EigenPtr<VectorX<T>> v) const override {
    DRAKE_ASSERT(qdot.size() == kNq);
    DRAKE_ASSERT(v != nullptr);
    DRAKE_ASSERT(v->size() == kNv);
    *v = qdot;
  }

 protected:
  std::unique_ptr<Mobilizer<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const override {
    return TemplatedDoCloneToScalar(tree_clone);
  }

  std::unique_ptr<Mobilizer<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const override {
    return TemplatedDoCloneToScalar(tree_clone);
  }

 private:
  typedef MobilizerImpl<T, 2, 2> MobilizerBase;

  const Vector3<T> rotation_axis() const {
    return H_FM_.template block<3, 1>(0, 0);
  }

  const Vector3<T> translation_axis() const {
    return H_FM_.template block<3, 1>(3, 1);
  }

  const T get_rotation(const MultibodyTreeContext<T>& context) const {
    const auto& q = this->get_positions(context);
    return q[0];
  }

  const T get_translation(const MultibodyTreeContext<T>& context) const {
    const auto& q = this->get_positions(context);
    return q[1];
  }

  template <typename ToScalar>
  std::unique_ptr<Mobilizer<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const {
    const Frame<ToScalar>& inboard_frame_clone =
        tree_clone.get_variant(this->inboard_frame());
    const Frame<ToScalar>& outboard_frame_clone =
        tree_clone.get_variant(this->outboard_frame());
    return std::make_unique<FeatherstoneMobilizer<ToScalar>>(
        inboard_frame_clone, outboard_frame_clone);
  }

  using MobilizerBase::kNq;
  using MobilizerBase::kNv;

  Eigen::Matrix<T, 6, kNv> H_FM_;
};

// An articulated body inertia from Example 7.1, Pages 123 - 124 of
// [Featherstone 2008]. The test consists of a cylinder inside of a box. The
// cylinder is allowed to rotate about the x-axis and translate along the
// y-axis.
//
// This will test basic recursion and projection of articulated body inertias.
// One extension from the example from Featherstone is that this test uses a
// revolute joint to connect the box with the world (inertial) frame so further
// testing can be done.
//
/// - [Featherstone 2008] Featherstone, R., 2008.
///     Rigid body dynamics algorithms. Springer.
GTEST_TEST(ArticulatedBodyInertiaAlgorithm, FeatherstoneExample) {
  // Create box (B).
  const double Lx = 0.4, Ly = 1.0, Lz = 1.0;
  const UnitInertia<double> G_Bcm = UnitInertia<double>::SolidBox(Lx, Ly, Lz);
  const double mass_box = 2.0;
  const SpatialInertia<double> M_Bcm(mass_box, Vector3d::Zero(), G_Bcm);

  // Create cylinder (C) in box.
  // Note that the unit inertia of the cylinder is taken about the x-axis.
  const double r = 0.2, L = 0.3;
  const UnitInertia<double> G_Ccm =
      UnitInertia<double>::SolidCylinder(r, L, Vector3d::UnitX());
  const double mass_cylinder = 0.8;
  const SpatialInertia<double> M_Ccm(mass_cylinder, Vector3d::Zero(), G_Ccm);

  // Create model.
  MultibodyTree<double> model;

  // Add box body and SpaceXYZ mobilizer.
  const RigidBody<double>& box_link = model.AddBody<RigidBody>(M_Bcm);
  const Frame<double>& world_frame = model.world_frame();
  const Frame<double>& box_frame = box_link.body_frame();
  model.AddMobilizer<SpaceXYZMobilizer>(world_frame, box_frame);

  // Add cylinder body and Featherstone mobilizer.
  const RigidBody<double>& cylinder_link = model.AddBody<RigidBody>(M_Ccm);
  const Frame<double>& cylinder_frame = cylinder_link.body_frame();
  model.AddMobilizer<FeatherstoneMobilizer>(box_frame, cylinder_frame);

  // Finalize model.
  model.Finalize();

  // Create context.
  std::unique_ptr<Context<double>> context = model.CreateDefaultContext();

  // Update cache.
  PositionKinematicsCache<double> pc(model.get_topology());
  model.CalcPositionKinematicsCache(*context, &pc);

  // Compute articulated body cache.
  ArticulatedBodyInertiaCache<double> abc(model.get_topology());
  model.CalcArticulatedBodyInertiaCache(*context, pc,  &abc);

  // Get expected projected articulated body inertia of cylinder.
  Matrix6<double> M_cylinder_mat = M_Ccm.CopyToFullMatrix6();
  Matrix6<double> Pplus_BC_W_expected_mat = Matrix6<double>::Zero();
  Pplus_BC_W_expected_mat(1, 1) = M_cylinder_mat(1, 1);
  Pplus_BC_W_expected_mat(2, 2) = M_cylinder_mat(2, 2);
  Pplus_BC_W_expected_mat(3, 3) = mass_cylinder;
  Pplus_BC_W_expected_mat(5, 5) = mass_cylinder;

  // Compare results.
  const ArticulatedBodyInertia<double>& Pplus_BC_W_expected =
      ArticulatedBodyInertia<double>(Pplus_BC_W_expected_mat);
  const ArticulatedBodyInertia<double>& Pplus_BC_W_actual =
      abc.get_Pplus_PB_W(cylinder_link.node_index());
  EXPECT_TRUE(Pplus_BC_W_expected.CopyToFullMatrix6().isApprox(
      Pplus_BC_W_actual.CopyToFullMatrix6(), kEpsilon));

  // Get expected projected articulated body inertia of the articulated body
  // consisting of the box and cylinder.
  Matrix6<double> Pplus_WB_W_expected_mat = Matrix6<double>::Zero();
  Pplus_WB_W_expected_mat(3, 3) = mass_box + mass_cylinder;
  Pplus_WB_W_expected_mat(4, 4) = mass_box;
  Pplus_WB_W_expected_mat(5, 5) = mass_box + mass_cylinder;

  // Compare results.
  const ArticulatedBodyInertia<double>& P_WB_W_expected =
      ArticulatedBodyInertia<double>(Pplus_WB_W_expected_mat);
  const ArticulatedBodyInertia<double>& P_WB_W_actual =
      abc.get_Pplus_PB_W(box_link.node_index());
  EXPECT_TRUE(P_WB_W_expected.CopyToFullMatrix6().isApprox(
      P_WB_W_actual.CopyToFullMatrix6(), kEpsilon));
}

// A similar test to FeatherstoneExample. The main difference is that this
// test uses non-zero generalized positions and a non-square box.
GTEST_TEST(ArticulatedBodyInertiaAlgorithm, ModifiedFeatherstoneExample) {
  // Create box (B).
  const double Lx = 0.5, Ly = 1.2, Lz = 1.6;
  const UnitInertia<double> G_Bcm = UnitInertia<double>::SolidBox(Lx, Ly, Lz);
  const double mass_box = 2.4;
  const SpatialInertia<double> M_Bcm(mass_box, Vector3d::Zero(), G_Bcm);

  // Create cylinder (C) in box.
  // Note that the unit inertia of the cylinder is taken about the x-axis.
  const double r = 0.3, L = 0.3;
  const UnitInertia<double> G_Ccm =
      UnitInertia<double>::SolidCylinder(r, L, Vector3d::UnitX());
  const double mass_cylinder = 0.6;
  const SpatialInertia<double> M_Ccm(mass_cylinder, Vector3d::Zero(), G_Ccm);

  // Create model.
  MultibodyTree<double> model;

  // Add box body and SpaceXYZ mobilizer.
  const RigidBody<double>& box_link = model.AddBody<RigidBody>(M_Bcm);
  const Frame<double>& world_frame = model.world_frame();
  const Frame<double>& box_frame = box_link.body_frame();
  const SpaceXYZMobilizer<double>& WB_mobilizer =
      model.AddMobilizer<SpaceXYZMobilizer>(world_frame, box_frame);

  // Add cylinder body and Featherstone mobilizer.
  const RigidBody<double>& cylinder_link = model.AddBody<RigidBody>(M_Ccm);
  const Frame<double>& cylinder_frame = cylinder_link.body_frame();
  const FeatherstoneMobilizer<double>& BC_mobilizer =
    model.AddMobilizer<FeatherstoneMobilizer>(box_frame, cylinder_frame);

  // Finalize model.
  model.Finalize();

  // Create context.
  std::unique_ptr<Context<double>> context = model.CreateDefaultContext();

  // State of mobilizer connecting the world and box.
  Vector3d q_WB;
  q_WB << 0.0, -M_PI_2, 0.0;
  WB_mobilizer.set_angles(context.get(), q_WB);

  // State of the mobilizer connecting the box and cylinder.
  Vector2<double> q_BC;
  q_BC << M_PI_4, 0.2;
  BC_mobilizer.set_angles(context.get(), q_BC);

  // Update cache.
  PositionKinematicsCache<double> pc(model.get_topology());
  model.CalcPositionKinematicsCache(*context, &pc);

  // Compute articulated body cache.
  ArticulatedBodyInertiaCache<double> abc(model.get_topology());
  model.CalcArticulatedBodyInertiaCache(*context, pc,  &abc);

  // Rotate the spatial inertia about the y-axis to match the rotation of
  // q_WB.
  Eigen::Matrix3d R_ZX =
      Eigen::AngleAxisd(-M_PI_2, Vector3d::UnitY()).toRotationMatrix();
  Matrix6<double> M_cylinder_mat = M_Ccm.ReExpress(R_ZX).CopyToFullMatrix6();

  // Get expected projected articulated body inertia of cylinder.
  Matrix6<double> Pplus_BC_W_expected_mat = Matrix6<double>::Zero();
  Pplus_BC_W_expected_mat(0, 0) = M_cylinder_mat(0, 0);
  Pplus_BC_W_expected_mat(1, 1) = M_cylinder_mat(1, 1);
  Pplus_BC_W_expected_mat(3, 3) = mass_cylinder;
  Pplus_BC_W_expected_mat(5, 5) = mass_cylinder;

  // Compare results.
  const ArticulatedBodyInertia<double>& Pplus_BC_W_expected =
      ArticulatedBodyInertia<double>(Pplus_BC_W_expected_mat);
  const ArticulatedBodyInertia<double>& Pplus_BC_W_actual =
      abc.get_Pplus_PB_W(cylinder_link.node_index());
  EXPECT_TRUE(Pplus_BC_W_expected.CopyToFullMatrix6().isApprox(
      Pplus_BC_W_actual.CopyToFullMatrix6(), kEpsilon));

  // H_WB_W does not change with q.
  Eigen::Matrix<double, 6, 3> H_WB_W = Eigen::Matrix<double, 6, 3>::Zero();
  H_WB_W.block<3, 3>(0, 0) = Matrix3<double>::Identity();

  // The articulated body inertia of the box is the sum of the rotated spatial
  // inertia of B and the shifted Pplus_BC_W.
  const Matrix6<double> P_B_W =
      Pplus_BC_W_expected.Shift(Vector3d(0.0, -0.2, 0.0)).CopyToFullMatrix6()
          + M_Bcm.ReExpress(R_ZX).CopyToFullMatrix6();

  // Get expected projected articulated body inertia of the articulated body
  // consisting of the box and cylinder.
  Matrix6<double> Pplus_WB_W_expected_mat =
      (Matrix6<double>::Identity() - P_B_W * H_WB_W
          * (H_WB_W.transpose() * P_B_W * H_WB_W).inverse()
          * H_WB_W.transpose()) * P_B_W;

  // Compare results.
  const ArticulatedBodyInertia<double>& P_WB_W_expected =
      ArticulatedBodyInertia<double>(Pplus_WB_W_expected_mat);
  const ArticulatedBodyInertia<double>& P_WB_W_actual =
      abc.get_Pplus_PB_W(box_link.node_index());
  EXPECT_TRUE(P_WB_W_expected.CopyToFullMatrix6().isApprox(
      P_WB_W_actual.CopyToFullMatrix6(), kEpsilon));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
