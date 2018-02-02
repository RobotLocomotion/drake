#include <gtest/gtest.h>

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
    H_FM_ = Matrix6X<T>(6, 2);
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

  Isometry3<T> CalcAcrossMobilizerTransform(
      const MultibodyTreeContext<T>& context) const override {
    const auto& q = this->get_positions(context);
    Isometry3<T> X_FM = Isometry3<T>::Identity();

    Vector3<T> axis_linear = H_FM_.template block<3, 1>(0, 0);
    X_FM.linear() = Eigen::AngleAxis<T>(q[0], axis_linear).toRotationMatrix();

    Vector3<T> axis_translation = H_FM_.template block<3, 1>(3, 1);
    X_FM.translation() = q[1] * axis_translation;

    return X_FM;
  }

  SpatialVelocity<T> CalcAcrossMobilizerSpatialVelocity(
      const MultibodyTreeContext<T>& context,
      const Eigen::Ref<const VectorX<T>>& v) const override {
    SpatialVelocity<T> V_FM;
    V_FM.get_coeffs() = H_FM_ * v;
    return V_FM;
  }

  SpatialAcceleration<T> CalcAcrossMobilizerSpatialAcceleration(
      const MultibodyTreeContext<T>& context,
      const Eigen::Ref<const VectorX<T>>& vdot) const override {
    SpatialAcceleration<T> A_FM;
    A_FM.get_coeffs() = H_FM_ * vdot;
    return A_FM;
  }

  void ProjectSpatialForce(
      const MultibodyTreeContext<T>& context,
      const SpatialForce<T>& F_Mo_F,
      Eigen::Ref<VectorX<T>> tau) const override {
    tau = H_FM_.transpose() * F_Mo_F.get_coeffs();
  }

  void MapVelocityToQDot(
      const MultibodyTreeContext<T>& context,
      const Eigen::Ref<const VectorX<T>>& v,
      EigenPtr<VectorX<T>> qdot) const override {
    *qdot = v;
  }

  void MapQDotToVelocity(
      const MultibodyTreeContext<T>& context,
      const Eigen::Ref<const VectorX<T>>& qdot,
      EigenPtr<VectorX<T>> v) const override {
    *v = qdot;
  }

 protected:
  template <typename ToScalar>
  std::unique_ptr<Mobilizer<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const {
    const Frame<ToScalar>& inboard_frame_clone =
        tree_clone.get_variant(this->get_inboard_frame());
    const Frame<ToScalar>& outboard_frame_clone =
        tree_clone.get_variant(this->get_outboard_frame());
    return std::make_unique<FeatherstoneMobilizer<ToScalar>>(
        inboard_frame_clone, outboard_frame_clone);
  }

  std::unique_ptr<Mobilizer<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const override {
    return TemplatedDoCloneToScalar(tree_clone);
  }

  std::unique_ptr<Mobilizer<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const override {
    return TemplatedDoCloneToScalar(tree_clone);
  }

 private:
  Matrix6X<T> H_FM_;
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
  const UnitInertia<double> I_box = UnitInertia<double>::SolidBox(Lx, Ly, Lz);
  const double mass_box = 2.0;
  const SpatialInertia<double> M_box(mass_box, Vector3d::Zero(), I_box);

  // Create cylinder (C) in box.
  const double r = 0.2, L = 0.3;
  const double Ix = r * r / 2;
  const double Iy = (3 * r * r + L * L) / 12;
  const UnitInertia<double> I_cylinder = UnitInertia<double>(Ix, Iy, Iy);
  const double mass_cylinder = 0.8;
  const SpatialInertia<double> M_cylinder(
      mass_cylinder, Vector3d::Zero(), I_cylinder);

  // Create model.
  MultibodyTree<double> model;

  // Add box body and fixed mobilizer.
  const RigidBody<double>& box_link = model.AddBody<RigidBody>(M_box);
  const Frame<double>& world_inboard_frame = model.get_world_frame();
  const Frame<double>& box_outboard_frame = box_link.get_body_frame();
  model.AddMobilizer(std::make_unique<SpaceXYZMobilizer<double>>(
      world_inboard_frame, box_outboard_frame));

  // Add cylinder body and Featherstone mobilizer.
  const RigidBody<double>& cylinder_link = model.AddBody<RigidBody>(M_cylinder);
  const Frame<double>& box_inboard_frame = box_link.get_body_frame();
  const Frame<double>& cylinder_outboard_frame = cylinder_link.get_body_frame();
  model.AddMobilizer(std::make_unique<FeatherstoneMobilizer<double>>(
      box_inboard_frame, cylinder_outboard_frame));

  // Finalize model.
  model.Finalize();

  // Create context.
  std::unique_ptr<Context<double>> context = model.CreateDefaultContext();

  // Update cache.
  PositionKinematicsCache<double> pc(model.get_topology());
  VelocityKinematicsCache<double> vc(model.get_topology());
  model.CalcPositionKinematicsCache(*context, &pc);
  model.CalcVelocityKinematicsCache(*context, pc, &vc);

  // No forces.
  MultibodyForces<double> forces = MultibodyForces<double>(model);
  forces.SetZero();

  // Compute articulated body cache.
  ArticulatedBodyCache<double> abc(model.get_topology());
  model.CalcArticulatedBodyCache(*context, pc, vc, forces, &abc);

  // Get expected projected articulated body inertia of cylinder.
  const Matrix6<double> M_cylinder_mat = M_cylinder.CopyToFullMatrix6();
  Matrix6<double> P_BC_W_expected_mat = Matrix6<double>::Zero();
  P_BC_W_expected_mat(1, 1) = M_cylinder_mat(1, 1);
  P_BC_W_expected_mat(2, 2) = M_cylinder_mat(2, 2);
  P_BC_W_expected_mat(3, 3) = mass_cylinder;
  P_BC_W_expected_mat(5, 5) = mass_cylinder;

  // Compare results.
  const ArticulatedBodyInertia<double>& P_BC_W_expected =
      ArticulatedBodyInertia<double>(P_BC_W_expected_mat);
  const ArticulatedBodyInertia<double>& P_BC_W_actual =
      abc.get_P_PB_W(BodyNodeIndex(2));
  EXPECT_TRUE(P_BC_W_expected.CopyToFullMatrix6().isApprox(
      P_BC_W_actual.CopyToFullMatrix6(), kEpsilon));

  // Get expected projected articulated body inertia of the articulated body
  // consisting of the box and cylinder.
  Matrix6<double> P_WB_W_expected_mat = Matrix6<double>::Zero();
  P_WB_W_expected_mat(3, 3) = mass_box + mass_cylinder;
  P_WB_W_expected_mat(4, 4) = mass_box;
  P_WB_W_expected_mat(5, 5) = mass_box + mass_cylinder;

  // Compare results.
  const ArticulatedBodyInertia<double>& P_WB_W_expected =
      ArticulatedBodyInertia<double>(P_WB_W_expected_mat);
  const ArticulatedBodyInertia<double>& P_WB_W_actual =
      abc.get_P_PB_W(BodyNodeIndex(1));
  EXPECT_TRUE(P_WB_W_expected.CopyToFullMatrix6().isApprox(
      P_WB_W_actual.CopyToFullMatrix6(), kEpsilon));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
