#pragma once

#include <limits>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/compute_numerical_gradient.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace multibody {

/**
 * Adds two free bodies to a MultibodyPlant.
 */
template <typename T>
void AddTwoFreeBodiesToPlant(MultibodyPlant<T>* model);

/**
 * Constructs a MultibodyPlant consisting of two free bodies.
 * @tparam_nonsymbolic_scalar
 */
template <typename T>
std::unique_ptr<MultibodyPlant<T>> ConstructTwoFreeBodiesPlant();

/**
 * Constructs a MultibodyPlant consisting of an Iiwa robot.
 */
std::unique_ptr<MultibodyPlant<double>> ConstructIiwaPlant(
    const std::string& file_path, double time_step, int num_iiwa = 1);

/**
 * Compares if two eigen matrices of AutoDiff have the same values and
 * gradients.
 */
template <typename DerivedA, typename DerivedB>
typename std::enable_if_t<
    std::is_same_v<typename DerivedA::Scalar, typename DerivedB::Scalar> &&
    std::is_same_v<typename DerivedA::Scalar, AutoDiffXd>>
CompareAutoDiffVectors(const Eigen::MatrixBase<DerivedA>& a,
                       const Eigen::MatrixBase<DerivedB>& b, double tol) {
  EXPECT_TRUE(CompareMatrices(math::ExtractValue(a),
                              math::ExtractValue(b), tol));
  EXPECT_TRUE(CompareMatrices(math::ExtractGradient(a),
                              math::ExtractGradient(b), tol));
}

/**
 * Convert an Eigen::Quaternion to a vector 4d in the order (w, x, y, z).
 */
Eigen::Vector4d QuaternionToVectorWxyz(const Eigen::Quaterniond& q);

// We test kinematic constraints on two robots: an IIWA robot and two free
// bodies. The IIWA test confirms that the bounds and the Eval function of each
// constraint computes the expected result. The two free bodies test confirms
// that the equations in Eval function semantically makes the two free bodies to
// satisfy the kinematic constraints.
class IiwaKinematicConstraintTest : public ::testing::Test {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaKinematicConstraintTest)

  IiwaKinematicConstraintTest();

 protected:
  std::unique_ptr<systems::Diagram<double>> diagram_{};
  MultibodyPlant<double>* plant_{};
  geometry::SceneGraph<double>* scene_graph_{};
  std::unique_ptr<systems::Context<double>> diagram_context_;
  systems::Context<double>* plant_context_;
  // Autodiff, without scene graph.
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_autodiff_;
  std::unique_ptr<systems::Context<AutoDiffXd>> plant_context_autodiff_;
};

// Test kinematic constraints on two free floating bodies.
class TwoFreeBodiesConstraintTest : public ::testing::Test {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TwoFreeBodiesConstraintTest)

  TwoFreeBodiesConstraintTest();

  ~TwoFreeBodiesConstraintTest() override {}

 protected:
  std::unique_ptr<systems::Diagram<double>> diagram_;
  MultibodyPlant<double>* plant_{};
  std::unique_ptr<systems::Context<double>> diagram_context_;
  systems::Context<double>* plant_context_;
  FrameIndex body1_index_;
  FrameIndex body2_index_;
  // Autodiff, without scene graph.
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_autodiff_;
  std::unique_ptr<systems::Context<AutoDiffXd>> plant_context_autodiff_;
};

class TwoFreeSpheresTest : public ::testing::Test {
 public:
  TwoFreeSpheresTest();

  template <typename T>
  geometry::GeometryId GetSphereGeometryId(const MultibodyPlant<T>& plant,
                                           FrameIndex sphere_index) const {
    return plant.GetCollisionGeometriesForBody(
        plant.get_frame(sphere_index).body())[0];
  }

 protected:
  double radius1_{0.1};
  double radius2_{0.2};
  std::unique_ptr<systems::Diagram<double>> diagram_double_;
  std::unique_ptr<systems::Diagram<AutoDiffXd>> diagram_autodiff_;
  MultibodyPlant<double>* plant_double_{nullptr};
  MultibodyPlant<AutoDiffXd>* plant_autodiff_{nullptr};
  geometry::SceneGraph<double>* scene_graph_double_{nullptr};
  geometry::SceneGraph<AutoDiffXd>* scene_graph_autodiff_{nullptr};

  FrameIndex sphere1_index_;
  FrameIndex sphere2_index_;

  // The pose of sphere 1's collision geometry in sphere 1's body frame.
  math::RigidTransformd X_B1S1_;
  // The pose of sphere 2's collision geometry in sphere 2's body frame.
  math::RigidTransformd X_B2S2_;

  std::unique_ptr<systems::Context<double>> diagram_context_double_;
  std::unique_ptr<systems::Context<AutoDiffXd>> diagram_context_autodiff_;
  systems::Context<double>* plant_context_double_{nullptr};
  systems::Context<AutoDiffXd>* plant_context_autodiff_{nullptr};
};

/**
 * Compute the signed distance between a box and a sphere.
 * @param box_size The size of the box.
 * @param radius The radius of the sphere.
 * @param X_WB the pose of the box (B) in the world frame (W).
 * @param X_WS the pose of the sphere (S) in the world frame (W).
 * @tparam_nonsymbolic_scalar
 */
template <typename T>
T BoxSphereSignedDistance(const Eigen::Ref<const Eigen::Vector3d>& box_size,
                          double radius, const math::RigidTransform<T>& X_WB,
                          const math::RigidTransform<T>& X_WS);

class BoxSphereTest : public ::testing::Test {
 public:
  BoxSphereTest();

 protected:
  Eigen::Vector3d box_size_;
  double radius_{0};
  // The pose of the box collision geometry frame Gb in the box body frame B.
  math::RigidTransformd X_BGb_{};
  std::unique_ptr<systems::Diagram<double>> diagram_double_;
  std::unique_ptr<systems::System<AutoDiffXd>> owned_diagram_autodiff_;
  systems::Diagram<AutoDiffXd>* diagram_autodiff_;
  MultibodyPlant<double>* plant_double_{nullptr};
  const MultibodyPlant<AutoDiffXd>* plant_autodiff_{nullptr};
  geometry::SceneGraph<double>* scene_graph_double_{nullptr};
  const geometry::SceneGraph<AutoDiffXd>* scene_graph_autodiff_{nullptr};
  std::unique_ptr<systems::Context<double>> diagram_context_double_;
  std::unique_ptr<systems::Context<AutoDiffXd>> diagram_context_autodiff_;
  systems::Context<double>* plant_context_double_{nullptr};
  systems::Context<AutoDiffXd>* plant_context_autodiff_{nullptr};
  FrameIndex sphere_frame_index_{};
  FrameIndex box_frame_index_{};
  geometry::GeometryId sphere_geometry_id_{};
  geometry::GeometryId box_geometry_id_{};
};

/**
 * Since we can construct the kinematic constraint using both
 * MultibodyPlant<double> (as @p constraint_from_double) and
 * MultibodyPlant<AutoDiffXd> (as @p constraint_from_autodiff), and evaluate the
 * constraint with both VectorX<double> and VectorX<AutoDiffXd>, we check if the
 * following evaluation results match:
 * 1. constraint_from_double.Eval(x_double) =
 *    constraint_from_autodiff.Eval(x_double).
 * 2. constraint_from_double.Eval(x_autodiff) =
 *    constraint_from_autodiff.Eval(x_autodiff).
 * 3. constraint_from_double.Eval(x_double) =
 *    constraint_from_double.Eval(x_autodiff).value()
 * 4. numerical_gradient(constraint_from_double.Eval(x_double)) ≈
 *    constraint_from_double.Eval(x_autodiff).
 * @param constraint_from_double A kinematic constraint constructed from
 * MultibodyPlant<double>
 * @param constraint_from_autodiff The same kinematic constraint, but
 * constructed from MultibodyPlant<AutoDiffXd>.
 * @param x_double The value of x passed to the constraint Eval function.
 * @param dx The gradient of x_double. dx must have the same number of rows as
 * x_double.
 * @param gradient_tol The tolerance for checking the 4th condition above, that
 * the numerical gradient should be close to the analytical gradient.
 */
template <typename ConstraintType>
void TestKinematicConstraintEval(
    const ConstraintType& constraint_from_double,
    const ConstraintType& constraint_from_autodiff,
    const Eigen::Ref<const Eigen::VectorXd>& x_double,
    const Eigen::Ref<const Eigen::MatrixXd>& dx, double gradient_tol) {
  const double tol = 1000 * std::numeric_limits<double>::epsilon();
  // condition 1.
  Eigen::VectorXd y1_left, y1_right;
  constraint_from_double.Eval(x_double, &y1_left);
  constraint_from_autodiff.Eval(x_double, &y1_right);
  EXPECT_TRUE(CompareMatrices(y1_left, y1_right, tol));

  // condition 2
  const auto x_autodiff =
      math::InitializeAutoDiff(x_double, dx);
  AutoDiffVecXd y2_left, y2_right;
  constraint_from_double.Eval(x_autodiff, &y2_left);
  constraint_from_autodiff.Eval(x_autodiff, &y2_right);
  EXPECT_TRUE(CompareMatrices(math::ExtractValue(y2_left),
                              math::ExtractValue(y2_right), tol));
  EXPECT_TRUE(CompareMatrices(math::ExtractGradient(y2_left),
                              math::ExtractGradient(y2_right), tol));

  // condition 3
  Eigen::VectorXd y3_left;
  AutoDiffVecXd y3_right;
  constraint_from_double.Eval(x_double, &y3_left);
  constraint_from_double.Eval(x_autodiff, &y3_right);
  EXPECT_TRUE(CompareMatrices(y3_left, math::ExtractValue(y3_right), tol));

  // condition 4
  std::function<void(const Eigen::Ref<const Eigen::VectorXd>&,
                     Eigen::VectorXd*)>
      eval_fun = [&constraint_from_double](
                     const Eigen::Ref<const Eigen::VectorXd>& x,
                     Eigen::VectorXd* y) { constraint_from_double.Eval(x, y); };
  const auto dy_dx_numeric = math::ComputeNumericalGradient(eval_fun, x_double);
  const Eigen::MatrixXd y_grad_numeric = dy_dx_numeric * dx;
  EXPECT_TRUE(CompareMatrices(y_grad_numeric, math::ExtractGradient(y2_right),
                              gradient_tol));
}

}  // namespace multibody
}  // namespace drake
