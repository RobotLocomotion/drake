#pragma once

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/benchmarks/kuka_iiwa_robot/make_kuka_iiwa_model.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace multibody {
/**
 * Constructs a MultibodyTree consisting of two free bodies.
 */
template <typename T>
std::unique_ptr<MultibodyTree<T>> ConstructTwoFreeBodies();

/**
 * Constructs a MultibodyPlant consisting of two free bodies.
 */
template <typename T>
std::unique_ptr<MultibodyPlant<T>> ConstructTwoFreeBodiesPlant();

/**
 * Constructs a MultibodyPlant consisting of an Iiwa robot.
 */
std::unique_ptr<MultibodyPlant<double>> ConstructIiwaPlant(
    const std::string& iiwa_sdf_name, double time_step);

/**
 * Compares if two eigen matrices of AutoDiff have the same values and
 * gradients.
 */
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_same<typename DerivedA::Scalar, typename DerivedB::Scalar>::value &&
    std::is_same<typename DerivedA::Scalar, AutoDiffXd>::value>::type
CompareAutoDiffVectors(const Eigen::MatrixBase<DerivedA>& a,
                       const Eigen::MatrixBase<DerivedB>& b, double tol) {
  EXPECT_TRUE(CompareMatrices(math::autoDiffToValueMatrix(a),
                              math::autoDiffToValueMatrix(b), tol));
  EXPECT_TRUE(CompareMatrices(math::autoDiffToGradientMatrix(a),
                              math::autoDiffToGradientMatrix(b), tol));
}

/**
 * Convert an Eigen::Quaternion to a vector 4d in the order (w, x, y, z).
 */
Eigen::Vector4d QuaternionToVectorWxyz(const Eigen::Quaterniond& q);

namespace internal {
// We test kinematic constraints on two robots: an IIWA robot and two free
// bodies. The IIWA test confirms that the bounds and the Eval function of each
// constraint computes the expected result. The two free bodies test confirms
// that the equations in Eval function semantically makes the two free bodies to
// satisfy the kinematic constraints.
class IiwaKinematicConstraintTest : public ::testing::Test {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaKinematicConstraintTest)

  IiwaKinematicConstraintTest();

  FrameIndex GetFrameIndex(const std::string& name) {
    // TODO(hongkai.dai): call GetFrameByName() directly.
    return iiwa_autodiff_.tree().GetFrameByName(name).index();
  }

 protected:
  std::unique_ptr<systems::Diagram<double>> diagram_{};
  MultibodyPlant<double>* plant_{};
  geometry::SceneGraph<double>* scene_graph_{};
  std::unique_ptr<systems::Context<double>> diagram_context_;
  systems::Context<double>* plant_context_;
  MultibodyTreeSystem<AutoDiffXd> iiwa_autodiff_;
  MultibodyTreeSystem<double> iiwa_double_;
  std::unique_ptr<systems::Context<AutoDiffXd>> context_autodiff_;
  std::unique_ptr<systems::Context<double>> context_double_;
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
  MultibodyTreeSystem<AutoDiffXd> two_bodies_autodiff_;
  MultibodyTreeSystem<double> two_bodies_double_;
  FrameIndex body1_index_;
  FrameIndex body2_index_;
  std::unique_ptr<systems::Context<AutoDiffXd>> context_autodiff_;
  std::unique_ptr<systems::Context<double>> context_double_;
};
}  // namespace internal
}  // namespace multibody
}  // namespace drake
