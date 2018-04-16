#include "drake/systems/primitives/barycentric_system.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/barycentric.h"
#include "drake/systems/primitives/matrix_gain.h"

namespace drake {
namespace systems {
namespace {

using Eigen::Vector2d;

// Use a BarycentricSystem to reproduce a MatrixGain.
GTEST_TEST(BarycentricSystemTest, MatrixGain) {
  // Create a small MatrixGain system.
  Eigen::Matrix2d A;
  A << 1., 2., 3., 4;
  MatrixGain<double> gain(A);

  // Create a BarycentricMesh in 2d.
  math::BarycentricMesh<double> bary({{-1., 1.},  // BR
                                      {-1., 1.}});

  // Compute mesh values to match the desired output of the MatrixGain.
  Eigen::MatrixXd values =
      bary.MeshValuesFrom([A](const auto& x) { return A * x; });

  // Quick sanity check of the barycentric mesh.
  Vector2d test{-.5, .5}, test_output;
  bary.Eval(values, test, &test_output);
  EXPECT_TRUE(CompareMatrices(test_output, A * test, 1e-8));

  // Create the new system.
  BarycentricMeshSystem<double> bary_sys(std::move(bary), values);

  // Check to make sure the two systems compute the same outputs (when the
  // inputs are inside the grid).
  auto gain_context = gain.CreateDefaultContext();
  auto bary_context = bary_sys.CreateDefaultContext();

  auto gain_output = gain.get_output_port().Allocate();
  auto bary_output = bary_sys.get_output_port().Allocate();
  for (double x : {-1., -.5, .3, .7}) {
    for (double y : {-.24, .4, .8}) {
      test = Vector2d{x, y};
      gain_context->FixInputPort(0, test);
      gain.get_output_port().Calc(*gain_context, gain_output.get());

      bary_context->FixInputPort(0, test);
      bary_sys.get_output_port().Calc(*bary_context, bary_output.get());

      EXPECT_TRUE(CompareMatrices(
          gain_output->GetValue<BasicVector<double>>().CopyToVector(), A * test,
          1e-8));

      EXPECT_TRUE(CompareMatrices(
          gain_output->GetValue<BasicVector<double>>().CopyToVector(),
          bary_output->GetValue<BasicVector<double>>().CopyToVector(), 1e-8));
    }
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake
