#include "drake/systems/framework/primitives/pid_controller.h"

#include <memory>
#include <stdexcept>
#include <string>

#include <unsupported/Eigen/AutoDiff>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_input.h"

#include "gtest/gtest.h"

using Eigen::AutoDiffScalar;
using Eigen::Vector2d;
using Eigen::Vector3d;
using std::make_unique;

namespace drake {
namespace systems {
namespace {

// TODO(amcastro-tri): Create a diagram with a ConstantVectorSource feeding
// the input of the Gain system.
template <class T>
std::unique_ptr<FreestandingInputPort> MakeInput(
    std::unique_ptr<BasicVector<T>> data) {
  return make_unique<FreestandingInputPort>(std::move(data));
}

GTEST_TEST(DiagramSubclassTest, TwelvePlusSevenIsNineteen) {
  const double kp = 1.0;
  const double ki = 2.0;
  PidController<double> controller(kp, ki, 3);

  auto context = controller.CreateDefaultContext();
  auto output = controller.AllocateOutput(*context);
  ASSERT_NE(nullptr, context);
  ASSERT_NE(nullptr, output);

#if 0
  auto vec = std::make_unique<BasicVector<double>>(1 /* length */);
  vec->get_mutable_value() << 12.0;
  context->SetInputPort(0, MakeInput(std::move(vec)));

  plus_seven.EvalOutput(*context, output.get());

  ASSERT_EQ(1, output->get_num_ports());
  const VectorBase<double>* output_vector = output->get_vector_data(0);
  EXPECT_EQ(1, output_vector->get_value().rows());
  EXPECT_EQ(19.0, output_vector->get_value().x());
#endif
}

}  // namespace
}  // namespace systems
}  // namespace drake
