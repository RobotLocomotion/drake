#include "drake/systems/primitives/constant_vector_source.h"

#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/test_utilities/my_vector.h"

using Eigen::Matrix;
using std::make_unique;

namespace drake {
namespace systems {
namespace {

class ConstantVectorSourceTest : public ::testing::Test {
 protected:
  void SetUpEigenModel() {
    source_ = make_unique<ConstantVectorSource<double>>(kConstantVectorSource);
    context_ = source_->CreateDefaultContext();
    output_ = source_->AllocateOutput(*context_);
  }

  void SetUpBasicVectorModel() {
    MyVector<3, double> vec;
    vec.get_mutable_value() << 42.0, 43.0, 44.0;

    source_ = make_unique<ConstantVectorSource<double>>(vec);
    context_ = source_->CreateDefaultContext();
    output_ = source_->AllocateOutput(*context_);
  }

  const Matrix<double, 2, 1, Eigen::DontAlign> kConstantVectorSource{2.0, 1.5};
  std::unique_ptr<System<double>> source_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
};

// Tests that the output of the ConstantVectorSource is correct with an Eigen
// model of double.
TEST_F(ConstantVectorSourceTest, EigenModel) {
  SetUpEigenModel();
  ASSERT_EQ(source_->get_num_input_ports(), 0);
  ASSERT_EQ(source_->get_num_output_ports(), 1);

  source_->CalcOutput(*context_, output_.get());

  const BasicVector<double>* output_vector = output_->get_vector_data(0);
  ASSERT_NE(nullptr, output_vector);
  EXPECT_TRUE(kConstantVectorSource.isApprox(
      output_vector->get_value(), Eigen::NumTraits<double>::epsilon()));
}

// Tests that the output of the ConstantVectorSource is correct with a
// BasicVector<double> model.
TEST_F(ConstantVectorSourceTest, BasicVectorModel) {
  SetUpBasicVectorModel();
  ASSERT_EQ(source_->get_num_input_ports(), 0);
  ASSERT_EQ(source_->get_num_output_ports(), 1);

  source_->CalcOutput(*context_, output_.get());

  auto output_vector =
      dynamic_cast<const MyVector<3, double>*>(output_->get_vector_data(0));
  ASSERT_NE(nullptr, output_vector);
  EXPECT_EQ(43.0, output_vector->GetAtIndex(1));
}

// Tests that ConstantVectorSource allocates no state variables in its context.
TEST_F(ConstantVectorSourceTest, ConstantVectorSourceIsStateless) {
  SetUpEigenModel();
  EXPECT_TRUE(context_->is_stateless());
}

}  // namespace
}  // namespace systems
}  // namespace drake
