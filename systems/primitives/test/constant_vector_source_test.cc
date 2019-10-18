#include "drake/systems/primitives/constant_vector_source.h"

#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/test_utilities/my_vector.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

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
  }

  void SetUpBasicVectorModel() {
    MyVector3d vec;
    vec.get_mutable_value() << 42.0, 43.0, 44.0;

    source_ = make_unique<ConstantVectorSource<double>>(vec);
    context_ = source_->CreateDefaultContext();
  }

  const Matrix<double, 2, 1, Eigen::DontAlign> kConstantVectorSource{2.0, 1.5};
  std::unique_ptr<ConstantVectorSource<double>> source_;
  std::unique_ptr<Context<double>> context_;
};

// Tests that the output of the ConstantVectorSource is correct with an Eigen
// model of double.
TEST_F(ConstantVectorSourceTest, EigenModel) {
  SetUpEigenModel();
  ASSERT_EQ(source_->num_input_ports(), 0);
  ASSERT_EQ(source_->num_output_ports(), 1);

  EXPECT_TRUE(kConstantVectorSource.isApprox(
      source_->get_output_port().Eval(*context_),
      Eigen::NumTraits<double>::epsilon()));

  // Tests that the output reflects changes to the parameter value in the
  // context.
  BasicVector<double>& source_value =
      source_->get_mutable_source_value(context_.get());
  source_value.SetFromVector(2.0 * source_value.get_value());

  EXPECT_TRUE(kConstantVectorSource.isApprox(
      0.5 * source_->get_output_port().Eval(*context_),
      Eigen::NumTraits<double>::epsilon()));
}

// Tests that the output of the ConstantVectorSource is correct with a
// BasicVector<double> model.
TEST_F(ConstantVectorSourceTest, BasicVectorModel) {
  SetUpBasicVectorModel();
  ASSERT_EQ(source_->num_input_ports(), 0);
  ASSERT_EQ(source_->num_output_ports(), 1);

  EXPECT_EQ(43.0, source_->get_output_port().Eval<MyVector3d>(*context_)[1]);

  // Tests that the output reflects changes to the parameter value in the
  // context.
  BasicVector<double>& source_value =
      static_cast<ConstantVectorSource<double>*>(source_.get())
          ->get_mutable_source_value(context_.get());
  source_value.SetFromVector(2.0 * source_value.get_value());
  EXPECT_EQ(
      43.0, 0.5 * source_->get_output_port().Eval<MyVector3d>(*context_)[1]);
}

// Tests that ConstantVectorSource allocates no state variables in its context.
TEST_F(ConstantVectorSourceTest, ConstantVectorSourceIsStateless) {
  SetUpEigenModel();
  EXPECT_TRUE(context_->is_stateless());
}

TEST_F(ConstantVectorSourceTest, ToAutoDiffPass) {
  SetUpEigenModel();
  EXPECT_TRUE(is_autodiffxd_convertible(*source_, [&](const auto& converted) {
    EXPECT_EQ(0, converted.num_input_ports());
    EXPECT_EQ(1, converted.num_output_ports());
    EXPECT_EQ(2, converted.get_output_port().size());

    auto new_context = converted.CreateDefaultContext();
    const BasicVector<AutoDiffXd>& new_default =
        converted.get_source_value(*new_context);
    EXPECT_EQ(new_default.size(), kConstantVectorSource.rows());
    EXPECT_EQ(new_default[0].value(), kConstantVectorSource[0]);
    EXPECT_EQ(new_default[1].value(), kConstantVectorSource[1]);
    EXPECT_EQ(new_default[0].derivatives().size(), 0);
    EXPECT_EQ(new_default[1].derivatives().size(), 0);
  }));
}

TEST_F(ConstantVectorSourceTest, ToAutoDiffFail) {
  SetUpBasicVectorModel();
  // This is not yet supported.  It would be nice to do so, but getting the
  // model_value subtype converted to a different underlying scalar is not yet
  // supported by the BasicVector API.
  EXPECT_FALSE(is_autodiffxd_convertible(*source_));
}

TEST_F(ConstantVectorSourceTest, ToSymbolic) {
  SetUpEigenModel();
  EXPECT_TRUE(is_symbolic_convertible(*source_, [&](const auto& converted) {
    auto new_context = converted.CreateDefaultContext();
    const BasicVector<symbolic::Expression>& new_default =
        converted.get_source_value(*new_context);
    EXPECT_EQ(new_default[0], kConstantVectorSource[0]);
  }));
}

}  // namespace
}  // namespace systems
}  // namespace drake
