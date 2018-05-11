#include "drake/systems/framework/single_output_vector_source.h"

#include <memory>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace systems {
namespace {

const int kSize = 3;

class TestSource : public SingleOutputVectorSource<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestSource)

  TestSource() : SingleOutputVectorSource<double>(kSize) {}

  // N.B. This method signature might be used by many downstream projects.
  // Change it only with good reason and with a deprecation period first.
  void DoCalcVectorOutput(
      const Context<double>& context,
      Eigen::VectorBlock<Eigen::VectorXd>* output) const final {
    *output = Eigen::Vector3d::Ones();
  }
};

class SingleOutputVectorSourceTest : public ::testing::Test {
 protected:
  void SetUp() override {
    source_ = std::make_unique<TestSource>();
    context_ = source_->CreateDefaultContext();
    output_ = source_->AllocateOutput(*context_);
  }

  std::unique_ptr<System<double>> source_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
};

// Tests that the output is correct.
TEST_F(SingleOutputVectorSourceTest, OutputTest) {
  ASSERT_EQ(context_->get_num_input_ports(), 0);
  ASSERT_EQ(output_->get_num_ports(), 1);

  std::unique_ptr<AbstractValue> output =
      source_->get_output_port(0).Allocate();
  source_->get_output_port(0).Calc(*context_, output.get());

  const auto& output_vector = output->GetValueOrThrow<BasicVector<double>>();
  EXPECT_EQ(output_vector.get_value(), Eigen::Vector3d::Ones());
}

// Tests that the state is empty.
TEST_F(SingleOutputVectorSourceTest, IsStateless) {
  EXPECT_EQ(context_->get_continuous_state().size(), 0);
}

// Some tag types used to select which constructor gets called.
struct UseTransmogrify {};
struct UseVector {};

template <typename T>
class Convertable final : public SingleOutputVectorSource<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Convertable)

  using Base = SingleOutputVectorSource<T>;
  using Tag = SystemTypeTag<systems::Convertable>;

  Convertable() : Base(kSize) {}
  explicit Convertable(UseVector) : Base(*MakeVec()) {}
  explicit Convertable(UseTransmogrify) : Base(Tag{}, kSize) {}
  Convertable(UseTransmogrify, UseVector) : Base(Tag{}, *MakeVec()) {}

  template <typename U>
  explicit Convertable(const Convertable<U>&)
      : Convertable<T>(UseTransmogrify{}) {}

 private:
  auto MakeVec() const {
    return std::make_unique<BasicVector<T>>(VectorX<T>::Constant(kSize, 22.0));
  }

  void DoCalcVectorOutput(
      const Context<T>& context,
      Eigen::VectorBlock<VectorX<T>>* output) const final {
    *output = Eigen::Vector3d::Ones();
  }
};

GTEST_TEST(SingleOutputVectorSourceConvertableTest, ScalarTypes) {
  // The constructors without SystemScalarConverter do not support conversion.
  const Convertable<double> false1;
  const Convertable<double> false2{UseVector{}};
  EXPECT_FALSE(is_autodiffxd_convertible(false1));
  EXPECT_FALSE(is_autodiffxd_convertible(false2));
  EXPECT_FALSE(is_symbolic_convertible(false1));
  EXPECT_FALSE(is_symbolic_convertible(false2));

  // The constructors with SystemScalarConverter do support conversion.
  const Convertable<double> true1{UseTransmogrify{}};
  const Convertable<double> true2{UseTransmogrify{}, UseVector{}};
  EXPECT_TRUE(is_autodiffxd_convertible(true1));
  EXPECT_TRUE(is_autodiffxd_convertible(true2));
  EXPECT_TRUE(is_symbolic_convertible(true1));
  EXPECT_TRUE(is_symbolic_convertible(true2));
}

}  // namespace
}  // namespace systems
}  // namespace drake
