#include "drake/systems/framework/system_scalar_converter.h"

#include <gtest/gtest.h>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace {

// A System that has non-trivial non-T-typed member data and can convert into
// *anything*.
template <typename T>
class AnyToAnySystem : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AnyToAnySystem);

  // User constructor.
  explicit AnyToAnySystem(int magic)
      : LeafSystem<T>(SystemTypeTag<systems::AnyToAnySystem>{}),
        magic_(magic) {}

  // Transmogrification constructor.
  template <typename U>
  explicit AnyToAnySystem(const AnyToAnySystem<U>& other)
      : AnyToAnySystem(other.magic()) {}

  int magic() const { return magic_; }

 private:
  int magic_{};
};

// A System that has non-trivial non-T-typed member data and and *cannot*
// convert into symbolic form.
template <typename T>
class NonSymbolicSystem : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NonSymbolicSystem);

  // User constructor.
  explicit NonSymbolicSystem(int magic)
      : LeafSystem<T>(SystemTypeTag<systems::NonSymbolicSystem>{}),
        magic_(magic) {
    // Declare input-output relation of `y[0] = copysign(magic, u[0])`.
    this->DeclareVectorInputPort(BasicVector<T>(1));
    this->DeclareVectorOutputPort(
        BasicVector<T>(1),
        [this](const Context<T>& context, BasicVector<T>* output) {
          const BasicVector<T>& input = *(this->EvalVectorInput(context, 0));
          (*output)[0] = magic_copysign(this->magic(), input[0]);
        });
  }

  // Transmogrification constructor.
  template <typename U>
  explicit NonSymbolicSystem(const NonSymbolicSystem<U>& other)
      : NonSymbolicSystem(other.magic()) {}

  int magic() const { return magic_; }

 private:
  // A non-symbolic-compatible helper function.  If the transmogrification
  // implementation fails to avoid instantiating System<symbolic::Expression>
  // when told not to, then we would see compile errors that this function does
  // not compile with T = symbolic::Expression.
  static T magic_copysign(int magic, const T& value) {
    if ((magic < 0) == (value < 0)) {
      return value;
    } else {
      return -value;
    }
  }

  int magic_{};
};

// A System that has non-trivial non-T-typed member data and can convert
// only *from* double form.
template <typename T>
class FromDoubleSystem : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FromDoubleSystem);

  // User constructor.
  explicit FromDoubleSystem(int magic)
      : LeafSystem<T>(SystemTypeTag<systems::FromDoubleSystem>{}),
        magic_(magic) {}

  // Transmogrification constructor.  Note that we are *not* templated on U,
  // since FromDoubleSystem can only come from double.  Note that we need a
  // dummy argument in order to avoid conflicting with our copy constructor.
  explicit FromDoubleSystem(
      const FromDoubleSystem<double>& other,
      int dummy = 0)
      : FromDoubleSystem(other.magic()) {}

  int magic() const { return magic_; }

 private:
  int magic_{};
};


}  // namespace

namespace scalar_conversion {
template <> struct Traits<NonSymbolicSystem> : public NonSymbolicTraits {};
template <> struct Traits<FromDoubleSystem> : public FromDoubleTraits {};
}  // namespace scalar_conversion

namespace {

GTEST_TEST(SystemScalarConverterTest, TestAnyToAnySystem) {
  const AnyToAnySystem<double> system_a_double{22};
  const SystemScalarConverter dut(SystemTypeTag<AnyToAnySystem>{});

  // Convert to AutoDiffXd.
  const std::unique_ptr<System<AutoDiffXd>> system_autodiffxd =
      dut.Convert<AutoDiffXd, double>(system_a_double);
  ASSERT_TRUE(system_autodiffxd != nullptr);

  // Extract the subtype.
  const auto* system_a_autodiffxd =
      dynamic_cast<const AnyToAnySystem<AutoDiffXd>*>(system_autodiffxd.get());
  ASSERT_TRUE(system_a_autodiffxd != nullptr);

  // Confirm that the magic was preserved.
  EXPECT_EQ(system_a_autodiffxd->magic(), system_a_double.magic());

  // Convert to symbolic.
  const std::unique_ptr<System<symbolic::Expression>> system_symbolic =
      dut.Convert<symbolic::Expression, double>(system_a_double);
  ASSERT_TRUE(system_symbolic != nullptr);

  // Extract the subtype.
  const auto* system_a_symbolic =
      dynamic_cast<const AnyToAnySystem<symbolic::Expression>*>(
          system_symbolic.get());
  ASSERT_TRUE(system_a_symbolic != nullptr);

  // Confirm that the magic was preserved.
  EXPECT_EQ(system_a_symbolic->magic(), system_a_double.magic());
}

GTEST_TEST(SystemScalarConverterTest, TestNonSymbolicSystem) {
  const NonSymbolicSystem<double> system_b_double{22};
  const SystemScalarConverter dut(SystemTypeTag<NonSymbolicSystem>{});

  // Convert to AutoDiffXd.
  const std::unique_ptr<System<AutoDiffXd>> system_autodiffxd =
      dut.Convert<AutoDiffXd, double>(system_b_double);
  ASSERT_TRUE(system_autodiffxd != nullptr);

  // Extract the subtype.
  const auto* system_b_autodiffxd =
      dynamic_cast<const NonSymbolicSystem<AutoDiffXd>*>(
          system_autodiffxd.get());
  ASSERT_TRUE(system_b_autodiffxd != nullptr);

  // Confirm that the magic was preserved.
  EXPECT_EQ(system_b_autodiffxd->magic(), system_b_double.magic());

  // Convert to symbolic should not work.
  const std::unique_ptr<System<symbolic::Expression>> system_symbolic =
      dut.Convert<symbolic::Expression, double>(system_b_double);
  ASSERT_TRUE(system_symbolic == nullptr);
}

GTEST_TEST(SystemScalarConverterTest, TestFromDoubleSystem) {
  const FromDoubleSystem<double> system_c_double{22};
  const SystemScalarConverter dut(SystemTypeTag<FromDoubleSystem>{});

  // Convert to AutoDiffXd.
  const std::unique_ptr<System<AutoDiffXd>> system_autodiffxd =
      dut.Convert<AutoDiffXd, double>(system_c_double);
  ASSERT_TRUE(system_autodiffxd != nullptr);

  // Extract the subtype.
  const auto* system_c_autodiffxd =
      dynamic_cast<const FromDoubleSystem<AutoDiffXd>*>(
          system_autodiffxd.get());
  ASSERT_TRUE(system_c_autodiffxd != nullptr);

  // Confirm that the magic was preserved.
  EXPECT_EQ(system_c_autodiffxd->magic(), system_c_double.magic());
}

}  // namespace
}  // namespace systems
}  // namespace drake
