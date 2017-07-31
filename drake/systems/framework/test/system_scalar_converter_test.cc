#include "drake/systems/framework/system_scalar_converter.h"

#include <gtest/gtest.h>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace {

using symbolic::Expression;

// A system that has non-trivial non-T-typed member data.
// This system can convert between all scalar types.
template <typename T>
class AnyToAnySystem : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AnyToAnySystem);

  // User constructor.
  explicit AnyToAnySystem(int magic) : magic_(magic) {}

  // Copy constructor that converts to a different scalar type.
  template <typename U>
  explicit AnyToAnySystem(const AnyToAnySystem<U>& other)
      : AnyToAnySystem(other.magic()) {}

  int magic() const { return magic_; }

 private:
  int magic_{};
};

// A system that has non-trivial non-T-typed member data.
// This system does not support symbolic form.
template <typename T>
class NonSymbolicSystem : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NonSymbolicSystem);

  // User constructor.
  explicit NonSymbolicSystem(int magic)
      : magic_(magic) {
    // Declare input-output relation of `y[0] = copysign(magic, u[0])`.
    this->DeclareVectorInputPort(BasicVector<T>(1));
    this->DeclareVectorOutputPort(
        BasicVector<T>(1),
        [this](const Context<T>& context, BasicVector<T>* output) {
          const BasicVector<T>& input = *(this->EvalVectorInput(context, 0));
          (*output)[0] = magic_copysign(this->magic(), input[0]);
        });
  }

  // Copy constructor that converts to a different scalar type.
  template <typename U>
  explicit NonSymbolicSystem(const NonSymbolicSystem<U>& other)
      : NonSymbolicSystem(other.magic()) {}

  int magic() const { return magic_; }

 private:
  // A non-symbolic-compatible helper function.  If the SystemScalarConverter
  // implementation fails to avoid instantiating S<symbolic::Expression> when
  // told not to, then we would see compile errors that this function does not
  // compile with T = symbolic::Expression.
  static T magic_copysign(int magic, const T& value) {
    if ((magic < 0) == (value < 0)) {
      return value;
    } else {
      return -value;
    }
  }

  int magic_{};
};

// A system that has non-trivial non-T-typed member data.
// This system can only convert from a scalar type of double.
template <typename T>
class FromDoubleSystem : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FromDoubleSystem);

  // User constructor.
  explicit FromDoubleSystem(int magic) : magic_(magic) {}

  // Copy constructor that converts to a different scalar type.  Note that we
  // are not templated on U, since FromDoubleSystem can only come from double.
  // Note that we need a dummy argument in order to avoid conflicting with our
  // copy constructor.
  explicit FromDoubleSystem(
      const FromDoubleSystem<double>& other, int dummy = 0)
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

template <template <typename> class S, typename T, typename U, int kMagic>
void TestConversionPass() {
  // The device under test.
  const SystemScalarConverter dut(SystemTypeTag<S>{});

  // Do the conversion.
  EXPECT_TRUE((dut.IsConvertible<T, U>()));
  const S<U> original{kMagic};
  const std::unique_ptr<System<T>> converted = dut.Convert<T, U>(original);
  EXPECT_TRUE(converted != nullptr);

  // Confirm that the correct type came out.
  const auto* const downcast = dynamic_cast<const S<T>*>(converted.get());
  EXPECT_TRUE(downcast != nullptr);

  // Confirm that the magic was preserved.
  EXPECT_EQ(downcast ? downcast->magic() : 0, kMagic);
}

template <template <typename> class S, typename T, typename U>
void TestConversionFail() {
  const SystemScalarConverter dut(SystemTypeTag<S>{});

  // Do the conversion.
  EXPECT_FALSE((dut.IsConvertible<T, U>()));
  const S<U> original{0};
  const std::unique_ptr<System<T>> converted = dut.Convert<T, U>(original);

  // Confirm that nothing came out.
  EXPECT_TRUE(converted == nullptr);
}

GTEST_TEST(SystemScalarConverterTest, DefaualtConstructor) {
  // With the default ctor, nothing is convertible ...
  const SystemScalarConverter dut;
  EXPECT_FALSE((dut.IsConvertible<double,     double>()));
  EXPECT_FALSE((dut.IsConvertible<double,     AutoDiffXd>()));
  EXPECT_FALSE((dut.IsConvertible<double,     Expression>()));
  EXPECT_FALSE((dut.IsConvertible<AutoDiffXd, double>()));
  EXPECT_FALSE((dut.IsConvertible<AutoDiffXd, AutoDiffXd>()));
  EXPECT_FALSE((dut.IsConvertible<AutoDiffXd, Expression>()));
  EXPECT_FALSE((dut.IsConvertible<Expression, double>()));
  EXPECT_FALSE((dut.IsConvertible<Expression, AutoDiffXd>()));
  EXPECT_FALSE((dut.IsConvertible<Expression, Expression>()));

  // ... including non-standard scalar types.
  using AD2 = Eigen::AutoDiffScalar<Eigen::Vector2d>;
  EXPECT_FALSE((dut.IsConvertible<AD2, double>()));
  EXPECT_FALSE((dut.IsConvertible<double, AD2>()));
}

GTEST_TEST(SystemScalarConverterTest, TestAnyToAnySystem) {
  // We don't support T => T conversion -- that's the copy constructor, which
  // is usually disabled.
  TestConversionFail<AnyToAnySystem, double,     double>();
  TestConversionFail<AnyToAnySystem, AutoDiffXd, AutoDiffXd>();
  TestConversionFail<AnyToAnySystem, Expression, Expression>();

  // We support all remaining combinations of standard types.
  TestConversionPass<AnyToAnySystem, double,     AutoDiffXd, 1>();
  TestConversionPass<AnyToAnySystem, double,     Expression, 2>();
  TestConversionPass<AnyToAnySystem, AutoDiffXd, double,     3>();
  TestConversionPass<AnyToAnySystem, AutoDiffXd, Expression, 4>();
  TestConversionPass<AnyToAnySystem, Expression, double,     5>();
  TestConversionPass<AnyToAnySystem, Expression, AutoDiffXd, 6>();
}

GTEST_TEST(SystemScalarConverterTest, TestNonSymbolicSystem) {
  // The always-disabled cases.
  TestConversionFail<NonSymbolicSystem, double,     double>();
  TestConversionFail<NonSymbolicSystem, AutoDiffXd, AutoDiffXd>();

  // All Expression conversions are disabled.
  //
  // We cannot test TestConversionFail<foo, Expression>, because in order to
  // call Convert, we need a reference to the come-from type (Expression), and
  // instantiating a NonSymbolicSystem<Expression> does not even compile.
  // Instead, we just confirm that the converter claims to not support it.
  TestConversionFail<NonSymbolicSystem, Expression, double>();
  TestConversionFail<NonSymbolicSystem, Expression, AutoDiffXd>();
  const SystemScalarConverter dut(SystemTypeTag<NonSymbolicSystem>{});
  EXPECT_FALSE((dut.IsConvertible<double, Expression>()));
  EXPECT_FALSE((dut.IsConvertible<AutoDiffXd, Expression>()));

  // We support all remaining combinations of standard types.
  TestConversionPass<NonSymbolicSystem, double,     AutoDiffXd, 1>();
  TestConversionPass<NonSymbolicSystem, AutoDiffXd, double,     2>();
}

GTEST_TEST(SystemScalarConverterTest, TestFromDoubleSystem) {
  // The always-disabled cases.
  TestConversionFail<FromDoubleSystem, double,     double>();
  TestConversionFail<FromDoubleSystem, AutoDiffXd, AutoDiffXd>();
  TestConversionFail<FromDoubleSystem, Expression, Expression>();

  // The from-non-double conversions are disabled.
  TestConversionFail<FromDoubleSystem, double,     Expression>();
  TestConversionFail<FromDoubleSystem, AutoDiffXd, Expression>();
  TestConversionFail<FromDoubleSystem, Expression, AutoDiffXd>();
  TestConversionFail<FromDoubleSystem, double,     AutoDiffXd>();

  // We support all remaining combinations of standard types.
  TestConversionPass<FromDoubleSystem, Expression, double, 1>();
  TestConversionPass<FromDoubleSystem, AutoDiffXd, double, 2>();
}

GTEST_TEST(SystemScalarConverterTest, TestUserTypes) {
  // The device under test.
  SystemScalarConverter dut(SystemTypeTag<AnyToAnySystem>{});

  // We don't (by default) support non-standard types.
  using AD2 = Eigen::AutoDiffScalar<Eigen::Vector2d>;
  TestConversionFail<AnyToAnySystem, AD2, double>();

  // The user can opt-in to non-standard types.
  dut.AddIfSupported<AnyToAnySystem, AD2, double>();

  // Do the conversion.
  const AnyToAnySystem<double> original{0};
  const std::unique_ptr<System<AD2>> converted =
      dut.Convert<AD2, double>(original);
  EXPECT_TRUE(converted != nullptr);

  // Confirm that the correct type came out.
  const auto* const downcast =
      dynamic_cast<const AnyToAnySystem<AD2>*>(converted.get());
  EXPECT_TRUE(downcast != nullptr);
}

}  // namespace
}  // namespace systems
}  // namespace drake
