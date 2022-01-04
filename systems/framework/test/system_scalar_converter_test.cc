#include "drake/systems/framework/system_scalar_converter.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/is_dynamic_castable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

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
    this->DeclareVectorInputPort(kUseDefaultName, 1);
    this->DeclareVectorOutputPort(
        kUseDefaultName, 1,
        [this](const Context<T>& context, BasicVector<T>* output) {
          const auto& input = this->get_input_port(0).Eval(context);
          (*output)[0] = test::copysign_int_to_non_symbolic_scalar(
              this->magic(), input[0]);
        });
  }

  // Copy constructor that converts to a different scalar type.
  template <typename U>
  explicit NonSymbolicSystem(const NonSymbolicSystem<U>& other)
      : NonSymbolicSystem(other.magic()) {}

  int magic() const { return magic_; }

 private:
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

// A subclass of AnyToAnySystem.
template <typename T>
class SubclassOfAnyToAnySystem : public AnyToAnySystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SubclassOfAnyToAnySystem);

  // User constructor.
  SubclassOfAnyToAnySystem() : AnyToAnySystem<T>(22) {}

  // Copy constructor that converts to a different scalar type.
  template <typename U>
  explicit SubclassOfAnyToAnySystem(const SubclassOfAnyToAnySystem<U>&)
      : SubclassOfAnyToAnySystem() {}
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
  EXPECT_TRUE(dut.IsConvertible(typeid(T), typeid(U)));
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
  EXPECT_FALSE(dut.IsConvertible(typeid(T), typeid(U)));
  const S<U> original{0};
  const std::unique_ptr<System<T>> converted = dut.Convert<T, U>(original);

  // Confirm that nothing came out.
  EXPECT_TRUE(converted == nullptr);
}

GTEST_TEST(SystemScalarConverterTest, Empty) {
  const SystemScalarConverter dut1;
  const SystemScalarConverter dut2(SystemTypeTag<AnyToAnySystem>{});
  EXPECT_TRUE(dut1.empty());
  EXPECT_FALSE(dut2.empty());
}

GTEST_TEST(SystemScalarConverterTest, DefaualtConstructor) {
  // With the default ctor, nothing is convertible.
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

GTEST_TEST(SystemScalarConverterTest, SubclassMismatch) {
  // When correctly configured, converting the subclass type is successful.
  {
    SystemScalarConverter dut(SystemTypeTag<SubclassOfAnyToAnySystem>{});
    const SubclassOfAnyToAnySystem<double> original;
    const std::unique_ptr<System<AutoDiffXd>> converted =
        dut.Convert<AutoDiffXd, double>(original);
    EXPECT_NE(converted, nullptr);
  }

  // When incorrectly configured, converting the subclass type throws.
  {
    SystemScalarConverter dut(SystemTypeTag<AnyToAnySystem>{});
    const SubclassOfAnyToAnySystem<double> original;
    EXPECT_THROW(({
      try {
        dut.Convert<AutoDiffXd, double>(original);
      } catch (const std::runtime_error& e) {
        EXPECT_THAT(
            std::string(e.what()),
            testing::MatchesRegex(
                "SystemScalarConverter was configured to convert a "
                ".*::AnyToAnySystem<double> into a "
                ".*::AnyToAnySystem<drake::AutoDiffXd> but was called with a "
                ".*::SubclassOfAnyToAnySystem<double> at runtime"));
        throw;
      }
    }), std::runtime_error);
  }

  // However, if subtype checking is off, the conversion is allowed to upcast.
  {
    auto dut = SystemScalarConverter::MakeWithoutSubtypeChecking<
        AnyToAnySystem>();
    const SubclassOfAnyToAnySystem<double> original;
    EXPECT_TRUE(is_dynamic_castable<AnyToAnySystem<AutoDiffXd>>(
        dut.Convert<AutoDiffXd, double>(original)));
  }
}

GTEST_TEST(SystemScalarConverterTest, RemoveUnlessAlsoSupportedBy) {
  SystemScalarConverter dut(SystemTypeTag<AnyToAnySystem>{});
  const SystemScalarConverter non_symbolic(SystemTypeTag<NonSymbolicSystem>{});
  const SystemScalarConverter from_double(SystemTypeTag<FromDoubleSystem>{});

  // These are the defaults per TestAnyToAnySystem above.
  EXPECT_TRUE((dut.IsConvertible<double,     AutoDiffXd>()));
  EXPECT_TRUE((dut.IsConvertible<double,     Expression>()));
  EXPECT_TRUE((dut.IsConvertible<AutoDiffXd, double    >()));
  EXPECT_TRUE((dut.IsConvertible<AutoDiffXd, Expression>()));
  EXPECT_TRUE((dut.IsConvertible<Expression, double    >()));
  EXPECT_TRUE((dut.IsConvertible<Expression, AutoDiffXd>()));

  // We remove symbolic support.  Only double <=> AutoDiff remains.
  dut.RemoveUnlessAlsoSupportedBy(non_symbolic);
  EXPECT_TRUE((dut.IsConvertible< double,     AutoDiffXd>()));
  EXPECT_TRUE((dut.IsConvertible< AutoDiffXd, double    >()));
  EXPECT_FALSE((dut.IsConvertible<double,     Expression>()));
  EXPECT_FALSE((dut.IsConvertible<AutoDiffXd, Expression>()));
  EXPECT_FALSE((dut.IsConvertible<Expression, double    >()));
  EXPECT_FALSE((dut.IsConvertible<Expression, AutoDiffXd>()));

  // We remove from-AutoDiff support.  Only AutoDiff <= double remains.
  dut.RemoveUnlessAlsoSupportedBy(from_double);
  EXPECT_TRUE((dut.IsConvertible< AutoDiffXd, double    >()));
  EXPECT_FALSE((dut.IsConvertible<double,     AutoDiffXd>()));
  EXPECT_FALSE((dut.IsConvertible<double,     Expression>()));
  EXPECT_FALSE((dut.IsConvertible<AutoDiffXd, Expression>()));
  EXPECT_FALSE((dut.IsConvertible<Expression, double    >()));
  EXPECT_FALSE((dut.IsConvertible<Expression, AutoDiffXd>()));

  // The conversion still actually works.
  const AnyToAnySystem<double> system{22};
  EXPECT_TRUE(is_dynamic_castable<AnyToAnySystem<AutoDiffXd>>(
      dut.Convert<AutoDiffXd, double>(system)));
}

GTEST_TEST(SystemScalarConverterTest, Remove) {
  SystemScalarConverter dut(SystemTypeTag<AnyToAnySystem>{});

  // These are the defaults per TestAnyToAnySystem above.
  EXPECT_TRUE((dut.IsConvertible<double,     AutoDiffXd>()));
  EXPECT_TRUE((dut.IsConvertible<double,     Expression>()));
  EXPECT_TRUE((dut.IsConvertible<AutoDiffXd, double    >()));
  EXPECT_TRUE((dut.IsConvertible<AutoDiffXd, Expression>()));
  EXPECT_TRUE((dut.IsConvertible<Expression, double    >()));
  EXPECT_TRUE((dut.IsConvertible<Expression, AutoDiffXd>()));

  // We remove symbolic support.  Only double <=> AutoDiff remains.
  dut.Remove<double,     Expression>();
  dut.Remove<AutoDiffXd, Expression>();
  dut.Remove<Expression, double    >();
  dut.Remove<Expression, AutoDiffXd>();
  EXPECT_TRUE((dut.IsConvertible< double,     AutoDiffXd>()));
  EXPECT_FALSE((dut.IsConvertible<double,     Expression>()));
  EXPECT_TRUE((dut.IsConvertible< AutoDiffXd, double    >()));
  EXPECT_FALSE((dut.IsConvertible<AutoDiffXd, Expression>()));
  EXPECT_FALSE((dut.IsConvertible<Expression, double    >()));
  EXPECT_FALSE((dut.IsConvertible<Expression, AutoDiffXd>()));

  // The conversion still actually works.
  const AnyToAnySystem<double> system{22};
  EXPECT_TRUE(is_dynamic_castable<AnyToAnySystem<AutoDiffXd>>(
      dut.Convert<AutoDiffXd, double>(system)));
}

}  // namespace
}  // namespace systems
}  // namespace drake
