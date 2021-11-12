#include "drake/common/value.h"

#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/drake_copyable.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/framework/test_utilities/my_vector.h"

namespace drake {
namespace test {

// A type with no constructors.
struct BareStruct {
  int data;
};

// A copyable type with no default constructor.
struct CopyableInt {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CopyableInt);
  explicit CopyableInt(int i) : data{i} {}
  CopyableInt(int c1, int c2) : data{c1 * c2} {}

  int data;
};

// A clone-only type with no default constructor.
struct CloneableInt {
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CloneableInt);
  explicit CloneableInt(int i) : data{i} {}

  std::unique_ptr<CloneableInt> Clone() const {
    return std::make_unique<CloneableInt>(data);
  }

  const int data;
};

// A move-or-clone (not copy) type with a default constructor.
struct MoveOrCloneInt {
  MoveOrCloneInt() {}
  explicit MoveOrCloneInt(int i) : data{i} {}
  MoveOrCloneInt(MoveOrCloneInt&& other) {
    std::swap(data, other.data);
  }
  MoveOrCloneInt& operator=(MoveOrCloneInt&& other)  {
    std::swap(data, other.data);
    return *this;
  }
  MoveOrCloneInt(const MoveOrCloneInt&) = delete;
  void operator=(const MoveOrCloneInt&) = delete;

  std::unique_ptr<MoveOrCloneInt> Clone() const {
    return std::make_unique<MoveOrCloneInt>(data);
  }

  int data{};
};

// Helper for EXPECT_EQ to unwrap the data field.
template <typename T>
bool operator==(int i, const T& value) { return i == value.data; }

using systems::BasicVector;
using systems::MyVector2d;

// Boilerplate for tests that are identical across different types.  Our
// TYPED_TESTs will run using all of the below types as the TypeParam.
template <typename TypeParam> class TypedValueTest : public ::testing::Test {};
typedef ::testing::Types<
    int,
    CopyableInt,
    CloneableInt,
    MoveOrCloneInt
    > Implementations;
TYPED_TEST_SUITE(TypedValueTest, Implementations);

// Value<T>() should work if and only if T is default-constructible.
GTEST_TEST(ValueTest, DefaultConstructor) {
  const AbstractValue& value_int = Value<int>();
  EXPECT_EQ(0, value_int.get_value<int>());

  const AbstractValue& value_bare_struct = Value<BareStruct>();
  EXPECT_EQ(0, value_bare_struct.get_value<BareStruct>().data);

  static_assert(!std::is_default_constructible_v<Value<CopyableInt>>,
                "Value<CopyableInt>() should not work.");

  static_assert(!std::is_default_constructible_v<Value<CloneableInt>>,
                "Value<CloneableInt>() should not work.");

  const AbstractValue& value_move_or_clone_int = Value<MoveOrCloneInt>();
  EXPECT_EQ(0, value_move_or_clone_int.get_value<MoveOrCloneInt>().data);
}

// Value<T>(int) should work (possibly using forwarding).
TYPED_TEST(TypedValueTest, ForwardingConstructor) {
  using T = TypeParam;
  const AbstractValue& abstract_value = Value<T>(22);
  EXPECT_EQ(22, abstract_value.get_value<T>());
}

// A two-argument constructor should work using forwarding.  (The forwarding
// test case above is not quite enough, because the Value implementation treats
// the first argument and rest of the arguments separately.)
GTEST_TEST(ValueTest, ForwardingConstructorTwoArgs) {
  using T = CopyableInt;
  const AbstractValue& value = Value<T>(11, 2);
  EXPECT_EQ(22, value.get_value<T>());
}

// Passing a single reference argument to the Value<T> constructor should use
// the `(const T&)` constructor, not the forwarding constructor.
TYPED_TEST(TypedValueTest, CopyConstructor) {
  using T = TypeParam;
  T param{0};
  const T const_param{0};
  const Value<T> xvalue(T{0});          // Called with `T&&`.
  const Value<T> lvalue(param);         // Called with `T&`.
  const Value<T> crvalue(const_param);  // Called with `const T&`.
}

// Ditto for BareStruct.
GTEST_TEST(ValueTest, BareCopyConstructor) {
  using T = BareStruct;
  T param{};
  const T const_param{};
  const Value<T> xvalue(T{});           // Called with `T&&`.
  const Value<T> lvalue(param);         // Called with `T&`.
  const Value<T> crvalue(const_param);  // Called with `const T&`.
}

// Passing a unique_ptr<T> to Value<T> should take over the value.
TYPED_TEST(TypedValueTest, UniquePtrConstructor) {
  using T = TypeParam;
  auto original = std::make_unique<T>(22);
  const Value<T> value{std::move(original)};
  EXPECT_EQ(original.get(), nullptr);
  EXPECT_EQ(22, value.get_value());
}

TYPED_TEST(TypedValueTest, Make) {
  using T = TypeParam;
  // TODO(jwnimmer-tri) We should be able to forward this too, and lose the
  // explicit construction of T{42}.
  auto abstract_value = AbstractValue::Make<T>(T{42});
  EXPECT_EQ(42, abstract_value->template get_value<T>());
}

GTEST_TEST(TypedValueTest, MakeDefault) {
  EXPECT_EQ(0, AbstractValue::Make<int>()->get_value<int>());
  EXPECT_EQ("", AbstractValue::Make<std::string>()->get_value<std::string>());
}

GTEST_TEST(ValueTest, NiceTypeName) {
  auto double_value = AbstractValue::Make<double>(3.);
  auto string_value = AbstractValue::Make<std::string>("hello");
  auto base_value =
      std::make_unique<Value<BasicVector<double>>>(MyVector2d::Make(1., 2.));

  EXPECT_EQ(double_value->GetNiceTypeName(), "double");
  EXPECT_EQ(string_value->GetNiceTypeName(), "std::string");

  // Must return the name of the most-derived type.
  EXPECT_EQ(base_value->GetNiceTypeName(),
            "drake::systems::MyVector<double,2>");
}

GTEST_TEST(ValueTest, TypeInfo) {
  auto double_value = AbstractValue::Make<double>(3.);
  auto string_value = AbstractValue::Make<std::string>("hello");
  auto base_value =
      std::make_unique<Value<BasicVector<double>>>(MyVector2d::Make(1., 2.));

  EXPECT_EQ(double_value->static_type_info(), typeid(double));
  EXPECT_EQ(double_value->type_info(), typeid(double));

  EXPECT_EQ(string_value->static_type_info(), typeid(std::string));
  EXPECT_EQ(string_value->type_info(), typeid(std::string));

  // The static type is BasicVector, but the runtime type is MyVector2d.
  EXPECT_EQ(base_value->static_type_info(), typeid(BasicVector<double>));
  EXPECT_EQ(base_value->type_info(), typeid(MyVector2d));
}

// Check that maybe_get_value() returns nullptr for wrong-type requests,
// and returns the correct value for right-type requests.
GTEST_TEST(ValueTest, MaybeGetValue) {
  auto double_value = AbstractValue::Make<double>(3.);
  auto string_value = AbstractValue::Make<std::string>("hello");

  EXPECT_EQ(double_value->maybe_get_value<std::string>(), nullptr);
  EXPECT_EQ(string_value->maybe_get_value<double>(), nullptr);

  const double* const double_pointer =
      double_value->maybe_get_value<double>();
  const std::string* const string_pointer =
      string_value->maybe_get_value<std::string>();

  ASSERT_NE(double_pointer, nullptr);
  ASSERT_NE(string_pointer, nullptr);
  EXPECT_EQ(*double_pointer, 3.);
  EXPECT_EQ(*string_pointer, "hello");
}

// Check that maybe_get_mutable_value() returns nullptr for wrong-type
// requests, and returns the correct value for right-type requests.
GTEST_TEST(ValueTest, MaybeGetMutableValue) {
  auto double_value = AbstractValue::Make<double>(3.);
  auto string_value = AbstractValue::Make<std::string>("hello");

  EXPECT_EQ(double_value->maybe_get_mutable_value<std::string>(), nullptr);
  EXPECT_EQ(string_value->maybe_get_mutable_value<double>(), nullptr);

  double* const double_pointer =
      double_value->maybe_get_mutable_value<double>();
  std::string* const string_pointer =
      string_value->maybe_get_mutable_value<std::string>();

  ASSERT_NE(double_pointer, nullptr);
  ASSERT_NE(string_pointer, nullptr);
  EXPECT_EQ(*double_pointer, 3.);
  EXPECT_EQ(*string_pointer, "hello");

  *string_pointer = "goodbye";
  EXPECT_EQ(string_value->get_value<std::string>(), "goodbye");
}

TYPED_TEST(TypedValueTest, Access) {
  using T = TypeParam;
  Value<T> value(3);
  const AbstractValue& erased = value;

  EXPECT_EQ(3, erased.get_value<T>());
  ASSERT_NE(erased.maybe_get_value<T>(), nullptr);
  EXPECT_EQ(3, *erased.maybe_get_value<T>());
}

TYPED_TEST(TypedValueTest, Clone) {
  using T = TypeParam;
  Value<T> value(43);
  const AbstractValue& erased = value;
  std::unique_ptr<AbstractValue> cloned = erased.Clone();
  EXPECT_EQ(43, cloned->get_value<T>());
}

TYPED_TEST(TypedValueTest, Mutation) {
  using T = TypeParam;
  Value<T> value(3);
  value.set_value(T{4});
  AbstractValue& erased = value;

  EXPECT_EQ(4, erased.get_value<T>());
  erased.set_value<T>(T{5});
  EXPECT_EQ(5, erased.get_value<T>());
  erased.SetFrom(Value<T>(6));
  EXPECT_EQ(6, erased.get_value<T>());
}

TYPED_TEST(TypedValueTest, BadCast) {
  using T = TypeParam;
  Value<double> value(4);
  AbstractValue& erased = value;

  EXPECT_THROW(erased.get_value<T>(), std::logic_error);
  EXPECT_THROW(erased.get_mutable_value<T>(), std::logic_error);
  EXPECT_THROW(erased.set_value<T>(T{3}), std::logic_error);
  EXPECT_THROW(erased.SetFrom(Value<T>(2)), std::logic_error);
}

class PrintInterface {
 public:
  virtual ~PrintInterface() {}
  virtual std::string print() const = 0;
 protected:
  // Allow our subclasses to make these public.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PrintInterface)
  PrintInterface() = default;
};

// A trivial class that implements a trivial interface.
//
// N.B. Don't use this precise example in your own code!  Normally we would
// mark this class `final` in order to avoid the slicing problem during copy,
// move, and assignment; however, the unit tests below are specifically
// checking for weird corner cases, so we can't mark it as such here.
class Point : public PrintInterface {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Point)

  Point(int x, int y) : x_(x), y_(y) {}
  virtual ~Point() {}

  int x() const { return x_; }
  int y() const { return y_; }
  void set_x(int x) { x_ = x; }
  void set_y(int y) { y_ = y; }

  std::string print() const override {
    std::ostringstream out;
    out << x_ << "," << y_;
    return out.str();
  }

 private:
  int x_, y_;
};

// Tests that classes can be erased in an AbstractValue.
GTEST_TEST(ValueTest, ClassType) {
  Point point(1, 2);
  Value<Point> value(point);
  AbstractValue& erased = value;
  erased.get_mutable_value<Point>().set_x(-1);
  EXPECT_EQ(-1, erased.get_value<Point>().x());
  EXPECT_EQ(2, erased.get_value<Point>().y());
  erased.get_mutable_value<Point>().set_y(-2);
  EXPECT_EQ(-1, erased.get_value<Point>().x());
  EXPECT_EQ(-2, erased.get_value<Point>().y());
}

class SubclassOfPoint : public Point {
 public:
  SubclassOfPoint() : Point(-1, -2) {}
};

// Tests that attempting to unerase an AbstractValue to a parent class of the
// original class throws std::logic_error.
GTEST_TEST(ValueTest, CannotUneraseToParentClass) {
  SubclassOfPoint point;
  Value<SubclassOfPoint> value(point);
  AbstractValue& erased = value;
  EXPECT_THROW(erased.get_mutable_value<Point>(), std::logic_error);
}

// A child class of Value<T> that requires T to satisfy PrintInterface, and
// also satisfies PrintInterface itself.
template <typename T>
class PrintableValue : public Value<T>, public PrintInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PrintableValue)

  explicit PrintableValue(const T& v) : Value<T>(v) {}

  std::unique_ptr<AbstractValue> Clone() const override {
    return std::make_unique<PrintableValue<T>>(this->get_value());
  }

  std::string print() const override {
    const PrintInterface& print_interface = Value<T>::get_value();
    return print_interface.print();
  }
};

// Tests that AbstractValues can be unerased to interfaces implemented by
// subclasses of Value<T>.
GTEST_TEST(ValueTest, SubclassOfValue) {
  Point point(3, 4);
  PrintableValue<Point> printable_value(point);
  AbstractValue* erased = &printable_value;
  PrintInterface* printable_erased = dynamic_cast<PrintInterface*>(erased);
  ASSERT_NE(nullptr, printable_erased);
  EXPECT_EQ("3,4", printable_erased->print());
}

// Tests that even after being cloned, PrintableValue can be unerased to
// PrintInterface.
GTEST_TEST(ValueTest, SubclassOfValueSurvivesClone) {
  Point point(5, 6);
  PrintableValue<Point> printable_value(point);
  const AbstractValue& erased = printable_value;
  std::unique_ptr<AbstractValue> cloned = erased.Clone();
  PrintInterface* printable_erased =
      dynamic_cast<PrintInterface*>(cloned.get());
  ASSERT_NE(nullptr, printable_erased);
  EXPECT_EQ("5,6", printable_erased->print());
}

// Tests an allowed type, and shows (by commented out examples) that pointers,
// arrays, const, volatile, and reference types should be forbidden.
GTEST_TEST(ValueTest, AllowedTypesMetaTest) {
  using T = int;
  Value<T>{};
  // - cvref
  // Value<const T>{};  // Triggers static assertion; fails without assertion.
  // Value<volatile T>{};  // Trigger static assertion; works without assertion.
  // Value<const T&>{};  // Triggers static assertion; fails without assertion.
  // Value<T&&>{};  // Triggers static assertion; fails without assertion.
  // - array / pointer
  // Value<T*>{};  // Triggers static assertion; works without assertion.
  // Value<T[2]>{};  // Triggers static assertion; fails without assertion.
}

// Check that TypeHash is extracting exactly the right strings from
// __PRETTY_FUNCTION__.
template <typename T>
void CheckHash(const std::string& name) {
  internal::FNV1aHasher hasher;
  hasher(name.data(), name.size());
  EXPECT_EQ(internal::TypeHash<T>::value, size_t(hasher))
      << "  for name\n"
      << "    Which is: " << name << "\n"
      << "  for __PRETTY_FUNCTION__\n"
      << "    Which is: " << __PRETTY_FUNCTION__;
}

namespace {

struct AnonStruct {};
class AnonClass {};
enum class AnonEnum { kFoo, kBar };

// A class with a non-type template argument is not hashable.
template <AnonEnum K>
class UnadornedAnonEnumTemplate {};

// To enable hashing, the user can add a `using` statement like this.
template <AnonEnum K>
class NiceAnonEnumTemplate {
 public:
  using NonTypeTemplateParameter = std::integral_constant<AnonEnum, K>;
};

}  // namespace

// Apple clang prior to version 13 needs a fixup for inline namespaces.
#if defined(__APPLE__) && defined(__clang__) && __clang_major__ < 13
constexpr bool kAppleInlineNamespace = true;
#else
constexpr bool kAppleInlineNamespace = false;
#endif

#ifdef __clang__
constexpr bool kClang = true;
#else
constexpr bool kClang = false;
#endif

#if __GNUC__ >= 9
constexpr bool kGcc9 = true;
#else
constexpr bool kGcc9 = false;
#endif

GTEST_TEST(TypeHashTest, WellKnownValues) {
  // Simple primitives, structs, and classes.
  CheckHash<int>("int");
  CheckHash<double>("double");
  CheckHash<Point>("drake::test::Point");

  // Anonymous structs and classes, and an enum class.
  CheckHash<AnonStruct>("drake::test::{anonymous}::AnonStruct");
  CheckHash<AnonClass>("drake::test::{anonymous}::AnonClass");
  CheckHash<AnonEnum>("drake::test::{anonymous}::AnonEnum");

  // Templated containers without default template arguments.
  const std::string stdcc = kAppleInlineNamespace ? "std::__1" : "std";
  CheckHash<std::shared_ptr<double>>(fmt::format(
      "{std}::shared_ptr<double>",
      fmt::arg("std", stdcc)));
  CheckHash<std::pair<int, double>>(fmt::format(
      "{std}::pair<int,double>",
      fmt::arg("std", stdcc)));

  // Templated classes *with* default template arguments.
  CheckHash<std::vector<double>>(fmt::format(
      "{std}::vector<double,{std}::allocator<double>>",
      fmt::arg("std", stdcc)));
  CheckHash<std::vector<BasicVector<double>>>(fmt::format(
      "{std}::vector<"
        "drake::systems::BasicVector<double>,"
        "{std}::allocator<drake::systems::BasicVector<double>>"
      ">", fmt::arg("std", stdcc)));

  // Const-qualified types.
  CheckHash<std::shared_ptr<const double>>(fmt::format(
      "{std}::shared_ptr<const double>",
      fmt::arg("std", stdcc)));

  // Eigen classes.
  CheckHash<Eigen::VectorXd>(
      "Eigen::Matrix<double,int=-1,int=1,int=0,int=-1,int=1>");
  CheckHash<Eigen::MatrixXd>(
      "Eigen::Matrix<double,int=-1,int=-1,int=0,int=-1,int=-1>");
  CheckHash<Eigen::Vector3d>(
      "Eigen::Matrix<double,int=3,int=1,int=0,int=3,int=1>");
  CheckHash<Eigen::Matrix3d>(
      "Eigen::Matrix<double,int=3,int=3,int=0,int=3,int=3>");

  // Vectors of Eigens.
  CheckHash<std::vector<Eigen::VectorXd>>(fmt::format(
      "{std}::vector<{eigen},{std}::allocator<{eigen}>>",
      fmt::arg("std", stdcc),
      fmt::arg("eigen",
          "Eigen::Matrix<double,int=-1,int=1,int=0,int=-1,int=1>")));

  // Everything together at once works.
  using BigType = std::vector<std::pair<
      const double, std::shared_ptr<Eigen::Matrix3d>>>;
  CheckHash<BigType>(fmt::format(
      "{std}::vector<"
        "{std}::pair<"
          "const double,"
          "{std}::shared_ptr<{eigen}>>,"
        "{std}::allocator<{std}::pair<"
          "const double,"
          "{std}::shared_ptr<{eigen}>>>>",
      fmt::arg("std", stdcc),
      fmt::arg("eigen",
          "Eigen::Matrix<double,int=3,int=3,int=0,int=3,int=3>")));

  // Templated on a value, but with the 'using NonTypeTemplateParameter'
  // decoration so that the hash works.
  const std::string kfoo =
      kClang || kGcc9 ? "drake::test::{anonymous}::AnonEnum::kFoo" : "0";
  CheckHash<NiceAnonEnumTemplate<AnonEnum::kFoo>>(
      "drake::test::{anonymous}::NiceAnonEnumTemplate<"
        "drake::test::{anonymous}::AnonEnum=" + kfoo + ">");
}

// Tests that a type mismatched is detected for a mismatched non-type template
// parameter, even in Release builds.  When the TypeHash fails (is zero), it's
// important that AbstractValue fall back to using typeinfo comparison instead.
GTEST_TEST(ValueTest, NonTypeTemplateParameter) {
  // We cannot compute hashes for non-type template parameters when the user
  // hasn't added a `using` statement to guide us.
  using T1 = UnadornedAnonEnumTemplate<AnonEnum::kFoo>;
  using T2 = UnadornedAnonEnumTemplate<AnonEnum::kBar>;
  ASSERT_EQ(internal::TypeHash<T1>::value, 0);
  ASSERT_EQ(internal::TypeHash<T2>::value, 0);

  // However, our getters and setters still catch type mismatches (by using the
  // std::typeinfo comparison).
  Value<T1> foo_value;
  Value<T2> bar_value;
  AbstractValue& foo = foo_value;
  DRAKE_EXPECT_NO_THROW(foo.get_value<T1>());
  EXPECT_THROW(foo.get_value<T2>(), std::exception);
  EXPECT_THROW(foo.get_value<int>(), std::exception);
  EXPECT_THROW(foo.SetFrom(bar_value), std::exception);
}

// When a cast fails, the error message should report the actual types found
// not to match. The request must match the static type of the Value container,
// not the dynamic possibly-more-derived type of the contained value. When the
// static and dynamic types differ, display both. See #15434.
GTEST_TEST(ValueTest, TypesInBadCastMessage) {
  using MyVector1d = systems::MyVector<double, 1>;

  {
    // Make a base-typed value container with a more-derived value inside.
    auto value =
        std::make_unique<Value<BasicVector<double>>>(MyVector1d{});
    AbstractValue& abstract = *value;

    // This request looks like it should work, but doesn't. The error message
    // should indicate why.
    DRAKE_EXPECT_THROWS_MESSAGE(
        abstract.get_value<MyVector1d>(),
        ".*request.*MyVector.*static.*BasicVector.*"
        "dynamic.*MyVector.*'\\)\\.$");

    // This is the proper request type.
    DRAKE_EXPECT_NO_THROW(abstract.get_value<BasicVector<double>>());
  }

  {
    // Make a value container that doesn't have the possibility of containing
    // derived types.
    Value<int> value;
    AbstractValue& abstract = value;

    // The error message in this case can be simpler. Test note: the regular
    // expression here specifically rejects parentheses near the end of the
    // line, to exclude the more elaborate error message matched in the case
    // above.
    DRAKE_EXPECT_THROWS_MESSAGE(
        abstract.get_value<double>(),
        ".*request.*double.*static.*int'\\.$");
  }
}

}  // namespace test
}  // namespace drake
