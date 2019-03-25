#include "drake/systems/framework/value_to_abstract_value.h"

#include <memory>
#include <type_traits>

#include <gtest/gtest.h>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/framework/test_utilities/my_vector.h"

namespace drake {
namespace systems {
namespace internal {
namespace {

// Define a collection of classes that exhibit all the copy/clone variants
// that ValueToAbstractValue is supposed to handle correctly.

class NotCopyableOrCloneable {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NotCopyableOrCloneable)
  explicit NotCopyableOrCloneable(double value) : value_{value} {}
  double value() const { return value_; }

 private:
  double value_{};
};

class CopyConstructible {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CopyConstructible)
  explicit CopyConstructible(double value) : value_{value} {}
  double value() const { return value_; }

 private:
  double value_{};
};

// When both copy and clone are around, we're supposed to prefer copy.
class CopyableAndCloneable {
 public:
  explicit CopyableAndCloneable(double value) : value_{value} {}
  CopyableAndCloneable(const CopyableAndCloneable& src)
      : value_(src.value()), was_copy_constructed_(true) {}
  CopyableAndCloneable& operator=(const CopyableAndCloneable& src) = default;

  std::unique_ptr<CopyableAndCloneable> Clone() const {
    ++num_clones_;
    return std::make_unique<CopyableAndCloneable>(value());
  }

  double value() const { return value_; }
  int num_clones() const { return num_clones_; }
  bool was_copy_constructed() const { return was_copy_constructed_; }

 private:
  double value_{};
  bool was_copy_constructed_{false};
  mutable int num_clones_{0};
};

class JustCloneable {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(JustCloneable)
  virtual ~JustCloneable() = default;
  explicit JustCloneable(double value) : value_{value} {}
  std::unique_ptr<JustCloneable> Clone() const { return DoClone(); }
  double value() const { return value_; }

 private:
  virtual std::unique_ptr<JustCloneable> DoClone() const {
    return std::make_unique<JustCloneable>(value_);
  }
  double value_{};
};

// This derived class is cloneable via its base class but doesn't have its own
// public Clone() method. The Value<T> we create for it should use the
// base class type for T.
class BaseClassCloneable : public JustCloneable {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BaseClassCloneable)
  BaseClassCloneable(std::string name, double value)
      : JustCloneable(value), name_(std::move(name)) {}
  std::string name() const { return name_; }

 private:
  std::unique_ptr<JustCloneable> DoClone() const final {
    return std::make_unique<BaseClassCloneable>(name_, value());
  }

  std::string name_;
};

// This derived class is cloneable via its own public Clone() method. The
// Value<T> we create for it should use the derived class type for T.
class DerivedClassCloneable : public JustCloneable {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DerivedClassCloneable)
  DerivedClassCloneable(std::string name, double value)
      : JustCloneable(value), name_(std::move(name)) {}
  std::unique_ptr<DerivedClassCloneable> Clone() const {
    return std::make_unique<DerivedClassCloneable>(name_, value());
  }
  std::string name() const { return name_; }

 private:
  std::unique_ptr<JustCloneable> DoClone() const final {
    return std::make_unique<DerivedClassCloneable>(name_, value());
  }

  std::string name_;
};

// Test each of the four ToAbstract() methods implemented for AbstractPolicy.
// For reference:
// (1) ToAbstract(AbstractValue) cloned directly without wrapping in Value<>
// (2) ToAbstract(const char*) special-cased shorthand for std::string
// (3) ToAbstract(Eigen object) must be a simple Eigen type
// (4) ToAbstract<V>(V) creates Value<V> for copyable or cloneable types V,
//                      or Value<B> if V's clone returns a base type B.
// Method (4) is supposed to prefer the copy constructor if it and Clone() are
// both available.
//
// No runtime errors occur with this policy. A compilation failure occurs
// if a non-copyable, non-cloneable type is provided.
GTEST_TEST(ValueToAbstractValue, AbstractPolicy) {
  // All methods return std::unique_ptr<AbstractValue>.
  using AbstractPolicy = ValueToAbstractValue;

  // First check handling of given AbstractValue objects (signature (1)).

  // Pass a value already of AbstractValue type.
  Value<std::string> value_str("hello");
  Value<double> value_double(1.25);
  AbstractValue& absval_double = value_double;

  const auto value_str_copy =
      AbstractPolicy::ToAbstract("MyApi", value_str);  // (1, with conversion)
  const auto absval_double_copy =
      AbstractPolicy::ToAbstract("MyApi", absval_double);  // (1)

  EXPECT_EQ(value_str_copy->get_value<std::string>(), "hello");
  EXPECT_EQ(absval_double_copy->get_value<double>(), 1.25);

  // Make sure we're not just looking at the same objects.
  value_str.get_mutable_value() = "goodbye";
  EXPECT_EQ(value_str_copy->get_value<std::string>(), "hello");  // no change

  absval_double.get_mutable_value<double>() = 2.5;
  EXPECT_EQ(absval_double_copy->get_value<double>(), 1.25);  // no change

  // Check char* sugar signature (2).

  // char* values are stored in std::string AbstractValues.
  const auto char_ptr_val = AbstractPolicy::ToAbstract("MyApi", "char*");
  EXPECT_EQ(char_ptr_val->get_value<std::string>(), "char*");  // (2)

  // Check signature (4) handling of simple objects.

  const auto int_val = AbstractPolicy::ToAbstract("MyApi", 5);  // (4)
  EXPECT_EQ(int_val->get_value<int>(), 5);

  const auto str_val =
      AbstractPolicy::ToAbstract("MyApi", std::string("string"));  // (4)
  EXPECT_EQ(str_val->get_value<std::string>(), "string");

  // Check signature (4) handling of objects that are: just copyable,
  // just cloneable (to self or base class), and both copyable and cloneable.

  const auto copyable_val = AbstractPolicy::ToAbstract(
      "MyApi", CopyConstructible(1.25));  // (4, copy)
  EXPECT_EQ(copyable_val->get_value<CopyConstructible>().value(), 1.25);

  const auto cloneable_val =
      AbstractPolicy::ToAbstract("MyApi", JustCloneable(0.75));  // (4, clone)
  EXPECT_EQ(cloneable_val->get_value<JustCloneable>().value(), 0.75);

  // Here ToAbstract() has to discover that the available Clone() method
  // returns a base class-typed object (here a JustCloneable), rather than the
  // concrete type we give it, so we have to store it in a Value<BaseType>.
  const auto base_cloneable_val = AbstractPolicy::ToAbstract(
      "MyApi",
      BaseClassCloneable("base_cloneable", 1.5));  // (4, clone-to-base)
  EXPECT_EQ(base_cloneable_val->get_value<JustCloneable>().value(), 1.5);
  EXPECT_EQ(dynamic_cast<const BaseClassCloneable&>(
                base_cloneable_val->get_value<JustCloneable>())
                .name(),
            "base_cloneable");

  // Here ToAbstract() should discover that the available Clone() method
  // returns the derived class-typed object, so we can store that directly
  // in a Value<DerivedType>.
  const auto derived_cloneable_val = AbstractPolicy::ToAbstract(
      "MyApi", DerivedClassCloneable("derived_cloneable", 0.25));  // (4, clone)
  EXPECT_EQ(derived_cloneable_val->get_value<DerivedClassCloneable>().value(),
            0.25);
  EXPECT_EQ(derived_cloneable_val->get_value<DerivedClassCloneable>().name(),
            "derived_cloneable");

  // When the value object is both copyable and cloneable, we expect the copy
  // constructor to be used.
  CopyableAndCloneable cc_value(2.25);
  EXPECT_EQ(cc_value.num_clones(), 0);
  EXPECT_FALSE(cc_value.was_copy_constructed());

  const auto cc_val_abstract_copy =
      AbstractPolicy::ToAbstract("MyApi", cc_value);  // (4, prefer copy)
  const auto& cc_val_copy =
      cc_val_abstract_copy->get_value<CopyableAndCloneable>();
  EXPECT_EQ(cc_val_copy.value(), 2.25);
  EXPECT_EQ(cc_value.num_clones(), 0);
  EXPECT_TRUE(cc_val_copy.was_copy_constructed());
  EXPECT_EQ(cc_val_copy.num_clones(), 0);

  // These are also (4) but should trigger a nice static_assert message if
  // uncommented, similar to:
  //   static assertion failed: ValueToAbstractValue(): value type must be
  //   copy constructible or have an accessible Clone() method that returns
  //   std::unique_ptr.
  // AbstractPolicy::ToAbstract("MyApi", NotCopyableOrCloneable(2.));
}

// AbstractPolicy should store vector objects as themselves, but Eigen vector
// expressions must get eval()'ed first and then stored as the eval() type.
GTEST_TEST(ValueToAbstractValue, AbstractPolicyOnVectors) {
  using AbstractPolicy = ValueToAbstractValue;

  const Eigen::Vector3d expected_vec(10., 20., 30.);
  const double expected_scalar = 3.125;

  // A scalar should not get turned into a Vector1.
  const auto scalar_val =
      AbstractPolicy::ToAbstract("MyApi", expected_scalar);  // (4)
  EXPECT_EQ(scalar_val->get_value<double>(), expected_scalar);

  // An Eigen vector object can't be supplied directly.
  DRAKE_EXPECT_THROWS_MESSAGE(
      AbstractPolicy::ToAbstract("MyApi", expected_vec),  // (3)
      std::logic_error,
      ".*MyApi.*Eigen.*cannot automatically be stored.*Drake abstract "
      "quantity.*");

  // But providing the type explicitly works.
  const auto vec3_val =
      AbstractPolicy::ToAbstract("MyApi", Value<Eigen::Vector3d>(expected_vec));
  EXPECT_EQ(vec3_val->get_value<Eigen::Vector3d>(), expected_vec);

  // More complicated Eigen expressions should throw also.
  const Eigen::Vector4d long_vec(.25, .5, .75, 1.);
  DRAKE_EXPECT_THROWS_MESSAGE(
      AbstractPolicy::ToAbstract("MyApi", long_vec.tail(3)),  // (3)
      std::logic_error,
      ".*MyApi.*Eigen.*cannot automatically be stored.*Drake abstract "
      "quantity.*");

  // A plain BasicVector is stored as a Value<BasicVector>, just as it is
  // under the VectorPolicy.
  const BasicVector<double> basic_vector3({7., 8., 9.});
  const auto basic_vector_val =
      AbstractPolicy::ToAbstract("MyApi", basic_vector3);  // (4)
  EXPECT_EQ(basic_vector_val->get_value<BasicVector<double>>().get_value(),
            Eigen::Vector3d(7., 8., 9.));

  // MyVector derives from BasicVector and has its own Clone() method
  // that returns unique_ptr<MyVector>. Under the AbstractPolicy, this is
  // handled like any other cloneable object by signature (4) and stored as
  // Value<MyVector>. That's different from the VectorPolicy treatment.
  const MyVector3d my_vector3(expected_vec);
  auto my_vector3_val = AbstractPolicy::ToAbstract("MyApi", my_vector3);  // (4)
  const MyVector3d& my_vector3_from_abstract =
      my_vector3_val->get_value<MyVector3d>();
  EXPECT_EQ(my_vector3_from_abstract.get_value(), expected_vec);
}

// Test each of the five ToAbstract() methods implemented for VectorPolicy.
// For reference:
// (1) ToAbstract(Eigen::Ref) should convert to BasicVector.
// (2) ToAbstract(scalar) should be treated like and Eigen Vector1.
// (3) ToAbstract(BasicVector) special-cased to make concrete vector types work
//         regardless of whether they overload Clone().
// (4) ToAbstract(AbstractValue) cloned directly without wrapping in Value<>,
//         but must resolve to Value<BasicVector>.
// (5) ToAbstract<V>(V) issues an std::logic_error since only the above types
//         are acceptable for vector objects.

// Test the non-vector signatures (4) and (5) here. These can produce runtime
// errors which should include the supplied API name. (All methods must take
// the name.)
GTEST_TEST(ValueToVectorValue, GenericObjects) {
  // All methods return std::unique_ptr<AbstractValue>.

  using VectorPolicy = ValueToVectorValue<double>;

  // Check that AbstractValue objects are acceptable to signature (4) only if
  // they resolve to Value<BasicVector>.
  const Value<int> bad_val(1);
  const Value<BasicVector<double>> good_val(Eigen::Vector3d(1., 2., 3.));

  DRAKE_EXPECT_THROWS_MESSAGE(
      VectorPolicy::ToAbstract("MyApi", bad_val), std::logic_error,
      ".*MyApi.*AbstractValue.*type int is not.*vector quantity.*");

  const auto good_val_copy = VectorPolicy::ToAbstract("MyApi", good_val);
  EXPECT_EQ(good_val_copy->get_value<BasicVector<double>>().get_value(),
            Eigen::Vector3d(1., 2., 3.));

  // Non-vector objects should throw via signature (5).
  DRAKE_EXPECT_THROWS_MESSAGE(VectorPolicy::ToAbstract("MyApi", 5),
                              std::logic_error,
                              ".*MyApi.*type int is not.*vector quantity.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      VectorPolicy::ToAbstract("MyApi", std::string("string")),
      std::logic_error, ".*MyApi.*type std::string is not.*vector quantity.*");

  DRAKE_EXPECT_THROWS_MESSAGE(
      VectorPolicy::ToAbstract("MyApi", CopyConstructible(1.25)),
      std::logic_error, ".*MyApi.*CopyConstructible is not.*vector quantity.*");

  DRAKE_EXPECT_THROWS_MESSAGE(
      VectorPolicy::ToAbstract("MyApi", JustCloneable(0.75)), std::logic_error,
      ".*MyApi.*JustCloneable is not.*vector quantity.*");
}

// Test signature (1), (2), and (3) which deal with Eigen vectors, scalars, and
// BasicVectors, respectively, under VectorPolicy. (Under AbstractPolicy,
// none of these get special treatment.)
GTEST_TEST(ValueToVectorValue, VectorPolicy) {
  const Eigen::Vector3d expected_vec(10., 20., 30.);
  const double expected_scalar = 3.125;

  // VectorPolicy handles Eigen vector, scalar, and BasicVector-derived objects
  // V by putting them in a Value<BasicVector>.
  using VectorPolicy = ValueToVectorValue<double>;

  // VectorPolicy puts a scalar into a 1-element Value<BasicVector<T>>, while
  // AbstractPolicy simply puts it in a Value<T>.
  auto scalar_val = VectorPolicy::ToAbstract("MyApi", expected_scalar);  // (2)
  EXPECT_EQ(scalar_val->get_value<BasicVector<double>>()[0], expected_scalar);

  // Any Eigen vector type should be stored as a BasicVector.
  auto vec3_val = VectorPolicy::ToAbstract("MyApi", expected_vec);  // (1)
  EXPECT_EQ(vec3_val->get_value<BasicVector<double>>().get_value(),
            expected_vec);

  // Pass a more complicated Eigen object; should still work.
  const Eigen::Vector4d long_vec(.25, .5, .75, 1.);
  auto tail3_val = VectorPolicy::ToAbstract("MyApi", long_vec.tail(3));  // (1)
  EXPECT_EQ(tail3_val->get_value<BasicVector<double>>().get_value(),
            Eigen::Vector3d(.5, .75, 1.));

  auto segment2_val =
      VectorPolicy::ToAbstract("MyApi", long_vec.segment<2>(1));  // (1)
  EXPECT_EQ(segment2_val->get_value<BasicVector<double>>().get_value(),
            Eigen::Vector2d(.5, .75));

  // A plain BasicVector is stored as a Value<BasicVector>. It is handled
  // by the BasicVector overload (3).
  const BasicVector<double> basic_vector3({7., 8., 9.});
  const auto basic_vector_val =
      VectorPolicy::ToAbstract(__func__, basic_vector3);  // (3)
  EXPECT_EQ(basic_vector_val->get_value<BasicVector<double>>().get_value(),
            Eigen::Vector3d(7., 8., 9.));

  // MyVector derives from BasicVector and has its own Clone() method
  // that returns unique_ptr<MyVector>. *Despite that*, under the VectorPolicy
  // it should be stored as a Value<BasicVector>, which is why signature (3)
  // exists for this policy. (AbstractPolicy would store this as a
  // Value<MyVector>.) However, the concrete type must be preserved.
  const MyVector3d my_vector3(expected_vec);
  auto my_vector3_val = VectorPolicy::ToAbstract("MyApi", my_vector3);  // (3)
  const BasicVector<double>& basic_vector =
      my_vector3_val->get_value<BasicVector<double>>();
  EXPECT_EQ(basic_vector.get_value(), expected_vec);

  // Check that the concrete type was preserved.
  EXPECT_EQ(dynamic_cast<const MyVector3d&>(basic_vector).get_value(),
            expected_vec);
}

}  // namespace
}  // namespace internal
}  // namespace systems
}  // namespace drake
