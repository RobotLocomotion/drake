#include "drake/systems/framework/value_to_abstract_value.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/drake_copyable.h"
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
  int num_clones() const {return num_clones_;}
  bool was_copy_constructed() const {return was_copy_constructed_;}

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
      : JustCloneable(value), name_(name) {}
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
      : JustCloneable(value), name_(name) {}
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

// Test each of the five ToAbstract() methods. For reference:
// (1) ToAbstract(Eigen::Ref) should convert to BasicVector
// (2) ToAbstract(BasicVector) special-cased to make concrete vector types work
//                             regardless of whether they overload Clone()
// (3) ToAbstract(AbstractValue) cloned directly without wrapping in Value<>
// (4) ToAbstract(const char*) special-cased shorthand for std::string
// (5) ToAbstract<V>(V) creates Value<V> for copyable or cloneable types V,
//                      or Value<B> if V's clone returns a base type B.

// Test that the generic method (5) and sugar (4) work properly for well-behaved
// concrete objects.
GTEST_TEST(ValueToAbstractValue, GenericObjects) {
  // All methods return std::unique_ptr<AbstractValue>.

  const auto int_val = ValueToAbstractValue<double>::ToAbstract(5);  // (5)
  EXPECT_EQ(int_val->GetValue<int>(), 5);

  const auto str_val =
      ValueToAbstractValue<double>::ToAbstract(std::string("string"));  // (5)
  EXPECT_EQ(str_val->GetValue<std::string>(), "string");

  // char* values are stored in std::string AbstractValues.
  const auto char_ptr_val = ValueToAbstractValue<double>::ToAbstract("char*");
  EXPECT_EQ(char_ptr_val->GetValue<std::string>(), "char*");  // (4)

  const auto copyable_val = ValueToAbstractValue<double>::ToAbstract(
      CopyConstructible(1.25));  // (5)
  EXPECT_EQ(copyable_val->GetValue<CopyConstructible>().value(), 1.25);

  const auto cloneable_val = ValueToAbstractValue<double>::ToAbstract(
      JustCloneable(0.75));  // (5)
  EXPECT_EQ(cloneable_val->GetValue<JustCloneable>().value(), 0.75);

  // Here ToAbstract() has to discover that the available Clone() method
  // returns a base class-typed object, rather than the concrete type we give
  // it here, so we have to store it in a Value<BaseType>.
  const auto base_cloneable_val = ValueToAbstractValue<double>::ToAbstract(
      BaseClassCloneable("base_cloneable", 1.5));  // (5)
  EXPECT_EQ(base_cloneable_val->GetValue<JustCloneable>().value(), 1.5);
  EXPECT_EQ(dynamic_cast<const BaseClassCloneable&>(
                base_cloneable_val->GetValue<JustCloneable>()).name(),
            "base_cloneable");

  // Here ToAbstract() should discover that the available Clone() method
  // returns the derived class-typed object, so we can store that directly
  // in a Value<DerivedType>.
  const auto derived_cloneable_val = ValueToAbstractValue<double>::ToAbstract(
      DerivedClassCloneable("derived_cloneable", 0.25));  // (5)
  EXPECT_EQ(derived_cloneable_val->GetValue<DerivedClassCloneable>().value(),
            0.25);
  EXPECT_EQ(derived_cloneable_val->GetValue<DerivedClassCloneable>().name(),
            "derived_cloneable");

  // When the value object is both copyable and cloneable, we expect the copy
  // constructor to be used.
  CopyableAndCloneable cc_value(2.25);
  EXPECT_EQ(cc_value.num_clones(), 0);
  EXPECT_FALSE(cc_value.was_copy_constructed());

  const auto cc_val_abstract_copy =
      ValueToAbstractValue<double>::ToAbstract(cc_value);  // (5)
  const auto& cc_val_copy =
      cc_val_abstract_copy->GetValue<CopyableAndCloneable>();
  EXPECT_EQ(cc_val_copy.value(), 2.25);
  EXPECT_EQ(cc_value.num_clones(), 0);
  EXPECT_TRUE(cc_val_copy.was_copy_constructed());
  EXPECT_EQ(cc_val_copy.num_clones(), 0);

  // This is also (5) but should trigger a nice static_assert message if
  // uncommented, similar to:
  //   static assertion failed: ValueToAbstractValue(): value type must be
  //   copy constructible or have an accessible Clone() method that returns
  //   std::unique_ptr.
  // ValueToAbstractValue<double>::ToAbstract(NotCopyableOrCloneable(2.));
}

// Test variants (1) and (2) which deal with Eigen vectors and BasicVectors,
// respectively.
GTEST_TEST(ValueToAbstractValue, Vectors) {
  const Eigen::Vector3d expected_vec(10., 20., 30.);

  // Any Eigen vector type should be stored as a BasicVector.
  const auto vec3_val =
      ValueToAbstractValue<double>::ToAbstract(expected_vec);  // (1)
  EXPECT_EQ(vec3_val->GetValue<BasicVector<double>>().get_value(),
      expected_vec);

  // Pass a more complicated Eigen object; should still work.
  const Eigen::Vector4d long_vec(.25, .5, .75, 1.);
  const auto tail3_val =
      ValueToAbstractValue<double>::ToAbstract(long_vec.tail(3));  // (1)
  EXPECT_EQ(tail3_val->GetValue<BasicVector<double>>().get_value(),
            Eigen::Vector3d(.5, .75, 1.));

  const auto segment2_val =
      ValueToAbstractValue<double>::ToAbstract(long_vec.segment<2>(1));  // (1)
  EXPECT_EQ(segment2_val->GetValue<BasicVector<double>>().get_value(),
            Eigen::Vector2d(.5, .75));

  // A plain BasicVector is stored as a Value<BasicVector>, but is still
  // processed by the BasicVector overload (2) of ToAbstract.
  const BasicVector<double> basic_vector3({7., 8., 9.});
  const auto basic_vector_val =
      ValueToAbstractValue<double>::ToAbstract(basic_vector3);  // (2)
  EXPECT_EQ(basic_vector_val->GetValue<BasicVector<double>>().get_value(),
            Eigen::Vector3d(7., 8., 9.));

  // MyVector derives from BasicVector and has its own Clone() method
  // that returns unique_ptr<MyVector>. *Despite that*, and unlike treatment
  // of other cloneable classes, it should be stored as a Value<BasicVector>,
  // which is why variant (2) of ToAbstract exists. However, the concrete
  // type must be preserved.
  const MyVector3d my_vector3(expected_vec);
  const auto my_vector3_val =
      ValueToAbstractValue<double>::ToAbstract(my_vector3);  // (2)
  const BasicVector<double>& basic_vector =
      my_vector3_val->GetValue<BasicVector<double>>();
  EXPECT_EQ(basic_vector.get_value(), expected_vec);
  EXPECT_EQ(dynamic_cast<const MyVector3d&>(basic_vector).get_value(),
            expected_vec);
}

// Test variant (3) of ToAbstract() that deals with AbstractValues.
GTEST_TEST(ValueToAbstractValue, AbstractValue) {
  // Pass a value already of AbstractValue type.
  std::unique_ptr<AbstractValue> absval_str =
      AbstractValue::Make<std::string>("hello");

  const auto absval_str_copy =
      ValueToAbstractValue<double>::ToAbstract(*absval_str);  // (3)
  EXPECT_EQ(absval_str_copy->GetValue<std::string>(), "hello");
  // Make sure we're not just looking at the same object.
  absval_str->GetMutableValue<std::string>() = "goodbye";
  EXPECT_EQ(absval_str_copy->GetValue<std::string>(), "hello");  // no change

  // Pass an object of AbstractValue-derived type.
  const auto val_int =
      ValueToAbstractValue<double>::ToAbstract(Value<int>(10));  // (3)
  EXPECT_EQ(val_int->GetValue<int>(), 10);
}

}  // namespace
}  // namespace internal
}  // namespace systems
}  // namespace drake
