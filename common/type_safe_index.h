#pragma once

#include <functional>
#include <limits>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/common/hash.h"
#include "drake/common/nice_type_name.h"

namespace drake {

/// A type-safe non-negative index class.
///
/// @note This is *purposely* a separate class from Identifier.
/// For more information, see @ref TypeSafeIndexVsIndentifier "this section".
///
/// This class serves as an upgrade to the standard practice of passing `int`s
/// around as indices. In the common practice, a method that takes indices into
/// multiple collections would have an interface like:
///
/// @code
/// void foo(int bar_index, int thing_index);
/// @endcode
///
/// It is possible for a programmer to accidentally switch the two index values
/// in an invocation.  This mistake would still be _syntactically_ correct; it
/// will successfully compile but lead to inscrutable run-time errors. The
/// type-safe index provides the same speed and efficiency of passing `int`s,
/// but provides compile-time checking. The function would now look like:
///
/// @code
/// void foo(BarIndex bar_index, ThingIndex thing_index);
/// @endcode
///
/// and the compiler will catch instances where the order is reversed.
///
/// The type-safe index is a _stripped down_ `int`. Each uniquely declared
/// index type has the following properties:
///
/// - Valid index values are _explicitly_ constructed from `int` values.
/// - The index is implicitly convertible to an `int` (to serve as an index).
/// - The index supports increment, decrement, and in-place addition and
///   subtraction to support standard index-like operations.
/// - An index _cannot_ be constructed or compared to an index of another
///   type.
/// - In general, indices of different types are _not_ interconvertible.
/// - Binary integer operators (e.g., +, -, |, *, etc.) _always_ produce `int`
///   return values. One can even use operands of different index types in
///   such a binary expression. It is the _programmer's_ responsibility to
///   confirm that the resultant `int` value has meaning.
///
/// While there _is_ the concept of an "invalid" index, this only exists to
/// support default construction _where appropriate_ (e.g., using indices in
/// STL containers). Using an invalid index in _any_ operation is considered
/// an error. In Debug build, attempts to compare, increment, decrement, etc. an
/// invalid index will throw an exception.
///
/// A function that returns %TypeSafeIndex values which need to communicate
/// failure should _not_ use an invalid index. It should return an
/// `std::optional<Index>` instead.
///
/// It is the designed intent of this class, that indices derived from this
/// class can be passed and returned by value. (Drake's typical calling
/// convention requires passing input arguments by const reference, or by value
/// when moved from. That convention does not apply to this class.)
///
/// This is the recommended method to create a unique index type associated with
/// class `Foo`:
///
/// @code
/// using FooIndex = TypeSafeIndex<class FooTag>;
/// @endcode
///
/// This references a non-existent, and ultimately anonymous, class `FooTag`.
/// This is sufficient to create a unique index type. It is certainly possible
/// to use an existing class (e.g., `Foo`). But this provides no functional
/// benefit.
///
/// __Construction from integral types__
///
/// C++ will do
/// [implicit integer conversions](https://en.cppreference.com/w/cpp/language/implicit_conversion#Integral_conversions).
/// This allows construction of %TypeSafeIndex values with arbitrary integral
/// types. Index values must lie in the range of [0, 2³¹). The constructor will
/// validate the input value (in Debug mode). Ultimately, the caller is
/// responsible for confirming that the values provided lie in the valid range.
///
/// __Examples of valid and invalid operations__
///
/// The TypeSafeIndex guarantees that index instances of different types can't
/// be compared or combined.  Efforts to do so will cause a compile-time
/// failure.  However, comparisons or operations on _other_ types that are
/// convertible to an int will succeed.  For example:
/// @code
///    using AIndex = TypeSafeIndex<class A>;
///    using BIndex = TypeSafeIndex<class B>;
///    AIndex a(1);
///    BIndex b(1);
///    if (a == 2) { ... }      // Ok.
///    size_t sz = 7;
///    if (a == sz) { ... }     // Ok.
///    if (a == b) { ... }      // <-- Compiler error.
///    AIndex invalid;          // Creates an invalid index.
///    ++invalid;               // Runtime error in Debug build.
/// @endcode
///
/// As previously stated, the intent of this class is to seamlessly serve as an
/// index into indexed objects (e.g., vector, array, etc.). At the same time, we
/// want to avoid implicit conversions _from_ int to an index.  These two design
/// constraints combined lead to a limitation in how TypeSafeIndex instances
/// can be used.  Specifically, we've lost a common index pattern:
///
/// @code
///    for (MyIndex a = 0; a < N; ++a) { ... }
/// @endcode
///
/// This pattern no longer works because it requires implicit conversion of int
/// to TypeSafeIndex. Instead, the following pattern needs to be used:
///
/// @code
///    for (MyIndex a(0); a < N; ++a) { ... }
/// @endcode
///
/// __Use with Eigen__
///
/// At the time of this writing when using the latest Eigen 3.4 preview branch,
/// a TypeSafeIndex cannot be directly used to index into an Eigen::Matrix; the
/// developer must explicitly introduce the `int` conversion:
/// @code
///    VectorXd some_vector = ...;
///    FooIndex foo_index = ...;
///    some_vector(foo_index) = 0.0;       // Fails to compile.
///    some_vector(int{foo_index}) = 0.0;  // Compiles OK.
/// @endcode
/// TODO(#15354) We hope to fix this irregularity in the future.
///
/// @sa drake::geometry::Identifier
///
/// @tparam Tag The name of the tag associated with a class type. The class
///             need not be a defined class.
template <class Tag>
class TypeSafeIndex {
 public:
  /// @name           Constructors
  ///@{

  /// Default constructor; the result is an _invalid_ index. This only
  /// exists to serve applications which require a default constructor.
  TypeSafeIndex() {}

  /// Construction from a non-negative `int` value. The value must lie in the
  /// range of [0, 2³¹). Constructor only promises to test validity in
  /// Debug build.
  explicit TypeSafeIndex(int64_t index) : index_(static_cast<int>(index)) {
    // NOTE: This tests the *input* value and not the result as an invalid
    // input can lead to a truncation that appears valid.
    DRAKE_ASSERT_VOID(
        AssertValid(index, "Explicitly constructing an invalid index."));
  }

  /// Disallow construction from another index type.
  template <typename U>
  TypeSafeIndex(const TypeSafeIndex<U>& idx) = delete;

  TypeSafeIndex(const TypeSafeIndex&) = default;

  TypeSafeIndex(TypeSafeIndex&& other) noexcept : index_(other.index_) {
    other.index_ = kDefaultInvalid;
  }
  ///@}

  /// @name       Assignment
  ///@{

  TypeSafeIndex& operator=(const TypeSafeIndex&) = default;

  TypeSafeIndex& operator=(TypeSafeIndex&& other) noexcept {
    index_ = other.index_;
    other.index_ = kDefaultInvalid;
    return *this;
  }

  ///@}

  /// @name     Utility methods
  ///@{

  /// Implicit conversion-to-int operator.
  operator int() const {
    DRAKE_ASSERT_VOID(AssertValid(index_, "Converting to an int."));
    return index_;
  }

  /// Reports if the index is valid--the only operation on an invalid index
  /// that doesn't throw an exception in Debug builds.
  bool is_valid() const {
    // All other error testing, with assert armed, indirectly enforces the
    // invariant that the only way to get an invalid index is via the default
    // constructor. This assertion will catch any crack in that effort.
    DRAKE_ASSERT((index_ >= 0) || (index_ == kDefaultInvalid));
    return index_ >= 0;
  }

  ///@}

  /// @name Arithmetic operators
  ///@{

  /// Prefix increment operator.
  const TypeSafeIndex& operator++() {
    DRAKE_ASSERT_VOID(
        AssertValid(index_, "Pre-incrementing an invalid index."));
    DRAKE_ASSERT_VOID(
        AssertNoOverflow(1, "Pre-incrementing produced an invalid index."));
    ++index_;
    return *this;
  }

  /// Postfix increment operator.
  TypeSafeIndex operator++(int) {
    DRAKE_ASSERT_VOID(
        AssertValid(index_, "Post-incrementing an invalid index."));
    DRAKE_ASSERT_VOID(
        AssertNoOverflow(1, "Post-incrementing produced an invalid index."));
    ++index_;
    return TypeSafeIndex(index_ - 1);
  }

  /// Prefix decrement operator.
  /// In Debug builds, this method asserts that the resulting index is
  /// non-negative.
  const TypeSafeIndex& operator--() {
    DRAKE_ASSERT_VOID(
        AssertValid(index_, "Pre-decrementing an invalid index."));
    --index_;
    DRAKE_ASSERT_VOID(
        AssertValid(index_, "Pre-decrementing produced an invalid index."));
    return *this;
  }

  /// Postfix decrement operator.
  /// In Debug builds, this method asserts that the resulting index is
  /// non-negative.
  TypeSafeIndex operator--(int) {
    DRAKE_ASSERT_VOID(
        AssertValid(index_, "Post-decrementing an invalid index."));
    --index_;
    DRAKE_ASSERT_VOID(AssertValid(
        index_, "Post-decrementing produced an invalid index."));
    return TypeSafeIndex(index_ + 1);
  }
  ///@}

  /// @name Compound assignment operators
  ///@{

  /// Addition assignment operator.
  /// In Debug builds, this method asserts that the resulting index is
  /// non-negative.
  TypeSafeIndex& operator+=(int i) {
    DRAKE_ASSERT_VOID(
        AssertValid(index_,
                    "In-place addition with an int on an invalid index."));
    DRAKE_ASSERT_VOID(AssertNoOverflow(
        i, "In-place addition with an int produced an invalid index."));
    index_ += i;
    DRAKE_ASSERT_VOID(AssertValid(
        index_, "In-place addition with an int produced an invalid index."));
    return *this;
  }

  /// Allow addition for indices with the same tag.
  TypeSafeIndex<Tag>& operator+=(const TypeSafeIndex<Tag>& other) {
    DRAKE_ASSERT_VOID(AssertValid(
        index_, "In-place addition with another index invalid LHS."));
    DRAKE_ASSERT_VOID(AssertValid(
        other.index_, "In-place addition with another index invalid RHS."));
    DRAKE_ASSERT_VOID(AssertNoOverflow(
        other.index_,
        "In-place addition with another index produced an invalid index."));
    index_ += other.index_;
    DRAKE_ASSERT_VOID(AssertValid(
        index_,
        "In-place addition with another index produced an invalid index."));
    return *this;
  }

  /// Prevent addition for indices of different tags.
  template <typename U>
  TypeSafeIndex<U>& operator+=(const TypeSafeIndex<U>& u) = delete;

  /// Subtraction assignment operator.
  /// In Debug builds, this method asserts that the resulting index is
  /// non-negative.
  TypeSafeIndex& operator-=(int i) {
    DRAKE_ASSERT_VOID(AssertValid(
        index_, "In-place subtraction with an int on an invalid index."));
    DRAKE_ASSERT_VOID(AssertNoOverflow(
        -i, "In-place subtraction with an int produced an invalid index."));
    index_ -= i;
    DRAKE_ASSERT_VOID(AssertValid(
        index_, "In-place subtraction with an int produced an invalid index."));
    return *this;
  }

  /// Allow subtraction for indices with the same tag.
  TypeSafeIndex<Tag>& operator-=(const TypeSafeIndex<Tag>& other) {
    DRAKE_ASSERT_VOID(AssertValid(
        index_, "In-place subtraction with another index invalid LHS."));
    DRAKE_ASSERT_VOID(AssertValid(other.index_,
        "In-place subtraction with another index invalid RHS."));
    // No test for overflow; it would only be necessary if other had a negative
    // index value. In that case, it would be invalid and that would be caught
    // by the previous assertion.
    index_ -= other.index_;
    DRAKE_ASSERT_VOID(AssertValid(
        index_,
        "In-place subtraction with another index produced an invalid index."));
    return *this;
  }

  /// Prevent subtraction for indices of different tags.
  template <typename U>
  TypeSafeIndex<U>& operator-=(const TypeSafeIndex<U>& u) = delete;

  ///@}

  /// @name Exclusive comparison operators
  ///
  /// In order to prevent indices _of different type_ being added together or
  /// compared against each other, we explicitly include indices of this type,
  /// but exclude indices of all other types.  This implicitly allows all
  /// _other_ objects that can be converted to int types.
  ///@{

  // Note for developers: Each comparison operator has a SFINAE-based version
  // for handling comparison with unsigned values (created to allow comparison
  // with size_t). The explicit methods are necessary to enable comparison
  // without unsigned/signed comparison warnings (which Drake considers to be an
  // error). Furthermore, the SFINAE is necessary to prevent ambiguity.
  // Index == int can be resolved two ways:
  //
  // - convert Index to int
  // - promote int to size_t
  //
  // SFINAE prevents the latter.

  /// Allow equality test with indices of this tag.
  bool operator==(const TypeSafeIndex<Tag>& other) const {
    DRAKE_ASSERT_VOID(AssertValid(index_, "Testing == with invalid LHS."));
    DRAKE_ASSERT_VOID(
        AssertValid(other.index_, "Testing == with invalid RHS."));
    return index_ == other.index_;
  }

  /// Allow equality test with unsigned integers.
  template <typename U>
  typename std::enable_if_t<
      std::is_integral_v<U> && std::is_unsigned_v<U>, bool>
  operator==(const U& value) const {
    DRAKE_ASSERT_VOID(AssertValid(index_, "Testing == with invalid index."));
    return value <= static_cast<U>(kMaxIndex) &&
        index_ == static_cast<int>(value);
  }

  /// Prevent equality tests with indices of other tags.
  template <typename U>
  bool operator==(const TypeSafeIndex<U>& u) const = delete;

  /// Allow inequality test with indices of this tag.
  bool operator!=(const TypeSafeIndex<Tag>& other) const {
    DRAKE_ASSERT_VOID(AssertValid(index_, "Testing != with invalid LHS."));
    DRAKE_ASSERT_VOID(
        AssertValid(other.index_, "Testing != with invalid RHS."));
    return index_ != other.index_;
  }

  /// Allow inequality test with unsigned integers.
  template <typename U>
  typename std::enable_if_t<
      std::is_integral_v<U> && std::is_unsigned_v<U>, bool>
  operator!=(const U& value) const {
    DRAKE_ASSERT_VOID(AssertValid(index_, "Testing != with invalid index."));
    return value > static_cast<U>(kMaxIndex) ||
        index_ != static_cast<int>(value);
  }

  /// Prevent inequality test with indices of other tags.
  template <typename U>
  bool operator!=(const TypeSafeIndex<U>& u) const = delete;

  /// Allow less than test with indices of this tag.
  bool operator<(const TypeSafeIndex<Tag>& other) const {
    DRAKE_ASSERT_VOID(AssertValid(index_, "Testing < with invalid LHS."));
    DRAKE_ASSERT_VOID(
        AssertValid(other.index_, "Testing < with invalid RHS."));
    return index_ < other.index_;
  }

  /// Allow less than test with unsigned integers.
  template <typename U>
  typename std::enable_if_t<
      std::is_integral_v<U> && std::is_unsigned_v<U>, bool>
  operator<(const U& value) const {
    DRAKE_ASSERT_VOID(AssertValid(index_, "Testing < with invalid index."));
    return value > static_cast<U>(kMaxIndex) ||
        index_ < static_cast<int>(value);
  }

  /// Prevent less than test with indices of other tags.
  template <typename U>
  bool operator<(const TypeSafeIndex<U>& u) const = delete;

  /// Allow less than or equals test with indices of this tag.
  bool operator<=(const TypeSafeIndex<Tag>& other) const {
    DRAKE_ASSERT_VOID(AssertValid(index_, "Testing <= with invalid LHS."));
    DRAKE_ASSERT_VOID(
        AssertValid(other.index_, "Testing <= with invalid RHS."));
    return index_ <= other.index_;
  }

  /// Allow less than or equals test with unsigned integers.
  template <typename U>
  typename std::enable_if_t<
      std::is_integral_v<U> && std::is_unsigned_v<U>, bool>
  operator<=(const U& value) const {
    DRAKE_ASSERT_VOID(AssertValid(index_, "Testing <= with invalid index."));
    return value > static_cast<U>(kMaxIndex) ||
        index_ <= static_cast<int>(value);
  }

  /// Prevent less than or equals test with indices of other tags.
  template <typename U>
  bool operator<=(const TypeSafeIndex<U>& u) const = delete;

  /// Allow greater than test with indices of this tag.
  bool operator>(const TypeSafeIndex<Tag>& other) const {
    DRAKE_ASSERT_VOID(AssertValid(index_, "Testing > with invalid LHS."));
    DRAKE_ASSERT_VOID(
        AssertValid(other.index_, "Testing > with invalid RHS."));
    return index_ > other.index_;
  }

  /// Allow greater than test with unsigned integers.
  template <typename U>
  typename std::enable_if_t<
      std::is_integral_v<U> && std::is_unsigned_v<U>, bool>
  operator>(const U& value) const {
    DRAKE_ASSERT_VOID(AssertValid(index_, "Testing > with invalid index."));
    return value <= static_cast<U>(kMaxIndex) &&
           index_ > static_cast<int>(value);
  }

  /// Prevent greater than test with indices of other tags.
  template <typename U>
  bool operator>(const TypeSafeIndex<U>& u) const = delete;

  /// Allow greater than or equals test with indices of this tag.
  bool operator>=(const TypeSafeIndex<Tag>& other) const {
    DRAKE_ASSERT_VOID(AssertValid(index_, "Testing >= with invalid LHS."));
    DRAKE_ASSERT_VOID(
        AssertValid(other.index_, "Testing >= with invalid RHS."));
    return index_ >= other.index_;
  }

  /// Allow greater than or equals test with unsigned integers.
  template <typename U>
  typename std::enable_if_t<
      std::is_integral_v<U> && std::is_unsigned_v<U>, bool>
  operator>=(const U& value) const {
    DRAKE_ASSERT_VOID(AssertValid(index_, "Testing >= with invalid index."));
    return value <= static_cast<U>(kMaxIndex) &&
           index_ >= static_cast<int>(value);
  }

  /// Prevent greater than or equals test with indices of other tags.
  template <typename U>
  bool operator>=(const TypeSafeIndex<U>& u) const = delete;

  ///@}

  /// Implements the @ref hash_append concept. And invalid index will
  /// successfully hash (in order to satisfy STL requirements), and it is up to
  /// the user to confirm it is valid before using it as a key (or other hashing
  /// application).
  template <typename HashAlgorithm>
  friend void hash_append(HashAlgorithm& hasher,
                          const TypeSafeIndex& i) noexcept {
    using drake::hash_append;
    hash_append(hasher, i.index_);
  }

 private:
  // Checks if this index lies in the valid range; throws an exception if not.
  // Invocations provide a string explaining the origin of the bad value.
  static void AssertValid(int64_t index, const char* source) {
    if (index < 0 || index > kMaxIndex) {
      throw std::runtime_error(
          std::string(source) + " Type \"" +
          drake::NiceTypeName::Get<TypeSafeIndex<Tag>>() +
          "\", has an invalid value; it must lie in the range [0, 2³¹ - 1].");
    }
  }

  // This tests for overflow conditions based on adding the given delta into
  // the current index value.
  void AssertNoOverflow(int delta, const char* source) const {
    if (delta > 0 && index_ > kMaxIndex - delta) {
      throw std::runtime_error(
          std::string(source) + " Type \"" +
          drake::NiceTypeName::Get<TypeSafeIndex<Tag>>() +
          "\", has overflowed.");
    }
  }

  // This value helps distinguish indices that are invalid through construction
  // or moves versus user manipulation.
  enum {
    kDefaultInvalid = -1234567
  };

  int index_{kDefaultInvalid};

  // The largest representable index.
  // Note: The handling of comparisons of TypeSafeIndex with unsigned integral
  // types with *fewer* bits relies on truncations of *this* value consisting
  // of all 1s. The maximum int satisfies that requirement. If, for whatever
  // reason, some *alternative* maximum index is preferred (or the underlying
  // integral type of TypeSafeIndex changes), keep this requirement in mind.
  // Otherwise, comparisons against smaller unsigned integral types is likely
  // to fail.
  static constexpr int kMaxIndex = std::numeric_limits<int>::max();
};

template <typename Tag, typename U>
typename std::enable_if_t<
    std::is_integral_v<U> && std::is_unsigned_v<U>, bool>
operator==(const U& value, const TypeSafeIndex<Tag>& tag) {
  return tag.operator==(value);
}

template <typename Tag, typename U>
typename std::enable_if_t<
    std::is_integral_v<U> && std::is_unsigned_v<U>, bool>
operator!=(const U& value, const TypeSafeIndex<Tag>& tag) {
  return tag.operator!=(value);
}

template <typename Tag, typename U>
typename std::enable_if_t<
    std::is_integral_v<U> && std::is_unsigned_v<U>, bool>
operator<(const U& value, const TypeSafeIndex<Tag>& tag) {
  return tag >= value;
}

template <typename Tag, typename U>
typename std::enable_if_t<
    std::is_integral_v<U> && std::is_unsigned_v<U>, bool>
operator<=(const U& value, const TypeSafeIndex<Tag>& tag) {
  return tag > value;
}

template <typename Tag, typename U>
typename std::enable_if_t<
    std::is_integral_v<U> && std::is_unsigned_v<U>, bool>
operator>(const U& value, const TypeSafeIndex<Tag>& tag) {
  return tag <= value;
}

template <typename Tag, typename U>
typename std::enable_if_t<
    std::is_integral_v<U> && std::is_unsigned_v<U>, bool>
operator>=(const U& value, const TypeSafeIndex<Tag>& tag) {
  return tag < value;
}

}  // namespace drake

namespace std {

/// Enables use of the type-safe index to serve as a key in STL containers.
/// @relates TypeSafeIndex
template <typename Tag>
struct hash<drake::TypeSafeIndex<Tag>> : public drake::DefaultHash {};
}  // namespace std
