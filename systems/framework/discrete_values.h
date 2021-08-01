#pragma once

#include <memory>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/drake_throw.h"
#include "drake/common/value.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/scalar_conversion_traits.h"

namespace drake {
namespace systems {

/// %DiscreteValues is a container for numerical but non-continuous state
/// and parameters. It may own its underlying data, for use with leaf Systems,
/// or not, for use with Diagrams.
///
/// %DiscreteValues is an ordered collection xd of BasicVector "groups" so
/// xd = [xd₀, xd₁...], where each group xdᵢ is a contiguous vector. Requesting
/// a specific group index from this collection is the most granular way
/// to retrieve discrete values from the Context, and thus is the unit of
/// cache invalidation. System authors are encouraged to partition their
/// DiscreteValues such that each cacheable computation within the System may
/// depend on only the elements of DiscreteValues that it needs.
///
/// None of the contained vectors (groups) may be null, although any of them may
/// be zero-length.
///
/// @tparam_default_scalar
template <typename T>
class DiscreteValues {
 public:
  // DiscreteValues is not copyable or moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiscreteValues)

  /// Constructs an empty %DiscreteValues object containing no groups.
  DiscreteValues() {}

  /// Constructs a %DiscreteValues that does not own the underlying @p data.
  /// The referenced data must outlive this DiscreteValues. Every entry must be
  /// non-null.
  /// @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
  explicit DiscreteValues(const std::vector<BasicVector<T>*>& data)
      : data_(data) {
    for (BasicVector<T>* basic_vector_ptr : data_) {
      if (basic_vector_ptr == nullptr)
        throw std::logic_error("DiscreteValues: null groups not allowed");
    }
  }

  /// Constructs a %DiscreteValues that owns the underlying @p data. Every entry
  /// must be non-null.
  explicit DiscreteValues(std::vector<std::unique_ptr<BasicVector<T>>>&& data)
      : owned_data_(std::move(data)) {
    // Initialize the unowned pointers.
    for (auto& datum : owned_data_) {
      if (datum == nullptr)
        throw std::logic_error("DiscreteValues: null groups not allowed");
      data_.push_back(datum.get());
    }
  }

  /// Constructs a one-group %DiscreteValues object that owns a single @p datum
  /// vector which may not be null.
  explicit DiscreteValues(std::unique_ptr<BasicVector<T>> datum) {
    AppendGroup(std::move(datum));
  }

  /// Adds an additional group that owns the given @p datum, which must be
  /// non-null. Returns the assigned group number, counting up from 0 for the
  /// first group.
  int AppendGroup(std::unique_ptr<BasicVector<T>> datum) {
    if (datum == nullptr) {
      throw std::logic_error(
          "DiscreteValues::AppendGroup(): null groups not allowed");
    }
    const int group_num = static_cast<int>(data_.size());
    data_.push_back(datum.get());
    owned_data_.push_back(std::move(datum));
    return group_num;
  }

  virtual ~DiscreteValues() {}

  int num_groups() const { return static_cast<int>(data_.size()); }

  const std::vector<BasicVector<T>*>& get_data() const { return data_; }

  //----------------------------------------------------------------------------
  /// @name Convenience accessors for %DiscreteValues with just one group.
  /// These will throw if there is not exactly one group in this %DiscreteValues
  /// object.
  //@{

  /// Returns the number of elements in the only %DiscreteValues group.
  int size() const {
    return get_vector().size();
  }

  /// Returns a mutable reference to an element in the _only_ group.
  T& operator[](std::size_t idx) {
    return get_mutable_vector()[idx];
  }

  /// Returns a const reference to an element in the _only_ group.
  const T& operator[](std::size_t idx) const {
    return get_vector()[idx];
  }

  /// Returns a const reference to the BasicVector containing the values for
  /// the _only_ group.
  const BasicVector<T>& get_vector() const {
    ThrowUnlessExactlyOneGroup();
    return get_vector(0);
  }

  /// Returns a mutable reference to the BasicVector containing the values for
  /// the _only_ group.
  BasicVector<T>& get_mutable_vector() {
    ThrowUnlessExactlyOneGroup();
    return get_mutable_vector(0);
  }

  /// Sets the vector to the given value for the _only_ group.
  void set_value(const Eigen::Ref<const VectorX<T>>& value) {
    ThrowUnlessExactlyOneGroup();
    get_mutable_vector(0).set_value(value);
  }

  /// Returns the entire vector as a const Eigen::VectorBlock for the _only_
  /// group.
  Eigen::VectorBlock<const VectorX<T>> get_value() const {
    ThrowUnlessExactlyOneGroup();
    return get_vector(0).get_value();
  }

  /// Returns the entire vector for the _only_ group as a mutable
  /// Eigen::VectorBlock, which allows mutation of the values, but does not
  /// allow resize() to be called on the vector.
  Eigen::VectorBlock<VectorX<T>> get_mutable_value() {
    ThrowUnlessExactlyOneGroup();
    return get_mutable_vector(0).get_mutable_value();
  }

  //@}

  /// Returns a const reference to the vector holding data for the indicated
  /// group.
  const BasicVector<T>& get_vector(int index) const {
    DRAKE_THROW_UNLESS(0 <= index && index < num_groups());
    return *data_[index];
  }

  /// Returns a mutable reference to the vector holding data for the indicated
  /// group.
  BasicVector<T>& get_mutable_vector(int index) {
    DRAKE_THROW_UNLESS(0 <= index && index < num_groups());
    return *data_[index];
  }

  /// Returns the entire vector as a const Eigen::VectorBlock for the indicated
  /// group.
  Eigen::VectorBlock<const VectorX<T>> get_value(int index) const {
    DRAKE_THROW_UNLESS(0 <= index && index < num_groups());
    return data_[index]->get_value();
  }

  /// Returns the entire vector for the indicated group as a mutable
  /// Eigen::VectorBlock, which allows mutation of the values, but does not
  /// allow resize() to be called on the vector.
  Eigen::VectorBlock<VectorX<T>> get_mutable_value(int index) {
    DRAKE_THROW_UNLESS(0 <= index && index < num_groups());
    return data_[index]->get_mutable_value();
  }

  /// Sets the vector to the given value for the indicated group.
  void set_value(
      int index, const Eigen::Ref<const VectorX<T>>& value) {
    get_mutable_vector(index).set_value(value);
  }

  /// Resets the values in this DiscreteValues from the values in @p other,
  /// possibly writing through to unowned data. Throws if the dimensions don't
  /// match.
  template <typename U>
  void SetFrom(const DiscreteValues<U>& other) {
    DRAKE_THROW_UNLESS(num_groups() == other.num_groups());
    for (int i = 0; i < num_groups(); i++) {
      BasicVector<T>& this_i = *data_[i];
      const BasicVector<U>& other_i = other.get_vector(i);
      DRAKE_THROW_UNLESS(this_i.size() == other_i.size());
      this_i.get_mutable_value() = other_i.get_value().unaryExpr(
          scalar_conversion::ValueConverter<T, U>{});
    }
  }

  /// Creates a deep copy of this object with the same substructure but with
  /// all data owned by the copy. That is, if the original was a
  /// DiagramDiscreteValues object that maintains a tree of sub-objects, the
  /// clone will not include any references to the original sub-objects and is
  /// thus decoupled from the Context containing the original. The concrete
  /// type of the BasicVector underlying each leaf DiscreteValue is preserved.
  std::unique_ptr<DiscreteValues<T>> Clone() const {
    auto result = std::unique_ptr<DiscreteValues<T>>(DoClone());
    result->set_system_id(get_system_id());
    return result;
  }

  /// @name System compatibility
  /// See @ref system_compatibility.
  //@{
  /// (Internal use only) Gets the id of the subsystem that created this object.
  internal::SystemId get_system_id() const { return system_id_; }

  /// (Internal use only) Records the id of the subsystem that created this
  /// object.
  void set_system_id(internal::SystemId id) { system_id_ = id; }
  //@}

 private:
  // Throw unless this object is compatible with convenience methods; i.e., it
  // has exactly one group.
  void ThrowUnlessExactlyOneGroup() const {
    static constexpr char message[] =
        "Cannot use DiscreteValues convenience methods unless there is"
        " exactly one group. num_groups() = {}";
    if (num_groups() != 1) {
      throw std::logic_error(fmt::format(message, num_groups()));
    }
  }

  // DiagramDiscreteValues must override this to maintain the necessary
  // internal substructure, and to perform a deep copy so that the result
  // owns all its own data. The default implementation here requires that this
  // is a leaf DiscreteValues object so that we need only clone BasicVectors.
  // The implementation should not set_system_id on the result, the caller
  // will set an id on the object after this method returns.
  virtual std::unique_ptr<DiscreteValues<T>> DoClone() const {
    std::vector<std::unique_ptr<BasicVector<T>>> cloned_data;
    // Make deep copies regardless of previous ownership.
    cloned_data.reserve(data_.size());
    for (const BasicVector<T>* datum : data_)
      cloned_data.push_back(datum->Clone());
    return std::make_unique<DiscreteValues<T>>(std::move(cloned_data));
  }

  // Pointers to the data comprising the values. If the data is owned,
  // corresponding pointers are stored in owned_data_.
  std::vector<BasicVector<T>*> data_;
  // Owned pointers to the data comprising the values. The only purpose of these
  // pointers is to maintain ownership in leaf DiscreteValues. They may be
  // populated at construction/append time, and are never accessed thereafter.
  std::vector<std::unique_ptr<BasicVector<T>>> owned_data_;

  // Unique id of the subsystem that created this object.
  internal::SystemId system_id_;
};

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiscreteValues)

}  // namespace systems
}  // namespace drake
