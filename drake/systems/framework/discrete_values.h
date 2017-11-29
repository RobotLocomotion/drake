#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/value.h"

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
/// @tparam T A mathematical type compatible with Eigen's Scalar.
template <typename T>
class DiscreteValues {
 public:
  // DiscreteValues is not copyable or moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiscreteValues)

  /// Constructs an empty %DiscreteValues.
  DiscreteValues() {}

  /// Constructs a %DiscreteValues that does not own the underlying @p data.
  /// The referenced data must outlive this DiscreteValues. Every entry must be
  /// non-null.
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
    if (datum == nullptr)
      throw std::logic_error("DiscreteValues: null groups not allowed");
    data_.push_back(datum.get());
    owned_data_.push_back(std::move(datum));
  }

  virtual ~DiscreteValues() {}

  int num_groups() const { return static_cast<int>(data_.size()); }

  const std::vector<BasicVector<T>*>& get_data() const { return data_; }

  //----------------------------------------------------------------------------
  /// @name Convenience accessors for %DiscreteValues with just one group.
  /// These will fail (at least in Debug builds) if there is more than one
  /// group in this %DiscreteValues object.
  //@{

  /// Returns the number of elements in the only %DiscreteValues group.
  int size() const {
    DRAKE_ASSERT(num_groups() == 1);
    return data_[0]->size();
  }

  /// Returns a mutable reference to an element in the _only_ group.
  T& operator[](std::size_t idx) {
    DRAKE_ASSERT(num_groups() == 1);
    return (*data_[0])[idx];
  }

  /// Returns a const reference to an element in the _only_ group.
  const T& operator[](std::size_t idx) const {
    DRAKE_ASSERT(num_groups() == 1);
    return (*data_[0])[idx];
  }

  /// Returns a const reference to the BasicVector containing the values for
  /// the _only_ group.
  const BasicVector<T>& get_vector() const {
    DRAKE_ASSERT(num_groups() == 1);
    return get_vector(0);
  }

  /// Returns a mutable reference to the BasicVector containing the values for
  /// the _only_ group.
  BasicVector<T>& get_mutable_vector() {
    DRAKE_ASSERT(num_groups() == 1);
    return get_mutable_vector(0);
  }
  //@}

  /// Returns a const reference to the vector holding data for the indicated
  /// group.
  const BasicVector<T>& get_vector(int index) const {
    DRAKE_ASSERT(index >= 0 && index < num_groups());
    DRAKE_ASSERT(data_[index] != nullptr);
    return *data_[index];
  }

  /// Returns a mutable reference to the vector holding data for the indicated
  /// group.
  BasicVector<T>& get_mutable_vector(int index) {
    DRAKE_ASSERT(index >= 0 && index < num_groups());
    DRAKE_ASSERT(data_[index] != nullptr);
    return *data_[index];
  }

  /// Writes the values from @p other into this DiscreteValues, possibly
  /// writing through to unowned data. Asserts if the dimensions don't match.
  void CopyFrom(const DiscreteValues<T>& other) { SetFromGeneric(other); }

  /// Resets the values in this DiscreteValues from the values in @p other,
  /// possibly writing through to unowned data. Asserts if the dimensions don't
  /// match.
  void SetFrom(const DiscreteValues<double>& other) { SetFromGeneric(other); }

  /// Returns a deep copy of all the data in this DiscreteValues. The clone
  /// will own its own data. This is true regardless of whether the values being
  /// cloned had ownership of its data or not.
  // TODO(sherm1) This slices DiagramDiscreteVariables and needs to be reworked.
  std::unique_ptr<DiscreteValues> Clone() const {
    std::vector<std::unique_ptr<BasicVector<T>>> cloned_data;
    cloned_data.reserve(data_.size());
    for (const BasicVector<T>* datum : data_) {
      cloned_data.push_back(datum->Clone());
    }
    return std::make_unique<DiscreteValues>(std::move(cloned_data));
  }

 private:
  // Pointers to the data comprising the values. If the data is owned, these
  // pointers are equal to the pointers in owned_data_.
  std::vector<BasicVector<T>*> data_;
  // Owned pointers to the data comprising the values. The only purpose of these
  // pointers is to maintain ownership. They may be populated at construction
  // time, and are never accessed thereafter.
  std::vector<std::unique_ptr<BasicVector<T>>> owned_data_;

  template <typename U>
  void SetFromGeneric(const DiscreteValues<U>& other) {
    DRAKE_ASSERT(num_groups() == other.num_groups());
    for (int i = 0; i < num_groups(); i++) {
      DRAKE_ASSERT(data_[i] != nullptr);
      data_[i]->set_value(
          other.get_vector(i).get_value().template cast<T>());
    }
  }
};

}  // namespace systems
}  // namespace drake
