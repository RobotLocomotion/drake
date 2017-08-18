#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

//==============================================================================
//                             DISCRETE VALUES
//==============================================================================
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

  /// Constructs an empty %DiscreteValues object containing no groups.
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

  /// Creates a deep copy of this object with the same substructure but with all
  /// data owned by the copy. That is, if the original was a
  /// DiagramDiscreteValues object that maintains a tree of substates, the clone
  /// will not include any references to the original substates and is thus
  /// decoupled from the Context containing the original. The concrete type of
  /// the BasicVector underlying each leaf DiscreteValue is preserved.
  std::unique_ptr<DiscreteValues<T>> Clone() const {
    return std::unique_ptr<DiscreteValues<T>>(DoClone());
  }

 private:
  // DiagramDiscreteValues must override this to maintain the necessary
  // internal substructure, and to perform a deep copy so that the result
  // owns all its own data. The default implementation here requires that this
  // is a leaf DiscreteValues object so that we need only clone BasicVectors.
  virtual DiscreteValues* DoClone() const {
    std::vector<std::unique_ptr<BasicVector<T>>> cloned_data;
    // Make deep copies regardless of previous ownership.
    for (const BasicVector<T>* datum : data_)
      cloned_data.push_back(datum->Clone());
    return new DiscreteValues(std::move(cloned_data));
  }

  template <typename U>
  void SetFromGeneric(const DiscreteValues<U>& other) {
    DRAKE_ASSERT(num_groups() == other.num_groups());
    for (int i = 0; i < num_groups(); i++) {
      DRAKE_ASSERT(data_[i] != nullptr);
      data_[i]->set_value(
          other.get_vector(i).get_value().template cast<T>());
    }
  }

  // Pointers to the data comprising the values. If the data is owned, these
  // pointers are equal to the pointers in owned_data_.
  std::vector<BasicVector<T>*> data_;
  // Owned pointers to the data comprising the values. The only purpose of these
  // pointers is to maintain ownership in leaf DiscreteValues. They may be
  // populated at construction time, and are never accessed thereafter.
  std::vector<std::unique_ptr<BasicVector<T>>> owned_data_;
};


//==============================================================================
//                          DIAGRAM DISCRETE VALUES
//==============================================================================
/// DiagramDiscreteValues is a version of DiscreteValue that owns
/// the constituent discrete states. As the name implies, it is only useful
/// for the discrete updates.
template <typename T>
class DiagramDiscreteValues final: public DiscreteValues<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiagramDiscreteValues)

  /// Constructs a DiagramDiscreteValues object that is composed of other
  /// DiscreteValues, which are not owned by this object and must outlive it.
  ///
  /// The DiagramDiscreteValues vector xd = [xd₁ xd₂ ...] where each of the xdᵢ
  /// is an array of BasicVector objects. These will have the same
  /// ordering as the @p subdiscretes parameter, which should be the order of
  /// the Diagram itself. That is, the substates should be indexed by
  /// SubsystemIndex in the same order as the subsystems are.
  explicit DiagramDiscreteValues(std::vector<DiscreteValues<T>*> subdiscretes)
      : DiscreteValues<T>(Flatten(subdiscretes)),
        subdiscretes_(std::move(subdiscretes)) {
    DRAKE_ASSERT_VOID(internal::CheckNonNull(subdiscretes_));
  }

  /// Constructs a DiagramDiscreteValues object that is composed (recursively)
  /// of other DiscreteValues objects, ownership of which is transferred here.
  explicit DiagramDiscreteValues(
      std::vector<std::unique_ptr<DiscreteValues<T>>>&& owned_subdiscretes)
      : DiagramDiscreteValues<T>(internal::Unpack(owned_subdiscretes)) {
    owned_subdiscretes_ = std::move(owned_subdiscretes);
    DRAKE_ASSERT_VOID(internal::CheckNonNull(owned_subdiscretes_));
  }

  /// Destructor deletes any owned DiscreteValues objects but does nothing if
  /// the referenced DiscreteValues objects are unowned.
  ~DiagramDiscreteValues() override {}

  /// Creates a deep copy of this %DiagramDiscreteValues object, with the same
  /// substructure but with new, owned data. Intentionally shadows the
  /// DiscreteValues::Clone() method but with a more-specific return type so
  /// you don't have to downcast.
  std::unique_ptr<DiagramDiscreteValues> Clone() const {
    return std::unique_ptr<DiagramDiscreteValues>(DoClone());
  }

  /// Returns the number of DiscreteValues objects referenced by this
  /// %DiagramDiscreteValues object, necessarily the same as the number of
  /// subcontexts in the containing DiagramContext.
  int num_subdiscretes() const {
    return static_cast<int>(subdiscretes_.size());
  }

  /// Returns a const reference to one of the referenced DiscreteValues
  /// objects which may or may not be owned locally.
  const DiscreteValues<T>& get_subdiscrete(SubsystemIndex index) const {
    DRAKE_DEMAND(0 <= index && index < num_subdiscretes());
    DRAKE_DEMAND(subdiscretes_[index] != nullptr);
    return *subdiscretes_[index];
  }

  /// Returns a mutable reference to one of the referenced DiscreteValues
  /// objects which may or may not be owned locally.
  DiscreteValues<T>& get_mutable_subdiscrete(SubsystemIndex index) {
    return const_cast<DiscreteValues<T>&>(get_subdiscrete(index));
  }

 private:
  // This completely replaces the base class default DoClone() so must
  // take care of the base class members as well as the local ones. That's done
  // by invoking a constructor that delegates to the base.
  DiagramDiscreteValues* DoClone() const override {
    std::vector<std::unique_ptr<DiscreteValues<T>>> owned_subdiscretes;
    // Make deep copies regardless of whether they were owned.
    for (auto discrete : subdiscretes_)
      owned_subdiscretes.push_back(discrete->Clone());
    return new DiagramDiscreteValues(std::move(owned_subdiscretes));
  }

  // Given a vector of DiscreteValues pointers, each potentially containing
  // multiple BasicVectors, return a longer vector of pointers to all the
  // contained BasicVectors, unpacked in order. Because each of the referenced
  // DiscreteValues has been similarly unpacked, the result is a complete, flat
  // list of all the leaf BasicVectors below `this` DiagramDiscreteValues.
  std::vector<BasicVector<T>*> Flatten(
      const std::vector<DiscreteValues<T>*>& in) const {
    std::vector<BasicVector<T>*> out;
    for (const DiscreteValues<T>* xd : in) {
      const std::vector<BasicVector<T>*>& xd_data = xd->get_data();
      out.insert(out.end(), xd_data.begin(), xd_data.end());
    }
    return out;
  }

  // Pointers to the underlying DiscreteValues objects that provide the actual
  // values. If these are owned, the pointers are equal to the pointers in
  // owned_subdiscretes_.
  std::vector<DiscreteValues<T>*> subdiscretes_;

  // Owned pointers to DiscreteValues objects that hold the actual values.
  // The only purpose of these pointers is to maintain ownership. They may be
  // populated at construction time, and are never accessed thereafter.
  std::vector<std::unique_ptr<DiscreteValues<T>>> owned_subdiscretes_;
};

}  // namespace systems
}  // namespace drake
