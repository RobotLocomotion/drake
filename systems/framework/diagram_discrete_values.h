#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/pointer_cast.h"
#include "drake/common/value.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/discrete_values.h"
#include "drake/systems/framework/framework_common.h"

namespace drake {
namespace systems {

/// DiagramDiscreteValues is a DiscreteValues container comprised recursively of
/// a sequence of child DiscreteValues objects. The API allows this to be
/// treated as though it were a single DiscreteValues object whose groups are
/// the concatenation of the groups in each child.
///
/// The child objects may be owned or not. When this is used to aggregate
/// LeafSystem discrete values in a Diagram, the child objects are not owned.
/// When this is cloned, deep copies are made that are owned here.
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
    DRAKE_ASSERT(internal::IsNonNull(subdiscretes_));
  }

  /// Constructs a DiagramDiscreteValues object that is composed (recursively)
  /// of other DiscreteValues objects, ownership of which is transferred here.
  explicit DiagramDiscreteValues(
      std::vector<std::unique_ptr<DiscreteValues<T>>> owned_subdiscretes)
      : DiagramDiscreteValues<T>(internal::Unpack(owned_subdiscretes)) {
    owned_subdiscretes_ = std::move(owned_subdiscretes);
    DRAKE_ASSERT(internal::IsNonNull(owned_subdiscretes_));
  }

  /// Destructor deletes any owned DiscreteValues objects but does nothing if
  /// the referenced DiscreteValues objects are unowned.
  ~DiagramDiscreteValues() override {}

  /// Creates a deep copy of this %DiagramDiscreteValues object, with the same
  /// substructure but with new, owned data. Intentionally shadows the
  /// DiscreteValues::Clone() method but with a more-specific return type so
  /// you don't have to downcast.
  std::unique_ptr<DiagramDiscreteValues> Clone() const {
    // Note that DoClone() below cannot be overridden so we can count on the
    // concrete type being DiagramDiscreteValues.
    return static_pointer_cast<DiagramDiscreteValues>(DoClone());
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
  // The returned concrete object is a DiagramDiscreteValues<T>.
  std::unique_ptr<DiscreteValues<T>> DoClone() const final {
    std::vector<std::unique_ptr<DiscreteValues<T>>> owned_subdiscretes;
    // Make deep copies regardless of whether they were owned.
    for (auto discrete : subdiscretes_)
      owned_subdiscretes.push_back(discrete->Clone());
    return std::make_unique<DiagramDiscreteValues>(
        std::move(owned_subdiscretes));
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

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiagramDiscreteValues)
