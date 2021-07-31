#pragma once

#include "drake/multibody/fixed_fem/dev/fem_indexes.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
/* %ElementCacheEntry provides basic caching capabilities for the
 per-element, state-dependent quantities used in an FEM simulation that are
 not states themselves. These quantities are stored in `ElementData`.
 @tparam ElementData    The state-dependent quantities of the element that are
 stored. `ElementData` should be of type `FooElement::Traits::Data` where
 `FooElement`, the element that this %ElementCacheEntry is holding data for,
 is the same type as the template parameter `DerivedElement` in FemElement. */
template <class ElementData>
class ElementCacheEntry {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ElementCacheEntry);

  static_assert(std::is_default_constructible_v<ElementData>,
                "The template parameter 'ElementData' in ElementCacheEntry "
                "must be default constructible. ");

  /* Constructs a new %ElementCacheEntry with default initialized data. */
  ElementCacheEntry() {}

  ElementData& mutable_element_data() { return element_data_; }

  const ElementData& element_data() const { return element_data_; }

  bool is_stale() const { return is_stale_; }

  void set_stale(bool stale) { is_stale_ = stale; }

 private:
  ElementData element_data_;
  bool is_stale_{true};
};
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
