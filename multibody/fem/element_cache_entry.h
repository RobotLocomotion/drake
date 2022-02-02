#pragma once

#include <type_traits>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* ElementCacheEntry provides basic caching capabilities for the
 per-element, state-dependent quantities used in an FEM simulation that are
 not states themselves. These quantities are stored in `ElementData`.
 @tparam ElementData  The state-dependent quantities of the element that are
 stored. `ElementData` is usually of the type `FooElement::Traits::Data` where
 `FooElement`, the element that this ElementCacheEntry is holding data for,
 is the same type as the template parameter `DerivedElement` in FemElement. It
 must be default constructible and copy/move constructible/assignable. */
template <class ElementData>
class ElementCacheEntry {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ElementCacheEntry);

  static_assert(std::is_default_constructible_v<ElementData>,
                "The template parameter 'ElementData' in ElementCacheEntry "
                "must be default constructible. ");
  static_assert(std::is_copy_constructible_v<ElementData>,
                "The template parameter 'ElementData' in ElementCacheEntry "
                "must be copy constructible. ");
  static_assert(std::is_move_constructible_v<ElementData>,
                "The template parameter 'ElementData' in ElementCacheEntry "
                "must be move constructible. ");
  static_assert(std::is_copy_assignable_v<ElementData>,
                "The template parameter 'ElementData' in ElementCacheEntry "
                "must be copy assignable. ");
  static_assert(std::is_move_assignable_v<ElementData>,
                "The template parameter 'ElementData' in ElementCacheEntry "
                "must be move assignable. ");

  /* Constructs a new ElementCacheEntry with default initialized data. */
  ElementCacheEntry() {}

  /* Returns a mutable reference to the cached data. Doesn't modify the
   staleness flag. The callers should modify the staleness flag through
   set_stale() when needed.  */
  ElementData& mutable_element_data() { return element_data_; }

  const ElementData& element_data() const { return element_data_; }

  /* Returns true if the element cache entry value is out of date. */
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
