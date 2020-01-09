#pragma once

// NOLINTNEXTLINE(whitespace/line_length)
#warning DRAKE DEPRECATED: The header drake/multibody/tree/multibody_tree_element.h is being removed on or after 2020-03-01. Please use drake/multibody/tree/multibody_element.h instead.

#include "drake/common/drake_deprecated.h"
#include "drake/multibody/tree/multibody_element.h"

namespace drake {
namespace multibody {

namespace internal {

template <class ElementType, typename ElementIndexType>
struct MultibodyTreeAlias;

template <
    template <typename> class ElementType, typename T,
    typename ElementIndexType>
struct MultibodyTreeAlias<ElementType<T>, ElementIndexType> {
  using type = MultibodyElement<ElementType, T, ElementIndexType>;
};

}  // namespace internal

template <typename... Args>
using MultibodyTreeElement
    DRAKE_DEPRECATED("2020-03-01", "Use MultibodyElement instead.")
    = typename internal::MultibodyTreeAlias<Args...>::type;

}  // namespace multibody
}  // namespace drake
