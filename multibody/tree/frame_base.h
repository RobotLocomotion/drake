#pragma once

#include "drake/multibody/tree/multibody_element.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {

/// %FrameBase is deprecated and will be removed on or after 2025-01-01.
/// @tparam_default_scalar
template <typename T>
class FrameBase : public MultibodyElement<T> {
 public:

  // TODO(jwnimmer-tri) On 2025-01-01 move this into `class Frame`.
  /// Returns this element's unique index.
  FrameIndex index() const { return this->template index_impl<FrameIndex>(); }

  ~FrameBase() override;

 protected:
  explicit FrameBase(ModelInstanceIndex model_instance)
      : MultibodyElement<T>(model_instance) {}
};

}  // namespace multibody
}  // namespace drake
