#pragma once

#include "drake/common/drake_copyable.h"

namespace drake {
namespace multibody {
namespace fem {

/** Abstract class for per element FEM data.
 @tparam_nonsymbolic_scalar */
template <typename T>
class ElementData {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FemState);

  /* Returns the number of elements that this ElementData has data for. */
  virtual size() const = 0;

 protected:
  ElementData() = default;
};

}  // namespace fem
}  // namespace multibody
}  // namespace drake
