#include "drake/multibody/plant/deformable_body.h"
#include "drake/multibody/tree/element_collection-inl.h"

namespace drake {
namespace multibody {
namespace internal {

using symbolic::Expression;

template class ElementCollection<double, DeformableBody, DeformableBodyIndex>;
template class ElementCollection<AutoDiffXd, DeformableBody,
                                 DeformableBodyIndex>;
template class ElementCollection<Expression, DeformableBody,
                                 DeformableBodyIndex>;

}  // namespace internal
}  // namespace multibody
}  // namespace drake
