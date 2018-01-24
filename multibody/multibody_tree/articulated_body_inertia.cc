#include "drake/multibody/multibody_tree/articulated_body_inertia.h"

namespace drake {
namespace multibody {

// Explicitly instantiates on the most common scalar types.
template class ArticulatedBodyInertia<double>;
template class ArticulatedBodyInertia<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
