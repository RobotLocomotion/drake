#include "drake/multibody/collision/collision_filter.h"

#include "drake/common/eigen_autodiff_types.h"

namespace DrakeCollision {

using drake::AutoDiffXd ;

template <typename T>
CollisionFilterGroup<T>::CollisionFilterGroup() : name_(""), model_id_(-1) {}

template <typename T>
CollisionFilterGroup<T>::CollisionFilterGroup(const std::string& name,
                                           int model_id)
    : name_(name), model_id_(model_id) {}

// Explicitly instantiates on the most common scalar types.
template class CollisionFilterGroup<double>;
template class CollisionFilterGroup<AutoDiffXd>;

}  // namespace DrakeCollision
