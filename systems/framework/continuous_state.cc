#include "drake/systems/framework/continuous_state.h"

#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/subvector.h"

namespace drake {
namespace systems {

template <typename T>
ContinuousState<T>::ContinuousState(std::unique_ptr<VectorBase<T>> state) {
  state_ = std::move(state);
  generalized_position_.reset(
      new Subvector<T>(state_.get(), 0, 0));
  generalized_velocity_.reset(
      new Subvector<T>(state_.get(), 0, 0));
  misc_continuous_state_.reset(
      new Subvector<T>(state_.get(), 0, state_->size()));
  DRAKE_ASSERT_VOID(DemandInvariants());
}

template <typename T>
ContinuousState<T>::ContinuousState(
    std::unique_ptr<VectorBase<T>> state, int num_q, int num_v, int num_z) {
  state_ = std::move(state);
  if (state_->size() != num_q + num_v + num_z) {
    throw std::out_of_range(
        "Continuous state of size " + std::to_string(state_->size()) +
        "cannot be partitioned as" + " q " + std::to_string(num_q) + " v " +
        std::to_string(num_v) + " z " + std::to_string(num_z));
  }
  if (num_v > num_q) {
    throw std::logic_error("Number of velocity variables " +
                           std::to_string(num_v) +
                           " must not exceed number of position variables " +
                           std::to_string(num_q));
  }
  generalized_position_.reset(new Subvector<T>(state_.get(), 0, num_q));
  generalized_velocity_.reset(new Subvector<T>(state_.get(), num_q, num_v));
  misc_continuous_state_.reset(
      new Subvector<T>(state_.get(), num_q + num_v, num_z));
  DRAKE_ASSERT_VOID(DemandInvariants());
}

template <typename T>
ContinuousState<T>::ContinuousState()
    : ContinuousState(std::make_unique<BasicVector<T>>(0)) {}

template <typename T>
ContinuousState<T>::~ContinuousState() {}

template <typename T>
std::unique_ptr<ContinuousState<T>> ContinuousState<T>::Clone() const {
  auto result = DoClone();
  result->set_system_id(this->get_system_id());
  return result;
}

template <typename T>
ContinuousState<T>::ContinuousState(
    std::unique_ptr<VectorBase<T>> state,
    std::unique_ptr<VectorBase<T>> q,
    std::unique_ptr<VectorBase<T>> v,
    std::unique_ptr<VectorBase<T>> z)
    : state_(std::move(state)),
      generalized_position_(std::move(q)),
      generalized_velocity_(std::move(v)),
      misc_continuous_state_(std::move(z)) {
  DRAKE_ASSERT_VOID(DemandInvariants());
}

template <typename T>
std::unique_ptr<ContinuousState<T>> ContinuousState<T>::DoClone() const {
  auto state = dynamic_cast<const BasicVector<T>*>(state_.get());
  DRAKE_DEMAND(state != nullptr);
  return std::make_unique<ContinuousState>(state->Clone(), num_q(), num_v(),
                                           num_z());
}

template <typename T>
void ContinuousState<T>::DemandInvariants() const {
  // Nothing is nullptr.
  DRAKE_DEMAND(generalized_position_ != nullptr);
  DRAKE_DEMAND(generalized_velocity_ != nullptr);
  DRAKE_DEMAND(misc_continuous_state_ != nullptr);

  // The sizes are consistent.
  DRAKE_DEMAND(num_q() >= 0);
  DRAKE_DEMAND(num_v() >= 0);
  DRAKE_DEMAND(num_z() >= 0);
  DRAKE_DEMAND(num_v() <= num_q());
  const int num_total = (num_q() + num_v() + num_z());
  DRAKE_DEMAND(state_->size() == num_total);

  // The storage addresses of `state_` elements contain no duplicates.
  std::unordered_set<const T*> state_element_pointers;
  for (int i = 0; i < num_total; ++i) {
    const T* element = &(state_->GetAtIndex(i));
    state_element_pointers.emplace(element);
  }
  DRAKE_DEMAND(static_cast<int>(state_element_pointers.size()) == num_total);

  // The storage addresses of (q, v, z) elements contain no duplicates, and
  // are drawn from the set of storage addresses of `state_` elements.
  // Therefore, the `state_` vector and (q, v, z) vectors form views into the
  // same unique underlying data, just with different indexing.
  std::unordered_set<const T*> qvz_element_pointers;
  for (int i = 0; i < num_q(); ++i) {
    const T* element = &(generalized_position_->GetAtIndex(i));
    qvz_element_pointers.emplace(element);
    DRAKE_DEMAND(state_element_pointers.count(element) == 1);
  }
  for (int i = 0; i < num_v(); ++i) {
    const T* element = &(generalized_velocity_->GetAtIndex(i));
    qvz_element_pointers.emplace(element);
    DRAKE_DEMAND(state_element_pointers.count(element) == 1);
  }
  for (int i = 0; i < num_z(); ++i) {
    const T* element = &(misc_continuous_state_->GetAtIndex(i));
    qvz_element_pointers.emplace(element);
    DRAKE_DEMAND(state_element_pointers.count(element) == 1);
  }
  DRAKE_DEMAND(static_cast<int>(qvz_element_pointers.size()) == num_total);
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::ContinuousState)
