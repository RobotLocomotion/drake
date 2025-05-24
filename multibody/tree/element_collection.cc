#include "drake/multibody/tree/element_collection.h"

#include <stdexcept>
#include <string>
#include <utility>

#include <fmt/format.h>

#include "drake/common/nice_type_name.h"
#include "drake/common/unused.h"
#include "drake/multibody/tree/deformable_body.h"
#include "drake/multibody/tree/joint.h"
#include "drake/multibody/tree/joint_actuator.h"
#include "drake/multibody/tree/model_instance.h"
#include "drake/multibody/tree/rigid_body.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T, template <typename> class Element, typename Index>
ElementCollection<T, Element, Index>::ElementCollection() = default;

template <typename T, template <typename> class Element, typename Index>
ElementCollection<T, Element, Index>::~ElementCollection() = default;

template <typename T, template <typename> class Element, typename Index>
Element<T>& ElementCollection<T, Element, Index>::Add(
    std::unique_ptr<Element<T>> element) {
  return AddImpl(std::move(element));
}

template <typename T, template <typename> class Element, typename Index>
Element<T>& ElementCollection<T, Element, Index>::AddBorrowed(
    Element<T>* element) {
  // The shared_ptr will point to `element`, but with a no-op deleter.
  using Shared = std::shared_ptr<Element<T>>;
  return AddImpl(Shared(Shared{}, element));
}

template <typename T, template <typename> class Element, typename Index>
Element<T>& ElementCollection<T, Element, Index>::AddImpl(
    std::shared_ptr<Element<T>> maybe_owned) {
  Element<T>* element = maybe_owned.get();
  DRAKE_DEMAND(element != nullptr);
  const Index index = element->index();
  if (index == ssize(elements_by_index_)) {
    elements_by_index_.push_back(std::move(maybe_owned));
    names_map_.emplace(std::string(element->name()), index);
    elements_packed_.push_back(element);
    indices_packed_.push_back(index);
  } else {
    DRAKE_DEMAND(index >= 0);
    DRAKE_DEMAND(index < ssize(elements_by_index_));
    DRAKE_DEMAND(elements_by_index_[index] == nullptr);
    elements_by_index_[index] = std::move(maybe_owned);
    names_map_.emplace(std::string(element->name()), index);
    // The elements_packed and indices_packed represent congruent views over the
    // non-null items in elements_by_index, with the former viewing the element
    // pointers and the latter viewing the element indices. To figure out where
    // to insert this new element, we'll do a binary search for where its index
    // should live within indices_packed. Because the two vectors are congruent,
    // that same offset is also applicable to elements_packed.
    const size_t packed_offset =
        std::distance(indices_packed_.begin(),
                      std::lower_bound(indices_packed_.begin(),
                                       indices_packed_.end(), index));
    elements_packed_.insert(elements_packed_.begin() + packed_offset, element);
    indices_packed_.insert(indices_packed_.begin() + packed_offset, index);
  }
  return *element;
}

template <typename T, template <typename> class Element, typename Index>
typename string_unordered_multimap<Index>::const_iterator
ElementCollection<T, Element, Index>::FindNamesIterator(std::string_view name,
                                                        Index index) const {
  auto [lower, upper] = names_map_.equal_range(name);
  for (auto iter = lower; iter != upper; ++iter) {
    if (iter->second == index) {
      return iter;
    }
  }
  return names_map_.end();
}

template <typename T, template <typename> class Element, typename Index>
void ElementCollection<T, Element, Index>::Remove(Index index) {
  // Find the name in the map *before* we delete the element; this also
  // validates the index by side effect of calling get_element().
  const auto names_iter = FindNamesIterator(get_element(index).name(), index);
  DRAKE_DEMAND(names_iter != names_map_.end());

  // Delete the element.
  elements_by_index_[index] = nullptr;

  // De-index the name.
  names_map_.erase(names_iter);

  // Remove it from the packed vectors caches.
  const size_t packed_offset = std::distance(
      indices_packed_.begin(),
      std::lower_bound(indices_packed_.begin(), indices_packed_.end(), index));
  DRAKE_DEMAND(indices_packed_.at(packed_offset) == index);
  elements_packed_.erase(elements_packed_.begin() + packed_offset);
  indices_packed_.erase(indices_packed_.begin() + packed_offset);
}

template <typename T, template <typename> class Element, typename Index>
void ElementCollection<T, Element, Index>::Rename(Index index,
                                                  std::string name) {
  // Currently, ModelInstance is the only Element that has `set_name`.
  if constexpr (std::is_same_v<Element<T>, ModelInstance<T>>) {
    const auto old_name_iter =
        FindNamesIterator(get_element(index).name(), index);
    DRAKE_DEMAND(old_name_iter != names_map_.end());
    names_map_.erase(old_name_iter);
    names_map_.emplace(name, index);
    get_mutable_element(index).set_name(std::move(name));
  } else {
    DRAKE_UNREACHABLE();
  }
}

template <typename T, template <typename> class Element, typename Index>
void ElementCollection<T, Element, Index>::AppendNull() {
  elements_by_index_.push_back(nullptr);
}

namespace {
std::string RemoveTemplates(std::string type_name) {
  const auto offset = type_name.find('<');
  DRAKE_DEMAND(offset != std::string::npos);
  type_name.erase(offset);
  return type_name;
}
}  // namespace

template <typename T, template <typename> class Element, typename Index>
void ElementCollection<T, Element, Index>::ThrowInvalidIndexException(
    Index index) const {
  const std::string nice_type = RemoveTemplates(
      NiceTypeName::RemoveNamespaces(NiceTypeName::Get<Element<T>>()));
  if (!index.is_valid()) {
    throw std::logic_error(fmt::format(
        "The given default-constructed {}Index() cannot be used. You must "
        "pass a valid integer as the index.",
        nice_type));
  }
  if (index >= next_index()) {
    throw std::logic_error(fmt::format(
        "The given {}Index({}) is out of bounds (must be less than {})",
        nice_type, index, next_index()));
  }
  throw std::logic_error(
      fmt::format("The {}Index({}) has been removed", nice_type, index));
}

using symbolic::Expression;

template class ElementCollection<double, Frame, FrameIndex>;
template class ElementCollection<AutoDiffXd, Frame, FrameIndex>;
template class ElementCollection<Expression, Frame, FrameIndex>;

template class ElementCollection<double, Joint, JointIndex>;
template class ElementCollection<AutoDiffXd, Joint, JointIndex>;
template class ElementCollection<Expression, Joint, JointIndex>;

template class ElementCollection<double, JointActuator, JointActuatorIndex>;
template class ElementCollection<AutoDiffXd, JointActuator, JointActuatorIndex>;
template class ElementCollection<Expression, JointActuator, JointActuatorIndex>;

template class ElementCollection<double, ModelInstance, ModelInstanceIndex>;
template class ElementCollection<AutoDiffXd, ModelInstance, ModelInstanceIndex>;
template class ElementCollection<Expression, ModelInstance, ModelInstanceIndex>;

template class ElementCollection<double, RigidBody, BodyIndex>;
template class ElementCollection<AutoDiffXd, RigidBody, BodyIndex>;
template class ElementCollection<Expression, RigidBody, BodyIndex>;

template class ElementCollection<double, DeformableBody, DeformableBodyIndex>;
template class ElementCollection<AutoDiffXd, DeformableBody,
                                 DeformableBodyIndex>;
template class ElementCollection<Expression, DeformableBody,
                                 DeformableBodyIndex>;

}  // namespace internal
}  // namespace multibody
}  // namespace drake
