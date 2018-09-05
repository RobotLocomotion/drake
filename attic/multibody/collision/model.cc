#include "drake/multibody/collision/model.h"

#include <iostream>

#include "drake/common/drake_assert.h"

using Eigen::Isometry3d;
using std::move;
using std::unique_ptr;
using std::vector;

namespace drake {
namespace multibody {
namespace collision {

Element* Model::AddElement(std::unique_ptr<Element> element) {
  ElementId id = element->getId();
  const auto& itr = elements.find(id);
  if (itr == elements.end()) {
    elements.insert(make_pair(id, move(element)));
    Element* raw_element = elements[id].get();
    DoAddElement(*raw_element);
    return raw_element;
  }
  throw std::runtime_error(
      "Attempting to add an element with a duplicate"
      "id: " +
      std::to_string(id));
}

bool Model::RemoveElement(ElementId id) {
  DoRemoveElement(id);
  return elements.erase(id) > 0;
}

const Element* Model::FindElement(ElementId id) const {
  auto element_iter = elements.find(id);
  if (element_iter != elements.end()) {
    return element_iter->second.get();
  } else {
    return nullptr;
  }
}

Element* Model::FindMutableElement(ElementId id) {
  auto element_iter = elements.find(id);
  if (element_iter != elements.end()) {
    return element_iter->second.get();
  } else {
    return nullptr;
  }
}

void Model::GetTerrainContactPoints(ElementId id0,
                                    Eigen::Matrix3Xd* terrain_points) {
  DRAKE_DEMAND(terrain_points != nullptr);
  auto element_iter = elements.find(id0);
  if (element_iter != elements.end()) {
    element_iter->second->getTerrainContactPoints(*terrain_points);
  } else {
    *terrain_points = Eigen::Matrix3Xd();
  }
}

bool Model::UpdateElementWorldTransform(ElementId id,
                                        const Isometry3d& X_WL) {
  auto elem_itr = elements.find(id);
  if (elem_itr != elements.end()) {
    elem_itr->second->updateWorldTransform(X_WL);
    return true;
  } else {
    return false;
  }
}

bool Model::TransformCollisionFrame(
    const drake::multibody::collision::ElementId& eid,
    const Eigen::Isometry3d& transform_body_to_joint) {
  auto element = elements.find(eid);
  if (element != elements.end()) {
    element->second->SetLocalTransform(transform_body_to_joint *
                                       element->second->getLocalTransform());
    return true;
  } else {
    return false;
  }
}

void Model::DoAddElement(const Element&) {}

void Model::DoRemoveElement(ElementId) {}

/**
 * A toString for the collision model.
 */
std::ostream& operator<<(std::ostream& os, const Model& model) {
  if (model.elements.size() == 0) {
    os << "No collision elements.";
  } else {
    for (auto it = model.elements.begin(); it != model.elements.end(); ++it)
      os << " ElementID: " << it->first << ":\n"
         << "    - world transform:\n"
         << it->second->getWorldTransform().matrix() << "\n"
         << "    - local transform:\n"
         << it->second->getLocalTransform().matrix() << "\n"
         << "    - has geometry? " << (it->second->hasGeometry() ? "yes" : "no")
         << "\n";
  }
  return os;
}

}  // namespace collision
}  // namespace multibody
}  // namespace drake
