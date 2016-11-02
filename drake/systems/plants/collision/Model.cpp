#include "drake/systems/plants/collision/Model.h"

#include <iostream>

using Eigen::Isometry3d;
using std::move;
using std::unique_ptr;
using std::vector;

namespace DrakeCollision {
ElementId Model::addElement(const Element& element) {
  unique_ptr<Element> element_local(element.clone());
  ElementId id = element_local->getId();
  this->elements.insert(make_pair(id, move(element_local)));
  return id;
}

bool Model::removeElement(const ElementId& id) {
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

void Model::getTerrainContactPoints(ElementId id0,
                                    Eigen::Matrix3Xd& terrain_points) {
  auto element_iter = elements.find(id0);
  if (element_iter != elements.end()) {
    element_iter->second->getTerrainContactPoints(terrain_points);
  } else {
    terrain_points = Eigen::Matrix3Xd();
  }
}

bool Model::updateElementWorldTransform(const ElementId id,
                                        const Isometry3d& T_elem_to_world) {
  auto elem_itr = elements.find(id);
  if (elem_itr != elements.end()) {
    elem_itr->second->updateWorldTransform(
        T_elem_to_world);  // fixme: this is taking T_local_to_world, not
                           // T_elem_to_world.  so this method name is wrong
    return true;
  } else {
    return false;
  }
}

bool Model::transformCollisionFrame(
    const DrakeCollision::ElementId& eid,
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

bool closestPointsAllToAll(
    const vector<ElementId>& ids_to_check,
    const bool use_margins,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    vector<PointPair>& closest_points) {
  return false;
}

bool collisionPointsAllToAll(
    const bool use_margins,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    vector<PointPair>& points) {
  return false;
}

bool closestPointsPairwise(
    const vector<ElementIdPair>& id_pairs,
    const bool use_margins,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    vector<PointPair>& closest_points) {
  return false;
}

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

}  // namespace DrakeCollision
