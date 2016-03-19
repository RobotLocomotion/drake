#include "Model.h"

#include <iostream>

using namespace std;
using namespace Eigen;

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

const Element* Model::readElement(ElementId id) {
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

bool closestPointsAllToAll(const vector<ElementId>& ids_to_check,
                           const bool use_margins,
                           vector<PointPair>& closest_points) {
  return false;
};

bool collisionPointsAllToAll(const bool use_margins,
                             vector<PointPair>& points) {
  return false;
};

bool closestPointsPairwise(const vector<ElementIdPair>& id_pairs,
                           const bool use_margins,
                           vector<PointPair>& closest_points) {
  return false;
};

/**
 * A toString for the collision model.
 */
std::ostream& operator<<(std::ostream &os, const Model &model) {
  if (model.elements.size() == 0)
    os << "No collision elements.";
  else {
    for (auto it = model.elements.begin(); it != model.elements.end(); ++it)
      os << " ElementID: " << it->first << ":\n"
         << "    - world transform:\n"
         << it->second->getWorldTransform().matrix() << "\n"
         << "    - local transform:\n"
         << it->second->getLocalTransform().matrix() << "\n"
         << "    - has geometry? " << (it->second->hasGeometry() ? "yes" : "no") << "\n";
  }
  return os;
}

#define PRINT_STMT(x) std::cout << "DrakeCollision::Model: EQUAL: " << x << std::endl;

bool operator==(const Model & m1, const Model & m2) {
  PRINT_STMT("Checking if two collision models are equal...")
  bool result = true;

  for (auto it1 = m1.elements.begin(); result && it1 != m1.elements.end(); ++it1) {
    auto& element1 = it1->second;
    bool matchFound = false;
    for (auto it2 = m2.elements.begin(); !matchFound && it2 != m2.elements.end(); ++it2) {
      auto& element2 = it2->second;
      if (*element1 == *element2)
        matchFound = true;
    }
    if (!matchFound) {
      std::stringstream msg;
      msg << "No match found for LHS collision element:\n" << *element1 << "\n"
          << "Available collision elements in RHS model:\n";
      for (auto& ce : m2.elements) {
        msg << *ce.second << "\n";
      }
      PRINT_STMT(msg.str())
      result = false;
    }
  }

  return result;
}

#undef PRINT_STMT

bool operator!=(const Model & m1, const Model & m2) {
  return !operator==(m1, m2);
}

}  // namespace DrakeCollision