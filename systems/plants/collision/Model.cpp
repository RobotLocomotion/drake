#include "Model.h"

#include <iostream>

using namespace std;
using namespace Eigen;

namespace DrakeCollision
{
  ElementId Model::addElement(std::unique_ptr<Element> element)
  {
    if ((element != nullptr) && (element->getGeometry() != nullptr)) {
      ElementId id = element->getId();
      this->elements.insert(make_pair(id, move(element)));
      return id;
    } else {
      return 0;
    }
  }

  const Element* Model::readElement(ElementId id)
  {
    auto element_iter = elements.find(id);
    if (element_iter != elements.end()) {
      return element_iter->second.get();
    } else {
      return nullptr;
    }
  }

  bool Model::updateElementWorldTransform(const ElementId id, const Matrix4d& T_elem_to_world)
  {
    auto elem_itr = elements.find(id);
    if (elem_itr != elements.end()) {
      elem_itr->second->updateWorldTransform(T_elem_to_world);
      return true;
    } else {
      return false;
    }
  }

  bool closestPointsAllToAll(const vector<ElementId>& ids_to_check, 
      const bool use_margins,
      vector<PointPair>& closest_points)
  { return false; };

  bool collisionPointsAllToAll(const bool use_margins,
      vector<PointPair>& points)
  { return false; };

  bool closestPointsPairwise(const vector<ElementIdPair>& id_pairs, 
      const bool use_margins,
      vector<PointPair>& closest_points)
  { return false; };
}
