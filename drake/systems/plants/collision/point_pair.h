#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "Element.h"
#include "drake/drakeCollision_export.h"

namespace DrakeCollision {

/** Structure containing the results of a collision query. **/
struct DRAKECOLLISION_EXPORT PointPair {
  PointPair() {}

  PointPair(const Element* elementA, const Element* elementB,
            const Eigen::Vector3d ptA, const Eigen::Vector3d ptB,
            const Eigen::Vector3d normal, double distance)
      : elementA_(elementA), elementB_(elementB),
        idA_(elementA->getId()), idB_(elementB->getId()),
        ptA_(ptA), ptB_(ptB),
        normal_(normal),
        distance_(distance) {}

  void getResults(Eigen::Vector3d* ptA, Eigen::Vector3d* ptB,
                  Eigen::Vector3d* normal) const;
  void getResults(Eigen::Vector3d* ptA, Eigen::Vector3d* ptB,
                  Eigen::Vector3d* normal, double* distance) const;
  ElementId getIdA() const;
  ElementId getIdB() const;
  const Element* get_elementA() const;
  const Element* get_elementB() const;
  Eigen::Vector3d getNormal() const;
  Eigen::Vector3d getPtA() const;
  Eigen::Vector3d getPtB() const;
  double getDistance() const;

  bool operator<(const PointPair& pt) const {
    return (distance_ < pt.distance_);
  }

  bool operator==(const PointPair& pt) const {
    return (distance_ == pt.distance_);
  }

  bool operator!=(const PointPair& pt) const {
    return (distance_ != pt.distance_);
  }

  bool operator<=(const PointPair& pt) const {
    return (distance_ <= pt.distance_);
  }

  bool operator>(const PointPair& pt) const {
    return (distance_ > pt.distance_);
  }

  bool operator>=(const PointPair& pt) const {
    return (distance_ >= pt.distance_);
  }

  /** Element A in the pair participating in the collision. **/
  const Element* elementA_{nullptr};

  /** Element B in the pair participating in the collision. **/
  const Element* elementB_{nullptr};

  /** Id of element A participating in the collision. **/
  ElementId idA_{0};

  /** Id of element B participating in the collision. **/
  ElementId idB_{0};

  /** Collision point on the surface of body A. **/
  Eigen::Vector3d ptA_;

  /** Collision point on the surface of body B. **/
  Eigen::Vector3d ptB_;

  /** Outwards normal on body B. On body A it points in the opposite
  direction. **/
  Eigen::Vector3d normal_;

  /** Distance between the point on body A and the point on body B. **/
  double distance_{};
};

}  // namespace DrakeCollision
