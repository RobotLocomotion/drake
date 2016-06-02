#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "collision_element.h"
#include "drake/drakeCollision_export.h"

namespace DrakeCollision {

class DRAKECOLLISION_EXPORT PointPair {
 public:
  PointPair(const CollisionElementId idA, const CollisionElementId idB,
            const Eigen::Vector3d ptA, const Eigen::Vector3d ptB,
            const Eigen::Vector3d normal, double distance)
      : idA_(idA),
        idB_(idB),
        ptA_(ptA),
        ptB_(ptB),
        normal_(normal),
        distance_(distance) {}

  void getResults(Eigen::Vector3d* ptA, Eigen::Vector3d* ptB,
                  Eigen::Vector3d* normal) const;
  void getResults(Eigen::Vector3d* ptA, Eigen::Vector3d* ptB,
                  Eigen::Vector3d* normal, double* distance) const;
  CollisionElementId getIdA() const;
  CollisionElementId getIdB() const;
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

 protected:
  CollisionElementId idA_;
  CollisionElementId idB_;
  Eigen::Vector3d ptA_;
  Eigen::Vector3d ptB_;
  Eigen::Vector3d normal_;
  double distance_;
};

}  // namespace DrakeCollision
