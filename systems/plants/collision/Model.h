#ifndef __DrakeCollisionModel_H__
#define __DrakeCollisionModel_H__

#include <memory>
#include <unordered_map>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "drakeCollisionMacros.h"
#include "Element.h"
#include "PointPair.h"

namespace DrakeCollision
{
  typedef std::pair<ElementId, ElementId> ElementIdPair;

  class DLLEXPORT_drakeCollision Model {
    public:
      Model() {}

      virtual ~Model(){};

      virtual ElementId addElement(const Element& element);

      virtual const Element* readElement(ElementId id);
      
      virtual void getTerrainContactPoints(ElementId id0, Eigen::Matrix3Xd &terrain_points);

      virtual void updateModel() {};

      virtual bool updateElementWorldTransform(const ElementId id, 
          const Eigen::Matrix4d& T_local_to_world);

      virtual bool closestPointsAllToAll(const std::vector<ElementId>& ids_to_check, 
          const bool use_margins,
          std::vector<PointPair>& closest_points)
      { return false; };

      virtual bool collisionPointsAllToAll(const bool use_margins,
          std::vector<PointPair>& points)
      { return false; };

      virtual bool closestPointsPairwise(const std::vector<ElementIdPair>& id_pairs, 
          const bool use_margins,
          std::vector<PointPair>& closest_points)
      { return false; };

      virtual std::vector<PointPair> potentialCollisionPoints(const bool use_margins) 
      { return std::vector<PointPair>(); };

      virtual std::vector<size_t> collidingPoints(
          const std::vector<Eigen::Vector3d>& points, 
          double collision_threshold)
      { return std::vector<size_t>(); };

      //
      // Performs raycasting collision detecting (like a LIDAR / laser rangefinder)
      //
      // @param origin Vector3d specifying the position of the ray's origin
      // @param ray_endpoint Vector3d specifying a second point on the ray in world coordinates
      // @param distance to the first collision, or -1 on no collision
      //
      virtual bool collisionRaycast(const Eigen::Matrix3Xd &origin, const Eigen::Matrix3Xd &ray_endpoint, bool use_margins, Eigen::VectorXd &distances) { return false; };

    protected:
      std::unordered_map< ElementId, std::unique_ptr<Element> >  elements;

    private:
      Model(const Model&) {}
      Model& operator=(const Model&) { return *this; }
  };

}
  
#endif
