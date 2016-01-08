#ifndef __DrakeCollisionModel_H__
#define __DrakeCollisionModel_H__

#include <memory>
#include <unordered_map>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "Element.h"
#include "PointPair.h"
#include "drakeCollision_export.h"

namespace DrakeCollision
{
  typedef std::pair<ElementId, ElementId> ElementIdPair;

  class DRAKECOLLISION_EXPORT Model {
    public:
      Model() {}

      virtual ~Model(){};

      /** \brief Add a collision element to this model.    
      * \param element the collision element to be added to this model
      * \return an ElementId that uniquely identifies the added element within
      * this model
      */
      virtual ElementId addElement(const Element& element);

      virtual const Element* readElement(ElementId id);
      
      virtual void getTerrainContactPoints(ElementId id0, Eigen::Matrix3Xd &terrain_points);

      virtual void updateModel() {};

      virtual bool updateElementWorldTransform(const ElementId id, 
          const Eigen::Matrix4d& T_local_to_world);

      /** \brief Compute the points of closest approach between all eligible
       * pairs of collision elements drawn from a specified set of elements
       * \param ids_to_check the vector of ElementId for which the all-to-all
       * collision detection should be performed
       * \param use_margins flag indicating whether or not to use the version
       * of this model with collision margins
       * \param[out] closest_points reference to a vector of PointPair objects
       * that contains the closest point information after this method is
       * called
       * \return true if this method ran successfully
       */
      virtual bool closestPointsAllToAll(const std::vector<ElementId>& ids_to_check, 
          const bool use_margins,
          std::vector<PointPair>& closest_points)
      { return false; };

      /** \brief Compute the points of closest approach between all eligible
       * pairs of collision elements in this model
       * \param use_margins flag indicating whether or not to use the version
       * of this model with collision margins
       * \param[out] closest_points reference to a vector of PointPair objects
       * that contains the closest point information after this method is
       * called
       * \return true if this method ran successfully
       */
      virtual bool collisionPointsAllToAll(const bool use_margins,
          std::vector<PointPair>& points)
      { return false; };

      /** \brief Compute the points of closest approach between specified pairs
       * of collision elements
       * \param id_pairs vector of ElementIdPair specifying which pairs of
       * elements to consider
       * \param use_margins flag indicating whether or not to use the version
       * of this model with collision margins
       * \param[out] closest_points reference to a vector of PointPair objects
       * that contains the closest point information after this method is
       * called
       * \return true if this method ran successfully
       */
      virtual bool closestPointsPairwise(const std::vector<ElementIdPair>& id_pairs, 
          const bool use_margins,
          std::vector<PointPair>& closest_points)
      { return false; };

      /** \brief Compute the set of potential collision points for all
       * eligible pairs of collision geometries in this model. This includes
       * the points of closest approach, but may also include additional points
       * that are "close" to being in contact. This can be useful when
       * simulating scenarios in which two collision elements have more than
       * one point of contact.
       * \param use_margins flag indicating whether or not to use the version
       * of this model with collision margins
       * \return a vector of PointPair objects containing the potential
       * collision points
       */
      virtual std::vector<PointPair> potentialCollisionPoints(const bool use_margins) 
      { return std::vector<PointPair>(); };

      virtual bool collidingPointsCheckOnly(
          const std::vector<Eigen::Vector3d>& points, 
          double collision_threshold)
      { return false; };

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
