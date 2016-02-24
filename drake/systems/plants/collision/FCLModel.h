#ifndef __DrakeCollisionFCLModel_H__
#define __DrakeCollisionFCLModel_H__

#include "Element.h"
#include "Model.h"
#include "ResultCollector.h"

namespace DrakeCollision
{
  class FCLModel;    // forward declaration

  //#define PERTURBATION_ITERATIONS 8 
  //#define MINIMUM_POINTS_PERTURBATION_THRESHOLD 8
  
  class FCLModel : public Model
  {
    public:
      FCLModel() {};

      virtual ~FCLModel(){};

      //Required member functions for Model interface
      virtual void updateModel();

      virtual void resize(int num_bodies) {};

      virtual ElementId addElement(const Element& element);

      virtual bool updateElementWorldTransform(const ElementId, 
                                               const Eigen::Isometry3d& T_local_to_world);

      virtual bool closestPointsAllToAll(const std::vector<ElementId>& ids_to_check, 
                                         const bool use_margins,
                                         std::vector<PointPair>& closest_points);

      virtual bool collisionPointsAllToAll(const bool use_margins,
                                           std::vector<PointPair>& points);

      virtual bool closestPointsPairwise(const std::vector<ElementIdPair>& id_pairs, 
                                         const bool use_margins,
                                         std::vector<PointPair>& closest_points);

      virtual bool findClosestPointsBtwElements(const ElementId elemA,
                                                const ElementId elemB,
                                                const bool use_margins,
                                                std::unique_ptr<ResultCollector>& c);


      virtual void collisionDetectFromPoints(const Eigen::Matrix3Xd& points,
                                           bool use_margins,
                                           std::vector<PointPair>& closest_points);

      virtual bool collisionRaycast(const Eigen::Matrix3Xd &origins, 
              const Eigen::Matrix3Xd &ray_endpoints, bool use_margins, 
              Eigen::VectorXd &distances, Eigen::Matrix3Xd &normals);

      virtual std::vector<PointPair> potentialCollisionPoints(bool use_margins);

      virtual bool collidingPointsCheckOnly(const std::vector<Eigen::Vector3d>& points, 
                                            double collision_threshold);

      virtual std::vector<size_t> collidingPoints(
          const std::vector<Eigen::Vector3d>& points, 
          double collision_threshold);

      // END Required member functions
      
    protected:

      //static constexpr double small_margin = 1e-9;
      //static constexpr double large_margin = 0.05;

      class unknownShapeException : public std::exception
      {
        public:
          unknownShapeException(DrakeShapes::Shape shape);
          virtual const char* what() const throw();
          virtual ~unknownShapeException() throw() {};
        protected:
          std::string shape_str;
      };
    private:
      FCLModel(const FCLModel&) {}
      FCLModel& operator=(const FCLModel&) { return *this; }
  };
}
#endif
