#ifndef __DrakeCollision_H__
#define __DrakeCollision_H__

#include <memory>
#include <set>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <stdexcept>
#include <bitset>

#include <stdint.h>

#if defined(WIN32) || defined(WIN64)
  #if defined(drakeCollision_EXPORTS)
    #define DLLEXPORT __declspec( dllexport )
  #else
    #define DLLEXPORT __declspec( dllimport )
  #endif
#else
    #define DLLEXPORT
#endif


namespace DrakeCollision
{
  enum DLLEXPORT Shape {
    UNKNOWN,
    BOX,
    SPHERE,
    CYLINDER,
    MESH,
    CAPSULE
  };

  enum DLLEXPORT ModelType {
    NONE,
    AUTO,
    BULLET
  };

  class DLLEXPORT Model {
  public:
    virtual void resize(int num_bodies) {};
    
    virtual void addElement(const int body_idx, const int parent_idx, 
			    const Eigen::Matrix4d& T_element_to_link, Shape shape, 
			    const std::vector<double>& params, 
          const std::string& group_name,
			    bool is_static,
          bool use_margins = true) {};

    virtual bool updateElementsForBody(const int body_idx, 
				       const Eigen::Matrix4d& T_link_to_world) { return false; };
      
    virtual bool setCollisionFilter(const int body_idx, const uint16_t group, 
				    const uint16_t mask) { return false; };

    virtual bool getPointCollision(const int body_idx, 
				   const int body_collision_idx, 
				   Eigen::Vector3d &ptA, Eigen::Vector3d &ptB, 
				   Eigen::Vector3d &normal) { return false; };

    virtual bool getPairwiseCollision(const int bodyA_idx, const int bodyB_idx, 
				      Eigen::MatrixXd& ptsA, Eigen::MatrixXd& ptsB, 
				      Eigen::MatrixXd& normals) { return false; };

    virtual bool getPairwisePointCollision(const int bodyA_idx, const int bodyB_idx, 
					   const int body_collisionA_idx, 
					   Eigen::Vector3d &ptA, Eigen::Vector3d &ptB, 
					   Eigen::Vector3d &normal) { return false; };

    virtual bool getClosestPoints(const int bodyA_idx, const int bodyB_idx,
				  Eigen::Vector3d& ptA, Eigen::Vector3d& ptB, Eigen::Vector3d& normal,
				  double& distance) { return false; };

    virtual bool closestPointsAllBodies(std::vector<int>& bodyA_idx, 
					std::vector<int>& bodyB_idx, 
					Eigen::MatrixXd& ptsA, Eigen::MatrixXd& ptsB,
					Eigen::MatrixXd& normal, 
					Eigen::VectorXd& distance,
					const std::vector<int>& bodies_idx,
          const std::set<std::string>& active_element_groups) { return false; };

    bool closestPointsAllBodies(std::vector<int>& bodyA_idx, 
        std::vector<int>& bodyB_idx, 
        Eigen::MatrixXd& ptsA, Eigen::MatrixXd& ptsB,
        Eigen::MatrixXd& normal, 
        Eigen::VectorXd& distance,
        const std::set<std::string>& active_element_groups);

    bool closestPointsAllBodies(std::vector<int>& bodyA_idx, 
        std::vector<int>& bodyB_idx, 
        Eigen::MatrixXd& ptsA, Eigen::MatrixXd& ptsB,
        Eigen::MatrixXd& normal, 
        Eigen::VectorXd& distance,
        const std::vector<int>& bodies_idx);

    bool closestPointsAllBodies(std::vector<int>& bodyA_idx, 
        std::vector<int>& bodyB_idx, 
        Eigen::MatrixXd& ptsA, Eigen::MatrixXd& ptsB,
        Eigen::MatrixXd& normal, 
        Eigen::VectorXd& distance);

    virtual bool allCollisions(std::vector<int>& bodyA_idx, 
			       std::vector<int>& bodyB_idx, 
			       Eigen::MatrixXd& ptsA, Eigen::MatrixXd& ptsB) { return false; };
                                  
    //
    // Performs raycasting collision detecting (like a LIDAR / laser rangefinder)
    //
    // @param origin Vector3d specifying the position of the ray's origin
    // @param ray_endpoint Vector3d specifying a second point on the ray in world coordinates
    // @param distance to the first collision, or -1 on no collision
    //
    virtual bool collisionRaycast(const Eigen::Matrix3Xd &origin, const Eigen::Matrix3Xd &ray_endpoint, Eigen::VectorXd &distances) { return false; };

    protected:
      virtual const std::vector<int> bodyIndices() const;
      virtual const std::set<std::string> elementGroupNames() const;
  };

  DLLEXPORT std::shared_ptr<Model> newModel();

  DLLEXPORT std::shared_ptr<Model> newModel(ModelType model_type);


  
  typedef std::bitset<16> bitmask;
  // Constants
  extern const DLLEXPORT bitmask ALL_MASK;
  extern const DLLEXPORT bitmask NONE_MASK;
  extern const DLLEXPORT bitmask DEFAULT_GROUP;

  // Exceptions

  class DLLEXPORT noClosestPointsResultException : public std::exception {};

  class DLLEXPORT badShapeException : public std::exception
  {
    public:
      badShapeException();
      badShapeException(Shape shape);
      virtual const char* what() const throw();
      virtual ~badShapeException() throw() {};
    protected:
      std::string shape_str;
  };

  class DLLEXPORT zeroRadiusSphereException : public badShapeException
  {
    public:
      virtual const char* what() const throw();
      virtual ~zeroRadiusSphereException() throw() {};
  };

  class DLLEXPORT unknownShapeException : public badShapeException
  {
    public:
      unknownShapeException(Shape shape) : badShapeException(shape){};
      virtual const char* what() const throw();
      virtual ~unknownShapeException() throw() {};
  };

  class DLLEXPORT unsupportedShapeException : public badShapeException
  {
    public:
      unsupportedShapeException(Shape shape) : badShapeException(shape){};
      virtual const char* what() const throw();
      virtual ~unsupportedShapeException() throw() {};
  };

  template<typename T> int sgn(T val) {
      return (T(0) < val) - (val < T(0));
  }

}
#endif

