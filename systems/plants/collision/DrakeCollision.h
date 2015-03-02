#ifndef __DrakeCollision_H__
#define __DrakeCollision_H__

#include <map>
#include <memory>
#include <set>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <stdexcept>
#include <bitset>

#include <stdint.h>

#undef DLLEXPORT
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

  class DLLEXPORT Geometry {
    public:
      const Shape getShape() const;
    protected:
      Geometry(Shape shape) : shape(shape) {};
      const Shape shape;
  };

  class DLLEXPORT Sphere: public Geometry {
    public:
      Sphere(double radius);
      double radius;
  };

  class DLLEXPORT Box : public Geometry {
    public:
      Box(const Eigen::Vector3d& size);
      Eigen::Vector3d size;
  };

  class DLLEXPORT Cylinder : public Geometry {
    public:
      Cylinder(double radius, double length);
      double radius;
      double length;
  };

  class DLLEXPORT Capsule : public Geometry {
    public:
      Capsule(double radius, double length);
      double radius;
      double length;
  };

  class DLLEXPORT Mesh : public Geometry {
    public:
      Mesh(const Eigen::Matrix3Xd& points);
      Eigen::Matrix3Xd points;
  };

  typedef uintptr_t ElementId;
  typedef std::pair<ElementId, ElementId> ElementIdPair;

  class DLLEXPORT PointPair {
    public:
      PointPair(const ElementId idA, const ElementId idB,
                const Eigen::Vector3d ptA, const Eigen::Vector3d ptB,
                const Eigen::Vector3d normal, double distance)
      : idA(idA), idB(idB), ptA(ptA),
        ptB(ptB), normal(normal),
        distance(distance)
      {}

      const ElementId idA;
      const ElementId idB;
      const Eigen::Vector3d ptA;
      const Eigen::Vector3d ptB;
      const Eigen::Vector3d normal;
      double distance;

      void getResults(Eigen::Vector3d& ptA, Eigen::Vector3d& ptB, Eigen::Vector3d& normal);
      void getResults(Eigen::Vector3d& ptA, Eigen::Vector3d& ptB, Eigen::Vector3d& normal, double& distance);

      bool operator < (const PointPair& pt) const
      {
        return (distance < pt.distance);
      }

      bool operator == (const PointPair& pt) const
      {
        return (distance == pt.distance);
      }

      bool operator != (const PointPair& pt) const
      {
        return (distance != pt.distance);
      }

      bool operator <= (const PointPair& pt) const
      {
        return (distance <= pt.distance);
      }

      bool operator > (const PointPair& pt) const
      {
        return (distance > pt.distance);
      }

      bool operator >= (const PointPair& pt) const
      {
        return (distance >= pt.distance);
      }
  };

  class DLLEXPORT Element {
    public:
      Element(std::unique_ptr<Geometry> geometry, const Eigen::Matrix4d T_element_to_local = Eigen::Matrix4d::Identity());

      const Eigen::Matrix4d& getWorldTransform() const; 

      const Eigen::Matrix4d& getLocalTransform() const; 

      virtual void updateWorldTransform(const Eigen::Matrix4d& T_local_to_world);

      const Shape getShape() const;

      ElementId getId() const;

      const Geometry* getGeometry() const;

      virtual bool isStatic() const { return false; };

      virtual bool collidesWith( const Element* other) const { return true; };

    protected:

      virtual void setWorldTransform(const Eigen::Matrix4d& T_elem_to_world);
      Eigen::Matrix4d T_element_to_world;
      const Eigen::Matrix4d T_element_to_local;
      std::unique_ptr<Geometry> geometry;
      const ElementId id;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  class DLLEXPORT Model {
  public:
    
    virtual ElementId addElement(std::unique_ptr<Element> element);

    virtual const Element* readElement(ElementId id);

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
    //
    // Performs raycasting collision detecting (like a LIDAR / laser rangefinder)
    //
    // @param origin Vector3d specifying the position of the ray's origin
    // @param ray_endpoint Vector3d specifying a second point on the ray in world coordinates
    // @param distance to the first collision, or -1 on no collision
    //
    virtual bool collisionRaycast(const Eigen::Matrix3Xd &origin, const Eigen::Matrix3Xd &ray_endpoint, bool use_margins, Eigen::VectorXd &distances) { return false; };

    protected:
      std::map< ElementId, std::unique_ptr<Element> >  elements;
  };

  DLLEXPORT std::unique_ptr<Model> newModel();

  DLLEXPORT std::unique_ptr<Model> newModel(ModelType model_type);
  
  typedef std::bitset<16> bitmask;
  // Constants
  extern const DLLEXPORT bitmask ALL_MASK;
  extern const DLLEXPORT bitmask NONE_MASK;
  extern const DLLEXPORT bitmask DEFAULT_GROUP;

  // Exceptions

  class noClosestPointsResultException : public std::exception {};

  class badShapeException : public std::exception
  {
    public:
      badShapeException();
      badShapeException(Shape shape);
      virtual const char* what() const throw();
      virtual ~badShapeException() throw() {};
    protected:
      std::string shape_str;
  };

  class zeroRadiusSphereException : public badShapeException
  {
    public:
      virtual const char* what() const throw();
      virtual ~zeroRadiusSphereException() throw() {};
  };

  class unknownShapeException : public badShapeException
  {
    public:
      unknownShapeException(Shape shape) : badShapeException(shape){};
      virtual const char* what() const throw();
      virtual ~unknownShapeException() throw() {};
  };

  class unsupportedShapeException : public badShapeException
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

