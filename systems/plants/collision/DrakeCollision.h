#ifndef __DrakeCollision_H__
#define __DrakeCollision_H__

#include <memory>
#include <map>
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;

namespace DrakeCollision
{
  // Forward declarations
  class Model;
  class Element;
  class BulletModel;
  class BulletElement;

  enum Shape {
    UNKNOWN,
    BOX,
    SPHERE,
    CYLINDER,
    MESH
  };

  class badShapeException : public std::exception
  {
    public:
      badShapeException(Shape shape);
      virtual const char* what() const throw();
      virtual ~badShapeException() throw() {};
    protected:
      std::string shape_str;
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

  enum ModelType {
    NONE,
    AUTO,
    BULLET
  };

  typedef std::weak_ptr<Model> mdl_ptr;
  typedef std::shared_ptr<Element> ElementShPtr;
  typedef std::shared_ptr<BulletElement> BulletElementShPtr;

  std::shared_ptr<Model> newModel();
  std::shared_ptr<Model> newModel(ModelType model_type);
}
#endif

