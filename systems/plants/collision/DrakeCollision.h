#ifndef __DrakeCollision_H__
#define __DrakeCollision_H__

#include <memory>
#include <map>
#include <Eigen/Dense>
#include <iostream>
#include <stdexcept>
#include <bitset>
#include "Model.h"

using namespace Eigen;

namespace DrakeCollision
{
  typedef std::bitset<16> bitmask;
  // Constants
  const std::bitset<16> ALL_MASK(std::string(16,'1'));
  const std::bitset<16> NONE_MASK(0);
  const std::bitset<16> DEFAULT_GROUP(1);

  // Forward declarations
  class GenericModel;
  class Element;
  class BulletModel;
  class BulletElement;
  class ResultCollector;

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
  

  typedef std::shared_ptr< ResultCollector > ResultCollShPtr;

  std::shared_ptr<Model> newModel();
  std::shared_ptr<Model> newModel(ModelType model_type);
  ResultCollShPtr newResultCollector();
}
#endif

