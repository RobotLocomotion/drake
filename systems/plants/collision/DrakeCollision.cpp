#include "DrakeCollision.h"
#include "GenericModel.h"
#include "ResultCollector.h"

#ifdef BULLET_COLLISION
#include "BulletModel.h"
#include "BulletResultCollector.h"
#endif

using namespace std;

namespace DrakeCollision
{
  badShapeException::badShapeException()
    : shape_str()
  {}
  badShapeException::badShapeException(Shape shape)
    : shape_str(to_string(shape))
  {}

  const char* badShapeException::what() const throw()
  {
    return "Ignoring this collision element";
  }

  const char* zeroRadiusSphereException::what() const throw()
  {
    return "Ignoring zero-radius sphere";
  };

  const char* unknownShapeException::what() const throw()
  {
    return ("Unknown collision shape: " + shape_str + ". " + badShapeException::what()).c_str();
  }

  const char* unsupportedShapeException::what() const throw()
  {
    return ("Unsupported collision shape: " + shape_str + ". " +  badShapeException::what()).c_str();
  }

  ResultCollShPtr newResultCollector()
  {
#ifdef BULLET_COLLISION
    return ResultCollShPtr(new BulletResultCollector());
#else
    return ResultCollShPtr(new ResultCollector());
#endif
  }
};
