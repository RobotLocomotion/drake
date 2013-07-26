#include "DrakeCollision.h"
#include "Model.h"
#ifdef BULLET_COLLISION
#include "BulletModel.h"
#endif

using namespace std;

namespace DrakeCollision
{
  badShapeException::badShapeException(Shape shape)
    : shape_str(to_string(shape))
  {}

  const char* badShapeException::what() const throw()
  {
    return "Ignoring this collision element.";
  }

  const char* unknownShapeException::what() const throw()
  {
    return ("Unknown collision shape: " + shape_str + ". " + badShapeException::what()).c_str();
  }

  const char* unsupportedShapeException::what() const throw()
  {
    return ("Unsupported collision shape: " + shape_str + ". " +  badShapeException::what()).c_str();
  }

  shared_ptr<Model> newModel()
  {
#ifdef BULLET_COLLISION
    return newModel(ModelType::BULLET);
#else
    return newModel(ModelType::NONE);
#endif
  }

  shared_ptr<Model> newModel(ModelType model_type)
  {
    switch (model_type) {
      case NONE:
        return shared_ptr<Model>(new Model());
        break;
      case BULLET:
#ifdef BULLET_COLLISION
        return shared_ptr<Model>(new BulletModel());
#else
        cerr << "Recompile with Bullet enabled (-DBULLET_COLLISION) to use Bullet collision models." << endl;
#endif
        break;
      default:
        cerr << model_type << " is not a recognized collision model type." << endl;
    }
  };
};
