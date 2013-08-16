#include "DrakeCollision.h"
#include "Model.h"
#include "GenericModel.h"
#include "BulletModel.h"

using namespace std;

namespace DrakeCollision
{
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
        return shared_ptr<Model>(new GenericModel());
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
    return shared_ptr<Model>();
  };

}
