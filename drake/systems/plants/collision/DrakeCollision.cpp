#include "drake/systems/plants/collision/DrakeCollision.h"

#include <iostream>

#include "drake/common/drake_assert.h"
#ifdef BULLET_COLLISION
#include "drake/systems/plants/collision/bullet_model.h"
#endif

using namespace std;
using namespace Eigen;

namespace DrakeCollision {

const bitmask ALL_MASK(bitmask(0).set());
const bitmask NONE_MASK(0);
const bitmask DEFAULT_GROUP(1);

enum DRAKECOLLISION_EXPORT ModelType { NONE, AUTO, BULLET };

unique_ptr<Model> newModel(ModelType model_type) {
  switch (model_type) {
    case NONE:
      return nullptr;
      break;
    case BULLET:
#ifdef BULLET_COLLISION
      return unique_ptr<Model>(new BulletModel());
#else
      cerr << "Recompile with Bullet enabled (-DBULLET_COLLISION) to use "
              "Bullet collision models." << endl;
#endif
      break;
    default:
      cerr << model_type << " is not a recognized collision model type."
           << endl;
  }
  return unique_ptr<Model>();
}

unique_ptr<Model> newModel() {
#ifdef BULLET_COLLISION
  return newModel(BULLET);
#else
  DRAKE_ABORT_UNLESS(!"DrakeCollision must be compiled with Bullet.");
  // This return statement is actually never reached.
  // It is placed here to avoid a compiler error on not having a return
  // statement.
  return unique_ptr<Model>();
#endif
}
};
