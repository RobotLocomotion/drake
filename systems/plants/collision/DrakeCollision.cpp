
#include <iostream>
#include <map>

#include "DrakeCollision.h"

#ifdef BULLET_COLLISION
#include "BulletModel.h"
#endif

using namespace std;
using namespace Eigen;

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
    return shared_ptr<Model>();
  };
  

  const bitmask ALL_MASK(std::string(16,'1'));
  const bitmask NONE_MASK(0);
  const bitmask DEFAULT_GROUP(1);

  badShapeException::badShapeException()
    : shape_str()
  {}
  badShapeException::badShapeException(Shape shape)
  { 
    std::ostringstream ostr; 
    ostr << shape; 
    this->shape_str = ostr.str(); 
  }
 
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

  bool Model::closestPointsAllBodies(std::vector<int>& bodyA_idx, 
      std::vector<int>& bodyB_idx, 
      MatrixXd& ptsA, MatrixXd& ptsB,
      MatrixXd& normal, 
      VectorXd& distance,
      const std::set<std::string>& active_element_groups)
  {
    return closestPointsAllBodies(bodyA_idx,bodyB_idx,ptsA,ptsB,normal,distance,
        bodyIndices(),active_element_groups);
  }

  bool Model::closestPointsAllBodies(std::vector<int>& bodyA_idx, 
      std::vector<int>& bodyB_idx, 
      MatrixXd& ptsA, MatrixXd& ptsB,
      MatrixXd& normal, 
      VectorXd& distance,
      const std::vector<int>& bodies_idx)
  {
    return closestPointsAllBodies(bodyA_idx,bodyB_idx,ptsA,ptsB,normal,distance,
        bodies_idx,elementGroupNames());
  }

  bool Model::closestPointsAllBodies(std::vector<int>& bodyA_idx, 
      std::vector<int>& bodyB_idx, 
      MatrixXd& ptsA, MatrixXd& ptsB,
      MatrixXd& normal, 
      VectorXd& distance)
  {
    return closestPointsAllBodies(bodyA_idx,bodyB_idx,ptsA,ptsB,normal,distance,
        bodyIndices(),elementGroupNames());
  }

  const std::vector<int> Model::bodyIndices() const 
  {
    const std::vector<int> bodies_idx;
    return bodies_idx;
  }
  const std::set<std::string> Model::elementGroupNames() const 
  {
    const std::set<std::string> active_element_groups;
    return active_element_groups;
  }

};
