
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

  unique_ptr<Model> newModel()
  {
#ifdef BULLET_COLLISION
    return newModel(BULLET);
#else
    return newModel(NONE);
#endif
  }

  unique_ptr<Model> newModel(ModelType model_type)
  {
    switch (model_type) {
      case NONE:
        return unique_ptr<Model>(new Model());
        break;
      case BULLET:
#ifdef BULLET_COLLISION
        return unique_ptr<Model>(new BulletModel());
#else
        cerr << "Recompile with Bullet enabled (-DBULLET_COLLISION) to use Bullet collision models." << endl;
#endif
        break;
      default:
        cerr << model_type << " is not a recognized collision model type." << endl;
    }
    return unique_ptr<Model>();
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

  Element::Element(unique_ptr<Geometry> geometry, const Matrix4d T_element_to_local) : geometry(move(geometry)), T_element_to_local(T_element_to_local), id((ElementId) this) {}

  const Matrix4d& Element::getWorldTransform() const
  {
    return T_element_to_world;
  }

  const Matrix4d& Element::getLocalTransform() const
  {
    return T_element_to_local;
  }

  const Shape Element::getShape() const
  {
    return geometry->getShape();
  }

  const Geometry* Element::getGeometry() const
  {
    return geometry.get();
  }

  ElementId Element::getId() const
  {
    return id;
  }

  void Element::updateWorldTransform(const Eigen::Matrix4d& T_local_to_world)
  {
    setWorldTransform(T_local_to_world*(this->T_element_to_local));
  }

  void Element::setWorldTransform(const Matrix4d& T_element_to_world)
  {
    this->T_element_to_world = T_element_to_world;
  }

  const Shape Geometry::getShape() const
  {
    return shape;
  }

  Sphere::Sphere(double radius)
    : Geometry(SPHERE), radius(radius) {}

  Box::Box(const Eigen::Vector3d& size)
    : Geometry(BOX), size(size) {}

  Cylinder::Cylinder(double radius, double length)
    : Geometry( CYLINDER), radius(radius), length(length) {}

  Capsule::Capsule(double radius, double length)
    : Geometry(CAPSULE), radius(radius), length(length) {}

  Mesh::Mesh(const Eigen::Matrix3Xd& points) 
    : Geometry(MESH), points(points) {}

  ElementId Model::addElement(std::unique_ptr<Element> element)
  {
    if ((element != nullptr) && (element->getGeometry() != nullptr)) {
      ElementId id = element->getId();
      this->elements.insert(make_pair(id, move(element)));
      return id;
    } else {
      return 0;
    }
  }

  const Element* Model::readElement(ElementId id)
  {
    auto element_iter = elements.find(id);
    if (element_iter != elements.end()) {
      return element_iter->second.get();
    } else {
      return nullptr;
    }
  }

  bool Model::updateElementWorldTransform(const ElementId id, const Matrix4d& T_elem_to_world)
  {
    auto elem_itr = elements.find(id);
    if (elem_itr != elements.end()) {
      elem_itr->second->updateWorldTransform(T_elem_to_world);
      return true;
    } else {
      return false;
    }
  }

};
