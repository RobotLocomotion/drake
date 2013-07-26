#include "Element.h"

using namespace std;

namespace DrakeCollision
{
  Element::Element(Matrix4d T_elem_to_link, Shape shape, std::vector<double> params) 
    : T_elem_to_link(T_elem_to_link) 
  {
    setWorldTransform(Matrix4d::Identity());
  }
  void Element::updateWorldTransform(const Matrix4d T_link_to_world)
  {
    setWorldTransform(T_link_to_world*(this->T_elem_to_link));
  }

  void Element::setWorldTransform(const Matrix4d T_elem_to_world)
  {
    // DEBUG
    //cout << "Element::setWorldTransform" << endl;
    // DEBUG
    this->T_elem_to_world = T_elem_to_world;
    //shared_ptr<Model> model_tmp = model.lock();
    //model.lock()->updateElement(this);
  }
}
