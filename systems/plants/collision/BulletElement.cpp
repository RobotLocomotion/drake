#include "BulletElement.h"
#include "BulletModel.h"

using namespace std;

namespace DrakeCollision
{
  BulletElement::BulletElement(const Matrix4d& T_elem_to_link, Shape shape, 
                                const vector<double>& params)
    : Element(T_elem_to_link, shape, params)
  {
    //DEBUG
    //std::cout << "BulletElement::BulletElement: START" << std::endl;
    //END_DEBUG
    btCollisionShape* bt_shape;
    switch (shape) {
      case BOX:
        //DEBUG
        //std::cout << "BulletElement::BulletElement: Create BOX ..." << std::endl;
        //END_DEBUG
        bt_shape = new btBoxShape( btVector3(params[0]/2,params[1]/2,params[2]/2) );
        //DEBUG
        //std::cout << "BulletElement::BulletElement: Created BOX" << std::endl;
        //END_DEBUG
        break;
      case SPHERE:
        if (params[0] != 0) {
          //DEBUG
          //std::cout << "BulletElement::BulletElement: Create SPHERE ..." << std::endl;
          //END_DEBUG
          bt_shape = new btSphereShape(params[0]) ;
          //DEBUG
          //std::cout << "BulletElement::BulletElement: Created SPHERE" << std::endl;
          //END_DEBUG
        } else {
          //DEBUG
          //std::cout << "BulletElement::BulletElement: THROW" << std::endl;
          //END_DEBUG
          throw zeroRadiusSphereException();
        }
        break;
      case CYLINDER:
        //DEBUG
        //std::cout << "BulletElement::BulletElement: Create CYLINDER ..." << std::endl;
        //END_DEBUG
        bt_shape = new btCylinderShapeZ( btVector3(params[0],params[0],params[1]/2) );
        //DEBUG
        //std::cout << "BulletElement::BulletElement: Created CYLINDER ..." << std::endl;
        //END_DEBUG
        break;
      case MESH:
        //DEBUG
        //std::cout << "BulletElement::BulletElement: Create MESH ..." << std::endl;
        //END_DEBUG
        bt_shape = new btConvexHullShape( (btScalar*) params.data(), 
                                          params.size()/3,
                                          (int) 3*sizeof(double) );
        //DEBUG
        //std::cout << "BulletElement::BulletElement: Created MESH ..." << std::endl;
        //END_DEBUG
        break;
      default:
        cerr << "Warning: Collision element has an unknown type " << shape << endl;
        //DEBUG
        //std::cout << "BulletElement::BulletElement: THROW" << std::endl;
        //END_DEBUG
        throw unknownShapeException(shape);
        break;
    }
    bt_obj = make_shared<btCollisionObject>();
    bt_obj->setCollisionShape(bt_shape);
    setWorldTransform(this->T_elem_to_world);
    //DEBUG
    //std::cout << "BulletElement::BulletElement: END" << std::endl;
    //END_DEBUG
  }
  void BulletElement::setWorldTransform(const Matrix4d& T)
  {
    Element::setWorldTransform(T);

    btMatrix3x3 rot;
    btVector3 pos;
    btTransform btT;

    rot.setValue( T(0,0), T(0,1), T(0,2),
        T(1,0), T(1,1), T(1,2),
        T(2,0), T(2,1), T(2,2) );
    btT.setBasis(rot);
    pos.setValue( T(0,3), T(1,3), T(2,3) );
    btT.setOrigin(pos);

    bt_obj->setWorldTransform(btT);
  }
}
