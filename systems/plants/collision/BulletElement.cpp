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
        bt_shape->setMargin(0.0);
        //DEBUG
        //std::cout << "BulletElement::BulletElement: Created BOX" << std::endl;
        //END_DEBUG
        break;
      case SPHERE:
        if (true || params[0] != 0) {
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
        //bt_shape = new btConvexHullShape( (btScalar*) params.data(), 
                                          //params.size()/3,
                                          //(int) 3*sizeof(double) );
        bt_shape = new btConvexHullShape();
        bt_shape->setMargin(0.05);
        for (int i=0; i<params.size(); i+=3){
          //DEBUG
          //std::cout << "BulletElement::BulletElement: Adding point " << i/3 + 1 << std::endl;
          //END_DEBUG
          dynamic_cast<btConvexHullShape*>(bt_shape)->addPoint(btVector3(params[i],params[i+1],params[i+2]));
        }
        //DEBUG
        //std::cout << "BulletElement::BulletElement: Created MESH ..." << std::endl;
        //END_DEBUG
        break;
      case CAPSULE:
        bt_shape = new btConvexHullShape();
        dynamic_cast<btConvexHullShape*>(bt_shape)->addPoint(btVector3(0,0,-params[1]/2));
        dynamic_cast<btConvexHullShape*>(bt_shape)->addPoint(btVector3(0,0,params[1]/2));
        bt_shape->setMargin(params[0]);
        break;
      default:
        cerr << "Warning: Collision element has an unknown type " << shape << endl;
        //DEBUG
        //std::cout << "BulletElement::BulletElement: THROW" << std::endl;
        //END_DEBUG
        throw unknownShapeException(shape);
        break;
    }
    //DEBUG
    //cout << "BulletElement::BulletElement: Creating btCollisionObject" << endl;
    //END_DEBUG
    bt_obj = make_shared<btCollisionObject>();
    //DEBUG
    //cout << "BulletElement::BulletElement: Setting bt_shape for bt_ob" << endl;
    //END_DEBUG
    bt_obj->setCollisionShape(bt_shape);
    //DEBUG
    //cout << "BulletElement::BulletElement: Setting world transform for bt_ob" << endl;
    //END_DEBUG
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
