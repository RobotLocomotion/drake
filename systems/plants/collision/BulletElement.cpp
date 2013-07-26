#include "BulletElement.h"
#include "BulletModel.h"

using namespace std;

namespace DrakeCollision
{
  BulletElement::BulletElement( Matrix4d T_elem_to_link, Shape shape, vector<double> params)
    : Element(T_elem_to_link, shape, params)
  {
    switch (shape) {
      case BOX:
        //DEBUG
        //cout << "BOX" << endl;
        //DEBUG
        bt_shape = new btBoxShape( btVector3(params[0]/2,params[1]/2,params[2]/2) );
        break;
      case SPHERE:
        //DEBUG
        //cout << "SPHERE" << endl;
        //DEBUG
        bt_shape = new btSphereShape(params[0]) ;
        break;
      case CYLINDER:
        //DEBUG
        //cout << "CYLINDER" << endl;
        //DEBUG
        bt_shape = new btCylinderShapeZ( btVector3(params[0],params[0],params[1]/2) );
        break;
      case MESH:
        //DEBUG
        //cout << "MESH (params.size() = " << params.size() << ")" << endl;
        //DEBUG
        bt_shape = new btConvexHullShape( (btScalar*) params.data(), params.size() ,(int) 3*sizeof(double) );
        //DEBUG
        //cout << "Created mesh bt_shape" << endl;
        //DEBUG
        //cerr << "Warning: mesh collision elements are not supported yet." << endl;
        //throw unsupportedShapeException(shape);
        break;
      default:
        cerr << "Warning: Collision element has an unknown type " << shape << endl;
        throw unknownShapeException(shape);
        //mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs", "Body %d collision shape %d has an unknown type %d", i+1,j+1,type);
        break;
    }
    bt_obj = new btCollisionObject();
    bt_obj->setCollisionShape(bt_shape);
    setWorldTransform(this->T_elem_to_world);
  }
  BulletElement::~BulletElement()
  {
    delete bt_obj;
    delete bt_shape;
  }
  void BulletElement::setWorldTransform(const Matrix4d T)
  {
    // DEBUG
    //cout << "BulletElement::setWorldTransform" << endl;
    // DEBUG
    Element::setWorldTransform(T);

    // DEBUG
    //cout << "BulletElement::setWorldTransform:Creating Bullet objects" << endl;
    // DEBUG
    btMatrix3x3 rot;
    btVector3 pos;
    btTransform btT;

    rot.setValue( T(0,0), T(0,1), T(0,2),
        T(1,0), T(1,1), T(1,2),
        T(2,0), T(2,1), T(2,2) );
    btT.setBasis(rot);
    pos.setValue( T(0,3), T(1,3), T(2,3) );
    btT.setOrigin(pos);

    // DEBUG
    //cout << "BulletElement::setWorldTransform: Updating bt_obj" << endl;
    // DEBUG
    bt_obj->setWorldTransform(btT);
  }
}
