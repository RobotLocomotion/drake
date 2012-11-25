#include <Eigen/Dense>
#include "PlanarRigidBody.h"
using namespace Eigen;
using namespace std;

class Model {
public:
  int NB;
  int *jcode;
  int *parent;
  Matrix3d *Xtree;
  Matrix3d* I;
  Vector3d a_grav;
  
  Vector3d* S;
  Matrix3d* Xup;
  Vector3d* v;
  Vector3d* avp;
  Vector3d* fvp;
  Matrix3d* IC;
  MatrixXd H;
  MatrixXd C;
  
  //Variables for gradient calculations
  Matrix3d* dXupdq;
  Matrix3d** dIC;
  MatrixXd dH;
  MatrixXd dC;
  
  MatrixXd* dvdq;
  MatrixXd* dvdqd;
  MatrixXd* davpdq;
  MatrixXd* davpdqd;
  MatrixXd* dfvpdq;
  MatrixXd* dfvpdqd;
  MatrixXd dvJdqd_mat;
  MatrixXd dcross;
  
  PlanarRigidBody* bodies;
  bool kinematicsInit;
  double *cached_q;

  Model(int n) {
    this->NB = n;
    this->jcode = new int[n];
    this->parent = new int[n];
    this->Xtree = new Matrix3d[n];
    this->I = new Matrix3d[n];
    this->a_grav << 0.0, 0.0, 0.0;
    
    this->S = new Vector3d[n];
    this->Xup = new Matrix3d[n];
    this->v = new Vector3d[n];
    this->avp = new Vector3d[n];
    this->fvp = new Vector3d[n];
    this->IC = new Matrix3d[n];
    
    this->H = MatrixXd::Zero(n,n);
    this->C.resize(n,1); // C gets over-written completely by the algorithm below.
    
    //Variable allocation for gradient calculations
    this->dXupdq = new Matrix3d[n];
    this->dIC = new Matrix3d*[n];
    for(int i=0; i < n; i++) {
     this->dIC[i] = new Matrix3d[n]; 
    }
    this->dH = MatrixXd::Zero(n*n,n);
    this->dvJdqd_mat = MatrixXd::Zero(3,n);
//     this->dcross.resize(3,n);
    this->dC = MatrixXd::Zero(n,2*n);
    
    this->dvdq = new MatrixXd[n];
    this->dvdqd = new MatrixXd[n];
    this->davpdq = new MatrixXd[n];
    this->davpdqd = new MatrixXd[n];
    this->dfvpdq = new MatrixXd[n];
    this->dfvpdqd = new MatrixXd[n];
    
    for(int i=0; i < n; i++) {
     this->dvdq[i] = MatrixXd::Zero(3,n);
     this->dvdqd[i] = MatrixXd::Zero(3,n);
     this->davpdq[i] = MatrixXd::Zero(3,n);
     this->davpdqd[i] = MatrixXd::Zero(3,n);
     this->dfvpdq[i] = MatrixXd::Zero(3,n);
     this->dfvpdqd[i] = MatrixXd::Zero(3,n);
     
    }        
    //This assumes that there is only one "world" object
    this->bodies = new PlanarRigidBody[n+1];
    
    for(int i=0; i < n+1; i++) {
      this->bodies[i].setN(n);
    }
    
    this->kinematicsInit = false;
    this->cached_q = new double[n];
  }
  
  ~Model() {
    delete[] this->jcode;
    delete[] this->parent;
    delete[] this->Xtree;
    delete[] this->I;
    
    delete[] S;
    delete[] Xup;
    delete[] v;
    delete[] avp;
    delete[] fvp;
    delete[] IC;
    
    delete[] dXupdq;
    delete[] dIC;
    delete[] dvdq;
    delete[] dvdqd;
    delete[] davpdq;
    delete[] davpdqd;
    delete[] dfvpdq;
    delete[] dfvpdqd;
  }
};