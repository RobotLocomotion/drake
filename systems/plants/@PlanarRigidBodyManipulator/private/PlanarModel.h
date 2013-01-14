#include <Eigen/Dense>
#include "PlanarRigidBody.h"
using namespace Eigen;
using namespace std;

class PlanarModel {
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
  int secondDerivativesCached;
  double *cached_q;

  PlanarModel(int n) {
    NB = n;
    jcode = new int[n];
    parent = new int[n];
    Xtree = new Matrix3d[n];
    I = new Matrix3d[n];
    a_grav << 0.0, 0.0, 0.0;
    
    S = new Vector3d[n];
    Xup = new Matrix3d[n];
    v = new Vector3d[n];
    avp = new Vector3d[n];
    fvp = new Vector3d[n];
    IC = new Matrix3d[n];
    
    H = MatrixXd::Zero(n,n);
    C.resize(n,1); // C gets over-written completely by the algorithm below.
    
    //Variable allocation for gradient calculations
    dXupdq = new Matrix3d[n];
    dIC = new Matrix3d*[n];
    for(int i=0; i < n; i++) {
      dIC[i] = new Matrix3d[n]; 
    }
    dH = MatrixXd::Zero(n*n,n);
    dvJdqd_mat = MatrixXd::Zero(3,n);
//     dcross.resize(3,n);
    dC = MatrixXd::Zero(n,2*n);
    
    dvdq = new MatrixXd[n];
    dvdqd = new MatrixXd[n];
    davpdq = new MatrixXd[n];
    davpdqd = new MatrixXd[n];
    dfvpdq = new MatrixXd[n];
    dfvpdqd = new MatrixXd[n];
    
    for(int i=0; i < n; i++) {
     dvdq[i] = MatrixXd::Zero(3,n);
     dvdqd[i] = MatrixXd::Zero(3,n);
     davpdq[i] = MatrixXd::Zero(3,n);
     davpdqd[i] = MatrixXd::Zero(3,n);
     dfvpdq[i] = MatrixXd::Zero(3,n);
     dfvpdqd[i] = MatrixXd::Zero(3,n);
    }        
    //This assumes that there is only one "world" object
    bodies = new PlanarRigidBody[n+1];
    
    for(int i=0; i < n+1; i++) {
      bodies[i].setN(n);
    }
    
    kinematicsInit = false;
    secondDerivativesCached = 0;
    cached_q = new double[n];
  }
  
  ~PlanarModel() {
    delete[] jcode;
    delete[] parent;
    delete[] Xtree;
    delete[] I;
    
    delete[] S;
    delete[] Xup;
    delete[] v;
    delete[] avp;
    delete[] fvp;
    delete[] IC;
    
    delete[] dXupdq;
    for (int i=0; i<NB; i++) {
      delete[] dIC[i]; 
    }
    delete[] dIC;
    delete[] dvdq;
    delete[] dvdqd;
    delete[] davpdq;
    delete[] davpdqd;
    delete[] dfvpdq;
    delete[] dfvpdqd;
    
    delete[] bodies;
    delete[] cached_q;
  }
};
