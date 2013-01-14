#include <Eigen/Dense>
#include "RigidBody.h"
using namespace Eigen;
using namespace std;

class Model 
{
public:
  int NB;
  int *pitch;
  int *parent;
  MatrixXd* Xtree;
  MatrixXd* I;
  VectorXd a_grav;
  
  VectorXd* S;
  MatrixXd* Xup;
  VectorXd* v;
  VectorXd* avp;
  VectorXd* fvp;
  MatrixXd* IC;
  MatrixXd H;
  MatrixXd C;
  
  //Variables for gradient calculations
  MatrixXd* dXupdq;
  MatrixXd** dIC;
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
  
  RigidBody* bodies;
  bool kinematicsInit;
  double *cached_q;
  int secondDerivativesCached;

  Model(int n) {
    NB = n;
    pitch = new int[n];
    parent = new int[n];
    Xtree = new MatrixXd[n];
    I = new MatrixXd[n];
    a_grav = VectorXd::Zero(6);
    
    S = new VectorXd[n];
    Xup = new MatrixXd[n];
    v = new VectorXd[n];
    avp = new VectorXd[n];
    fvp = new VectorXd[n];
    IC = new MatrixXd[n];
    
        
    for(int i=0; i < n; i++) {
      Xtree[i] = MatrixXd::Zero(6,6);
      I[i] = MatrixXd::Zero(6,6);
      S[i] = VectorXd::Zero(6);
      Xup[i] = MatrixXd::Zero(6,6);
      v[i] = VectorXd::Zero(6);
      avp[i] = VectorXd::Zero(6);
      fvp[i] = VectorXd::Zero(6);
      IC[i] = MatrixXd::Zero(6,6);
    }
    
    H = MatrixXd::Zero(n,n);
    C.resize(n,1); // C gets over-written completely by the algorithm below.
    
    //Variable allocation for gradient calculations
    dXupdq = new MatrixXd[n];
    dIC = new MatrixXd*[n];
    for(int i=0; i < n; i++) {
      dIC[i] = new MatrixXd[n];
	    for(int j=0; j < n; j++) {
        dIC[i][j] = MatrixXd::Zero(6,6);
      }
    }
    dH = MatrixXd::Zero(n*n,n);
    dvJdqd_mat = MatrixXd::Zero(6,n);
//     dcross.resize(6,n);
    dC = MatrixXd::Zero(n,3*n);
    
    dvdq = new MatrixXd[n];
    dvdqd = new MatrixXd[n];
    davpdq = new MatrixXd[n];
    davpdqd = new MatrixXd[n];
    dfvpdq = new MatrixXd[n];
    dfvpdqd = new MatrixXd[n];
    
    for(int i=0; i < n; i++) {
      dvdq[i] = MatrixXd::Zero(6,n);
      dvdqd[i] = MatrixXd::Zero(6,n);
      davpdq[i] = MatrixXd::Zero(6,n);
      davpdqd[i] = MatrixXd::Zero(6,n);
      dfvpdq[i] = MatrixXd::Zero(6,n);
      dfvpdqd[i] = MatrixXd::Zero(6,n);
    }
    //This assumes that there is only one "world" object
    bodies = new RigidBody[n+1];
    
    for(int i=0; i < n+1; i++) {
      bodies[i].setN(n);
    }
    
    kinematicsInit = false;
    cached_q = new double[n];
    secondDerivativesCached = 0;
  }
  
  ~Model() {
    delete[] pitch;
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
