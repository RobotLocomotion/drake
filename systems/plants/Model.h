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
    this->NB = n;
    this->pitch = new int[n];
    this->parent = new int[n];
    this->Xtree = new MatrixXd[n];
    this->I = new MatrixXd[n];
    this->a_grav = VectorXd::Zero(6);
    
    this->S = new VectorXd[n];
    this->Xup = new MatrixXd[n];
    this->v = new VectorXd[n];
    this->avp = new VectorXd[n];
    this->fvp = new VectorXd[n];
    this->IC = new MatrixXd[n];
    
        
    for(int i=0; i < n; i++) {
     this->Xtree[i] = MatrixXd::Zero(6,6);
     this->I[i] = MatrixXd::Zero(6,6);
     this->S[i] = VectorXd::Zero(6);
     this->Xup[i] = MatrixXd::Zero(6,6);
     this->v[i] = VectorXd::Zero(6);
     this->avp[i] = VectorXd::Zero(6);     
     this->fvp[i] = VectorXd::Zero(6);
     this->IC[i] = MatrixXd::Zero(6,6);
    } 
    
    this->H = MatrixXd::Zero(n,n);
    this->C.resize(n,1); // C gets over-written completely by the algorithm below.
    
    //Variable allocation for gradient calculations
    this->dXupdq = new MatrixXd[n];
    this->dIC = new MatrixXd*[n];
    for(int i=0; i < n; i++) {
     this->dIC[i] = new MatrixXd[n]; 
	    for(int j=0; j < n; j++) {
	    	this->dIC[i][j] = MatrixXd::Zero(6,6);
		}
    }
    this->dH = MatrixXd::Zero(n*n,n);
    this->dvJdqd_mat = MatrixXd::Zero(6,n);
//     this->dcross.resize(6,n);
    this->dC = MatrixXd::Zero(n,3*n);
    
    this->dvdq = new MatrixXd[n];
    this->dvdqd = new MatrixXd[n];
    this->davpdq = new MatrixXd[n];
    this->davpdqd = new MatrixXd[n];
    this->dfvpdq = new MatrixXd[n];
    this->dfvpdqd = new MatrixXd[n];
    
    for(int i=0; i < n; i++) {
     this->dvdq[i] = MatrixXd::Zero(6,n);
     this->dvdqd[i] = MatrixXd::Zero(6,n);
     this->davpdq[i] = MatrixXd::Zero(6,n);
     this->davpdqd[i] = MatrixXd::Zero(6,n);
     this->dfvpdq[i] = MatrixXd::Zero(6,n);
     this->dfvpdqd[i] = MatrixXd::Zero(6,n);
    }        
    //This assumes that there is only one "world" object
    this->bodies = new RigidBody[n+1];
    
    for(int i=0; i < n+1; i++) {
      this->bodies[i].setN(n);
    }
    
    this->kinematicsInit = false;
    this->cached_q = new double[n];
    this->secondDerivativesCached = 0;
  }
  
  ~Model() {
    delete[] this->pitch;
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
