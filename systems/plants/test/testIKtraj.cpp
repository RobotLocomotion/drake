#include "RigidBodyIK.h"
#include "RigidBodyManipulator.h"
#include "../constraint/RigidBodyConstraint.h"

#include "../IKoptions.h"
#include <iostream>
#include <cstdlib>
#include <limits>

using namespace std;
using namespace Eigen;
int main()
{
  RigidBodyManipulator rbm("examples/Atlas/urdf/atlas_minimal_contact.urdf");
  RigidBodyManipulator* model = &rbm;

  if(!model)
  {
    cerr<<"ERROR: Failed to load model"<<endl;
  }
  Vector2d tspan;
  tspan<<0,1;
  int l_hand;
  int r_hand;
  //int l_foot;
  //int r_foot;
  for(int i = 0;i<model->num_bodies;i++)
  {
    if(model->bodies[i]->linkname.compare(string("l_hand")))
    {
      l_hand = i;
    }
    else if(model->bodies[i]->linkname.compare(string("r_hand")))
    {
      r_hand = i;
    }
    //else if(model->bodies[i].linkname.compare(string("l_foot")))
    //{
    //  l_foot = i;
    //}
    //else if(model->bodies[i].linkname.compare(string("r_foot")))
    //{
    //  r_foot = i;
    //}
  }
  int nq = model->num_positions;
  VectorXd qstar = VectorXd::Zero(nq);
  qstar(3) = 0.8;
  model->doKinematics(qstar);
  Vector3d com0;
  model->getCOM(com0);
  Vector3d l_hand_pt;
  l_hand_pt << 0.0, 0.0, 0.0;
  Vector3d r_hand_pt;
  r_hand_pt << 0.0, 0.0, 0.0;
  Vector3d lhand_pos0;
  model->forwardKin(l_hand,l_hand_pt,0,lhand_pos0);
  Vector3d rhand_pos0;
  model->forwardKin(r_hand,r_hand_pt,0,rhand_pos0);
  int nT = 4;
  double* t = new double[nT];
  double dt = 1.0/(nT-1);
  for(int i = 0;i<nT;i++)
  {
    t[i] = dt*i;
  }
  MatrixXd q0 = qstar.replicate(1,nT);
  VectorXd qdot0 = VectorXd::Zero(model->num_velocities);
  Vector3d com_lb = com0;
  com_lb(0) = std::numeric_limits<double>::quiet_NaN();
  com_lb(1) = std::numeric_limits<double>::quiet_NaN();
  Vector3d com_ub = com0;
  com_ub(0) = std::numeric_limits<double>::quiet_NaN();
  com_ub(1) = std::numeric_limits<double>::quiet_NaN();
  com_ub(2) = com0(2)+0.5;
  WorldCoMConstraint* com_kc = new WorldCoMConstraint(model,com_lb,com_ub);
  Vector3d rhand_pos_lb = rhand_pos0;
  rhand_pos_lb(0) +=0.1;
  rhand_pos_lb(1) +=0.05;
  rhand_pos_lb(2) +=0.25;
  Vector3d rhand_pos_ub = rhand_pos_lb;
  rhand_pos_ub(2) += 0.25;
  Vector2d tspan_end;
  tspan_end<<t[nT-1],t[nT-1];
  WorldPositionConstraint* kc_rhand = new WorldPositionConstraint(model,r_hand,r_hand_pt,rhand_pos_lb,rhand_pos_ub,tspan_end);
  int num_constraints = 2;
  RigidBodyConstraint** constraint_array = new RigidBodyConstraint*[num_constraints];
  constraint_array[0] = com_kc;
  constraint_array[1] = kc_rhand;
  IKoptions ikoptions(model);
  MatrixXd q_sol(model->num_positions,nT);
  MatrixXd qdot_sol(model->num_velocities,nT);
  MatrixXd qddot_sol(model->num_positions,nT);
  int info = 0;
  vector<string> infeasible_constraint;
  inverseKinTraj(model,nT,t,qdot0,q0,q0,num_constraints,constraint_array,q_sol,qdot_sol,qddot_sol,info,infeasible_constraint,ikoptions);
  printf("INFO = %d\n",info);
  if(info != 1)
  {
    cerr<<"Failure"<<endl;
    return 1;
  }
  ikoptions.setFixInitialState(false);
  ikoptions.setMajorIterationsLimit(500);
  inverseKinTraj(model,nT,t,qdot0,q0,q0,num_constraints,constraint_array,q_sol,qdot_sol,qddot_sol,info,infeasible_constraint,ikoptions);
  printf("INFO = %d\n",info);
  if(info != 1)
  {
    cerr<<"Failure"<<endl;
    return 1;
  }
  RowVectorXd t_inbetween(5);
  t_inbetween << 0.1,0.15,0.3,0.4,0.6;
  ikoptions.setAdditionaltSamples(t_inbetween);
  inverseKinTraj(model,nT,t,qdot0,q0,q0,num_constraints,constraint_array,q_sol,qdot_sol,qddot_sol,info,infeasible_constraint,ikoptions);
  printf("INFO = %d\n",info);
  if(info != 1)
  {
    cerr<<"Failure"<<endl;
    return 1;
  }
  delete com_kc;
  delete[] constraint_array;
  delete[] t;
  return 0;
}
