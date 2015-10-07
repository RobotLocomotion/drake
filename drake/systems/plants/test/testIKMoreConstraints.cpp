#include <drake/RigidBodyIK.h>
#include <drake/RigidBodyManipulator.h>
#include <drake/RigidBodyConstraint.h>

#include <drake/IKoptions.h>
#include <iostream>
#include <cstdlib>
#include <limits>
#include <boost/shared_ptr.hpp>

using namespace std;
using namespace Eigen;



int main()
{

  RigidBodyManipulator rbm("examples/Atlas/urdf/atlas_minimal_contact.urdf");
  RigidBodyManipulator* model = &rbm;


  int r_hand, l_foot, r_foot;
  for(int i = 0;i<model->bodies.size();i++)
  {
    if(model->bodies[i]->linkname.compare(string("r_hand")) ==0 )
    {
      r_hand = i;
    }
    if(model->bodies[i]->linkname.compare(string("l_foot")) ==0 )
    { 
      l_foot = i;
    }
    if(model->bodies[i]->linkname.compare(string("r_foot")) ==0 )
    {
      r_foot = i;
    }
  }


  if(!model)
  {
    cerr<<"ERROR: Failed to load model"<<endl;
  }

  Vector2d tspan;
  tspan<<0,1;

  // Default Atlas v5 posture:
  VectorXd qstar(model->num_positions);
qstar <<  -0.0260, 0,  0.8440, 0, 0, 0, 0, 0, 0,  0.2700, 0,  0.0550, -0.5700,  1.1300, -0.5500, -0.0550, -1.3300,  2.1530,  0.5000,  0.0985, 0,  0.0008, -0.2700, 0, -0.0550, -0.5700,  1.1300, -0.5500,  0.0550,  1.3300,  2.1530, -0.5000,  0.0985, 0,  0.0008,  0.2564;
  //model->doKinematics(qstar, v);

  // 1 CoM Constraint
  Vector3d com_lb = Vector3d::Zero(); 
  Vector3d com_ub = Vector3d::Zero(); 
  com_lb(0) = -0.1;
  com_lb(1) = -0.1;
  com_lb(2) = 0.6;
  com_ub(0) = 0.1;
  com_ub(1) = 0.1;
  com_ub(2) = 1.0;
  WorldCoMConstraint* kc_com = new WorldCoMConstraint(model,com_lb,com_ub,tspan);

  // 2 Back Posture Constraint
  PostureConstraint* kc_posture_back = new PostureConstraint(model, tspan);
  std::vector<int> back_idx;
  back_idx.push_back(6);
  back_idx.push_back(7);
  back_idx.push_back(8);
  VectorXd back_lb = VectorXd::Zero(3);
  VectorXd back_ub = VectorXd::Zero(3);
  kc_posture_back->setJointLimits(3, back_idx.data(), back_lb, back_ub);

  // 3 Knees Constraint
  PostureConstraint* kc_posture_knees = new PostureConstraint(model, tspan);
  std::vector<int> knee_idx;
  knee_idx.push_back(13);
  knee_idx.push_back(26);
  VectorXd knee_lb = VectorXd::Zero(2);
  knee_lb(0) = 1.0; // usually use 0.6
  knee_lb(1) = 1.0; // usually use 0.6
  VectorXd knee_ub = VectorXd::Zero(2);
  knee_ub(0) = 2.5;
  knee_ub(1) = 2.5;
  kc_posture_knees->setJointLimits(2, knee_idx.data(), knee_lb, knee_ub);

  // 4 Left Arm Constraint
  PostureConstraint* kc_posture_larm = new PostureConstraint(model, tspan);
  std::vector<int> larm_idx;
  larm_idx.push_back(9);
  larm_idx.push_back(16);
  larm_idx.push_back(17);
  larm_idx.push_back(18);
  larm_idx.push_back(19);
  larm_idx.push_back(20);
  larm_idx.push_back(21);
  VectorXd larm_lb = VectorXd::Zero(7);
  larm_lb(0) = 0.27;
  larm_lb(1) = -1.33;
  larm_lb(2) = 2.153;
  larm_lb(3) = 0.500;
  larm_lb(4) = 0.0985;
  larm_lb(5) = 0;
  larm_lb(6) = 0.0008;
  VectorXd larm_ub = larm_lb;
  kc_posture_larm->setJointLimits(7, larm_idx.data(), larm_lb, larm_ub);

  // 5 Left Foot Position and Orientation Constraints
  Vector3d l_foot_pt = Vector3d::Zero();
  Vector3d lfoot_pos0;
  // = model->forwardKin(l_foot_pt, l_foot, 0, 0, 0).value();
  lfoot_pos0(0) = 0;
  lfoot_pos0(1) = 0.13;
  lfoot_pos0(2) = 0.08;
  Vector3d lfoot_pos_lb = lfoot_pos0;
  lfoot_pos_lb(0) +=0.001;
  lfoot_pos_lb(1) +=0.001;
  lfoot_pos_lb(2) +=0.001;
  Vector3d lfoot_pos_ub = lfoot_pos_lb;
  lfoot_pos_ub(2) += 0.01;
  // std::cout << lfoot_pos0.transpose() << " lfoot\n" ;
  WorldPositionConstraint* kc_lfoot_pos = new WorldPositionConstraint(model,l_foot,l_foot_pt,lfoot_pos_lb,lfoot_pos_ub,tspan);
  Eigen::Vector4d quat_des(1,0,0,0);
  double tol = 0.0017453292519943296;
  WorldQuatConstraint* kc_lfoot_quat = new  WorldQuatConstraint(model, l_foot, quat_des, tol, tspan);

  // 5 Right Foot Position and Orientation Constraints
  Vector3d r_foot_pt = Vector3d::Zero();
  Vector3d rfoot_pos0;
  //Vector3d rfoot_pos0 = model->forwardKin(r_foot_pt, r_foot, 0, 0, 0).value();
  rfoot_pos0(0) = 0;
  rfoot_pos0(1) = -0.13;
  rfoot_pos0(2) = 0.08;
  Vector3d rfoot_pos_lb = rfoot_pos0;
  rfoot_pos_lb(0) +=0.001;
  rfoot_pos_lb(1) +=0.001;
  rfoot_pos_lb(2) +=0.001;
  Vector3d rfoot_pos_ub = rfoot_pos_lb;
  rfoot_pos_ub(2) += 0.001;
  //std::cout << rfoot_pos0.transpose() << " lfoot\n" ;
  WorldPositionConstraint* kc_rfoot_pos = new WorldPositionConstraint(model,r_foot,r_foot_pt,rfoot_pos_lb,rfoot_pos_ub,tspan);
  WorldQuatConstraint* kc_rfoot_quat = new  WorldQuatConstraint(model, r_foot, quat_des, tol, tspan);


  // 6 Right Position Constraints (actual reaching constraint)
  Vector3d r_hand_pt = Vector3d::Zero();
  Vector3d rhand_pos0;
  //Vector3d rhand_pos0 = model->forwardKin(r_hand_pt, r_hand, 0, 0, 0).value();
  rhand_pos0(0) = 0.2;
  rhand_pos0(1) = -0.5;
  rhand_pos0(2) = 0.4;

  Vector3d rhand_pos_lb = rhand_pos0;
  rhand_pos_lb(0) +=0.05;
  rhand_pos_lb(1) +=0.05;
  rhand_pos_lb(2) +=0.05;
  Vector3d rhand_pos_ub = rhand_pos_lb;
  rhand_pos_ub(2) += 0.05;
  //std::cout << rhand_pos_ub.transpose() << " rhand\n" ;
  WorldPositionConstraint* kc_rhand = new WorldPositionConstraint(model,r_hand,r_hand_pt,rhand_pos_lb,rhand_pos_ub,tspan);


  // 7 QuasiStatic Constraints
  QuasiStaticConstraint* kc_quasi = new QuasiStaticConstraint(model, tspan);
  kc_quasi->setShrinkFactor(0.2);
  kc_quasi->setActive(true);
  Eigen::Matrix3Xd l_foot_pts = Eigen::Matrix3Xd::Zero(3, 4);
  l_foot_pts <<    -0.0820 ,  -0.0820 ,   0.1780 ,   0.1780   , 0.0624  , -0.0624  ,  0.0624 ,  -0.0624  , -0.0811,   -0.0811  , -0.0811 ,  -0.0811;
  //std::cout << l_foot_pts <<"\n";
  kc_quasi->addContact(1, &l_foot, &l_foot_pts);
  Eigen::Matrix3Xd r_foot_pts = Eigen::Matrix3Xd::Zero(3, 4);
  r_foot_pts <<    -0.0820 ,  -0.0820 ,   0.1780 ,   0.1780   , 0.0624  , -0.0624  ,  0.0624 ,  -0.0624  , -0.0811,   -0.0811  , -0.0811 ,  -0.0811;
  kc_quasi->addContact(1, &r_foot, &r_foot_pts);



  int num_constraints = 9;
  RigidBodyConstraint** constraint_array = new RigidBodyConstraint*[num_constraints];
  constraint_array[0] = kc_quasi;
  constraint_array[1] = kc_posture_knees;
  constraint_array[2] = kc_lfoot_pos;
  constraint_array[3] = kc_lfoot_quat;
  constraint_array[4] = kc_rfoot_pos;
  constraint_array[5] = kc_rfoot_quat;
  constraint_array[6] = kc_rhand;
  constraint_array[7] = kc_posture_larm;
  constraint_array[8] = kc_posture_back;
  //  constraint_array[0] = kc_com;

  IKoptions ikoptions(model);
  VectorXd q_sol(model->num_positions);
  int info;
  vector<string> infeasible_constraint;
  inverseKin(model,qstar,qstar,num_constraints,constraint_array,q_sol,info,infeasible_constraint,ikoptions);
  printf("INFO = %d\n",info);
  if(info != 1)
  {
    return 1;
  }



  /////////////////////////////////////////
  KinematicsCache<double> cache = model->doKinematics(q_sol, 0);
  Vector3d com = model->centerOfMass(cache, 0).value();
  printf("%5.2f\n%5.2f\n%5.2f\n",com(0),com(1),com(2));

  delete kc_com;
  delete kc_quasi;
  delete kc_posture_knees;
  delete kc_lfoot_pos;
  delete kc_lfoot_quat;
  delete kc_rfoot_pos;
  delete kc_rfoot_quat;
  delete kc_rhand;
  delete kc_posture_larm;
  delete kc_posture_back;
  delete[] constraint_array;
  return 0;
}
