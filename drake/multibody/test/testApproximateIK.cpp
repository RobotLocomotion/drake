#include "drake/multibody/rigid_body_ik.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/constraint/rigid_body_constraint.h"
#include <iostream>
#include <cstdlib>
#include <limits>

// TODO(jwnimmer-tri) Someone with gurobi needs to fix these.
using namespace std;  // NOLINT(build/namespaces)
using namespace Eigen;  // NOLINT(build/namespaces)

int main() {
  RigidBodyTree<double>* model =
      new RigidBodyTree<double>(
          "examples/Atlas/urdf/atlas_minimal_contact.urdf");
  if (!model) {
    cerr << "ERROR: Failed to load model" << endl;
  }
  Vector2d tspan;
  tspan << 0, 1;
  VectorXd q0 = model->getZeroConfiguration();
  q0(3) = 0.8;
  Vector3d com_des = Vector3d::Zero();
  com_des(2) = std::numeric_limits<double>::quiet_NaN();
  WorldCoMConstraint* com_kc = new WorldCoMConstraint(model, com_des, com_des);
  int num_constraints = 1;
  RigidBodyConstraint** constraint_array =
      new RigidBodyConstraint* [num_constraints];
  constraint_array[0] = com_kc;
  IKoptions ikoptions(model);
  VectorXd q_sol(model->get_num_positions());
  int info;
  approximateIK(model, q0, q0, num_constraints, constraint_array,
                ikoptions, &q_sol, &info);
  printf("info = %d\n", info);
  delete com_kc;
  delete[] constraint_array;
  return 0;
}
