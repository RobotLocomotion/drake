
#include "RigidBodySystem.h"
#include "RigidBodyIK.h"  // required for resolving initial conditions

using namespace Drake;

RigidBodySystem::StateVector<double> Drake::getInitialState(const RigidBodySystem& sys) {

  Eigen::VectorXd x0 = Eigen::Matrix<double,Eigen::Dynamic,1>::Random(sys.tree->num_positions+sys.tree->num_velocities);

  if (sys.tree->getNumPositionConstraints()) {
    // todo: move this up to the system level?

    using namespace std;
    using namespace Eigen;

    std::vector<RigidBodyLoop,Eigen::aligned_allocator<RigidBodyLoop> > const& loops = sys.tree->loops;

    int nq = sys.tree->num_positions;
    int num_constraints = 2*loops.size();
    RigidBodyConstraint** constraint_array = new RigidBodyConstraint*[num_constraints];

    Matrix<double,7,1> bTbp = Matrix<double,7,1>::Zero();  bTbp(3)=1.0;
    Vector2d tspan; tspan<<-std::numeric_limits<double>::infinity(),std::numeric_limits<double>::infinity();
    Vector3d zero = Vector3d::Zero();
    for (int i=0; i<loops.size(); i++) {
      constraint_array[2*i] = new RelativePositionConstraint(sys.tree.get(), zero, zero, zero, loops[i].frameA->frame_index, loops[i].frameB->frame_index,bTbp,tspan);
      constraint_array[2*i+1] = new RelativePositionConstraint(sys.tree.get(), loops[i].axis, loops[i].axis, loops[i].axis, loops[i].frameA->frame_index, loops[i].frameB->frame_index,bTbp,tspan);
    }

    int info;
    vector<string> infeasible_constraint;
    IKoptions ikoptions(sys.tree.get());

    VectorXd q_guess = x0.topRows(nq);
    VectorXd q(nq);

    inverseKin(sys.tree.get(),q_guess,q_guess,num_constraints,constraint_array,q,info,infeasible_constraint,ikoptions);
    if (info>=10) {
      cout << "INFO = " << info << endl;
      cout << infeasible_constraint.size() << " infeasible constraints:";
      size_t limit = infeasible_constraint.size();
      if (limit>5) { cout << " (only printing first 5)" << endl; limit=5; }
      cout << endl;
      for (int i=0; i<limit; i++)
        cout << infeasible_constraint[i] << endl;
    }
    x0 << q, VectorXd::Zero(sys.tree->num_velocities);

    for (int i=0; i<num_constraints; i++) {
      delete constraint_array[i];
    }
    delete[] constraint_array;
  }
  return x0;
}

