#include <math.h>

#include <set>

#include <gurobi_c++.h>

#include "drake/multibody/constraint/rigid_body_constraint.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/multibody/rigid_body_tree.h"

// TODO(jwnimmer-tri) Someone with gurobi needs to fix these.
// NOLINTNEXTLINE(build/namespaces)
using namespace std;
// NOLINTNEXTLINE(build/namespaces)
using namespace Eigen;

template <typename DerivedA, typename DerivedB, typename DerivedC>
void approximateIK(RigidBodyTree<double>* model,
                   const MatrixBase<DerivedA>& q_seed,
                   const MatrixBase<DerivedB>& q_nom, const int num_constraints,
                   const RigidBodyConstraint* const* constraint_array,
                   const IKoptions& ikoptions,
                   MatrixBase<DerivedC>* q_sol, int* info) {
  int num_kc = 0;
  int nq = model->get_num_positions();
  const SingleTimeKinematicConstraint** kc_array =
      new const SingleTimeKinematicConstraint* [num_constraints];
  double* joint_lb = new double[nq];
  double* joint_ub = new double[nq];
  for (int j = 0; j < nq; j++) {
    joint_lb[j] = model->joint_limit_min[j];
    joint_ub[j] = model->joint_limit_max[j];
  }
  for (int i = 0; i < num_constraints; i++) {
    int constraint_category = constraint_array[i]->getCategory();
    if (constraint_category ==
        RigidBodyConstraint::SingleTimeKinematicConstraintCategory) {
      kc_array[num_kc] = static_cast<
        const SingleTimeKinematicConstraint*>(constraint_array[i]);

      num_kc++;
    } else if (constraint_category ==
               RigidBodyConstraint::PostureConstraintCategory) {
      VectorXd joint_min, joint_max;
      const PostureConstraint* pc =
          static_cast<const PostureConstraint*>(constraint_array[i]);
      pc->bounds(nullptr, joint_min, joint_max);
      for (int j = 0; j < nq; j++) {
        joint_lb[j] = (joint_lb[j] < joint_min[j] ? joint_min[j] : joint_lb[j]);
        joint_ub[j] = (joint_ub[j] > joint_max[j] ? joint_max[j] : joint_ub[j]);
        if (joint_lb[j] > joint_ub[j]) {
          cerr << "Drake:approximateIK:posture constraint has lower bound "
                  "larger than upper bound" << endl;
        }
      }
    } else {
      cerr
          << "Drake:approximateIK: The constraint category is not supported yet"
          << endl;
    }
  }
  MatrixXd Q;
  ikoptions.getQ(Q);
  int error;
  GRBenv* grb_env = nullptr;
  GRBmodel* grb_model = nullptr;

  VectorXd qtmp = -2 * Q * q_nom;

  // create gurobi environment
  error = GRBloadenv(&grb_env, nullptr);
  if (error) {
    printf("Load Gurobi environment error %s\n", GRBgeterrormsg(grb_env));
  }

  // set solver params
  // (http://www.gurobi.com/documentation/5.5/reference-manual/node798#sec:Parameters)
  error = GRBsetintparam(grb_env, "outputflag", 0);
  if (error) {
    printf("Set Gurobi outputflag error %s\n", GRBgeterrormsg(grb_env));
  }
  /*error = GRBsetintparam(grb_env,"method", 2);
  error = GRBsetintparam(grb_env,"presolve", 0);
  error = GRBsetintparam(grb_env,"bariterlimit", 20);
  error = GRBsetintparam(grb_env,"barhomogenous", 0);
  error = GRBsetdblparam(grb_env,"barconvtol", 1e-4);*/
  error = GRBnewmodel(grb_env, &grb_model, "ApproximateIK", nq, qtmp.data(),
                      joint_lb, joint_ub, nullptr, nullptr);
  if (error) {
    printf("Create Gurobi model error %s\n", GRBgeterrormsg(grb_env));
  }

  // set up cost function
  // cost: (q-q_nom)'*Q*(q-q_nom)
  error = GRBsetdblattrarray(grb_model, "Obj", 0, nq, qtmp.data());
  if (error) {
    printf("Set Gurobi obj error %s\n", GRBgeterrormsg(grb_env));
  }
  for (int i = 0; i < nq; i++) {
    for (int j = 0; j < nq; j++) {
      if (abs(Q(i, j)) > 1e-10) {
        error = GRBaddqpterms(grb_model, 1, &i, &j, Q.data() + i + j * nq);
        if (error) {
          printf("Gurobi error %s\n", GRBgeterrormsg(grb_env));
        }
      }
    }
  }
  int* allIndsData = new int[nq];
  for (int j = 0; j < nq; j++) {
    allIndsData[j] = j;
  }
  // TODO(tkoolen): pass this into the function?
  KinematicsCache<double> cache = model->doKinematics(q_seed);
  int kc_idx, c_idx;
  for (kc_idx = 0; kc_idx < num_kc; kc_idx++) {
    int nc = kc_array[kc_idx]->getNumConstraint(nullptr);
    VectorXd lb(nc);
    VectorXd ub(nc);
    VectorXd c(nc);
    MatrixXd dc(nc, nq);
    kc_array[kc_idx]->bounds(nullptr, lb, ub);
    kc_array[kc_idx]->eval(nullptr, cache, c, dc);
    for (c_idx = 0; c_idx < nc; c_idx++) {
      VectorXd rowVec = dc.row(c_idx);
      double* Jrow = rowVec.data();
      double c_seed = c(c_idx) - dc.row(c_idx) * q_seed;
      double rhs_row;
      if (std::isinf(-lb(c_idx))) {
        rhs_row = ub(c_idx) - c_seed;
        int gerror = GRBaddconstr(grb_model, nq, allIndsData, Jrow,
                                  GRB_LESS_EQUAL, rhs_row, nullptr);
        if (gerror) {
          printf("Gurobi error %s\n", GRBgeterrormsg(grb_env));
        }
      } else if (std::isinf(ub(c_idx))) {
        if (std::isinf(lb(c_idx))) {
          cerr << "Drake:approximateIK: lb and ub cannot be both infinity, "
                  "check the getConstraintBnds output of the "
                  "KinematicConstraint" << endl;
        }
        rhs_row = lb(c_idx) - c_seed;
        error = GRBaddconstr(grb_model, nq, allIndsData, Jrow,
                             GRB_GREATER_EQUAL, rhs_row, nullptr);
        if (error) {
          printf("Gurobi error %s\n", GRBgeterrormsg(grb_env));
        }
      } else if (ub(c_idx) - lb(c_idx) < 1e-10) {
        rhs_row = lb(c_idx) - c_seed;
        error = GRBaddconstr(grb_model, nq, allIndsData, Jrow, GRB_EQUAL,
                             rhs_row, nullptr);
        if (error) {
          printf("Gurobi error %s\n", GRBgeterrormsg(grb_env));
        }
      } else {
        double rhs_row1 = ub(c_idx) - c_seed;
        error = GRBaddconstr(grb_model, nq, allIndsData, Jrow, GRB_LESS_EQUAL,
                             rhs_row1, nullptr);
        if (error) {
          printf("Gurobi error %s\n", GRBgeterrormsg(grb_env));
        }
        double rhs_row2 = lb(c_idx) - c_seed;
        error = GRBaddconstr(grb_model, nq, allIndsData, Jrow,
                             GRB_GREATER_EQUAL, rhs_row2, nullptr);
        if (error) {
          printf("Gurobi error %s\n", GRBgeterrormsg(grb_env));
        }
      }
    }
  }
  error = GRBupdatemodel(grb_model);
  if (error) {
    printf("Gurobi error %s\n", GRBgeterrormsg(grb_env));
  }

  error = GRBoptimize(grb_model);
  if (error) {
    printf("Gurobi error %s\n", GRBgeterrormsg(grb_env));
  }

  VectorXd q_sol_data(nq);
  error =
      GRBgetdblattrarray(grb_model, GRB_DBL_ATTR_X, 0, nq, q_sol_data.data());
  (*q_sol) = q_sol_data;

  error = GRBgetintattr(grb_model, GRB_INT_ATTR_STATUS, info);
  if ((*info) == 2) {
    (*info) = 0;
  } else {
    (*info) = 1;
  }
  // debug only
  /*GRBwrite(grb_model,"gurobi_approximateIK.lp");
  int num_gurobi_cnst;
  GRBgetintattr(grb_model, GRB_INT_ATTR_NUMCONSTRS,&num_gurobi_cnst);
  MatrixXd J(num_gurobi_cnst, nq);
  VectorXd rhs(num_gurobi_cnst);
  for(int i = 0;i<num_gurobi_cnst;i++)
  {
    for(int j = 0;j<nq;j++)
    {
      GRBgetcoeff(grb_model, i, j,&J(i, j));
    }
    GRBgetdblattrarray(grb_model, GRB_DBL_ATTR_RHS, 0, num_gurobi_cnst, rhs.data());
  }
  */

  GRBfreemodel(grb_model);
  GRBfreeenv(grb_env);
  delete[] joint_lb;
  delete[] joint_ub;
  delete[] allIndsData;
  delete[] kc_array;
  return;
}

template void approximateIK(
  RigidBodyTree<double>*, const MatrixBase<Map<VectorXd>>& ,
  const MatrixBase<Map<VectorXd>>& , const int,
  const RigidBodyConstraint* const*,
  const IKoptions&, MatrixBase<Map<VectorXd>>*, int*);

template void approximateIK(
  RigidBodyTree<double>*, const MatrixBase<VectorXd>& ,
  const MatrixBase<VectorXd>& , const int,
  const RigidBodyConstraint* const*,
  const IKoptions&, MatrixBase<VectorXd>*, int*);
