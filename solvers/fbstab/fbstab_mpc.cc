#include "drake/solvers/fbstab/fbstab_mpc.h"

#include <memory>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>

#include "drake/solvers/fbstab/components/mpc_data.h"
#include "drake/solvers/fbstab/components/mpc_feasibility.h"
#include "drake/solvers/fbstab/components/mpc_residual.h"
#include "drake/solvers/fbstab/components/mpc_variable.h"
#include "drake/solvers/fbstab/components/riccati_linear_solver.h"

namespace drake {
namespace solvers {
namespace fbstab {

FBstabMpc::FBstabMpc(int N, int nx, int nu, int nc) {
  if (N < 1 || nx < 1 || nu < 1 || nc < 1) {
    throw std::runtime_error(
        "In FBstabMpc::FBstabMpc: problem sizes must be positive.");
  }

  N_ = N;
  nx_ = nx;
  nu_ = nu;
  nc_ = nc;
  nz_ = (nx + nu) * (N + 1);
  nl_ = nx * (N + 1);
  nv_ = nc * (N + 1);

  // create the components
  x1_ = std::make_unique<MpcVariable>(N, nx, nu, nc);
  x2_ = std::make_unique<MpcVariable>(N, nx, nu, nc);
  x3_ = std::make_unique<MpcVariable>(N, nx, nu, nc);
  x4_ = std::make_unique<MpcVariable>(N, nx, nu, nc);

  r1_ = std::make_unique<MpcResidual>(N, nx, nu, nc);
  r2_ = std::make_unique<MpcResidual>(N, nx, nu, nc);

  linear_solver_ = std::make_unique<RiccatiLinearSolver>(N, nx, nu, nc);

  feasibility_checker_ = std::make_unique<MpcFeasibility>(N, nx, nu, nc);

  algorithm_ = std::make_unique<FBstabAlgoMpc>(
      x1_.get(), x2_.get(), x3_.get(), x4_.get(), r1_.get(), r2_.get(),
      linear_solver_.get(), feasibility_checker_.get());
}

SolverOut FBstabMpc::Solve(const QPData& qp, const QPVariable* x,
                           bool use_initial_guess) {
  MpcData data(qp.Q, qp.R, qp.S, qp.q, qp.r, qp.A, qp.B, qp.c, qp.E, qp.L, qp.d,
               qp.x0);
  MpcVariable x0(x->z, x->l, x->v, x->y);

  if (data.N_ != N_ || data.nx_ != nx_ || data.nu_ != nu_ || data.nc_ != nc_) {
    throw std::runtime_error(
        "In FBstabMpc::Solve: mismatch between *this and data dimensions.");
  }
  if (x0.nz_ != nz_ || x0.nl_ != nl_ || x0.nv_ != nv_) {
    throw std::runtime_error(
        "In FBstabMpc::Solve: mismatch between *this and initial guess "
        "dimensions.");
  }

  if (!use_initial_guess) {
    x0.Fill(0.0);
  }

  return algorithm_->Solve(&data, &x0);
}

void FBstabMpc::UpdateOption(const char* option, double value) {
  algorithm_->UpdateOption(option, value);
}
void FBstabMpc::UpdateOption(const char* option, int value) {
  algorithm_->UpdateOption(option, value);
}
void FBstabMpc::UpdateOption(const char* option, bool value) {
  algorithm_->UpdateOption(option, value);
}
void FBstabMpc::SetDisplayLevel(FBstabAlgoMpc::Display level) {
  algorithm_->set_display_level(level);
}

// Explicit instantiation.
template class FBstabAlgorithm<MpcVariable, MpcResidual, MpcData,
                               RiccatiLinearSolver, MpcFeasibility>;

}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
