#pragma once

#include <set>
#include <vector>

#include <Eigen/Dense>

int fastQPThatTakesQinv(
  std::vector<Eigen::MatrixXd*> QinvblkDiag, const Eigen::VectorXd& f,
  const Eigen::MatrixXd& Aeq, const Eigen::VectorXd& beq,
  const Eigen::MatrixXd& Ain, const Eigen::VectorXd& bin,
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  std::set<int>& active, Eigen::VectorXd& x);

int fastQP(
  std::vector<Eigen::MatrixXd*> QblkDiag, const Eigen::VectorXd& f,
  const Eigen::MatrixXd& Aeq, const Eigen::VectorXd& beq,
  const Eigen::MatrixXd& Ain, const Eigen::VectorXd& bin,
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  std::set<int>& active, Eigen::VectorXd& x);

// int fastQP(std::vector< Eigen::MatrixXd* > QblkDiag, const Eigen::VectorXd&
// f, const Eigen::MatrixXd& Aeq, const Eigen::VectorXd& beq, const
// Eigen::MatrixXd& Ain, const Eigen::VectorXd& bin, std::set<int>& active,
// Eigen::VectorXd& x, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub);

/* TODO: restore templated versions
template <typename tA, typename tB, typename tC, typename tD, typename tE,
typename tF, typename tG>
int fastQP(std::vector< Eigen::Map<tA> > QblkDiag, const Eigen::MatrixBase<tB>&
f, const Eigen::MatrixBase<tC>& Aeq, const Eigen::MatrixBase<tD>& beq, const
Eigen::MatrixBase<tE>& Ain, const Eigen::MatrixBase<tF>& bin, std::set<int>&
active, Eigen::MatrixBase<tG>& x);

template <typename tA, typename tB, typename tC, typename tD, typename tE,
typename tF, typename tG>
int fastQPThatTakesQinv(std::vector< Eigen::Map<tA> > QinvblkDiag, const
Eigen::MatrixBase<tB>& f, const Eigen::MatrixBase<tC>& Aeq, const
Eigen::MatrixBase<tD>& beq, const Eigen::MatrixBase<tE>& Ain, const
Eigen::MatrixBase<tF>& bin, std::set<int>& active, Eigen::MatrixBase<tG>& x);
*/
