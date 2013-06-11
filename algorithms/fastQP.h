
#ifndef __FAST_QP__
#define __FAST_QP__

#include <Eigen/Dense>
#include <vector>
#include <set>

// todo: template this
int fastQP(std::vector< Eigen::Map<Eigen::MatrixXd> > QblkDiag, const Eigen::Map<Eigen::VectorXd> f, const Eigen::Map<Eigen::MatrixXd> Aeq, const Eigen::Map<Eigen::VectorXd> beq, const Eigen::Map<Eigen::MatrixXd> Ain, const Eigen::Map<Eigen::VectorXd> bin, std::set<int>& active, Eigen::Map<Eigen::VectorXd>& x);

#endif
