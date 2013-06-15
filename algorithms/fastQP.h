
#ifndef __FAST_QP__
#define __FAST_QP__
#include <Eigen/Dense>
#include <vector>
#include <set>


// todo: template this
template <typename tA, typename tB, typename tC, typename tD, typename tE, typename tF, typename tG>
int fastQP(std::vector< Eigen::Map<tA> > QblkDiag, const Eigen::MatrixBase<tB>& f, const Eigen::MatrixBase<tC>& Aeq, const Eigen::MatrixBase<tD>& beq, const Eigen::MatrixBase<tE>& Ain, const Eigen::MatrixBase<tF>& bin, std::set<int>& active, Eigen::MatrixBase<tG>& x);


#include <gurobi_c++.h>

#define CGE( call, env ) {int gerror; gerror = call; if (gerror) std::cerr << "Gurobi error " << GRBgeterrormsg( env ) << std::endl; }


template <typename tA, typename tB, typename tC, typename tD, typename tE>
GRBmodel* gurobiQP(GRBenv *env, std::vector< Eigen::Map<tA> > QblkDiag, Eigen::VectorXd& f, const Eigen::MatrixBase<tB>& Aeq, const Eigen::MatrixBase<tC>& beq, const Eigen::MatrixBase<tD>& Ain, const Eigen::MatrixBase<tE>& bin, Eigen::VectorXd& lb, Eigen::VectorXd& ub, std::set<int>& active, Eigen::VectorXd& x);



#endif


