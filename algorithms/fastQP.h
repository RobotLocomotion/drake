
#ifndef __FAST_QP__
#define __FAST_QP__
#include <Eigen/Dense>
#include <vector>
#include <set>


int fastQPThatTakesQinv(std::vector< Eigen::MatrixXd* > QinvblkDiag, const Eigen::VectorXd& f, const Eigen::MatrixXd& Aeq, const Eigen::VectorXd& beq, const Eigen::MatrixXd& Ain, const Eigen::VectorXd& bin, std::set<int>& active, Eigen::VectorXd& x);
int fastQP(std::vector< Eigen::MatrixXd* > QblkDiag, const Eigen::VectorXd& f, const Eigen::MatrixXd& Aeq, const Eigen::VectorXd& beq, const Eigen::MatrixXd& Ain, const Eigen::VectorXd& bin, std::set<int>& active, Eigen::VectorXd& x);
//int fastQP(std::vector< Eigen::MatrixXd* > QblkDiag, const Eigen::VectorXd& f, const Eigen::MatrixXd& Aeq, const Eigen::VectorXd& beq, const Eigen::MatrixXd& Ain, const Eigen::VectorXd& bin, std::set<int>& active, Eigen::VectorXd& x, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub);

/* TODO: restore templated versions
template <typename tA, typename tB, typename tC, typename tD, typename tE, typename tF, typename tG>
int fastQP(std::vector< Eigen::Map<tA> > QblkDiag, const Eigen::MatrixBase<tB>& f, const Eigen::MatrixBase<tC>& Aeq, const Eigen::MatrixBase<tD>& beq, const Eigen::MatrixBase<tE>& Ain, const Eigen::MatrixBase<tF>& bin, std::set<int>& active, Eigen::MatrixBase<tG>& x);

template <typename tA, typename tB, typename tC, typename tD, typename tE, typename tF, typename tG>
int fastQPThatTakesQinv(std::vector< Eigen::Map<tA> > QinvblkDiag, const Eigen::MatrixBase<tB>& f, const Eigen::MatrixBase<tC>& Aeq, const Eigen::MatrixBase<tD>& beq, const Eigen::MatrixBase<tE>& Ain, const Eigen::MatrixBase<tF>& bin, std::set<int>& active, Eigen::MatrixBase<tG>& x);
*/

#include <gurobi_c++.h>

#define CGE( call, env ) {int gerror; gerror = call; if (gerror) std::cerr << "Gurobi error " << GRBgeterrormsg( env ) << std::endl; }

GRBmodel* gurobiQP(GRBenv *env, std::vector< Eigen::MatrixXd* > QblkDiag, Eigen::VectorXd& f, const Eigen::MatrixXd& Aeq, const Eigen::VectorXd& beq, const Eigen::MatrixXd& Ain, const Eigen::VectorXd& bin, Eigen::VectorXd& lb, Eigen::VectorXd& ub, std::set<int>& active, Eigen::VectorXd& x, double active_set_slack_tol=1e-4);

//template <typename tA, typename tB, typename tC, typename tD, typename tE>
//GRBmodel* gurobiQP(GRBenv *env, std::vector< Eigen::Map<tA> > QblkDiag, Eigen::VectorXd& f, const Eigen::MatrixBase<tB>& Aeq, const Eigen::MatrixBase<tC>& beq, const Eigen::MatrixBase<tD>& Ain, const Eigen::MatrixBase<tE>& bin, Eigen::VectorXd& lb, Eigen::VectorXd& ub, std::set<int>& active, Eigen::VectorXd& x);



#endif


