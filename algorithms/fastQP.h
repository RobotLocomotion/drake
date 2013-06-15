
#ifndef __FAST_QP__
#define __FAST_QP__
#include <Eigen/Dense>
#include <vector>
#include <set>

// todo: template this
template <typename tA, typename tB, typename tC, typename tD, typename tE, typename tF, typename tG>
int fastQP(std::vector< Eigen::Map<tA> > QblkDiag, const Eigen::MatrixBase<tB>& f, const Eigen::MatrixBase<tC>& Aeq, const Eigen::MatrixBase<tD>& beq, const Eigen::MatrixBase<tE>& Ain, const Eigen::MatrixBase<tF>& bin, std::set<int>& active, Eigen::MatrixBase<tG>& x);
#endif
