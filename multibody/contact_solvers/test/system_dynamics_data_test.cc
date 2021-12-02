#include "drake/multibody/contact_solvers/system_dynamics_data.h"

#include <Eigen/SparseCore>
#include <gtest/gtest.h>

#include "drake/multibody/contact_solvers/sparse_linear_operator.h"

using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

GTEST_TEST(SystemDynamicsData, ForwardDynamicsDataConstruction) {
  const int nv = 10;
  const Eigen::SparseMatrix<double> Ainv_sparse(nv, nv);
  const SparseLinearOperator<double> Ainv_op("Aop", &Ainv_sparse);
  const VectorXd v_star = VectorXd::LinSpaced(nv, 1, nv);
  SystemDynamicsData<double> data(&Ainv_op, &v_star);
  EXPECT_EQ(data.num_velocities(), nv);
  EXPECT_TRUE(data.has_forward_dynamics());
  EXPECT_FALSE(data.has_inverse_dynamics());
  EXPECT_EQ(&data.get_Ainv(), &Ainv_op);
  EXPECT_EQ(&data.get_v_star(), &v_star);
}

GTEST_TEST(SystemDynamicsData, ForwardDynamicsDataConstructionAlternative) {
  const int nv = 10;
  const Eigen::SparseMatrix<double> Ainv_sparse(nv, nv);
  const SparseLinearOperator<double> Ainv_op("Aop", &Ainv_sparse);
  const VectorXd v_star = VectorXd::LinSpaced(nv, 1, nv);
  SystemDynamicsData<double> data(nullptr, &Ainv_op, &v_star);
  EXPECT_EQ(data.num_velocities(), nv);
  EXPECT_TRUE(data.has_forward_dynamics());
  EXPECT_FALSE(data.has_inverse_dynamics());
  EXPECT_EQ(&data.get_Ainv(), &Ainv_op);
  EXPECT_EQ(&data.get_v_star(), &v_star);
}

GTEST_TEST(SystemDynamicsData, InverseDynamicsDataConstruction) {
  const int nv = 10;
  const Eigen::SparseMatrix<double> Asparse(nv, nv);
  const SparseLinearOperator<double> Aop("Aop", &Asparse);
  const VectorXd v_star = VectorXd::LinSpaced(nv, 1, nv);
  SystemDynamicsData<double> data(&Aop, nullptr, &v_star);
  EXPECT_EQ(data.num_velocities(), nv);
  EXPECT_FALSE(data.has_forward_dynamics());
  EXPECT_TRUE(data.has_inverse_dynamics());
  EXPECT_EQ(&data.get_A(), &Aop);
  EXPECT_EQ(&data.get_v_star(), &v_star);
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
