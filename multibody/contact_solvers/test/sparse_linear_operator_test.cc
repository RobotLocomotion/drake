#include "drake/multibody/contact_solvers/sparse_linear_operator.h"

#include <memory>

#include <Eigen/SparseCore>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/contact_solvers/block_sparse_matrix.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

using SparseMatrixd = Eigen::SparseMatrix<double>;
using SparseVectord = Eigen::SparseVector<double>;
using Eigen::VectorXd;
using Triplet = Eigen::Triplet<double>;

// This method makes a sparse matrix that emulates the contact Jacobian that we
// would have in an application in which a subset of the vertices of a mesh is
// in contact. More specifically, if nc is the number of vertices in contact,
// vc ∈ ℝ³ˣⁿᶜ is the vector that concatenates the 3D contact velocities of all
// nc contact points and, v is the vector that concatenates the 3D velocities of
// all nv vertices in the mesh, then the contact Jacobian Jc is defined such
// that vc = Jc⋅v. Jc is of size 3nc x 3nv.
//
// N.B. Though inspired in a real application, this is only meant for test
// purposes while in a real application with meshes the computation of this
// Jacobian is more complex. Our goal is to generate a sparse matrix so that we
// can attach a LinearOperator interface to it for testing purposes.
//
// num_vertices:
//   The number of vertices nv in a hypothetical mesh. The number of
//   generalized velocities in this case equals 3 * num_vertices.
// vertices_in_contact:
//   set of the vertices that are in "contact". The size of vertices_in_contact
//   is nc, the number of contact points.
//
// For each contact point ic, its velocity simply equals the velocity for that
// vertex iv. Therefore the Jacobian contains an identity matrix at 3x3 block
// (ic, iv). All other entries are zero. Thus the Jacobian is very sparse.
SparseMatrixd MakeMeshInContactJacobian(
    int num_vertices, const std::vector<int>& vertices_in_contact) {
  const int num_contacts = static_cast<int>(vertices_in_contact.size());
  const int nnz = 3 * num_contacts;
  std::vector<Triplet> triplets;
  triplets.reserve(nnz);
  for (int ic = 0; ic < num_contacts; ++ic) {
    const int v = vertices_in_contact[ic];
    DRAKE_DEMAND(v < num_vertices);
    triplets.emplace_back(3 * ic, 3 * v, 1.0);
    triplets.emplace_back(3 * ic + 1, 3 * v + 1, 1.0);
    triplets.emplace_back(3 * ic + 2, 3 * v + 2, 1.0);
  }
  SparseMatrixd J(3 * num_contacts, 3 * num_vertices);
  J.setFromTriplets(triplets.begin(), triplets.end());
  return J;
}

class ContactJacobianTest : public ::testing::Test {
 protected:
  void SetUp() override {
    J_ = MakeMeshInContactJacobian(kNumVertices_, contact_set_);
    Jop_ = std::make_unique<SparseLinearOperator<double>>("Jc", &J_);
    num_rows_ = 3 * static_cast<int>(contact_set_.size());
  }

  const int kNumVertices_ = 300;
  const std::vector<int> contact_set_{12, 3, 75, 100, 99, 233, 7};
  int num_rows_{};
  int kCols_{3 * kNumVertices_};
  SparseMatrixd J_;
  std::unique_ptr<SparseLinearOperator<double>> Jop_;
};

TEST_F(ContactJacobianTest, Construction) {
  EXPECT_EQ(Jop_->name(), "Jc");
  EXPECT_EQ(Jop_->rows(), num_rows_);
  EXPECT_EQ(Jop_->cols(), kCols_);
}

TEST_F(ContactJacobianTest, MultiplyDense) {
  VectorXd y(num_rows_);
  const VectorXd x = VectorXd::LinSpaced(kCols_, 0.0, 1.0);
  Jop_->Multiply(x, &y);

  VectorXd y_expected = J_ * x;
  // y's values should equal those in y_expected bit by bit.
  EXPECT_EQ(y, y_expected);
}

TEST_F(ContactJacobianTest, MultiplyByTransposeDense) {
  VectorXd y(kCols_);
  const VectorXd x = VectorXd::LinSpaced(num_rows_, 0.0, 1.0);
  Jop_->MultiplyByTranspose(x, &y);

  VectorXd y_expected = J_.transpose() * x;
  // y's values should equal those in y_expected bit by bit.
  EXPECT_EQ(y, y_expected);
}

TEST_F(ContactJacobianTest, MultiplySparse) {
  SparseVectord y(num_rows_);
  const SparseVectord x = VectorXd::LinSpaced(kCols_, 0.0, 1.0).sparseView();
  Jop_->Multiply(x, &y);

  VectorXd y_expected = J_ * x;
  // y's values should equal those in y_expected bit by bit.
  EXPECT_EQ(VectorXd(y), y_expected);
}

TEST_F(ContactJacobianTest, MultiplyByTransposeSparse) {
  SparseVectord y(kCols_);
  const SparseVectord x = VectorXd::LinSpaced(num_rows_, 0.0, 1.0).sparseView();
  Jop_->MultiplyByTranspose(x, &y);

  VectorXd y_expected = J_.transpose() * x;
  // y's values should equal those in y_expected bit by bit.
  EXPECT_EQ(VectorXd(y), y_expected);
}

TEST_F(ContactJacobianTest, AssembleMatrix) {
  SparseMatrixd Jcopy(Jop_->rows(), Jop_->cols());
  Jop_->AssembleMatrix(&Jcopy);

  // Required before we access their data pointers.
  Jcopy.makeCompressed();
  J_.makeCompressed();

  // We verify the Jcopy is an exact bit by bit copy of J_.
  // Eigen does not offer SparseMatrix::operator==() and therefore we compare
  // the results by explicitly comparing the individual components of the CCS
  // format.
  Eigen::Map<VectorX<double>> Jcopy_values(Jcopy.valuePtr(), Jcopy.nonZeros());
  Eigen::Map<VectorX<double>> J_values(J_.valuePtr(), J_.nonZeros());
  EXPECT_EQ(Jcopy_values, J_values);

  Eigen::Map<VectorX<int>> Jcopy_inner(Jcopy.innerIndexPtr(),
                                       Jcopy.innerSize());
  Eigen::Map<VectorX<int>> J_inner(J_.innerIndexPtr(), J_.innerSize());
  EXPECT_EQ(Jcopy_inner, J_inner);

  Eigen::Map<VectorX<int>> Jcopy_outer(Jcopy.outerIndexPtr(),
                                       Jcopy.outerSize());
  Eigen::Map<VectorX<int>> J_outer(J_.outerIndexPtr(), J_.outerSize());
  EXPECT_EQ(Jcopy_outer, J_outer);
}

// Thus far SparseLinearOperator does not implement assembly into a
// BlockSparseMatrix. We expect this method to throw.
GTEST_TEST(SparseLinearOperator, AssembleMatrixBlockSparseThrows) {
  SparseMatrixd Asparse;
  const SparseLinearOperator<double> Aop("A", &Asparse);
  BlockSparseMatrix<double> Ablock;
  DRAKE_EXPECT_THROWS_MESSAGE(
      Aop.AssembleMatrix(&Ablock), std::runtime_error,
      "DoAssembleMatrix().*must provide an implementation.");
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
