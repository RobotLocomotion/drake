#include <Eigen/Sparse>


namespace drake {
namespace planning {
using Eigen::SparseMatrix;
using Eigen::Triplet;
using Eigen::Dynamic;

namespace {
Eigen::SparseMatrix<bool> SymmetrizeSparseBooleanMatrix(const Eigen::Ref<Eigen::SparseMatrix<bool>>& mat) {
  return mat + Eigen::SparseMatrix<bool>(mat.transpose());
}


}

Eigen::SparseMatrix<bool> Kn(const int n) {
  return Eigen::MatrixXd::Ones(n,n).cast<bool>().sparseView();
}

Eigen::SparseMatrix<bool> BullGraph() {
  std::vector<Triplet<bool>> triplets;

  triplets.push_back(Triplet<bool>(0,1,true));
  triplets.push_back(Triplet<bool>(1,2,true));
  triplets.push_back(Triplet<bool>(1,3,true));
  triplets.push_back(Triplet<bool>(2,3,true));
  triplets.push_back(Triplet<bool>(3,4,true));

  Eigen::SparseMatrix<bool> upper_part(5,5);
  upper_part.setFromTriplets(triplets.begin(), triplets.end());
  return SymmetrizeSparseBooleanMatrix(upper_part);
};

Eigen::SparseMatrix<bool> ButterflyGraph() {
  std::vector<Triplet<bool>> triplets;
  triplets.push_back(Triplet<bool>(0,1,1));
  triplets.push_back(Triplet<bool>(0,2,1));
  triplets.push_back(Triplet<bool>(1,2,1));
  triplets.push_back(Triplet<bool>(2,3,1));
  triplets.push_back(Triplet<bool>(2,4,1));
  triplets.push_back(Triplet<bool>(3,4,1));

  Eigen::SparseMatrix<bool> upper_part(5,5);
  upper_part.setFromTriplets(triplets.begin(), triplets.end());
  return SymmetrizeSparseBooleanMatrix(upper_part);
}

Eigen::SparseMatrix<bool> PetersonGraph() {
  std::vector<Triplet<bool>> triplets;
  triplets.push_back(Triplet<bool>(0,2,1));
  triplets.push_back(Triplet<bool>(0,3,1));
  triplets.push_back(Triplet<bool>(1,3,1));
  triplets.push_back(Triplet<bool>(1,4,1));
  triplets.push_back(Triplet<bool>(2,4,1));

  triplets.push_back(Triplet<bool>(5,6,1));
  triplets.push_back(Triplet<bool>(6,7,1));
  triplets.push_back(Triplet<bool>(7,8,1));
  triplets.push_back(Triplet<bool>(8,9,1));
  triplets.push_back(Triplet<bool>(5,9,1));

  triplets.push_back(Triplet<bool>(0,5,1));
  triplets.push_back(Triplet<bool>(1,6,1));
  triplets.push_back(Triplet<bool>(2,7,1));
  triplets.push_back(Triplet<bool>(3,8,1));
  triplets.push_back(Triplet<bool>(4,9,1));

  Eigen::SparseMatrix<bool> upper_part(10,10);
  upper_part.setFromTriplets(triplets.begin(), triplets.end());
  return SymmetrizeSparseBooleanMatrix(upper_part);
}

}  // namespace planning
}  // namespace drake

