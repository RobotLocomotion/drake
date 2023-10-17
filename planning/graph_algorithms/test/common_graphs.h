#include <Eigen/Sparse>

namespace drake {
namespace planning {
namespace graph_algorithms {

// Complete graph on n vertices
Eigen::SparseMatrix<bool> Kn(const int n);

// Construct the bull graph
// 0         4
//  \       /
//   1  -  3
//    \   /
//      2
Eigen::SparseMatrix<bool> BullGraph();

// Construct the butterfly graph
// 0     3
// | \ / |
//    2  |
// | / \ |
// 1     4
Eigen::SparseMatrix<bool> ButterflyGraph();

// Construct the PetersonGraph https://en.wikipedia.org/wiki/Petersen_graph
Eigen::SparseMatrix<bool> PetersonGraph();
}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake