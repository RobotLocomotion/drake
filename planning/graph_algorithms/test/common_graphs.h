#pragma once
#include <Eigen/Sparse>

namespace drake {
namespace planning {
namespace graph_algorithms {
namespace internal {

// Complete graph on n vertices
Eigen::SparseMatrix<bool> MakeCompleteGraph(const int n);

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

// Construct the PetersenGraph https://en.wikipedia.org/wiki/Petersen_graph. The
// vertices of the internal star labelled 0-4, with 0 the top of the star and
// the nodes labelled counter clockwise. The vertices of the external pentagon
// labeled 5-9, with the top of the pentagon labelled 5 and the nodes labelled
// counter-clockwise.
Eigen::SparseMatrix<bool> PetersenGraph();
}  // namespace internal
}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
