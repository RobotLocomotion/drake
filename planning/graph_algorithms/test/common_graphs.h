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
// the nodes labelled counter-clockwise. The vertices of the external pentagon
// labeled 5-9, with the top of the pentagon labelled 5 and the nodes labelled
// counter-clockwise.
Eigen::SparseMatrix<bool> PetersenGraph();

// Fully connected graph and fully connected bipartite graph used to test that
// the answers from greedy max clique are different from the true maximum.
// Taken from
// https://cs.stackexchange.com/questions/63784/why-doesnt-greedy-work-for-clique.
//  0-┐  5---6
//  | |    x
//  1 |  4---7
//  | |    x
//  2-┘  3---8
// There are additionally edges between (3,6), (5,8) which are hard to draw in
// ascii art.
Eigen::SparseMatrix<bool> FullyConnectedPlusFullBipartiteGraph();

}  // namespace internal
}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
