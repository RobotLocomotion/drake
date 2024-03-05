#include "drake/planning/graph_algorithms/max_clique_solver_via_greedy.h"
#include "drake/common/fmt_eigen.h"

#include <algorithm>
// #include <iostream>
#include <vector>
#include <numeric>

namespace drake {
namespace planning {
namespace graph_algorithms {
using Eigen::SparseMatrix;

namespace{

Eigen::VectorXi compute_degree_of_vertices(const Eigen::SparseMatrix<bool>& adjacency_matrix) {
    const int num_vertices = adjacency_matrix.rows();
    // Initialize a vector to store the degree of each vertex
    Eigen::VectorXi degrees(num_vertices);
    // Compute the degree of each vertex
    for (int j = 0; j < num_vertices; ++j) {
        degrees(j) = 0;
        for (SparseMatrix<bool>::InnerIterator it(adjacency_matrix, j); it; ++it) {
            degrees(j) +=1;
        }
    }
    return degrees;
}

int pick_best(const std::vector<int>& degrees_sort_idx, std::vector<int>& available_nodes) {
    for (int d : degrees_sort_idx) {
        auto it = std::find(available_nodes.begin(), available_nodes.end(), d);
        if (it != available_nodes.end()) {
            return d;
        }
    }
    throw std::runtime_error("No valid element found in available_nodes.");
}

void update_available_nodes(const Eigen::SparseMatrix<bool>& mat, int point_to_add, std::vector<int>& available_nodes) {
    auto it = std::remove_if(available_nodes.begin(), available_nodes.end(),
                             [&](int a) { 
                                bool value = !mat.coeff(a, point_to_add) || (a == point_to_add);
                                // std::cout<<fmt::format("a {} val {}", a, value)<<std::endl;
                                return value;  });
    
    // Print the elements intended to be erased
    available_nodes.erase(it, available_nodes.end());
}

// Sets mat(i,:) and mat(:,i) to false if i is not a neighbour of index  
void mask_non_neighbours(Eigen::SparseMatrix<bool>& mat, int index) {
    for (int j = 0; j < mat.outerSize(); ++j) {
        if (!(mat.coeff(index, j))&& index!=j) {
            for (Eigen::SparseMatrix<bool>::InnerIterator it(mat, j); it; ++it) {
                int row = it.row();
                int col = it.col();
                mat.coeffRef(row, col) = false;
                mat.coeffRef(col, row) = false;
            }
        }
    }
    mat = mat.pruned();
}

std::vector<int> decreasing_argsort(const Eigen::Ref<Eigen::VectorXi>& values) {
  std::vector<int> index(values.size());
  std::iota(index.begin(), index.end(), 0);
  std::sort(index.begin(), index.end(), [&values](uint8_t a, uint8_t b) {
    return values(a) > values(b);
  });
  return index;
}

}//namespace

MaxCliqueSolverViaGreedy::MaxCliqueSolverViaGreedy(){};

VectorX<bool> MaxCliqueSolverViaGreedy::DoSolveMaxClique(
    const SparseMatrix<bool>& adjacency_matrix) const {

  const int n = adjacency_matrix.rows();
  std::vector<int> available_nodes(n);
  for(int i = 0; i<n; ++i){
    available_nodes.emplace_back(i);
  }

  std::vector<int> clique_members;
  SparseMatrix<bool> curr_ad_matrix = adjacency_matrix;
 
  while(true){
    Eigen::VectorXi degrees = compute_degree_of_vertices(curr_ad_matrix);
    // std::cout<<fmt::format("{}", fmt_eigen(degrees))<<std::endl;
    std::vector<int> degrees_sort_idx = decreasing_argsort(degrees);
    int point_to_add = pick_best(degrees_sort_idx, available_nodes);
    // std::cout<<fmt::format("point to add {}", point_to_add)<<std::endl;
    
    clique_members.push_back(point_to_add);
    update_available_nodes(curr_ad_matrix, point_to_add, available_nodes);
    // std::cout << "Available Nodes: ";
    // for (const auto& element : available_nodes) {
    //     std::cout << element << " ";
    // }
    // std::cout << std::endl;

    // std::cout<<fmt::format("Available nodes {}", available_nodes.size())<<std::endl;
    if(!available_nodes.size()){
        break;
    }
    mask_non_neighbours(curr_ad_matrix, point_to_add);

  }

  //fill in solution
  Eigen::VectorX<bool> is_clique_member = Eigen::VectorX<bool>::Constant(n, false); 
  for(auto i: clique_members){
    is_clique_member(i) = true;
  }
  return is_clique_member.cast<bool>();
}

}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake

