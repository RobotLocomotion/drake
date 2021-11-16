#include <numeric>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>
#include <benchmark/benchmark.h>
// The PETSc module that contains Krylov space solvers along with a variety of
// preconditioners. The only module (along with the modules it depends on) that
// we will be using in the foreseeable future.
#include <petscksp.h>

using Eigen::MatrixXd;
using Eigen::Vector3i;
using Eigen::Vector4i;
using Eigen::VectorXd;
using std::unordered_set;
using std::vector;

int CalcSequentialIndex(int i, int j, int k, const Vector3i& num_vertices) {
  return i * num_vertices.y() * num_vertices.z() + j * num_vertices.z() + k;
}

std::vector<Vector4i> GenerateDiamondCubicElements(
    const Vector3i& num_vertices) {
  vector<Vector4i> elements;
  const Vector3i num_cells = num_vertices - Vector3i(1, 1, 1);
  for (int i = 0; i < num_cells(0); ++i) {
    for (int j = 0; j < num_cells(1); ++j) {
      for (int k = 0; k < num_cells(2); ++k) {
        int v[8];
        int s = 0;
        for (int l = 0; l < 2; ++l) {
          for (int m = 0; m < 2; ++m) {
            for (int n = 0; n < 2; ++n) {
              v[s++] = CalcSequentialIndex(i + l, j + m, k + n, num_vertices);
            }
          }
        }
        if ((i + j + k) % 2 == 1) {
          elements.emplace_back(v[6], v[4], v[7], v[2]);
          elements.emplace_back(v[7], v[2], v[1], v[3]);
          elements.emplace_back(v[1], v[4], v[7], v[5]);
          elements.emplace_back(v[2], v[4], v[1], v[0]);
          elements.emplace_back(v[7], v[4], v[1], v[2]);
        } else {
          elements.emplace_back(v[0], v[6], v[5], v[4]);
          elements.emplace_back(v[3], v[6], v[0], v[2]);
          elements.emplace_back(v[5], v[6], v[3], v[7]);
          elements.emplace_back(v[3], v[0], v[5], v[1]);
          elements.emplace_back(v[0], v[6], v[3], v[5]);
        }
      }
    }
  }
  return elements;
}

// Make an arbitrary SPD element matrix sized 12x12.
Eigen::Matrix<double, 12, 12> dummy_matrix12x12() {
  Eigen::Matrix<double, 12, 12> A;
  for (int i = 0; i < 12; ++i) {
    for (int j = 0; j < 12; ++j) {
      A(i, j) = 3.14 * i + 2.7 * j;
    }
  }
  Eigen::Matrix<double, 12, 12> I = Eigen::Matrix<double, 12, 12>::Identity();
  return A * A.transpose() + I;
}

// Make an arbitrary vector of size 3*num_nodes.
Eigen::VectorXd MakeVector(int num_nodes) {
  Eigen::VectorXd x(3 * num_nodes);
  for (int i = 0; i < 3 * num_nodes; ++i) {
    x(i) = 3.14 * std::sin(i - 0.512);
  }
  return x;
}

// Initialize the Petsc vector b to be of size 3*num_nodes and populate it with
// arbitrary values.
void AssemblePetscRhs(int num_nodes, Vec* b) {
  std::vector<int> vector_indexes(3 * num_nodes);
  std::iota(vector_indexes.begin(), vector_indexes.end(), 0);
  Eigen::VectorXd b_ = MakeVector(num_nodes);
  VecCreate(PETSC_COMM_WORLD, b);
  VecSetSizes(*b, PETSC_DECIDE, 3 * num_nodes);
  VecSetFromOptions(*b);
  VecSetValues(*b, 3 * num_nodes, vector_indexes.data(), b_.data(),
               INSERT_VALUES);
  VecAssemblyBegin(*b);
  VecAssemblyEnd(*b);
}

// Initialize the Petsc vector x to be of size 3*num_nodes.
void InitializeSolution(int num_nodes, Vec* x) {
  VecCreate(PETSC_COMM_WORLD, x);
  VecSetSizes(*x, PETSC_DECIDE, 3 * num_nodes);
  VecSetFromOptions(*x);
}

class PetscFixture : public benchmark::Fixture {
 public:
  using benchmark::Fixture::SetUp;

  PetscFixture() = default;

  void SetUp(benchmark::State&) override {
    PetscInitialize(PETSC_NULL, PETSC_NULL, PETSC_NULL, PETSC_NULL);
    const int nnps = 6; /* Number of nodes per side. */
    num_nodes_ = nnps * nnps * nnps;
    elements_ = GenerateDiamondCubicElements(Vector3i(nnps, nnps, nnps));
    BuildSparsityPattern();

    /* Create rhs b. */
    AssemblePetscRhs(num_nodes_, &b_);
    /* Create x. */
    InitializeSolution(num_nodes_, &x_);

    SetUpMatrix();
    SetUpBlockMatrix();
    SetUpSolver();
  }

  // Build sparsity pattern from connectivity info from `elements_` and store
  // the sparsity pattern in nnz_ such that nnz_[i] containts the number of
  // nonzero entries in row i.
  void BuildSparsityPattern() {
    vector<unordered_set<int>> neighbors(num_nodes_);
    for (const auto& e : elements_) {
      for (int i = 0; i < 4; ++i) {
        const int vi = e(i);
        for (int j = 0; j < 4; ++j) {
          const int vj = e(j);
          neighbors[vi].insert(vj);
        }
      }
    }
    /* Find number of nonzero entries in each row. */
    nnz_.resize(3 * num_nodes_);
    for (int i = 0; i < num_nodes_; ++i) {
      for (int d = 0; d < 3; ++d) {
        nnz_[3 * i + d] = 3 * neighbors[i].size();
      }
    }
  }

  void SetUpBlockMatrix() {
    MatCreateSeqSBAIJ(PETSC_COMM_SELF, 3, 3 * num_nodes_, 3 * num_nodes_, 0,
                      nnz_.data(), &A_block_);
  }

  void SetUpMatrix() {
    MatCreateSeqAIJ(PETSC_COMM_SELF, 3 * num_nodes_, 3 * num_nodes_, 0,
                    nnz_.data(), &A_);
  }

  void SetUpSolver() {
    /* Linear solver context. */
    KSPCreate(PETSC_COMM_WORLD, &ksp_);
    /* Get pc context from ksp context. */
    KSPGetPC(ksp_, &pc_);
    /* Set preconditioner type. */
    PCSetType(pc_, PCCHOLESKY);
    /* Use direct solver. */
    KSPSetType(ksp_, KSPPREONLY);
    KSPSetFromOptions(ksp_);
  }

  // Assemble Petsc matrix A_ with arbitrary SPD element matrices.
  void AssembleMatrix() {
    MatZeroEntries(A_);
    vector<int> indexes(12);
    for (const Vector4i& element : elements_) {
      const MatrixXd element_matrix = dummy_matrix12x12();
      for (int v = 0; v < 4; ++v) {
        for (int d = 0; d < 3; ++d) {
          indexes[3 * v + d] = 3 * element(v) + d;
        }
      }
      MatSetValues(A_, 12, indexes.data(), 12, indexes.data(),
                   element_matrix.data(), ADD_VALUES);
    }
    MatAssemblyBegin(A_, MAT_FINAL_ASSEMBLY);
    MatAssemblyEnd(A_, MAT_FINAL_ASSEMBLY);
  }

  // Assemble Petsc matrix A_block_ with arbitrary SPD element matrices.
  void AssembleBlockMatrix() {
    MatZeroEntries(A_block_);
    for (const Vector4i& element : elements_) {
      const MatrixXd element_matrix = dummy_matrix12x12();
      MatSetValuesBlocked(A_block_, 4, element.data(), 4, element.data(),
                          element_matrix.data(), ADD_VALUES);
    }
    MatAssemblyBegin(A_block_, MAT_FINAL_ASSEMBLY);
    MatAssemblyEnd(A_block_, MAT_FINAL_ASSEMBLY);
  }

  // Solve A_ * x_ = b_.
  void Solve() {
    /* Set matrix. */
    KSPSetOperators(ksp_, A_, A_);
    /* Solve! */
    KSPSolve(ksp_, b_, x_);
  }

  // Solve A_block_ * x_ = b_.
  void BlockSolve() {
    /* Set matrix. */
    KSPSetOperators(ksp_, A_block_, A_block_);
    /* Solve! */
    KSPSolve(ksp_, b_, x_);
  }

 protected:
  Vec x_, b_;   /* solution, rhs */
  Mat A_;       /* sparse matrix */
  Mat A_block_; /* block sparse matrix */
  KSP ksp_;     /* linear solver context */
  PC pc_;       /* preconditioner context */
  int num_nodes_{0};
  vector<Vector4i> elements_;
  vector<int> nnz_; /* sparsity pattern: nnz_[i] gives the number of nonzero
                       entries in the i-th row. */
};

BENCHMARK_F(PetscFixture, AssembleAndSolve)(benchmark::State& state) {
  for (auto _ : state) {
    AssembleMatrix();
    Solve();
  }
}

BENCHMARK_F(PetscFixture, BlockAssembleAndSolve)(benchmark::State& state) {
  for (auto _ : state) {
    AssembleBlockMatrix();
    BlockSolve();
  }
}

BENCHMARK_MAIN();
