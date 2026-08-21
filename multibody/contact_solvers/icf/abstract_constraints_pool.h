#pragma once

#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"
#include "drake/multibody/contact_solvers/icf/icf_data.h"
#include "drake/multibody/contact_solvers/icf/reduced_mapping.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

// Forward declaration to break circular dependencies.
template <typename T>
class IcfModel;

/* Provides documentation and type signature checking for common methods of
constraint pools.

For convenience, only the `double` scalar type is used here. Typically scalar
type compatibility will be defined and tested elsewhere.

Clients must invoke the concept on the conforming class after its definition is
complete:

@code
template <typename T>
class MyConstraintsPool {
// ... stuff ...
};
static_assert(IsAbstractConstraintsPool<MyConstraintsPool>);
@endcode
*/
template <template <typename> typename Pool>
concept IsAbstractConstraintsPool = requires(
    Pool<double> pool, const IcfData<double>& data, VectorX<double>* gradient,
    std::span<const int> constraints, std::span<const int> clique_to_block,
    int island,
    contact_solvers::internal::BlockSparseSymmetricMatrix<MatrixX<double>>*
        hessian,
    const ReducedMapping& mapping, Pool<double>* reduced_pool) {
  /* Returns a reference to the parent model. */
  { pool.model() } -> std::same_as<const IcfModel<double>&>;

  /* Returns the total number of constraints stored in this pool. */
  { pool.num_constraints() } -> std::same_as<int>;

  /* Adds the gradient contribution of this constraint, ∇ℓ(v) = −γ, to the
  model-wide gradient. */
  pool.AccumulateGradient(data, gradient);

  /* Island-filtered overload: gradient for only the listed constraints. */
  pool.AccumulateGradient(data, constraints, gradient);

  /* Adds the contribution of this constraint to the model-wide Hessian. */
  pool.AccumulateHessian(data, hessian);

  /* Island-filtered overload: accumulates the Hessian for only the listed
  constraints into the island's local sub-Hessian. `clique_to_block` maps a
  global clique index to its block index in the island's sub-Hessian, and
  `island` selects the per-island scratch space. */
  pool.AccumulateHessian(data, constraints, clique_to_block, island, hessian);

  /* Makes a "reduced" constraints pool in `reduced_pool`, guided by the
  `mapping`.

  The mapping indicates that certain DOFs are locked; that is, their prescribed
  velocity is 0. The semantics of the reduced pool will be equivalent to that
  of a full pool, iff the full input velocity (not used here, see IcfData) is 0
  in all the DOFs that would have been locked.

  @param mapping the mapping computed by `this->model().ReduceInto()`, or
                 equivalent.

  @param[in,out] reduced_pool the pool to write the reduced constraints
                 into. The result of `reduced_pool.model()` is assumed to be
                 valid and remains unchanged; all the rest of the object is
                 rewritten.

  @pre reduced_pool != nullptr.  */
  pool.ReduceInto(mapping, reduced_pool);
};

template <template <typename> typename Pool,
          template <typename> typename DataPool>
concept HasJointVelocityCalcData = requires(  // BR
    Pool<double> pool, const VectorX<double>& v, DataPool<double>* data_pool,
    std::span<const int> constraints) {
  /* Computes problem data as a function of the generalized velocities `v`
  for the full plant. */
  pool.CalcData(v, data_pool);

  /* Island-filtered overload: computes data only for the listed constraints
  and returns the sum of their costs (does not write the pool-wide cost
  scalar). */
  { pool.CalcData(v, constraints, data_pool) } -> std::same_as<double>;
};

template <template <typename> typename Pool,
          template <typename> typename DataPool>
concept HasSpatialVelocityCalcData = requires(  // BR
    Pool<double> pool, const EigenPool<Vector6<double>>& V_WB,
    DataPool<double>* data_pool, std::span<const int> constraints) {
  /* Computes problem data as a function of the body spatial velocities V_WB
  for the full IcfModel. */
  pool.CalcData(V_WB, data_pool);

  /* Island-filtered overload: computes data only for the listed constraints
  and returns the sum of their costs (does not write the pool-wide cost
  scalar). */
  { pool.CalcData(V_WB, constraints, data_pool) } -> std::same_as<double>;
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
