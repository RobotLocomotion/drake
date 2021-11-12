#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/random.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
/**
 * Performs linear transformation on the random signal w_in as
 * w_out = A*w_in + b.
 * The user can obtain the probability density of w_out. When the class
 * is instantiated with autodiff scalar, the user can also obtain the gradient
 * of the probability density of w_out.
 *
 * @system
 * name: LinearTransformDensity
 * input_ports:
 * - w_in
 * - A
 * - b
 * output_ports:
 * - w_out
 * - w_out_density
 * @endsystem
 *
 * The `b` port can remain disconnected, in which case it defaults to zero.
 *
 * `A` should be a matrix using a column-major order.
 *
 * The user should make sure that the input port `w_in` is connected from the
 * output port of a RandomSource with the same distribution. A recommended way
 * is to use `AddRandomInputs()`.
 *
 * @warning The code cannot verify that the distribution type of w_in matches
 * between where w_in comes from and the w_in input port of this sytem. This
 * class will quitely produce incorrect behavior if the distribution types don't
 * match.
 *
 * @see @ref stochastic_systems
 *
 * @ingroup primitive_systems
 *
 * @tparam_nonsymbolic_scalar
 */
template <typename T>
class LinearTransformDensity final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearTransformDensity)

  /**
   * @param distribution The random input w_in should satisfy this distribution.
   * @param input_size The dimension of the input w_in.
   * @param output_size The dimension of the output w_out.
   * @note The matrix A will have `output_size` columns and `input_size` rows.
   * The vector b will have `output_size` columns.
   */
  LinearTransformDensity(RandomDistribution distribution, int input_size,
                         int output_size);

  /** Scalar-converting copy constructor. See @ref system_scalar_conversion. */
  template <typename U>
  explicit LinearTransformDensity(const LinearTransformDensity<U>&);

  /** Gets the input port for w_in. */
  const InputPort<T>& get_input_port_w_in() const {
    return this->get_input_port(w_in_port_id_);
  }

  /** Gets the input port for A. */
  const InputPort<T>& get_input_port_A() const {
    return this->get_input_port(A_port_id_);
  }

  /** Gets the input port for b. */
  const InputPort<T>& get_input_port_b() const {
    return this->get_input_port(b_port_id_);
  }

  const OutputPort<T>& get_output_port_w_out() const {
    return this->get_output_port(w_out_port_id_);
  }

  const OutputPort<T>& get_output_port_w_out_density() const {
    return this->get_output_port(w_out_density_port_id_);
  }

  /** Gets the random distribution type. */
  RandomDistribution get_distribution() const { return distribution_; }

  /**
   * Fix the input port `A` to a constant value in a given context.
   * @param context The context into which A's value is set.
   * @param A The value to which the port is fixed. The matrix A has num_output
   * rows and num_input columns, note that A is column-majored.
   */
  FixedInputPortValue& FixConstantA(
      Context<T>* context, const Eigen::Ref<const MatrixX<T>>& A) const;

  /**
   * Fix the input port `b` to a constant value in a given context.
   * @param context The context into which b's value is set.
   * @param b The value to which the port is fixed. The vector b has num_output
   * rows.
   */
  FixedInputPortValue& FixConstantB(
      Context<T>* context, const Eigen::Ref<const VectorX<T>>& b) const;

  /**
   * Compute the density (pdf) of a sampled output w_out.
   *
   * When T=AutoDiffXd, this function computes the gradient of the function
   * density(w_out_sample). Namely given an output sample, we want to know
   * how the probability of drawing this sample would change, when the
   * parameters of the distribution (like A and b) change. Such information is
   * locally expressed in the gradient. Note this is different from computing
   * the density of the input.
   *
   * @throw std::exception if A is not an invertible matrix.
   */
  T CalcDensity(const Context<T>& context) const;

 private:
  void CalcOutput(const Context<T>& context, BasicVector<T>* w_out) const;

  void CalcOutputDensity(const Context<T>& context,
                         BasicVector<T>* w_out_density) const;

  Eigen::Map<const MatrixX<T>> GetA(const Context<T>& context) const;

  const RandomDistribution distribution_;
  const int input_size_{};
  const int output_size_{};
  InputPortIndex w_in_port_id_;
  InputPortIndex A_port_id_;
  InputPortIndex b_port_id_;
  OutputPortIndex w_out_port_id_;
  OutputPortIndex w_out_density_port_id_;
};

// Exclude symbolic::Expression from the scalartype conversion of
// LinearTransformDensity
namespace scalar_conversion {
template <>
struct Traits<LinearTransformDensity> : public NonSymbolicTraits {};
}  // namespace scalar_conversion
}  // namespace systems
}  // namespace drake
