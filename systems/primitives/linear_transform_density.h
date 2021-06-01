#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/random.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
/**
 * Perform linear transformation on the random signal w_in as
 * w_out = A*w_in + b.
 *
 * @system
 * name: LinearTransformDensity
 * input_ports:
 * - random signal w_in
 * - matrix A
 * - vector b
 * output_ports:
 * - w_out
 * @endsystem
 *
 * The `A` port can remain disconnected, in which case it defaults to identity.
 * The `b` port can remain disconnected, in which case it defaults to zero.
 *
 * @see @ref stochastic_systems
 *
 * @ingroup primitive_systems
 */
template <typename T>
class LinearTransformDensity final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearTransformDensity)

  /**
   * @param distribution The random input w_in should satisfy this distribution.
   * @param input_size The dimension of the input w_in.
   * @param output_size The dimension of the output w_out
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

  /** Gets the input port for B. */
  const InputPort<T>& get_input_port_b() const {
    return this->get_input_port(b_port_id_);
  }

  RandomDistribution get_distribution() const { return distribution_; }

 private:
  void CalcOutput(const Context<T>& context, BasicVector<T>* w_out) const;

  const RandomDistribution distribution_;
  const int input_size_{};
  const int output_size_{};
  InputPortIndex w_in_port_id_;
  InputPortIndex A_port_id_;
  InputPortIndex b_port_id_;
};

// Exclude symbolic::Expression from the scalartype conversion of
// LinearTransformDensity
namespace scalar_conversion {
template <>
struct Traits<LinearTransformDensity> : public NonSymbolicTraits {};
}  // namespace scalar_conversion
}  // namespace systems
}  // namespace drake
