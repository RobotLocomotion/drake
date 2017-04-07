#pragma once

#include <cstdint>
#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/output_port_value.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

/// An adder for arbitrarily many inputs of equal size.
/// @ingroup primitive_systems
/// @tparam T The type of mathematical object being added.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
template <typename T>
class Adder : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Adder)

  /// Construct an %Adder System.
  /// @param num_inputs is the number of input ports to be added.
  /// @param size number of elements in each input and output signal.
  Adder(int num_inputs, int size);

  /// Returns the output port on which the sum is presented.
  const OutputPort<T>& get_output_port() const {
    DRAKE_ASSERT(output_port_ != nullptr);
    return *output_port_;
  }

  /// Returns an Adder<AutoDiffXd> with the same dimensions as this Adder.
  std::unique_ptr<Adder<AutoDiffXd>> ToAutoDiffXd() const {
    return std::unique_ptr<Adder<AutoDiffXd>>(DoToAutoDiffXd());
  }

 private:
  // Sums the input ports into a value suitable for the output port. If the
  // input ports are not the appropriate count or size, std::runtime_error will
  // be thrown.
  void CalcSum(const Context<T>& context, BasicVector<T>* sum) const;

  // System<T> override. Returns an Adder<AutoDiffXd> with the same dimensions
  // as this Adder.
  Adder<AutoDiffXd>* DoToAutoDiffXd() const override;

  // System<T> override. Returns an Adder<symbolic::Expression> with the same
  // dimensions as this Adder.
  Adder<symbolic::Expression>* DoToSymbolic() const override;

  // This is set in the constructor and can't be nullptr.
  const OutputPort<T>* output_port_;
};

}  // namespace systems
}  // namespace drake
