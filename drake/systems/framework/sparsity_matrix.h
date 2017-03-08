#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

/// The SparsityMatrix reports the connectivity between fields in a System's
/// Context, and outputs computed by that System.
///
/// TODO(david-german-tri): Extend SparsityMatrix to report input-to-state,
/// state-to-derivatives, state-to-discrete-updates, and state-to-output
/// sparsity.
///
/// A SparsityMatrix is only interesting if the Context contains purely
/// vector-valued elements. If any abstract-valued elements are present, the
/// SparsityMatrix will report no sparsity. Why? In the presence of opaque
/// variables, proving sparsity is equivalent to solving the halting problem.
///
/// It would be possible to report sparsity for a specific configuration of
/// the abstract inputs, state, or parameters. We intentionally do not provide
/// such an analysis, because it would invite developers to shoot themselves
/// in the foot by accidentally overstating sparsity, for instance if a given
/// input affects a given output in some modes, but not the mode tested.
///
/// Even with that limitation on scope, SparsityMatrix has risks. If the System
/// contains C++ native conditionals like "if" or "switch", accurate analysis
/// is once again equivalent to the halting problem. symbolic::Expression does
/// not provide an implicit conversion to `bool`, so it is unlikely that anyone
/// will accidentally write a System that both uses native conditionals and
/// compiles with a symbolic::Expression scalar type. However, it is possible,
/// for instance using an explicit cast, or `std::equal_to`.
class SparsityMatrix {
 public:
  /// Constructs a sparsity matrix for the given @p system by initializing
  /// every vector-valued element in the Context with symbolic variables.
  explicit SparsityMatrix(const System<symbolic::Expression>& system);

  ~SparsityMatrix() = default;

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SparsityMatrix)

  /// Returns true if the input port at the given @p input_port_index is or
  /// might possibly be a term in the output at the given @p output_port_index.
  bool IsConnectedInputToOutput(int input_port_index,
                                int output_port_index) const;

 private:
  // Populates the @p system inputs in the context_ with symbolic variables.
  void InitializeVectorInputs(const System<symbolic::Expression>& system);
  // Populates the continuous state in the context_ with symbolic variables.
  void InitializeContinuousState();
  // Populates the discrete state in the context_ with symbolic variables.
  void InitializeDiscreteState();

  // Returns true if any field in the @p context is abstract-valued.
  static bool IsAbstract(const System<symbolic::Expression>& system,
                         const Context<symbolic::Expression>& context);

  const std::unique_ptr<Context<symbolic::Expression>> context_;
  const std::unique_ptr<SystemOutput<symbolic::Expression>> output_;

  // The symbolic expression attached to each input port in the `context_`.
  std::vector<std::vector<symbolic::Expression>> input_expressions_;

  // The types of the output ports.
  std::vector<PortDataType> output_port_types_;

  // True if the `context_` contains any abstract elements.
  const bool context_is_abstract_{false};
};

}  // namespace systems
}  // namespace drake
