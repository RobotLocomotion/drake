#pragma once

#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

/// The SystemSymbolicInspector uses symbolic::Expressions to analyze various
/// properties of the System, such as time invariance and input-to-output
/// sparsity, along with many others.
///
/// A SystemSymbolicInspector is only interesting if the Context contains purely
/// vector-valued elements.  If any abstract-valued elements are present, the
/// SystemSymbolicInspector will not be able to parse the governing equations
/// reliably.
///
/// It would be possible to report system properties for a specific
/// configuration of the abstract inputs, state, or parameters. We intentionally
/// do not provide such an analysis, because it would invite developers to shoot
/// themselves in the foot by accidentally overstating sparsity, for instance if
/// a given input affects a given output in some modes, but not the mode tested.
///
/// Even with that limitation on scope, SystemSymbolicInspector has risks, if
/// the System contains C++ native conditionals like "if" or "switch".
/// symbolic::Expression does not provide an implicit conversion to `bool`, so
/// it is unlikely that anyone will accidentally write a System that both uses
/// native conditionals and compiles with a symbolic::Expression scalar type.
/// However, it is possible, for instance using an explicit cast, or
/// `std::equal_to`.
class SystemSymbolicInspector {
 public:
  /// Constructs a SystemSymbolicInspector for the given @p system by
  /// initializing every vector-valued element in the Context with symbolic
  /// variables.
  explicit SystemSymbolicInspector(const System<symbolic::Expression>& system);

  ~SystemSymbolicInspector() = default;

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SystemSymbolicInspector)

  /// Returns true if the input port at the given @p input_port_index is or
  /// might possibly be a term in the output at the given @p output_port_index.
  bool IsConnectedInputToOutput(int input_port_index,
                                int output_port_index) const;

  /// Returns true if there is no dependence on time in the dynamics (continuous
  /// nor discrete) nor the outputs.
  bool IsTimeInvariant() const;

  /// Returns true iff all of the derivatives and discrete updates have at
  /// most an affine dependence on state and input.  Note that the return value
  /// does NOT depend on the output methods (they may be affine or not).
  bool HasAffineDynamics() const;

  /// Returns true if any field in the @p context is abstract-valued.
  static bool IsAbstract(const System<symbolic::Expression>& system,
                         const Context<symbolic::Expression>& context);

  /// @name Reference symbolic components
  /// A set of accessor methods that provide direct access to the symbolic
  /// forms of the System.  This class carefully sets up and names all
  /// of the symbolic elements of the Context, and other methods should
  /// be able to reap the benefits.
  /// @{
  /// Returns a reference to the symbolic representation of time.
  const symbolic::Variable& time() const { return time_; }

  /// Returns a reference to the symbolic representation of the input.
  /// @param i The input port number.
  const VectorX<symbolic::Variable>& input(int i) const {
    DRAKE_DEMAND(i >= 0 && i < static_cast<int>(input_variables_.size()));
    return input_variables_[i];
  }

  /// Returns a reference to the symbolic representation of the continuous
  /// state.
  const VectorX<symbolic::Variable>& continuous_state() const {
    return continuous_state_variables_;
  }

  /// Returns a reference to the symbolic representation of the discrete state.
  /// @param i The discrete state group number.
  const VectorX<symbolic::Variable>& discrete_state(int i) const {
    DRAKE_DEMAND(i >= 0 &&
                 i < static_cast<int>(discrete_state_variables_.size()));
    return discrete_state_variables_[i];
  }

  /// Returns a reference to the symbolic representation of the numeric
  /// parameters.
  /// @param i The numeric parameter group number.
  const VectorX<symbolic::Variable>& numeric_parameters(int i) const {
    DRAKE_DEMAND(i >= 0 && i < static_cast<int>(numeric_parameters_.size()));
    return numeric_parameters_[i];
  }

  /// Returns a copy of the symbolic representation of the continuous-time
  /// dynamics.
  VectorX<symbolic::Expression> derivatives() const {
    return derivatives_->CopyToVector();
  }

  /// Returns a reference to the symbolic representation of the discrete-time
  /// dynamics.
  /// @param i The discrete state group number.
  const VectorX<symbolic::Expression>& discrete_update(int i) const {
    DRAKE_DEMAND(i >= 0 && i < context_->num_discrete_state_groups());
    return discrete_updates_->value(i);
  }

  /// Returns a reference to the symbolic representation of the output.
  /// @param i The output port number.
  const VectorX<symbolic::Expression>& output(int i) const {
    DRAKE_DEMAND(output_port_types_[i] == kVectorValued);
    return output_->get_vector_data(i)->value();
  }

  /// Returns a reference to the symbolic representation of the constraints.
  const std::set<symbolic::Formula>& constraints() const {
    return constraints_;
  }
  /// @}

 private:
  // Populates the @p system inputs in the context_ with symbolic variables.
  void InitializeVectorInputs(const System<symbolic::Expression>& system);
  // Populates the continuous state in the context_ with symbolic variables.
  void InitializeContinuousState();
  // Populates the discrete state in the context_ with symbolic variables.
  void InitializeDiscreteState();
  // Populates the parameters in the context_ with symbolic variables.
  void InitializeParameters();

  const std::unique_ptr<Context<symbolic::Expression>> context_;
  // Rather than maintain a Context of symbolic::Variables (which are not a
  // proper Eigen scalar type, since they are not closed under the basic vector
  // operations), we maintain member variables for the supported elements of the
  // Context here.  Internal methods must keep these in sync with context_.
  symbolic::Variable time_;
  std::vector<VectorX<symbolic::Variable>> input_variables_;
  VectorX<symbolic::Variable> continuous_state_variables_;
  std::vector<VectorX<symbolic::Variable>> discrete_state_variables_;
  std::vector<VectorX<symbolic::Variable>> numeric_parameters_;

  // Maintains symbolic representations of the primary system methods.
  const std::unique_ptr<SystemOutput<symbolic::Expression>> output_;
  const std::unique_ptr<ContinuousState<symbolic::Expression>> derivatives_;
  const std::unique_ptr<DiscreteValues<symbolic::Expression>> discrete_updates_;
  std::set<symbolic::Formula> constraints_;

  // The types of the output ports.
  std::vector<PortDataType> output_port_types_;

  // True if the `context_` contains any abstract elements.
  const bool context_is_abstract_{false};
};

}  // namespace systems
}  // namespace drake
