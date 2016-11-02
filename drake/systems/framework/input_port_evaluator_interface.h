#pragma once

#include <sstream>

#include "drake/common/drake_export.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/system_port_descriptor.h"

namespace drake {
namespace systems {

template <typename T> class Context;

namespace detail {

/// InputPortEvaluatorInterface is implemented by classes that are able to
/// evaluate the OutputPort connected to a particular InputPort.
///
/// This interface is a Drake-internal detail. Users should never implement
/// it. In fact, only Diagram should implement it. It exists primarily to
/// narrow the methods Systems can invoke on their parent pointer, and
/// secondarily to break circular includes between the System hierarchy and
/// the Context hierarchy.
///
/// @tparam T A mathematical type that is a valid Eigen scalar.
template <typename T>
class DRAKE_EXPORT InputPortEvaluatorInterface {
 public:
  virtual ~InputPortEvaluatorInterface() {}

  /// Evaluates the input port with the given @p id in the given @p context.
  /// The subsystem having the input port must be owned by this Diagram.
  /// Aborts if @p context is nullptr and there is any evaluation to do.
  virtual void EvaluateSubsystemInputPort(
      const Context<T>* context, const SystemPortDescriptor<T>& id) const = 0;

  /// Writes the full path of the evaluator in the tree of Systems to @p output.
  /// The path has the form (::ancestor_system_name)*::this_system_name.
  ///
  /// TODO(david-german-tri): Instead return a stringstream, once Drake no
  /// longer supports old compilers that suffer from
  /// https://gcc.gnu.org/bugzilla/show_bug.cgi?id=54316
  virtual void GetPath(std::stringstream* output) const = 0;
};

}  // namespace detail
}  // namespace systems
}  // namespace drake
