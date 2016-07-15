#pragma once

#include <utility>

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system_interface.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace systems {

template <typename T>
using PortIdentifier = std::pair<const SystemInterface<T>*, int>;

template <typename T>
class DiagramContext : public Context<T> {
 public:
  void AddConstituentSystem(const SystemInterface<T>* sys) {
    contexts_[sys] = sys->CreateDefaultContext();
    outputs_[sys] = sys->AllocateOutput();
  }

  void Connect(const PortIdentifier<T>& src, const PortIdentifier<T>& dest) {
    // TODO: bounds checking
    OutputPort<T>* output_port = outputs_[src.first]->ports[src.second].get();
    // TODO: Sample rate?
    auto input_port = std::make_unique<DependentInputPort<T>>(output_port, 0.0);
    contexts_[dest.first]->SetInputPort(dest.second, std::move(input_port));
  }

  SystemOutput<T>* GetSubsystemOutput(const SystemInterface<T>* sys) const {
    auto it = outputs_.find(sys);
    if (it == nullptr) {
      return nullptr;
    }
    return (*it).get();
  }

  const Context<T>* GetSubsystemContext(const SystemInterface<T>* sys) const {
    auto it = contexts_.find(sys);
    if (it == nullptr) {
      return nullptr;
    }
    return (*it).get();
  }

 private:
  // Order is critical: Output ports must outlive the DependentInputPorts that
  // depend on them, because the input ports unregister themselves from output
  // port change notifications at destruction time. Thus, outputs_ must be
  // declared before contexts_, so that contexts_ is destroyed before outputs_.
  std::map<const SystemInterface<T>*, std::unique_ptr<SystemOutput<T>>> outputs_;
  std::map<const SystemInterface<T>*, std::unique_ptr<Context<T>>> contexts_;
};

}  // namespace systems
}  // namespace drake
