#include "drake/systems/framework/system_base.h"

#include <algorithm>
#include <atomic>
#include <limits>
#include <mutex>
#include <regex>
#include <stdexcept>
#include <unordered_set>

#include <fmt/format.h>
#include <fmt/ranges.h>

#include "drake/common/hash.h"
#include "drake/common/text_logging.h"
#include "drake/common/unused.h"
#include "drake/systems/framework/fixed_input_port_value.h"

namespace {

// Output a string like "System::EvalInput()".
std::string FmtFunc(const char* func) {
  return fmt::format("System::{}()", func);
}

}  // namespace

namespace drake {
namespace systems {

SystemBase::~SystemBase() {}

internal::SystemId SystemBase::get_next_id() {
  return internal::SystemId::get_new_id();
}

std::string SystemBase::GetMemoryObjectName() const {
  // Remove the template parameter(s).
  const std::string type_name_without_templates = std::regex_replace(
      NiceTypeName::Get(*this), std::regex("<.*>$"), std::string());

  // Replace "::" with "/" because ":" is System::GetSystemPathname's separator.
  // TODO(sherm1) Change the separator to "/" and avoid this!
  const std::string default_name = std::regex_replace(
      type_name_without_templates, std::regex(":+"), std::string("/"));

  // Append the address spelled like "@0123456789abcdef".
  const uintptr_t address = reinterpret_cast<uintptr_t>(this);
  return fmt::format("{}@{:0>16x}", default_name, address);
}

std::string SystemBase::GetSystemPathname() const {
  const std::string parent_path =
      get_parent_service() ? get_parent_service()->GetParentPathname()
                           : std::string();
  return parent_path + internal::SystemMessageInterface::path_separator() +
         GetSystemName();
}

namespace {
/* Returns a copy of the given text, with HTML's special characters replaced
with their safe equivalents. */
std::string HtmlEscape(std::string_view text) {
  std::string result;
  for (char ch : text) {
    // N.B. This works fine even with utf-8 text.
    if (ch == '<') {
      result += "&lt;";
      continue;
    }
    if (ch == '>') {
      result += "&gt;";
      continue;
    }
    if (ch == '&') {
      result += "&amp;";
      continue;
    }
    result += ch;
  }
  return result;
}
}  // namespace

std::string SystemBase::GetGraphvizString(
    std::optional<int> max_depth,
    const std::map<std::string, std::string>& options) const {
  const GraphvizFragment result = GetGraphvizFragment(max_depth, options);
  return fmt::format(
      "digraph _{} {{\n"
      "rankdir=LR\n"
      "{}"
      "}}\n",
      system_id_, fmt::join(result.fragments, ""));
}

SystemBase::GraphvizFragment SystemBase::GetGraphvizFragment(
    std::optional<int> max_depth,
    const std::map<std::string, std::string>& options) const {
  DRAKE_THROW_UNLESS(max_depth.value_or(0) >= 0);

  GraphvizFragmentParams params;
  params.max_depth = max_depth.value_or(std::numeric_limits<int>::max());
  params.options = options;
  params.node_id = fmt::format("s{}", system_id_.get_value());

  // The header opens with our class name, sans namespaces and templates.
  const std::string system_type = std::regex_replace(
      NiceTypeName::RemoveNamespaces(NiceTypeName::Get(*this)),
      std::regex("<.*>$"), std::string());
  params.header_lines.push_back(
      fmt::format("<B>{}</B>", HtmlEscape(system_type)));

  // If we have a name, then add it to the header.
  const std::string system_name = this->get_name();
  if (!system_name.empty() && system_name != this->GetMemoryObjectName()) {
    params.header_lines.push_back(
        fmt::format("name={}", HtmlEscape(system_name)));
  }

  return DoGetGraphvizFragment(params);
}

SystemBase::GraphvizFragment SystemBase::DoGetGraphvizFragment(
    const GraphvizFragmentParams& params) const {
  GraphvizFragment result;

  // Unpack the options.
  const bool split_in_twain = [&params]() {
    auto iter = params.options.find("split");
    return (iter != params.options.end()) && (iter->second == "I/O");
  }();
  const std::string node_id_input =
      split_in_twain ? params.node_id + "in" : params.node_id;
  const std::string node_id_output =
      split_in_twain ? params.node_id + "out" : params.node_id;

  // Prepare some helpful constants.
  const int num_inputs = num_input_ports();
  const int num_outputs = num_output_ports();
  const int min_num_inputs_outputs = std::min(num_inputs, num_outputs);
  const int max_num_inputs_outputs = std::max(num_inputs, num_outputs);
  result.input_ports.reserve(num_inputs);
  result.output_ports.reserve(num_outputs);

  // Prepare a table of port labels. Input ports are on the left (col 0), and
  // output ports are on the right (col 1). A nullopt label indicates that the
  // <TD> cell should not be emitted. This is also a convenient time to populate
  // our result's list of port identifiers.
  MatrixX<std::optional<std::string>> port_labels(max_num_inputs_outputs, 2);
  {
    std::vector<std::string> labels_input =
        GetGraphvizPortLabels(/* input= */ true);
    std::vector<std::string> labels_output =
        GetGraphvizPortLabels(/* input= */ false);
    for (int i = 0; i < num_inputs; ++i) {
      port_labels(i, 0) = std::move(labels_input[i]);
      result.input_ports.push_back(fmt::format("{}:u{}", node_id_input, i));
    }
    for (int i = 0; i < num_outputs; ++i) {
      port_labels(i, 1) = std::move(labels_output[i]);
      result.output_ports.push_back(fmt::format("{}:y{}", node_id_output, i));
    }
  }

  // Prepare the table's styling. We'll expand the shorter of the two columns to
  // match the other, longer column.
  MatrixX<std::string> port_styles(max_num_inputs_outputs, 2);
  if (num_inputs != num_outputs) {
    const int shorter_col = num_inputs < num_outputs ? 0 : 1;
    if (min_num_inputs_outputs == 1) {
      // When the shorter column is just a single port, expand that port to fill
      // the entire column.
      port_styles(0, shorter_col) =
          fmt::format("ROWSPAN=\"{}\"", max_num_inputs_outputs);
    } else {
      // Otherwise, use a (single) greyed-out cell to fill up the remainder.
      const int filler_row = min_num_inputs_outputs;
      const int remainder = max_num_inputs_outputs - min_num_inputs_outputs;
      port_labels(filler_row, shorter_col) = "";
      port_styles(filler_row, shorter_col) =
          fmt::format("COLOR=\"grey\" ROWSPAN=\"{}\" SIDES=\"{}\"", remainder,
                      shorter_col == 0 ? "BL" : "BR");
    }
  }

  // When in split_in_twain mode, we'll emit two nodes (the input node, then the
  // output node); otherwise we'll emit just one node.
  const int num_nodes = split_in_twain ? 2 : 1;
  for (int node = 0; node < num_nodes; ++node) {
    DRAKE_DEMAND(split_in_twain || node_id_input == node_id_output);
    const std::string& node_id = (node == 0) ? node_id_input : node_id_output;
    const int node_num_rows = (split_in_twain && node == 0) ? num_inputs
                              : (split_in_twain && node == 1)
                                  ? num_outputs
                                  : max_num_inputs_outputs;

    // Open the node, along with its label table and the table's header row.
    result.fragments.push_back(fmt::format(
        R"""({} [shape=none, border=0, label=<
<TABLE BORDER="0" CELLSPACING="0">
<TR><TD BORDER="1" COLSPAN="2">
{}
</TD></TR>
)""",
        node_id, fmt::join(params.header_lines, "<BR/>\n")));

    // Print the input and/or output port rows.
    for (int i = 0; i < node_num_rows; ++i) {
      result.fragments.push_back("<TR>\n");
      for (int j = 0; j < 2; ++j) {
        if (split_in_twain && j != node) {
          // The dead column of a twained node is just a single dotted box.
          if (i == 0) {
            result.fragments.push_back(fmt::format(
                R"""(<TD BORDER="1" STYLE="dotted" COLOR="cadetblue" ROWSPAN="{}">
<FONT COLOR="cadetblue">(split)</FONT>
</TD>
)""",
                (j == 0) ? num_outputs : num_inputs));
          }
          continue;
        }
        if (port_labels(i, j).has_value()) {
          result.fragments.push_back(fmt::format(
              R"""(<TD PORT="{}{}" BORDER="1" {}>{}</TD>
)""",
              (j == 0 ? 'u' : 'y'), i, port_styles(i, j), *port_labels(i, j)));
        }
      }
      result.fragments.push_back("</TR>\n");
    }

    // Close the table, label, attributes, and node.
    result.fragments.push_back(
        R"""(</TABLE>
>];
)""");
  }

  return result;
}

std::vector<std::string> SystemBase::GetGraphvizPortLabels(bool input) const {
  const int num_ports = input ? num_input_ports() : num_output_ports();
  std::vector<std::string> result;
  result.reserve(num_ports);
  for (int i = 0; i < num_ports; ++i) {
    const PortBase& port = [&]() -> const PortBase& {
      if (input) {
        return GetInputPortBaseOrThrow("", i, /* warn_deprecated = */ false);
      } else {
        return GetOutputPortBaseOrThrow("", i, /* warn_deprecated = */ false);
      }
    }();
    std::string label = HtmlEscape(port.get_name());
    // For deprecated ports, use strikethrough and a unicode headstone (🪦).
    if (port.get_deprecation().has_value()) {
      label = fmt::format("<S>{}</S>\xF0\x9F\xAA\xA6", label);
    }
    // For ports that cannot carry any data, grey out the name.
    if (port.get_data_type() == PortDataType::kVectorValued &&
        port.size() == 0) {
      label = fmt::format(R"""(<FONT COLOR="grey">{}</FONT>)""", label);
    }
    result.push_back(std::move(label));
  }
  return result;
}

CacheEntry& SystemBase::DeclareCacheEntry(
    std::string description, ValueProducer value_producer,
    std::set<DependencyTicket> prerequisites_of_calc) {
  return DeclareCacheEntryWithKnownTicket(
      assign_next_dependency_ticket(), std::move(description),
      std::move(value_producer), std::move(prerequisites_of_calc));
}

CacheEntry& SystemBase::DeclareCacheEntryWithKnownTicket(
    DependencyTicket known_ticket, std::string description,
    ValueProducer value_producer,
    std::set<DependencyTicket> prerequisites_of_calc) {
  // If the prerequisite list is empty the CacheEntry constructor will throw
  // a logic error.
  const CacheIndex index(num_cache_entries());
  cache_entries_.emplace_back(std::make_unique<CacheEntry>(
      this, index, known_ticket, std::move(description),
      std::move(value_producer), std::move(prerequisites_of_calc)));
  CacheEntry& new_entry = *cache_entries_.back();
  return new_entry;
}

bool SystemBase::IsObviouslyNotInputDependent(
    DependencyTicket dependency_ticket) const {
  // Everything `<= ceiling` is known to be non-input-dependent; this is
  // promised by the framework_common.h documentation.
  const auto ceiling = internal::kAllSourcesExceptInputPortsTicket;
  static_assert(internal::kAllInputPortsTicket > ceiling);
  static_assert(internal::kAllSourcesTicket > ceiling);
  static_assert(internal::kConfigurationTicket > ceiling);
  static_assert(internal::kKinematicsTicket > ceiling);
  return (dependency_ticket <= ceiling) ||
         std::any_of(discrete_state_tickets_.begin(),
                     discrete_state_tickets_.end(),
                     [&dependency_ticket](const auto& info) {
                       return info.ticket == dependency_ticket;
                     }) ||
         std::any_of(abstract_state_tickets_.begin(),
                     abstract_state_tickets_.end(),
                     [&dependency_ticket](const auto& info) {
                       return info.ticket == dependency_ticket;
                     });
  // We could also plausibly scan {numeric,abstract}_parameter_tickets_ to
  // return true, but we don't believe the cost is worth it. Usually there
  // are many parameters (so the scan would need to check many items), but
  // output ports usually depend on all_parameters_ticket() not each small
  // parameter one by one.
}

void SystemBase::InitializeContextBase(ContextBase* context_ptr) const {
  DRAKE_DEMAND(context_ptr != nullptr);
  ContextBase& context = *context_ptr;

  // Initialization should happen only once per Context.
  DRAKE_DEMAND(
      !internal::SystemBaseContextBaseAttorney::is_context_base_initialized(
          context));

  internal::SystemBaseContextBaseAttorney::set_system_name(&context,
                                                           get_name());
  internal::SystemBaseContextBaseAttorney::set_system_id(&context, system_id_);

  // Add the independent-source trackers and wire them up appropriately. That
  // includes input ports since their dependencies are external.
  CreateSourceTrackers(&context);

  DependencyGraph& graph = context.get_mutable_dependency_graph();

  // Create the Context cache containing a CacheEntryValue corresponding to
  // each CacheEntry, add a DependencyTracker and subscribe it to its
  // prerequisites as specified in the CacheEntry. Cache entries are
  // necessarily ordered such that the first cache entry can depend only
  // on the known source trackers created above, the second may depend on
  // those plus the first, and so on. Circular dependencies are not permitted.
  Cache& cache = context.get_mutable_cache();
  for (CacheIndex index(0); index < num_cache_entries(); ++index) {
    const CacheEntry& entry = get_cache_entry(index);
    CacheEntryValue& cache_value = cache.CreateNewCacheEntryValue(
        entry.cache_index(), entry.ticket(), entry.description(),
        entry.prerequisites(), &graph);
    // TODO(sherm1) Supply initial value on creation instead and get rid of
    // this separate call.
    cache_value.SetInitialValue(entry.Allocate());

    if (entry.is_disabled_by_default()) {
      cache_value.disable_caching();
    }
  }

  // Create the output port trackers yᵢ here. Nothing in this System may
  // depend on them; subscribers will be input ports from peer subsystems or
  // an exported output port in the parent Diagram. The associated cache entries
  // were just created above. Any intra-system prerequisites are set up now.
  for (const auto& oport : output_ports_) {
    internal::SystemBaseContextBaseAttorney::AddOutputPort(
        &context, oport->get_index(), oport->ticket(),
        oport->GetPrerequisite());
  }

  internal::SystemBaseContextBaseAttorney::mark_context_base_initialized(
      &context);
}

// Set up trackers for variable-numbered independent sources: discrete and
// abstract state, numerical and abstract parameters, and input ports.
// The generic trackers like "all parameters" are already present in the
// supplied Context, but we have to subscribe them to the individual
// elements now.
void SystemBase::CreateSourceTrackers(ContextBase* context_ptr) const {
  ContextBase& context = *context_ptr;

  // Define a lambda to do the repeated work below: create trackers for
  // individual entities and subscribe the group tracker to each of them.
  auto make_trackers =
      [&context](
          DependencyTicket subscriber_ticket,
          const std::vector<TrackerInfo>& system_ticket_info,
          void (*add_ticket_to_context)(ContextBase*, DependencyTicket)) {
        DependencyGraph& graph = context.get_mutable_dependency_graph();
        DependencyTracker& subscriber =
            graph.get_mutable_tracker(subscriber_ticket);

        for (const auto& info : system_ticket_info) {
          auto& source_tracker =
              graph.CreateNewDependencyTracker(info.ticket, info.description);
          add_ticket_to_context(&context, info.ticket);
          subscriber.SubscribeToPrerequisite(&source_tracker);
        }
      };

  // Allocate trackers for each discrete variable group xdᵢ, and subscribe
  // the "all discrete variables" tracker xd to those.
  make_trackers(
      xd_ticket(), discrete_state_tickets_,
      &internal::SystemBaseContextBaseAttorney::AddDiscreteStateTicket);

  // Allocate trackers for each abstract state variable xaᵢ, and subscribe
  // the "all abstract variables" tracker xa to those.
  make_trackers(
      xa_ticket(), abstract_state_tickets_,
      &internal::SystemBaseContextBaseAttorney::AddAbstractStateTicket);

  // Allocate trackers for each numeric parameter pnᵢ and each abstract
  // parameter paᵢ, and subscribe the pn and pa trackers to them.
  make_trackers(
      pn_ticket(), numeric_parameter_tickets_,
      &internal::SystemBaseContextBaseAttorney::AddNumericParameterTicket);
  make_trackers(
      pa_ticket(), abstract_parameter_tickets_,
      &internal::SystemBaseContextBaseAttorney::AddAbstractParameterTicket);

  // Allocate trackers for each input port uᵢ, and subscribe the "all input
  // ports" tracker u to them. Doesn't use TrackerInfo so can't use the lambda.
  for (const auto& iport : input_ports_) {
    internal::SystemBaseContextBaseAttorney::AddInputPort(
        &context, iport->get_index(), iport->ticket(),
        MakeFixInputPortTypeChecker(iport->get_index()));
  }
}

void SystemBase::set_parent_service(
    SystemBase* child,
    const internal::SystemParentServiceInterface* parent_service) {
  DRAKE_DEMAND(child != nullptr);
  if (parent_service != nullptr && child->parent_service_ != nullptr) {
    throw std::logic_error(fmt::format(
        "Cannot build subsystem '{}' into Diagram '{}' because it has already "
        "been built into a different Diagram '{}'",
        child->GetSystemName(), parent_service->GetParentPathname(),
        child->parent_service_->GetParentPathname()));
  }
  child->parent_service_ = parent_service;
}

// The only way for a system to evaluate its own input port is if that
// port is fixed. In that case the port's value is in the corresponding
// subcontext and we can just return it. Otherwise, the port obtains its value
// from some other system and we need our parent's help to get access to
// that system.
const AbstractValue* SystemBase::EvalAbstractInputImpl(
    const char* func, const ContextBase& context,
    InputPortIndex port_index) const {
  if (port_index >= num_input_ports())
    ThrowInputPortIndexOutOfRange(func, port_index);

  if (input_ports_[port_index]->get_deprecation().has_value())
    WarnPortDeprecation(/* is_input = */ true, port_index);

  const FixedInputPortValue* const free_port_value =
      context.MaybeGetFixedInputPortValue(port_index);

  if (free_port_value != nullptr)
    return &free_port_value->get_value();  // A fixed input port.

  // The only way to satisfy an input port of a root System is to make it fixed.
  // Since it wasn't fixed, it is unconnected.
  if (get_parent_service() == nullptr) return nullptr;

  // If this is a root Context, our parent can't evaluate it.
  if (context.is_root_context()) return nullptr;

  // This is not the root System, and the port isn't fixed, so ask our parent to
  // evaluate it.
  return get_parent_service()->EvalConnectedSubsystemInputPort(
      *internal::SystemBaseContextBaseAttorney::get_parent_base(context),
      get_input_port_base(port_index));
}

void SystemBase::ThrowNegativePortIndex(const char* func,
                                        int port_index) const {
  DRAKE_DEMAND(port_index < 0);
  throw std::out_of_range(
      fmt::format("{}: negative port index {} is illegal. (System {})",
                  FmtFunc(func), port_index, GetSystemPathname()));
}

void SystemBase::ThrowInputPortIndexOutOfRange(const char* func,
                                               InputPortIndex port) const {
  throw std::out_of_range(fmt::format(
      "{}: there is no input port with index {} because there are only {} "
      "input ports in system {}.",
      FmtFunc(func), port, num_input_ports(), GetSystemPathname()));
}

void SystemBase::ThrowOutputPortIndexOutOfRange(const char* func,
                                                OutputPortIndex port) const {
  throw std::out_of_range(fmt::format(
      "{}: there is no output port with index {} because there "
      "are only {} output ports in system {}.",
      FmtFunc(func), port, num_output_ports(), GetSystemPathname()));
}

void SystemBase::ThrowNotAVectorInputPort(const char* func,
                                          InputPortIndex port) const {
  throw std::logic_error(fmt::format(
      "{}: vector port required, but input port '{}' (index {}) was declared "
      "abstract. Even if the actual value is a vector, use "
      "EvalInputValue<V> instead for an abstract port containing a vector "
      "of type V. (System {})",
      FmtFunc(func), get_input_port_base(port).get_name(), port,
      GetSystemPathname()));
}

void SystemBase::ThrowInputPortHasWrongType(
    const char* func, InputPortIndex port, const std::string& expected_type,
    const std::string& actual_type) const {
  ThrowInputPortHasWrongType(func, GetSystemPathname(), port,
                             get_input_port_base(port).get_name(),
                             expected_type, actual_type);
}

void SystemBase::ThrowInputPortHasWrongType(const char* func,
                                            const std::string& system_pathname,
                                            InputPortIndex port,
                                            const std::string& port_name,
                                            const std::string& expected_type,
                                            const std::string& actual_type) {
  throw std::logic_error(fmt::format(
      "{}: expected value of type {} for input port '{}' (index {}) "
      "but the actual type was {}. (System {})",
      FmtFunc(func), expected_type, port_name, port, actual_type,
      system_pathname));
}

void SystemBase::ThrowCantEvaluateInputPort(const char* func,
                                            InputPortIndex port) const {
  throw std::logic_error(
      fmt::format("{}: input port '{}' (index {}) is neither connected nor "
                  "fixed so cannot be evaluated. (System {})",
                  FmtFunc(func), get_input_port_base(port).get_name(), port,
                  GetSystemPathname()));
}

void SystemBase::ThrowValidateContextMismatch(
    const ContextBase& context) const {
  const char* const info_link =
      "For more information about Context-System mismatches, see "
      "https://drake.mit.edu/"
      "troubleshooting.html#framework-context-system-mismatch";

  // Check if we are a subsystem within a Diagram and the user passed us the
  // root context instead of our subsystem context. In that case, we can provide
  // a more specific error message.
  if (get_parent_service() != nullptr) {
    // N.B. get_parent_service() is only non-null for subsystems in Diagrams.
    const internal::SystemId root_id =
        get_parent_service()->GetRootSystemBase().get_system_id();
    if (context.get_system_id() == root_id) {
      throw std::logic_error(fmt::format(
          "A function call on a {} system named '{}' was passed the root "
          "Diagram's Context instead of the appropriate subsystem Context. "
          "Use GetMyContextFromRoot() or similar to acquire the appropriate "
          "subsystem Context.\n{}",
          this->GetSystemType(), this->GetSystemPathname(), info_link));
    }
  }

  // Check if the context is a sub-context whose root context was created by
  // this Diagram. In that case, we can provide a more specific error message.
  const ContextBase& root_context = [&context]() -> const ContextBase& {
    const ContextBase* iterator = &context;
    while (true) {
      const ContextBase* parent =
          internal::SystemBaseContextBaseAttorney::get_parent_base(*iterator);
      if (parent == nullptr) {
        return *iterator;
      }
      iterator = parent;
    }
  }();
  if (root_context.get_system_id() == get_system_id()) {
    throw std::logic_error(fmt::format(
        "A function call on the root Diagram was passed a subcontext "
        "associated with its subsystem named '{}' instead of the root "
        "context. When calling a function on a the root Digram, you must "
        "pass a reference to the root Context, not a subcontext.\n{}",
        context.GetSystemPathname(), info_link));
  }

  throw std::logic_error(fmt::format(
      "A function call on a {} system named '{}' was passed the Context of "
      "a system named '{}' instead of the appropriate subsystem Context.\n{}",
      this->GetSystemType(), this->GetSystemPathname(),
      context.GetSystemPathname(), info_link));
}

void SystemBase::ThrowNotCreatedForThisSystemImpl(
    const std::string& nice_type_name, internal::SystemId id) const {
  if (!id.is_valid()) {
    throw std::logic_error(fmt::format(
        "{} was not associated with any System but should have been "
        "created for {} System {}",
        nice_type_name, GetSystemType(), GetSystemPathname()));
  } else {
    throw std::logic_error(fmt::format("{} was not created for {} System {}",
                                       nice_type_name, GetSystemType(),
                                       GetSystemPathname()));
  }
}

void SystemBase::WarnPortDeprecation(bool is_input, int port_index) const {
  // Locate the deprecated PortBase (while sanity-checking our arguments).
  PortBase* port;
  if (is_input) {
    port = input_ports_.at(port_index).get();
  } else {
    port = output_ports_.at(port_index).get();
  }
  DRAKE_DEMAND(port != nullptr);
  DRAKE_DEMAND(port->get_deprecation().has_value());

  // If this port object has already been warned about, then return quickly.
  std::atomic<bool>* const deprecation_already_warned =
      internal::PortBaseAttorney::deprecation_already_warned(port);
  const bool had_already_warned = deprecation_already_warned->exchange(true);
  if (had_already_warned) {
    return;
  }

  // The had_already_warned above is a *per instance* warning, for performance.
  // We'd like to warn at most once *per process*; therefore, we have a second
  // layer of checking, using a unique lookup key for SystemType + PortBase.
  drake::internal::FNV1aHasher hash;
  hash_append(hash, this->GetSystemType());
  hash_append(hash, is_input);
  hash_append(hash, port->get_name());
  const size_t key = size_t{hash};
  static std::mutex g_mutex;
  static std::unordered_set<size_t> g_warned_hash;
  {
    std::lock_guard lock(g_mutex);
    const bool inserted = g_warned_hash.insert(key).second;
    if (!inserted) {
      // The key was already in the map, which means that we've already
      // warned about this port name on this particular system subclass.
      return;
    }
  }

  // We hadn't warned yet, so we'll warn now.
  const std::string& description = port->GetFullDescription();
  const std::string& deprecation = port->get_deprecation().value();
  const char* const message = deprecation.size()
                                  ? deprecation.c_str()
                                  : "no deprecation details were provided";
  drake::log()->warn("{} is deprecated: {}", description, message);
}

std::string SystemBase::GetUnsupportedScalarConversionMessage(
    const std::type_info& source_type,
    const std::type_info& destination_type) const {
  unused(source_type);
  return fmt::format(
      "System {} of type {} does not support scalar conversion to type {}",
      GetSystemPathname(), GetSystemType(),
      NiceTypeName::Get(destination_type));
}

namespace internal {

std::string DiagramSystemBaseAttorney::GetUnsupportedScalarConversionMessage(
    const SystemBase& system, const std::type_info& source_type,
    const std::type_info& destination_type) {
  return system.GetUnsupportedScalarConversionMessage(source_type,
                                                      destination_type);
}

std::vector<std::string> DiagramSystemBaseAttorney::GetGraphvizPortLabels(
    const SystemBase& system, bool input) {
  return system.GetGraphvizPortLabels(input);
}

}  // namespace internal

}  // namespace systems
}  // namespace drake
