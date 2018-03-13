#pragma once

#include <string>

#include "yaml-cpp/yaml.h"

#include "drake/automotive/maliput/simplerulebook/simple_rulebook.h"

namespace drake {
namespace maliput {
namespace simplerulebook {

/// @file
/// YAML IO for SimpleRulebook.
///
/// The YAML format is a simple literal representation of one record for
/// each rule, like so:
/// ~~~~~~
/// ---
/// maliput_simple_rulebook_v1:  # rulebook content tag
///   speed_limit:  # SpeedLimitRule group
///     ID1:  # ID, as a string
///       zone: [LANE_ID, S0, S1]  # LaneSRange
///       severity: Strict  # a Severity enumerator, without the leading 'k'
///       limit:  [MIN, MAX]
///     ID2:
///       zone: [LANE_ID, S0, S1]
///       severity: Advisory
///       limit:  MAX  #  A lone MAX limit implies MIN = 0.
///     .
///     .
///     .
///   right_of_way:  # RightOfWayRule group
///     ID1:  # ID, as a string
///       controlled_zone:  # sequence of LaneSRange items
///         - [LANE_ID, S0, S1]
///         - [LANE_ID, S0, S1]
///         - [LANE_ID, S0, S1]
///       type: StopThenGo  # ...the Type enum, without the leading 'k'
///     .
///     .
///     .
/// ~~~~~~
///
/// Only YAML content within the node tagged with "maliput_simple_rulebook_v1"
/// is parsed.  All rules are grouped by rule type.  Within each group, each
/// rule entry is labeled with the rule's ID.  The contents of a rule entry
/// depend on the rule type, but are generally in 1-to-1 correspondence with
/// the arguments to the constructor for that rule type.


/// Load rules unpacked from `node` into `rulebook`.
///
/// This is an additive process, equivalent to calling `AddRule()` on each
/// parsed rule.  Any pre-existing content in `rulebook` is left as is.
///
/// `node` must be a YAML map, with an entry bearing the key
/// "maliput_simple_rulebook_v1".  Only this entry is parsed; any other
/// content is ignored.
///
/// Throws `std::runtime_error` on parse errors, or failure of `AddRule()`.
///
/// Usage hints:
/// * To load from a string:
///   `LoadYaml(YAML::Load(string), &rulebook)`
/// * To load from a file:
///   `LoadYaml(YAML::LoadFile(filename), &rulebook)`
void LoadYaml(YAML::Node node, SimpleRulebook* rulebook);

}  // namespace simplerulebook
}  // namespace maliput
}  // namespace drake
