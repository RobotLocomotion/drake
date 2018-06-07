#pragma once

#include <iosfwd>

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
///       zone:  # sequence of LaneSRange items
///         - [LANE_ID, S0, S1]
///         - [LANE_ID, S0, S1]
///         - [LANE_ID, S0, S1]
///       zone_type: StopExcluded  # ...ZoneType enum, without the leading 'k'
///       states:
///         STATE_ID1:
///           type: StopThenGo  # ...Type enum, without the leading 'k'
///           yield_to:
///             - OTHER_RULE_ID
///             - OTHER_RULE_ID
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


/// Loads rules parsed from `istream` into `rulebook`.
/// See yaml_io.h for a description of the YAML syntax.
///
/// This is an additive process, equivalent to calling `AddRule()` on each
/// parsed rule.  Any pre-existing content in `rulebook` is left as is.
///
/// `istream` must yield a YAML document containing a YAML map at its
/// root, with an entry bearing the key "maliput_simple_rulebook_v1".
/// Only this entry is parsed; any other content is ignored.
///
/// Throws `std::runtime_error` on parse errors, or failure of `AddRule()`.
/// Internally calls `YAML::Load(std::istream&)`, and thus does whatever
/// throwing/asserting which that function does.
// TODO(maddog@tri.global)  If/when it ever becomes ok to expose YAML::Node
//                          in a public drake/maliput header, this function
//                          should just take a YAML::Node instead of a
//                          std::istream.
void LoadYaml(std::istream* istream, SimpleRulebook* rulebook);

}  // namespace simplerulebook
}  // namespace maliput
}  // namespace drake
