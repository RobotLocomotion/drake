#include "drake/geometry/utilities.h"

#include <regex>

#include "drake/common/drake_assert.h"

namespace drake {
namespace geometry {
namespace internal {

std::string CanonicalizeStringName(const std::string& name) {
  // The definition of "canonical" is based on SDF and the functionality in
  // sdformat. Essentially, it trims off leading and trailing white space (as
  // shown here):
  // https://bitbucket.org/osrf/sdformat/src/2fa714812545abeb5ae05a8aebf0047aeb35d6a4/src/Types.cc?at=default&fileviewer=file-view-default#Types.cc-51
  // An issue has been filed to extend it to all whitespace characters:
  // https://bitbucket.org/osrf/sdformat/issues/194/string-trimming-only-considers-space-and
  // Keep this in sync in case it changes.

  // NOTE: The regular expression "." operator encompasses all characters
  // *except* new line characters. A "canonical" string implicitly allows
  // arbitrary internal whitespace (including newlines) so this trimming must do
  // so as well. To include them in the accepted set, we simply define the "not
  // empty" set of characters which *does* include  the newlines.
  std::regex trim_regex("[ \\t]*([^]*?)[ \\t]*");
  std::smatch matches;
  std::regex_match(name, matches, trim_regex);
  DRAKE_DEMAND(matches.size() == 2);
  return matches[1].str();
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
