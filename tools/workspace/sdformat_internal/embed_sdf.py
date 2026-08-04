# A re-implementation of upstream's sdf/embedSdf.rb tool in Python.

import sys
import xml.etree.ElementTree as ET

assert __name__ == "__main__"


def _minified_xml(*, filename):
    """Given a filename for an `*.sdf` schema file, returns a minified xml
    string with its contents, to conserve disk space in Drake's library.
    """
    tree = ET.parse(filename)
    # Remove all '<description>' elements.
    for item in tree.findall(".//description/.."):
        item.remove(item.find("description"))
    # Discard whitespace.
    for elem in tree.iter("*"):
        if elem.text is not None:
            elem.text = elem.text.strip()
        if elem.tail is not None:
            elem.tail = elem.tail.strip()
    return ET.tostring(tree.getroot(), encoding="utf-8", xml_declaration=False)


filenames = sorted(sys.argv[1:])
print("""
#include "EmbeddedSdf.hh"
#include <array>
#include "gz/utils/NeverDestroyed.hh"
namespace sdf { inline namespace SDF_VERSION_NAMESPACE {
const std::map<std::string, std::string>& GetEmbeddedSdf() {
  using Result = std::map<std::string, std::string>;
""")
print(
    "  constexpr std::array<{}, {}> pairs{{".format(
        "std::pair<const char*, const char*>",
        len(filenames),
    )
)
for filename in filenames:
    _, relative_path = filename.split("/sdf/")
    print("std::pair<const char*, const char*>{")
    print(f'"{relative_path}",')
    print('R"raw(')
    sys.stdout.flush()
    sys.stdout.buffer.write(_minified_xml(filename=filename))
    print(')raw"')
    print("},")
print("""
  };
  static const gz::utils::NeverDestroyed<Result> result{
      pairs.begin(), pairs.end()};
  return result.Access();
}}}
""")
