# A re-implementation of upstream's sdf/embedSdf.rb tool in Python.

import sys

from lxml import etree

assert __name__ == '__main__'


def _minified_xml(*, filename):
    """Given a filename for an `*.sdf` schema file, returns a minified xml
    string with its contents, to conserve disk space in Drake's library.
    """
    parser = etree.XMLParser(remove_blank_text=True, remove_comments=True)
    root = etree.parse(filename, parser)
    for item in root.xpath("//description"):
        item.getparent().remove(item)
    for elem in root.iter('*'):
        if elem.text is not None:
            elem.text = elem.text.strip()
        if elem.tail is not None:
            elem.tail = elem.tail.strip()
    return etree.tostring(root, encoding="utf-8", xml_declaration=False)


filenames = sorted(sys.argv[1:])
print("""
#include "EmbeddedSdf.hh"
#include <array>
#include "drake_vendor/ignition/utils/NeverDestroyed.hh"
namespace sdf { inline namespace SDF_VERSION_NAMESPACE {
const std::map<std::string, std::string>& GetEmbeddedSdf() {
  using Result = std::map<std::string, std::string>;
""")
print('  constexpr std::array<std::pair<const char*, const char*>, {}> pairs{{'
      .format(len(filenames)))
for filename in filenames:
    _, relative_path = filename.split('/sdf/')
    print('std::pair<const char*, const char*>{')
    print(f'"{relative_path}",')
    print('R"raw(')
    sys.stdout.flush()
    sys.stdout.buffer.write(_minified_xml(filename=filename))
    print(')raw"')
    print('},')
print("""
  };
  static const ignition::utils::NeverDestroyed<Result> result{
      pairs.begin(), pairs.end()};
  return result.Access();
}}}
""")
