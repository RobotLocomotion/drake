/** @defgroup yaml_serialization YAML Serialization
@ingroup technical_notes

TBD

@anchor Implementing_Serialize
<h2>Implementing Serialize</h2>

TBD serialize

Sample code:
@code{cpp}
#include <vector>

#include <drake/common/name_value.h>

struct MyData {
  double foo{NAN};
  std::vector<double> bar;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(foo));
    a->Visit(DRAKE_NVP(bar));
  }
};
@endcode

**TBD defaults**

[1] N.B. The std::map collections merge the contents of the node into the
defaults, keeping anything in the default that is unchanged.  Collections
like std::vector are entirely reset, even if they already had some values
values already in place.

**TBD Write**

Saves data from a C++ structure into a YAML file, using SaveYamlString()
or SaveYamlFile().

Sample code:
@code{cpp}
int main() {
  MyData data{1.0, {2.0, 3.0}};
  std::cout << SaveYamlString("root", data);
  return 0;
}
@endcode

Output:
@code{yaml}
root:
  foo: 1.0
  bar: [2.0, 3.0]
@endcode

Structures can be arbitrarily nested, as long as each `struct` has a
`Serialize` method.  Many common built-in types (int, double, std::string,
std::vector, std::array, std::map, std::unordered_map, std::optional,
std::variant, Eigen::Matrix) may also be used.

The EmitString output is always deterministic, even for unordered datatypes
like std::unordered_map.

For inspiration and background, see:
https://www.boost.org/doc/libs/release/libs/serialization/doc/tutorial.html

**TBD Read**

Loads data from a YAML file into a C++ structure, using LoadYamlString()
or LoadYamlFile().

Sample data:
@code{yaml}
doc:
  foo: 1.0
  bar: [2.0, 3.0]
@endcode

Sample code:
@code{cpp}
MyData LoadData(const std::string& filename) {
  return LoadYamlFile<MyData>(filename);
}
@endcode

Structures can be arbitrarily nested, as long as each `struct` has a
`Serialize` method.  Many common built-in types (int, double, std::string,
std::vector, std::array, std::optional, std::variant, Eigen::Matrix) may
also be used.

YAML's "merge keys" (https://yaml.org/type/merge.html) are supported.

When reading into a std::variant<>, we match its YAML tag to the shortened
C++ class name of the variant selection.  For example, to read into this
sample struct:

@code
struct Foo {
  std::string data;
};
struct Bar {
  std::variant<std::string, double, Foo> value;
};
@endcode

Some valid YAML examples are:

@code
# For the first type declared in the variant<>, the tag is optional.
bar:
  value: hello

# YAML has built-in tags for string, float, int.
bar2:
  value: !!str hello

# For any other type within the variant<>, the tag is required.
bar3:
  value: !!float 1.0

# User-defined types use a single exclamation point.
bar4:
  value: !Foo
    data: hello
@endcode

*/
