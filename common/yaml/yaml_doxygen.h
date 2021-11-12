/** @defgroup yaml_serialization YAML Serialization
@ingroup technical_notes

TODO(jwnimmer-tri) This section is still a work in progress, and needs
better overview text and organization.

<h2>Reading YAML files</h2>

Use LoadYamlFile() or LoadYamlString() to de-serialize YAML-formatted string
data into C++ structure.

Sample data:
@code{yaml}
foo: 1.0
bar: [2.0, 3.0]
@endcode

Sample code:
@code{cpp}
struct MyData {
  double foo{NAN};
  std::vector<double> bar;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(foo));
    a->Visit(DRAKE_NVP(bar));
  }
};

MyData LoadData(const std::string& filename) {
  return LoadYamlFile<MyData>(filename);
}
@endcode

There is also an option to load from a top-level child in the document:

@code{yaml}
data_1:
  foo: 1.0
  bar: [2.0, 3.0]
data_2:
  foo: 4.0
  bar: [6.0, 6.0]
@endcode

@code{cpp}
MyData LoadData(const std::string& filename) {
  return LoadYamlFile<MyData>(filename, "data_1");
}
@endcode

Structures can be arbitrarily nested, as long as each `struct` has a
`Serialize` method.  Many common built-in types (int, double, std::string,
std::vector, std::array, std::map, std::unordered_map, std::optional,
std::variant, Eigen::Matrix) may also be used.

YAML's "merge keys" (https://yaml.org/type/merge.html) are supported.
TODO(jwnimmer-tri) Give an example of merge keys.

For inspiration and background, see:
https://www.boost.org/doc/libs/release/libs/serialization/doc/tutorial.html

@anchor implementing_serialize
<h2>Implementing Serialize</h2>

Structured data sometimes provides a Serialize method to be compatible with a
variety of readers, writers, or any other code that needs to visit the data
generically.

Here is an example of implementing a Serialize method:
@code
struct DoubleStruct {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(value));
  }

  double value{0.0};
};
@endcode

By convention, we place the Serialize method prior to the data members per
<a href="https://drake.mit.edu/styleguide/cppguide.html#Declaration_Order">the
styleguide rule</a>.  Each data member has a matching `Visit` line in the
Serialize method, in the same order as the member fields appear.

By convention, we declare all of the member fields as public, since they are
effectively so anyway (because anything that calls the Serialize method
receives a mutable pointer to them).  The typical way to do this is to declare
the data as a `struct`, instead of a `class`.

However, if
<a href="https://drake.mit.edu/styleguide/cppguide.html#Structs_vs._Classes">the
styleguide rule</a> for struct vs class points towards using a `class` instead,
then we follow that advice and make it a `class`, but we explicitly label the
member fields as `public`.  We also omit the trailing underscore from the field
names, so that the Serialize API presented to the caller of the class is
indifferent to whether it is phrased as a `struct` or a `class`.

TBD serialize

<h2>Writing YAML files</h2>

Use SaveYamlFile() or SaveYamlString() to output a YAML-formatted serialization
of a C++ structure.

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

<!--
TODO(jwnimmer-tri) Describe the special case for top-level collections without
a Serialize function, i.e., std::map<std::string, Serializable>.

Usually we require that we have a serializable struct that matches the YAML
document. However, for the special case of a C++ collection type that matches
a YAML node type exactly, sometimes it's convenient to parse the document
directly into that collection, without the need to define an enclosing struct.

This PR adds std::map<std::string, Serializable> along those lines. In the
future we could imagine adding std::vector<Serializable> as well just for
completeness, though I no examples of needing that come to mind. Those are the
only two collection types that we'd want to allow in this kind of special case.
(They are the only ones that directly align with YAML node semantics.)
-->

**TBD Re-file this section**

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
