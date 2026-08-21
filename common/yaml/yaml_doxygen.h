/** @defgroup yaml_serialization YAML Serialization
@ingroup technical_notes

<h2>Overview</h2>

Drake provides infrastructure for reading YAML files into C++ structs, and
writing C++ structs into YAML files. These functions are often used to read
or write configuration data, but may also be used to serialize runtime data
such as Diagram connections or OutputPort traces. Any C++ struct to be
serialized must provide a @ref implementing_serialize "Serialize()" function
to enumerate its fields.

Identical functionality is provided in Python via
[pydrake.common.yaml](/pydrake/pydrake.common.yaml.html).

<h2>Examples</h2>

Given a struct definition:

@code{cpp}
struct MyData {
  ...

  double foo{0.0};
  std::vector<double> bar;
};
@endcode

<h3>Loading</h3>

Given a YAML data file:

@code{yaml}
foo: 1.0
bar: [2.0, 3.0]
@endcode

We can use LoadYamlFile() to load the file:

@code{cpp}
int main() {
  const MyData data = LoadYamlFile<MyData>("filename.yaml");
  std::cout << fmt::format("foo = {:.1f}\n", data.foo);
  std::cout << fmt::format("bar = {:.1f}\n", fmt::join(data.bar, ", "));
}
@endcode

Output:

@code{txt}
foo = 1.0
bar = 2.0, 3.0
@endcode

<h3>Saving</h3>

We can use SaveYamlFile() to save to a file:

@code{cpp}
int main() {
  MyData data{4.0, {5.0, 6.0}};
  SaveYamlFile("filename.yaml", data);
}
@endcode

Output file:

@code{yaml}
foo: 4.0
bar: [5.0, 6.0]
@endcode

The following sections explain each of these steps in more detail, along with
the customization options that are available for each one.

@anchor implementing_serialize
<h2>Implementing Serialize</h2>

Any C++ struct to be serialized must provide a templated `Serialize()` function
that enumerates the fields. Typically, `Serialize()` will be implemented via a
member function on the struct, but if necessary it can also be a free function
obtained via argument-dependent lookup.

Here is an example of implementing a Serialize member function:

@code{cpp}
struct MyData {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(foo));
    a->Visit(DRAKE_NVP(bar));
  }

  double foo{0.0};
  std::vector<double> bar;
};
@endcode

Structures can be arbitrarily nested, as long as each `struct` has a
`Serialize()` function:

@code{cpp}
struct MoreData {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(baz));
    a->Visit(DRAKE_NVP(quux));
  }

  std::string baz;
  std::map<std::string, MyData> quux;
};
@endcode

For background information about visitor-based serialization, see also the <a
href="https://www.boost.org/doc/libs/release/libs/serialization/doc/tutorial.html">
Boost.Serialization Tutorial</a>, which served as the inspiration for Drake's
design.

<h3>Style guide for Serialize</h3>

By convention, we place the Serialize function prior to the data members per
<a href="https://drake.mit.edu/styleguide/cppguide.html#Declaration_Order">the
styleguide rule</a>. Each data member has a matching `Visit` line in the
Serialize function, in the same order as the member fields appear.

By convention, we declare all of the member fields as public, since they are
effectively so anyway (because anything that calls the Serialize function
receives a mutable pointer to them). The typical way to do this is to declare
the data as a `struct`, instead of a `class`.

However, if
<a href="https://drake.mit.edu/styleguide/cppguide.html#Structs_vs._Classes">the
styleguide rule</a> for struct vs class points towards using a `class` instead,
then we follow that advice and make it a `class`, but we explicitly label the
member fields as `public`. We also omit the trailing underscore from the field
names, so that the Serialize API presented to the caller of the class is
indifferent to whether it is phrased as a `struct` or a `class`.
See drake::schema::Gaussian for an example of this situation.

If the member fields have invariants that must be immediately enforced during
de-serialization, then we add invariant checks to the end of the `Serialize()`
function to enforce that, and we mark the class fields private (adding back the
usual trailing underscore). See drake::math::BsplineBasis for an example of
this situation.

<h3>Built-in types</h3>

Drake's YAML I/O functions provide built-in support for many common types:
- bool
- double
- float
- int32_t
- int64_t
- uint32_t
- uint64_t
- std::array
- std::map
- std::optional
- std::string
- std::unordered_map
- std::variant
- std::vector
- Eigen::Matrix (including 1-dimensional matrices, i.e., vectors)

<h3>YAML correspondence</h3>

The simple types (`std::string`, `bool`, floating-point number, integers) all
serialize to a <a href="https://yaml.org/spec/1.2.2/#scalars">Scalar</a>
node in YAML.

The array-like types (`std::array`, `std::vector`, `Eigen::Matrix`) all
serialize to a <a href="https://yaml.org/spec/1.2.2/#collections">Sequence</a>
node in YAML.

User-defined structs and the native maps (`std::map`, `std::unordered_map`) all
serialize to a <a href="https://yaml.org/spec/1.2.2/#collections">Mapping</a>
node in YAML.

For the treatment of `std::optional`, refer to
@ref serialize_nullable "Nullable types", below.
For the treatment of `std::variant`, refer to
@ref serialize_variant "Sum types", below.

<h2>Reading YAML files</h2>

Use LoadYamlFile() or LoadYamlString() to de-serialize YAML-formatted string
data into C++ structure.

It's often useful to write a helper function to load using a specific schema,
in this case the `MyData` schema:

@code{cpp}
MyData LoadMyData(const std::string& filename) {
  return LoadYamlFile<MyData>(filename);
}
int main() {
  const MyData data = LoadMyData("filename.yaml");
  std::cout << fmt::format("foo = {:.1f}\n", data.foo);
  std::cout << fmt::format("bar = {:.1f}\n", fmt::join(data.bar, ", "));
}
@endcode

Sample data in `filename.yaml`:
@code{yaml}
foo: 1.0
bar: [2.0, 3.0]
@endcode

Sample output:

@code{txt}
foo = 1.0
bar = 2.0, 3.0
@endcode

There is also an option to load from a top-level child in the document:

@code{yaml}
data_1:
  foo: 1.0
  bar: [2.0, 3.0]
data_2:
  foo: 4.0
  bar: [5.0, 6.0]
@endcode

@code{cpp}
MyData LoadMyData2(const std::string& filename) {
  return LoadYamlFile<MyData>(filename, "data_2");
}
@endcode

Sample output:

@code{txt}
foo = 4.0
bar = 5.0, 6.0
@endcode

<h3>Defaults</h3>

The LoadYamlFile() function offers a `defaults = ...` argument. When provided,
the yaml file's contents will overwrite the provided defaults, but any fields
that are not mentioned in the yaml file will remain intact at their default
values.

When merging file data atop any defaults, any `std::map` or `std::unordered_map`
collections will merge the contents of the file alongside the existing map
values, keeping anything in the default that is unchanged. Any other
collections such as `std::vector` are entirely reset, even if they already
had some values in place (in particular, they are not merely appended to).

<h3>Merge keys</h3>

YAML's "merge keys" (https://yaml.org/type/merge.html) are supported during
loading. (However, the graph-aliasing relationship implied by nominal YAML
semantics is not implemented; the merge keys are fully deep-copied.)

Example:

@code{yaml}
_template: &common_foo
  foo: 1.0
data_1:
  << : *common_foo
  bar: [2.0, 3.0]
data_2:
  << : *common_foo
  bar: [5.0, 6.0]
@endcode

<h2>Writing YAML files</h2>

Use SaveYamlFile() or SaveYamlString() to output a YAML-formatted serialization
of a C++ structure.

The serialized output is always deterministic, even for unordered datatypes such
as `std::unordered_map`.

@code{cpp}
struct MyData {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(foo));
    a->Visit(DRAKE_NVP(bar));
  }

  double foo{0.0};
  std::vector<double> bar;
};

int main() {
  MyData data{1.0, {2.0, 3.0}};
  std::cout << SaveYamlString(data, "root");
  return 0;
}
@endcode

Output:
@code{yaml}
root:
  foo: 1.0
  bar: [2.0, 3.0]
@endcode

<h2>Document root</h2>

Usually, YAML reading or writing requires a serializable struct that matches
the top-level YAML document. However, sometimes it's convenient to parse the
document in the special case of a C++ `std::map` at the top level, without
the need to define an enclosing struct.

@code{yaml}
data_1:
  foo: 1.0
  bar: [2.0, 3.0]
data_2:
  foo: 4.0
  bar: [5.0, 6.0]
@endcode

@code{cpp}
std::map<std::string, MyData> LoadAllMyData(const std::string& filename) {
  return LoadYamlFile<std::map<std::string, MyData>>(filename);
}
@endcode

@anchor serialize_nullable
<h2>Nullable types (std::optional)</h2>

When a C++ field of type `std::optional` is present, then:
- when saving its enclosing struct as YAML data, if the optional is nullopt,
  then no mapping entry will be emitted.
- when load its enclosing struct from YAML data, if no mapping entry is present
  in the YAML, then it is not an error.

@anchor serialize_variant
<h2>Sum types (std::variant)</h2>

When reading into a std::variant<>, we match its
<a href="https://yaml.org/spec/1.2.2/#tags">YAML tag</a>
to the shortened C++ class name of the variant selection. For example, to read
into this sample struct:

@code{cpp}
struct Foo {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(data));
  }

  std::string data;
};

struct Bar {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(value));
  }

  std::variant<std::string, double, Foo> value;
};
@endcode

Some valid YAML examples are:

@code{yaml}
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

Take particular note that the first type declared in the variant is a special
case: when no tag is given in the YAML, then the scalar is read into the first
type in the variant. For primitive types like `std::filesystem::path` that don't
have a corresponding `!!tag` defined in the YAML specification, there is no way
to load them into a variant unless they are the first type in the variant.

*/
