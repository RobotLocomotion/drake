/** @defgroup yaml_serialization YAML Serialization
@ingroup technical_notes

TODO(jwnimmer-tri) This section is still a work in progress, and needs
better overview text and organization.

<h2>Writing YAML files</h2>

Use SaveYamlFile() or SaveYamlString() to output a YAML-formatted serialization
of a C++ structure.

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

int main() {
  MyData data{1.0, {2.0, 3.0}};
  std::cout << drake::yaml::SaveYamlString(data);
  return 0;
}
@endcode

Output:
@code{yaml}
foo: 1.0
bar: [2.0, 3.0]
@endcode

Structures can be arbitrarily nested, as long as each `struct` has a
`Serialize` method.  Many common built-in types (int, double, std::string,
std::vector, std::array, std::map, std::unordered_map, std::optional,
std::variant, Eigen::Matrix) may also be used.

The output is always deterministic, even for unordered datatypes like
std::unordered_map.

For inspiration and background, see:
https://www.boost.org/doc/libs/release/libs/serialization/doc/tutorial.html
*/
