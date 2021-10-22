// TODO(jwnimmer-tri) Create some directory-overview documentation about our
// YAML file format, using the example content immediately below.

// Write
/// Saves data from a C++ structure into a YAML file, using the Serialize /
/// Archive pattern.
///
/// Sample code:
/// @code{cpp}
/// struct MyData {
///   double foo{NAN};
///   std::vector<double> bar;
///
///   template <typename Archive>
///   void Serialize(Archive* a) {
///     a->Visit(DRAKE_NVP(foo));
///     a->Visit(DRAKE_NVP(bar));
///   }
/// };
///
/// std::string SaveData(const MyData& data) {
///   common::YamlWriteArchive archive;
///   archive.Accept(data);
///   return archive.EmitString();
/// }
///
/// int main() {
///   MyData data{1.0, {2.0, 3.0}};
///   std::cout << SaveData(data);
///   return 0;
/// }
/// @endcode
///
/// Output:
/// @code{yaml}
/// root:
///   foo: 1.0
///   bar: [2.0, 3.0]
/// @endcode
///
/// Structures can be arbitrarily nested, as long as each `struct` has a
/// `Serialize` method.  Many common built-in types (int, double, std::string,
/// std::vector, std::array, std::map, std::unordered_map, std::optional,
/// std::variant, Eigen::Matrix) may also be used.
///
/// The EmitString output is always deterministic, even for unordered datatypes
/// like std::unordered_map.
///
/// For inspiration and background, see:
/// https://www.boost.org/doc/libs/release/libs/serialization/doc/tutorial.html

// Read
/// Sample data:
/// @code{yaml}
/// doc:
///   foo: 1.0
///   bar: [2.0, 3.0]
/// @endcode
///
/// Sample code:
/// @code{cpp}
/// struct MyData {
///   double foo{NAN};
///   std::vector<double> bar;
///
///   template <typename Archive>
///   void Serialize(Archive* a) {
///     a->Visit(DRAKE_NVP(foo));
///     a->Visit(DRAKE_NVP(bar));
///   }
/// };
///
/// MyData LoadData(const std::string& filename) {
///   return YamlReadArchive::LoadFile<MyData>(filename);
/// }
/// @endcode
///
/// Structures can be arbitrarily nested, as long as each `struct` has a
/// `Serialize` method.  Many common built-in types (int, double, std::string,
/// std::vector, std::array, std::optional, std::variant, Eigen::Matrix) may
/// also be used.
///
/// YAML's "merge keys" (https://yaml.org/type/merge.html) are supported.
///
/// When reading into a std::variant<>, we match its YAML tag to the shortened
/// C++ class name of the variant selection.  For example, to read into this
/// sample struct:
///
/// @code
/// struct Foo {
///   std::string data;
/// };
/// struct Bar {
///   std::variant<std::string, double, Foo> value;
/// };
/// @endcode
///
/// Some valid YAML examples are:
///
/// @code
/// # For the first type declared in the variant<>, the tag is optional.
/// bar:
///   value: hello
///
/// # YAML has built-in tags for string, float, int.
/// bar2:
///   value: !!str hello
///
/// # For any other type within the variant<>, the tag is required.
/// bar3:
///   value: !!float 1.0
///
/// # User-defined types use a single exclamation point.
/// bar4:
///   value: !Foo
///     data: hello
/// @endcode
