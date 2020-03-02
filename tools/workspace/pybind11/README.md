### Generating pybind coverage

To generate the file and class coverage, run:
`bazel build //bindings/pydrake:generate_pybind_coverage`

CSVs:
`bazel-bin/bindings/pydrake/file_coverage.csv`
`bazel-bin/bindings/pydrake/class_coverage.csv`
