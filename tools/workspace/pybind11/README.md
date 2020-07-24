# pybind Repository

## Generating pybind coverage

To generate the file and class coverage, run:
`bazel build //bindings/pydrake:generate_pybind_coverage`

CSVs:
`bazel-bin/bindings/pydrake/file_coverage.csv`
`bazel-bin/bindings/pydrake/class_coverage.csv`

## mkdoc

To update regression testswith the most recent build's generated
files:

```sh
bazel build \
    //tools/workspace/pybind11:generate_sample_header \
    //tools/workspace/pybind11:generate_sample_header_coverage
cp \
   bazel-bin/tools/workspace/pybind11/test/output/*.{h,csv,xml} \
   tools/workspace/pybind11/test/output_expected/
chmod -R u+w tools/workspace/pybind11/test/output_expected/*
chmod -R -x tools/workspace/pybind11/test/output_expected/*
sed -i \
    s'#GENERATED FILE DO NOT EDIT#<GENERIC MARKER SCRUBBED FOR REVIEWABLE>#g' \
    tools/workspace/pybind11/test/output_expected/*
```

and then manually put back the "SCRUBBED" change.
