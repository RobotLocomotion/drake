# Semi-Automated Python Binding Generation Tool

This provides an integration with
[`autopybind11`](https://gitlab.kitware.com/autopybind11/autopybind11). At
present, it is meant to *bootstrap* bindings (e.g., generate or regenerate
bindings for a class that may need some correction or customization or may be
incomplete), but is not meant to completely replace the manual binding process.

## Running

To output template bindings to `/tmp/autopybind11`, please run the following:
```bash
bazel run //tools/autopybind:generate -- --output_dir=/tmp/autopybind11
```

This is configured using the `bindings_to_generate.yaml` file in this
directory. For more information on the `autopybind11` schema, please review its
[documentation](https://gitlab.kitware.com/autopybind11/autopybind11).

## Troubleshooting

If you encounter an issue with our usage of `autopybind11`, please run the
following script that captures all output, commit the generate files to git,
and make a draft pull request pointing to the problem that you are having:
```bash
cd drake
./tools/autopybind/generate_debug
```
