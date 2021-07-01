# Automated Python Binding Generator

This provides an integration with
[`autopybind11`](https://gitlab.kitware.com/autopybind11/autopybind11). At present, these are meant to *bootstrap* bindings
(e.g. generate or regenerate bindings for a class), but are not meant to
completely replace the manual binding process.

## Running

To output template bindings to `/tmp/autopybind11`, please run the following:

```sh
bazel run //tools/autopybind:generate -- --output_dir=/tmp/autopybind11
```

This is configured using the `bindings_to_generate.yaml` file in this
directory. For more information on the `autopybind11` schema, please review its 
documentation:
<br/>
<https://gitlab.kitware.com/autopybind11/autopybind11>

## Example Workflow

TODO(eric): Add this in.

## Troubleshooting

If you encounter an issue w/ our usage of `autopybind11`, please run the
following script which captures all output, commit the generate files to git,
and make a draft PR pointing to what problem you're having:

```sh
cd drake
./tools/autopybind/generate_debug.sh
```
