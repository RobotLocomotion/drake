# Python Bindings

For more information, see:

* [User Documentation](https://drake.mit.edu/python_bindings.html)
* [Developer Documentation](https://drake.mit.edu/doxygen_cxx/group__python__bindings.html)

## Automated Bindings

At present, Drake bindings are still generally hand-curated. However, we are
making efforts to automate the minutae. We are working with Kitware to leverage
[`autopybind11`](https://gitlab.kitware.com/autopybind11/autopybind11).

To output template bindings to `/tmp/autopybind11`, please run the following:

```sh
bazel run //bindings/pydrake:autopybind11_example -- \
    --output_dir=/tmp/autopybind11
```

This is configured using the `autopybind11_example.yaml` in this directory. For more information on the `autopybind11` schema, please review its documentation:
<br/>
<https://gitlab.kitware.com/autopybind11/autopybind11>

## Debugging

To run, capture all output, and version control:

```sh
cd drake
./bindings/pydrake/autopybind11_example_debug.sh
```

### Additional Customization

The bazel command can take an additional argument to take advantage of a feature of AutoPyBind11
which is used to customize the code which is generated. It does this by passing a YAML file
with additional information and flags.  See the `Code Customziation` header within
 [`the README`]('https://gitlab.kitware.com/autopybind11/autopybind11/-/blob/master/README.rst') for more information.

The file that the script will look for is called `autopybind11_config.yaml`.
An example run with the configuration flag added would look as follows

```sh
bazel run //bindings/pydrake:autopybind11_example -- \
    --output_dir=/tmp/autopybind11 \
    --config_file=bindings/pydrake/autopybind11_config.yaml
```
