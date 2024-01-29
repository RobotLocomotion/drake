
# Tips for Drake Developers when upgrading VTK

When upgrading, the best first step is to get a handle on which libraries (aka
"modules") are required vs which should be excluded. Generally we want to try to
build as few modules as possible. Ignore configure_file complaints until you're
sure the list of modules is happy.

In rules.bzl there is a constant `VERBOSE = False`. Try changing that to `True`
when debugging / developing the module dependencies.

If there are too many error messages to understand at once, sometimes you can
make progress by working on one module at a time, starting from the bottom up.
For example, build the lowest-layer module first:

```
bazel build @vtk_internal//:vtkCommonCore
```

Then you can work your way up:

```
bazel build @vtk_internal//:vtkCommonDataModel
```
