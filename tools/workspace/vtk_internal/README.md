
# Tips for Drake Developers when upgrading VTK

The VTK source code version we download is determined by the `commit = ...`
git commit in `repository.bzl`.  Change that to the new commit, along with
the matching change to `sha256 = ...`. (To get the new sha256, set it to
`None` or `"0" * 64` and run a build; the required new sha256 will be printed.)

After changing the commit, next you might find that patch files have conflicts.
Check if any of the patches have already been upstreamed, and if so then drop
those from Drake. If there are still patch conflicts, you can either try to fix
them right away, or else comment out the patch in `repository.bzl` and circle
back to it later.

Then, the best next step is to get a handle on which libraries (aka "modules")
are required vs which should be excluded. Generally we want to try to build as
few modules as possible. Ignore configure_file complaints until you're sure the
list of modules is happy.

In rules.bzl there is a constant `VERBOSE = False`. Try changing that to `True`
when debugging / developing the module dependencies.

If there are too many error messages to understand at once, sometimes you can
make progress by working on one module at a time, starting from the bottom up.
For example, build the lowest-layer module first:

```
bazel build @vtk_internal//:vtkCommonCore
```

Note that vtkCommonCore in particular has a lot of weird codegen stuff, some of
which lives in `rules.bzl` (not just `settings.bzl`). If our codegen has fallen
out of date, consult the upstream code and try to match it back up.

Then you can work your way up:

```
bazel build @vtk_internal//:vtkCommonDataModel
```

For dealing with crashes from `cmake_configure_file()` about set-but-unused
values, just remove the now-unused values from `settings.bzl`. For crashes about
required-but-missing values, you can start by adding them to `cmake_undefines =
...` to try to reduce the quantity of errors; later (once the overall build is
healthier), you'll need to revisit what the proper value should be. Do that by
reading the VTK code (either the C++ code that uses the setting, or the
CMakeLists.txt or README files that explain the setting) and decide what the
value should be.
