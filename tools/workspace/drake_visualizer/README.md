# drake_visualizer

The files in this directory should by and large be ignored by consumers.
Developers may need to update files in this directory to support any further
modifications needed for `drake_visualizer`.

In-tree usage (`bazel run //tools:drake_visualizer`) is controlled by the file
`drake_visualizer_bazel.py`, which is responsible for modifying
`LD_LIBRARY_PATH` and `PYTHONPATH` appropriately.

Installation (`bazel run //:install -- ${prefix}`) is managed by
`package.BUILD.bazel`, which configures `drake_visualizer_installed.py.in`
to contain the appropriate modifications for `LD_LIBRARY_PATH` and `sys.path`
based on the provided `${prefix}`.

## Distributions

When a consumer compiles drake, the `.tar.gz` archive for
`drake_visualizer` is downloaded and extracted as part of the build.  The
artifacts are compiled once by developers and uploaded to drake's
`drake-packages` S3 bucket.  As such, any modifications to the build should
modify the naming scheme of the `drake_visualizer` tarball `dv*.tar.gz`.  Never
overwrite a build artifact of the same name on `drake-packages` as doing so will
break any users trying to perform a historical build.

Both Ubuntu and macOS patch director to be compatible with VTK-8, see the
subdirectory `image/director-patches`.  VTK-8 will come from different locations
depending on the distribution.

### Ubuntu

Compiled artifacts are produced by `build_binaries_with_docker`, and the
associated `Dockerfile` and `image/` directory in this folder.  Any rebuilds
should update the `image/package.sh` file's `<build_number>` component of the
created archive (at the bottom of the file).

The `drake_visualizer` tarball `dv*.tar.gz` will contain both director and a
precompiled VTK-8 (with python bindings, director uses the VTK python
interface).  These will be installed side-by-side with the VTK being used by the
rest of drake in (see also: `tools/workspace/vtk/README.md`).

### macOS

Compiled artifacts are produced by `build_mac_binaries` and uploaded to drake's
`drake-packages` S3 bucket.  If these binaries need to be rebuilt, update the
`filename` variable at the bottom of the file to introduce a `build_number`.

On macOS, VTK-8 is installed via
[`homebrew-director`](https://github.com/RobotLocomotion/homebrew-director) and
installed in `setup/mac/binary_distribution/Brewfile`.
