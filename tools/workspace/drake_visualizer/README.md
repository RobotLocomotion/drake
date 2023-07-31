# Deprecation

The legacy `drake_visualizer` application is deprecated for removal and is only
maintained on the Ubuntu 20.04 ("Focal") operating system.  When Drake drops
support for that platform (ETA spring of 2024), the `drake_visualizer` program
will also be dropped.

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

We patch director to be compatible with VTK-8, see the subdirectory
`image/director-patches`.

### Ubuntu

Compiled artifacts are produced by `build_binaries_with_docker`, and the
associated `Dockerfile` and `image/` directory in this folder.  Any rebuilds
should update the `image/package.sh` file's `<build_number>` component of the
created archive (at the bottom of the file).

The `drake_visualizer` tarball `dv*.tar.gz` will contain both director and a
precompiled VTK-8 (with python bindings, director uses the VTK python
interface).  These will be installed side-by-side with the VTK being used by the
rest of drake in (see also: `tools/workspace/vtk/README.md`).
