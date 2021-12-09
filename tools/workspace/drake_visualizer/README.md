# drake_visualizer

## Linux Build

On linux the director build uses vtk-8, and the vtk-8 libraries are packaged
into the director tarball -- they will be installed side-by-side with the
`tools/workspace/vtk` binaries (currently vtk-9).  Note the
`drake_visualizer_bazel.py` file modifies `LD_LIBRARY_PATH` to enable
`bazel run //tools:drake_visualizer` to find the vtk-8 libraries.  On the
install side, `tools/install/install.py.in` configures the vtk-8 libraries
to have an rpath of `$ORIGIN`, and the `python3.x/site-packages/vtkmodules/*.so`
receive `$ORIGIN/../../..` so that each installation can find the corresponding
vtk-8 libraries needed.

### Prerequisites

In order to build director, the vtk-8 tarballs are required.  They are copied
into the director docker build.  They must be in the current directory,
`tools/workspace/drake_visualizer`.

#### Option 1: Download the Existing vtk-8 Tarballs

```console
# Download the bionic tarball
$ wget https://drake-packages.csail.mit.edu/vtk/vtk-8.2.0-1-python-3.6.9-qt-5.9.5-bionic-x86_64.tar.gz
$ sha256sum vtk-8.2.0-1-python-3.6.9-qt-5.9.5-bionic-x86_64.tar.gz
d8d8bd13605f065839942d47eb9d556d8aa3f55e5759eb424773d05c46e805ee  vtk-8.2.0-1-python-3.6.9-qt-5.9.5-bionic-x86_64.tar.gz

# Download the focal tarball
$ wget https://drake-packages.csail.mit.edu/vtk/vtk-8.2.0-1-python-3.8.5-qt-5.12.8-focal-x86_64.tar.gz
$ sha256sum vtk-8.2.0-1-python-3.8.5-qt-5.12.8-focal-x86_64.tar.gz
927811bbecb1537c7d46c2eb73112ee7d46caf5ff765b5b8951b624ddf7d2928  vtk-8.2.0-1-python-3.8.5-qt-5.12.8-focal-x86_64.tar.gz
```

#### Option 2: Rebuild the vtk-8 Tarballs

The original build files from `tools/workspace/vtk` for the vtk-8 version have
been relocated to the `vtk-8` folder in this directory.  You can rebuild the
vtk-8 tarballs by doing

```console
# Navigate to the vtk-8 directory
$ cd tools/workspace/drake_visualizer/vtk-8

# Build the vtk-8 tarballs
$ ./build_binaries_with_docker
```

If successful, a `vtk-8.2.0*.tar.gz` and corresponding `.sha256` file will be
produced.  After checking the `sha256sum` of the `.tar.gz` archives for bionic
and focal, you must `cp vtk-8.2.0*.tar.gz ..` so that the director docker build
can access them (it looks in the same directory).

### Build the director Tarball

Now that you have a `vtk-8.2.0*.tar.gz` tarballs in the current working
directory, run `./build_binaries_with_docker` in the
`tools/workspace/drake_visualizer` directory.  This will produce `dv-*.tar.gz`
and associated `dv-*.sha256` files.  **If you are intending to rebuild with the
intent of distribution, you must update the build-number**.  See the bottom of
the `Dockerfile`, the created archives end in `.x86_64-<build_number>.tar.gz`.
Increase the build number in order to avoid overwriting artifacts on
`drake-packages.csail.mit.edu/director`.

If developing locally, you can also modify `tools/workspace/mirrors.bzl`:

```py
DEFAULT_MIRRORS = {
    # ...
    "director": [
        # Add a file:// url to point to where your tarball lives.
        "file:///abs/path/drake/tools/workspace/drake_visualizer/{archive}",
        # "https://drake-packages.csail.mit.edu/director/{archive}",
        # "https://s3.amazonaws.com/drake-packages/director/{archive}",
    ],
    # ...
}
```

Make sure to update the `sha256` hash in `repository.bzl` when developing
locally.  After you upload, you are advised to delete the `file://` mirror entry
and rebuild to confirm your artifacts are downloaded and used correctly.

## macOS Build

The `build_mac_binaries` file should produce the mac tarball.  You may need to
obtain a downgraded CMake and possibly macOS installation in order to succeed
with the build.
