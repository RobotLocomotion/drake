# VTK

The files in this directory should by and large be ignored by consumers.
Developers will need to make updates here in order to expose additional features
of VTK, or to update the version of VTK being used by drake.

The `repository.bzl` file is the core file that enumerates all of the VTK header
files and libraries that are in use by drake's codebase.  It does **not**
enumerate all possible files or libraries available in VTK.

## TODO(svenevs) rewrite this document

All of the builds (linux, linux_wheel, mac, mac_wheel) are expected to be
**sourcing** the following common files (symlinked to `tools/wheel/image` where
applicable) in this exact order:

- `image/common-definitions.sh` (defines `os_name`, `codename`, `architecture`)
- `image/clone-vtk.sh` (clones do `src` in the current working directory,
  defines the final output `vtk_archive_name` for `.tar.gz` output using
  `codename` and `architecture`)
- `image/vtk-cmake-args.sh` (defines `vtk_cmake_args` array with all of the
  VTK CMake configure arguments)
    - Assumes that `image/vtk-common-cmake-args` is in the same directory,
      docker based builds must make sure to create this.

To determine the `vtk_version` and subsequent `vtk_archive_name`, the VTK source
tree must be cloned in order to run `git describe`.

The consuming build (`.tar.gz` packaging or wheel) is responsible for wrapping
these scripts as appropriate.

`vtk-cmake-args.sh` is responsible for controlling all of the CMake configure
arguments for every single build flavor **except**:

- The generator (e.g., `-G Ninja`),
- The `CMAKE_BUILD_TYPE`,
- The `CMAKE_PREFIX_PATH` (where applicable, e.g., wheel builds),
- The `CMAKE_INSTALL_PREFIX`.

Wrapping build scripts per build flavor:

- Linux: `tools/workspace/vtk/image/build-and-package-vtk.sh`
- mac: `tools/workspace/vtk/build_mac_binary`
- Wheel (Linux and mac): `tools/wheel/image/build-vtk.sh`

## Distributions

### Ubuntu

Compiled artifacts are produced by `build_binaries_with_docker`, and the
associated `Dockerfile` and `image/` directory in this folder.  The artifacts
are compiled once by developers and uploaded to drake's `drake-packages` S3
bucket.  When a consumer compiles drake, the `.tar.gz` archive for VTK is
downloaded and extracted as part of the build.  As such, any modifications to
the VTK build should modify the naming scheme of the `.tar.gz` file by updating
the `image/package.sh` file's `build_number` variable.  Never overwrite a build
artifact of the same name on `drake-packages` as doing so will break any users
trying to perform a historical build.

The file `image/vtk-args` contains the full set of CMake configuration arguments
used to produce the `vtk*.tar.gz` binaries that are downloaded and extracted
during a normal build of drake.  This `vtk-args` file should be considered the
"gold standard" where VTK configurations are concerned.  Any differences to this
file for macOS or wheel builds should be documented in their corresponding build
file.

When developing an upgrade to a new VTK version, developers will need to modify:

- `image/build-vtk.sh`: update the version of VTK being downloaded to compile.
- `image/vtk-args`: update any CMake configurations needed.  Pay close attention
  to the docker build during the CMake configure phase.  Each VTK upgrade has
  potential to change, eliminate, or add new CMake configuration variables.  Any
  argument in `vtk-args` that is outdated will be warned about by CMake, along
  the lines of `Manually-specified variables were not used by the project:`.
  These usually indicate that a given CMake variable for VTK has changed.  Until
  you have arrived at your final changes to `vtk-args`, you are encouraged to
  add `-Werror=dev` to the `cmake` invocation in `image/build-vtk.sh`.
- `image/package.sh`: the `vtk_tag` field, which should match `build-vtk.sh`.
- `repository.bzl`: discussed below after the relevant changes for the other
  distributions.
- `image/prereqs`: in the event that additional libraries (or fewer) are needed
  to compile a new VTK version.  Make sure that the corresponding binary
  packages are installed in drake's
  `setup/ubuntu/binary_distribution/packages-*.txt` file.

While developing, the primary files to be editing will be `image/vtk-args` and
`repository.bzl`, using the `build_binaries_with_docker` script to produce the
`vtk*.tar.gz` archives.  As noted above, a typical consumer drake build will be
downloading archives from `drake-packages`, however for development editing the
`tools/workspace/mirrors.bzl` file will be sufficient to instruct `bazel` to
obtain the `vtk*.tar.gz` from an alternate location.  An example change:

```diff
 DEFAULT_MIRRORS = {
     # ... other projects ...
     "vtk": [
+    # NOTE:   ↓↓↓ there must be three slashes.
+        "file:///abs/path/to/drake/tools/workspace/vtk/{archive}",
+    #   "https://drake-packages.csail.mit.edu/vtk/{archive}",
+    #   "https://s3.amazonaws.com/drake-packages/vtk/{archive}",
-        "https://drake-packages.csail.mit.edu/vtk/{archive}",
-        "https://s3.amazonaws.com/drake-packages/vtk/{archive}",
     ],
 }
```

Comment out the existing archive URLs, and add a `file://` URL to the directory
where the artifacts from `build_binaries_with_docker` reside on your local
machine.  You will additionally need to modify `repository.bzl` to update the
`sha256` for the corresponding archives (search `sha256` in that file).

VTK archives for compiling drake hosted at
https://drake-packages.csail.mit.edu/vtk are accessible for 5 years after the
date of creation.  Afterward they are moved to glacier storage on s3 and will
not be available for public download.

### macOS

Compiled artifacts are produced by
[`homebrew-director`](https://github.com/RobotLocomotion/homebrew-director),
which is tapped and the corresponding VTK version installed in
`setup/mac/binary_distribution/Brewfile`.  The associated VTK `.rb` file under
the `Formula` directory in that repository contains the relevant CMake
configurations.  They should match the `image/vtk-args` file in this directory
as closely as possible.  The only differences should be possible external
dependencies available via `brew` that drake does not depend on, and some
`brew` specific items such as `-DCMAKE_INSTALL_RPATH:STRING=#{lib}`.  Any other
differences to `image/vtk-args` should be documented in that file.

Precompiled binaries for `brew` are called "bottles".  When the macOS consumer
of drake runs `setup/mac/install_prereqs.sh`, the precompiled bottles for this
new VTK formula in `homebrew-director` will be downloaded and installed from
the `drake-homebrew` S3 bucket.  Additional bottling instructions are available
in the "Drake Continuous Integration Details" internal document.

Note: when testing your pull request / bottles, make sure to request an
unprovisioned build from the jenkins bot for macOS.

### Pip Wheels (`manylinux`)

The wheel build docker infrastructure and the docker infrastructure in this
folder are as similar as possible, and generally speaking the instructions in
the Ubuntu section above apply here (except for `mirrors.bzl`, which is
irrelevant, and there is no correlated `package.sh`).  The primary differences
between the `image/vtk-args` in this directory and the wheel build:

- The wheel builds are static to avoid linker / `LD_LIBRARY_PATH` wrangling for
  the resultant pip wheels.
- The `CMAKE_CXX_FLAGS` found in `image/vtk-args` in this directory, for example
  `-D_FORTIFY_SOURCE=2`, are excluded as they introduce additional complexity
  for the wheel build that is not desirable.  Those flags are used in
  `image/vtk-args` in this directory to mirror what `bazel` compiles the rest
  of drake with, which does not apply to the wheel builds.
- The `vtk-args` for the wheel builds request that VTK builds more of its
  external dependencies as part of its build.  Any external dependency for the
  wheel builds must be packaged in `tools/wheel/image/dependencies/projects`.
  If an external dependency of VTK is already packaged there, it is reused.
  Otherwise, since the wheel builds are static, the configurations there request
  that VTK builds it internally.

Any discrepancies between `image/vtk-args` in this directory and
`tools/wheel/image/vtk-args` should be documented at the top of the file.

## `repository.bzl`

This file enumerates all of the header files and VTK libraries consumed directly
or indirectly (transitively) by drake.  There are two kinds of libraries: a
"private VTK library" (something vendored by VTK as part of its build, e.g.,
`vtksys`), or a "public VTK library".

### Private VTK Libraries

For each library, it is necessary to supply the appropriate dependencies as well
as for `manylinux` (wheel) supply the appropriate linker flags (e.g., add `-ldl`
and/or `-lpthread`).  Each internal / private library from VTK is all lower
case, which makes it easy to distinguish them from the public VTK libraries.
Assuming you have built a `vtk*.tar.gz` archive using
`build_binaries_with_docker`, and extracted that archive in a directory called
`vtk_extract`, you can create the following script to help identify what the
`deps` of a given `_vtk_cc_library` call will be:

```py
from pathlib import Path
import subprocess

this_file_dir = Path(__file__).parent.absolute()

# NOTE: update "vtk_extract" to wherever you have extracted the VTK .tar.gz.
lib_dir = this_file_dir / "vtk_extract" / "lib"
for lib in sorted(lib_dir.glob("*.so")):
    if lib.name.lower() == lib.name:
        vsep = "*" * 44
        print(f"{vsep}\n* {lib.name}\n{vsep}")
        subprocess.run(["ldd", str(lib)])
```

The private VTK libraries should not include any `hdrs` in their call to
`_vtk_cc_library`.  The `vtkkwiml` library is the only exception, its headers
are needed.  The above script will help you identify what the `deps` are.
Additionally, as you can see in the `repository.bzl`, any time that libdl or
pthread show up in the output for `ldd`, a switch is needed:

```py
if os_result.is_manylinux:
    file_content += _vtk_cc_library(
        os_result,
        "vtkloguru",
        linkopts = ["-ldl", "-pthread"],
    )
else:
    file_content += _vtk_cc_library(os_result, "vtkloguru")
```

### Public VTK Libraries

The majority of your time and effort when updating to a new VTK version will be
spent identifying any new or changed libraries that drake consumes directly.
The library names and targets do not usually change very often (except for
possibly the introduction of a new library or changed internal dependencies),
but you should expect that the header files will change or possibly be deleted.
The `_vtk_cc_library` command will by default mark any library that does not
have `hdrs` defined as "private visibility", meaning other packages within drake
will not be able to link directly against this.  Conversely, the libraries that
are currently being used elsewhere in drake will need to have the header files
that are `#include`ed (directly, and transitively).

What you will need to update are the `hdrs` and `deps` for each public library.
To see what the VTK module dependencies are, you can inspect VTK's source
tree.  For example, for `vtkCommonCore` and `vtkCommonDataModel`:

- `VTK/Common/Core/vtk.module`
- `VTK/Common/DataModel/vtk.module`

Make sure to include the sections from both `DEPENDS` and `PRIVATE_DEPENDS`.
The vtk.module file will help determine what the `deps` field for a given
library should be.  Where header files are concerned, a bit of manual effort is
required to determine which header files to include.  You can obtain a full list
of header files a given VTK module produces by running the `find_vtk_headers.py`
file.  This full list can be used to determine which libraries drake is actually
consuming.  You can obtain the current list of header files drake uses by:

```console
# invoke just the `grep` command to see which files.
$ grep -rH '#include <vtk.*\.h>' . | \
    sed 's/.*\(#include.*\)/\1/g' | sort | uniq > headers.txt
```

You can also find all of the locations where a drake component depends on a
vtk library by

```console
# Invoke just the `grep` command to see which files.
$ grep -rH '@vtk//' . | \
    sed 's#.*\(@vtk//.*\)#\1#g' | sort | uniq > deps.txt
```

Compare headers with the list(s) produced by `find_vtk_headers.py` to determine
which VTK modules are needed _directly_ by drake.  With your list of direct
dependencies, the `vtk.module` dependencies will enumerate the indirect
dependencies of libraries consumed by drake.

While there may be a more direct way to solve which transitive headers are
needed (e.g., with IWYU), the header list for each `_vtk_cc_library` call
started by enumerating every header available (commented out) and
iteratively re-compiling / un-commenting a given header file until all
compilation errors were resolved.  Then delete all remaining commented out
header files.  You can begin with `bazel build //geometry/render/...` which
currently contains a large subset of the VTK usage within drake, but you will
need to compile other targets such as `bazel build //manipulation/util:stl2obj`
and eventually `bazel build //...`.

When you believe that you have the final list of libraries updated, you must
run the tests.  To begin, the core tests that will affect your work are going
to be `bazel test //geometry/render/...`, but you will eventually need to make
sure that `bazel test //...` works as expected (for testing installation).
The VTK test tags are defined in `tools/skylark/test_tags.bzl:vtk_test_tags()`,
currently you can run

```console
$ bazel test \
    --test_tag_filters=no_drd,no_helgrind,no_lsan,no_memcheck,no-sandbox \
    //...
```

or to test the install logic specifically:

```console
$ bazel test //:py/install_test
```

## Bisecting VTK

During the upgrade, you may have failing tests particularly from
`bazel test //geometry/render/...` that may not be obvious to fix.  Since you
will likely be upgrading against a significant amount of changes within VTK, the
best approach will be to bisect VTK by temporarily manipulating the build system
in drake to point to an externally compiled (by you) source tree of VTK.  To
begin, you will need to update `repository.bzl` to point to the right locations.
Since some of the library dependencies or names may change, you may need to
temporarily update `*.bzl` files outside of the `tools/workspace/vtk` folder,
address those as needed.  For updates to `repository.bzl`,

1. In the `_vtk_cc_library` function, it may serve to your advantage to just
   make every library publicly visible (related to temporary updates outside of
   `tools/workspace/vtk`).

    ```diff
    + visibility = ["//visibility:public"]
    +
      if not deps:
          deps = []
    ```

2. In the `_vtk_cc_library` function, you might need to update to use linker
   options rather than specify srcs. In the past, we've used this change while
   bisecting, but it might not have been strictly required.

    ```diff
      elif os_result.is_ubuntu:
          if not header_only:
    -         srcs = ["lib/lib{}-{}.so.1".format(name, VTK_MAJOR_MINOR_VERSION)]
    +         lib_dir = "/abs/path/to/custom/vtk/install/lib"
    +         linkopts = linkopts + [
    +             "-L{}".format(lib_dir),
    +             "-l{}-{}".format(name, VTK_MAJOR_MINOR_VERSION)
    +         ]
    ```

3. In the `_impl(repository_ctx)` function, you will need to comment out the
   ubuntu section that sets `archive`, `sha256`, and calls
   `repository_ctx.download_and_extract`.  Replace it with something like

    ```py
    inc_dir = "/abs/path/to/custom/vtk/install/include"
    repository_ctx.symlink(inc_dir, "include")
    ```

4. At the top of `repository.bzl` you will need to update
   `VTK_MAJOR_MINOR_VERSION` (as it is used in the linker options from (2)).
   As you bisect, the VTK major/minor version can be found most easily by

    ```console
    $ find /abs/path/to/custom/vtk/lib -name "*.so"
    ```

    and examine the library suffixes.  You may see `libvtkFiltersPoints-8.90.so`
    (so `VTK_MAJOR_MINOR_VERSION = "8.90"`), or `libvtkFiltersPoints-9.0.so`
    (so `VTK_MAJOR_MINOR_VERSION = "9.0"`).

5. As you bisect, you may need to comment out `_vtk_cc_library` calls for
   private internal vtk libraries (such as `vtkmft`) and their corresponding
   usage in `deps` of other libraries (so comment out `":vtkmft"`).

6. Similarly, different header files may not be found, or new files that are
   not currently enumerated in `repository.bzl` may be required.  For header
   files reported as not found by `bazel`, simply comment them out.  For header
   files that are needed but not listed yet, the fastest solution to make them
   available to every location is by adding them to the `hdrs` of
   `"vtkCommonCore"`.

7. You will probably need to comment out the `fail()` clause in
   `_check_licenses_impl` the file `tools/install/check_licenses.bzl`.

8. Until your work is complete, you will be best served to keep your bisected
   builds / installation trees around.  If you need to bisect a second time,
   since you will be starting with the same `good` and `bad` commits, you will
   be able to reuse them until the bisect diverges.

A successful workflow was to have the following directory structure (outside of
your drake repository):

```
vtk-bisect/
├── rebuild.sh
├── vtk-build
├── vtk-installs
└── vtk-src
```

where

- `vtk-src` is a git clone of the VTK source tree.  This is where you will be
  bisecting.
- `vtk-build` is just a scratch build directory.
- `vtk-installs` is where each bisected commit is getting installed.
- `rebuild.sh` is a helper script to aid you in producing the installations,
  assuming you have `vtk-src` as your clone directory.  Its contents are below:

```bash
#!/usr/bin/env bash

set -x
# Assumes you are always running `./rebuild.sh`!  Updated accordingly...
here="$(pwd)"
# Wherever your `git clone` of the VTK source lives.
src="$here/vtk-src"
# As you bisect, each commit will produce different results for `git describe`.
inst="$here/vtk-installs/$(git -C "$src" describe)"
if ! [[ -d "$inst" ]]; then
    echo "I am building"
    rm -rf vtk-build
    mkdir vtk-build
    cd vtk-build
    # If desired, you can update the below to also add whatever is in
    # image/vtk-args to reduce the total size of the build by turning off
    # various dependencies or unnecessary VTK components.
    cmake -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=$inst \
        -DVTK_BUILD_TESTING=OFF \
        $src
    ninja install
else
    echo "DONE: $inst already exists!"
fi
```

With the infrastructure in place, you will need to begin bisecting.  In one
terminal in `vtk-src`, `git bisect start` and then use `git bisect good vX.Y.Z`
where `vX.Y.Z` is the VTK git tag that you are updating away from (what drake is
currently using).  Now `git bisect bad vA.B.C` where `vA.B.C` is the VTK git tag
that you are upgrading drake to.  The `git bisect` will then put you at some
specific commit to begin your work.  Now that the bisect has started, the
process works as follows:

1. Run `rebuild.sh` to produce a VTK installation of the current commit under
   inspection.

2. Update `repository.bzl` according to the instructions above to change the
   `VTK_MAJOR_MINOR_VERSION`, `lib_dir`, and `inc_dir` to point to the new
   locations.  If you forget which directory you were installing to, recall that
   in the terminal that is performing the `git bisect` you can run
   `git describe` and then look at `vtk-installs` to see which one it is.

3. Run your test or program, noting that you will need to set `LD_LIBRARY_PATH`
   for most scenarios.  If you were failing a specific test in a given test file
   then you could comment out all of the other tests, and then run
   `LD_LIBRARY_PATH=${lib_dir} bazel test //the/test/that:fails` where
   `${lib_dir}` would be the same absolute path you updated `repository.bzl` to
   and `//the/test/that:fails` is the actual test target.  Or, if you are
   inspecting visual results,
   `LD_LIBRARY_PATH=${lib_dir} bazel run //the/target/under:inspection`.

   During this phase is when you will encounter various build errors such as
   not being able to find a library that was created using `_vtk_cc_library`
   (step (5) above, comment it out), or comment out / add header files as needed
   (step (6) above).  Eventually your test or program will run after making the
   requisite modifications.  As you continue the bisect, the number of changes
   needed for libraries / headers will likely decrease.

4. Now that (3) runs, identify whether the test or application behaves correctly
   or not and in the `vtk-src` directory `git bisect good` or `git bisect bad`
   accordingly.  Now `rebuild.sh` using steps 1-4 here again until the process
   is complete.

The process is fairly time consuming as each step requires a rebuild and install
of VTK as well as modifying `LD_LIBRARY_PATH` for `bazel` commands will require
many items to be rebuilt.  Avoid rebuilding more of drake than you need to by
only testing / running the specific target that you need to.  The rest of drake
that depends on VTK will not be rebuilt.

## `drake_visualizer`

During the upgrade to VTK-9, the `tools/workspace/drake_visualizer` folder was
updated to build VTK-8 from source.  Its `Dockerfile` and related infrastructure
is closely related to the infrastructure found in this folder, but will become
irrelevant when `drake_visualizer` support is dropped.  Previously the
`drake_visualizer` build would consume the same `vtk*.tar.gz` archive that drake
used in its build for VTK-8.  As a result, it would contain libraries looking
for eventual linker references to VTK-8 libraries that would come together at
the end of build or install.  The VTK-9 update required keeping VTK-8 available
for `drake_visualizer`, which meant that

1. The `drake_visualizer` build needed to build VTK-8 internally and bundle it.
2. Both VTK-8 and VTK-9 are installed side-by-side with drake.

Where (2) is concerned, the special cases in

- `tools/install/bazel/drake.BUILD.bazel`
- `tools/install/installer.py`

for VTK-8 or "director" (what `drake_visualizer` uses) specific libraries should
be removed.

As part of the testing process for the VTK-9 upgrade, given the side-by-side
VTK-8 and VTK-9 install, it was required to include in the testing process
`rm -rf ${prefix} && bazel run //:install -- ${prefix}`, where `${prefix}` is
any temporary directory to install cleanly upon.  Then make sure that you can
run `${prefix}/bin/drake-visualizer` (to validate that the `LD_LIBRARY_PATH`,
rpath, and `PYTHONPATH` manipulations work correctly) in addition to validating
that `bazel run //tools:drake_visualizer` works.
