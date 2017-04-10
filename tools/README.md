
This directory contains build system files related to the Bazel build system.
  https://bazel.build/

Files named `*.BUILD` are Drake-specific build rules for external libraries or
tools that do not natively support Bazel.
  https://bazel.build/versions/master/docs/external.html#depending-on-non-bazel-projects

Files named `*.bzl` are Skylark extensions.
  https://bazel.build/versions/master/docs/skylark/concepts.html

`drake.cps` is the Common Package Specification for Drake, that provides the
necessary information for Drake to be consumed by other projects. Right now it
is hand edited; the list of includes should match the list of folders in
package_drake.sh's include/external folder. This is also used to generate
`drake-config.cmake` via `cps2cmake`.
  https://mwoehlke.github.io/cps/
  https://github.com/mwoehlke/pycps/

See `drake/doc/bazel.rst` for additional Drake-specific information.
