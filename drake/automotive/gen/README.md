
All C++ code in this directory is generated automatically by a script.
To re-generate the code, run ./simple_car_gen.sh in the parent directory:

  $ cd drake-distro/drake/automotive
  $ ./simple_car_gen.sh

Note that the above script searches for environment variable `CLANG_FORMAT` to
determine which version of `clang-format` to use. If this variable is not set,
it will default to a particular version that is hard-coded within the script.
To specify a different version, execute:

    $ env CLANG_FORMAT=clang-format-x.x ./simple_car_gen.sh

where `x.x` matches the version of your choice.

Note that clang-format 3.7.1 will not correctly alphabetize the `#include`
section. Please use at least clang-format 3.8.0.
