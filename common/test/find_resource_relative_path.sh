# This test should only be invoked via Bazel.
set -ex

find .  # Get some debugging output.

capture_cc_env="$1"
find_resource="$2"
find_resource_relative_path_test_cc="$3"

# Make sure we know what C++ compiler to use.
source "$capture_cc_env"
[[ ! -z "$BAZEL_CC" ]]
[[ -x "$BAZEL_CC" ]]
"$BAZEL_CC" --version

# TODO(#6996) Do this unconditionally once #6996 is fully merged.
if [[ ! -e ./drake ]]; then
  # Mimic `include_prefix = "drake"` per drake_cc_library's default.
  ln -s . drake
fi

# Compiles a test with a relative path to `libfind_resource.so`
OBJ="$TEST_TMPDIR/find_resource_relative_path_test_cc.o"
CXXFLAGS="-std=c++1y -I . -I external/stx/include \
    -c $find_resource_relative_path_test_cc \
    -o $OBJ"
"$BAZEL_CC" $CXXFLAGS
LDFLAGS="-ldl -lgflags -lstdc++ -lm -B/usr/bin -B/usr/bin"

if [[ "$OSTYPE" == "linux-gnu" ]]; then
  LDFLAGS="$LDFLAGS -fuse-ld=gold -Wl,-rpath=$(dirname $find_resource) \
      -Wl,--gc-sections -pthread"
fi

EXEC="$TEST_TMPDIR/find_resource_relative_path_test"
"$BAZEL_CC" -o $EXEC \
    $OBJ \
    $find_resource \
    $LDFLAGS

if [[ "$OSTYPE" == "darwin"* ]]; then
  # Force path to library to be relative. It doesn't matter on MacOS as the C
  # function that is used in `find_loaded_library()` is always absolute, but it
  # helps to verify that it is indeed the case.
  libfind_resource_path=$(otool -L $EXEC | grep $(basename $find_resource) | sed 's/(.*//')
  install_name_tool -change ${libfind_resource_path} common/$(basename $find_resource) $EXEC
  otool -L $EXEC
fi
# Run actual test that verifies that path to library is absolute
$EXEC $(basename $find_resource)
