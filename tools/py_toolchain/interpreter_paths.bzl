# -*- python -*-

# Default value of interpreter_path used by the py_runtime in the default
# Python toolchain registered on the @platforms//os:linux platform.
LINUX_INTERPRETER_PATH = "/usr/bin/python3"

# Default value of interpreter_path used by the py_runtime in the Python debug
# toolchain registered on the @platforms//os:linux platform when the
# --extra_toolchains=//tools/py_toolchain:linux_dbg_toolchain command line
# option is given.
LINUX_DBG_INTERPRETER_PATH = "/usr/bin/python3-dbg"

# Default value of interpreter_path used by the py_runtime in the default
# Python toolchain registered on the @platforms//os:osx platform when
# running i386 (i.e., x86_64) builds.
MACOS_I386_INTERPRETER_PATH = "/usr/local/bin/python3.11"

# Default value of interpreter_path used by the py_runtime in the default
# Python toolchain registered on the @platforms//os:osx platform when
# running arm64 builds.
MACOS_ARM64_INTERPRETER_PATH = "/opt/homebrew/bin/python3.11"
