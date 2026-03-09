"""
Wrapper Python script to ensure we can execute a C++ binary with access to
Python libraries using an environment established by Bazel.
"""

# TODO(eric.cousineau): See if there is a way to do this in pure C++, such
# that it is easier to debug.

import os
import subprocess
import sys

env = os.environ.copy()
env["PYTHONPATH"] = ":".join([p for p in sys.path if p])

assert len(sys.argv) >= 2
subprocess.check_call(sys.argv[1:], env=env)
