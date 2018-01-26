# This is a tool used by our package.BUILD.bazel file in a genrule().

import os
import sys

# Load godotengine's SCons implementation of this feature.
# See https://github.com/godotengine/godot/blob/master/methods.py.
from methods import update_version

# Turn the path "genfiles/external/godotengine/core/version_generated.gen.h"
# into just the base "genfiles/external/godotengine".
outs = sys.argv[1:]
suffix = "/core/version_generated.gen.h"
assert outs[0].endswith(suffix)
base = outs[0][:-len(suffix)]

# Call the upstream method, from the directory that it expects to see.
os.chdir(base)
if not os.path.isdir("core"):
    os.mkdir("core")
update_version()
