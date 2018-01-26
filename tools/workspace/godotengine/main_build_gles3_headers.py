# This is a tool used by our package.BUILD.bazel file in a genrule().

import os
import shutil
import sys

# Load godotengine's SCons implementation of this feature.
# See https://github.com/godotengine/godot/blob/master/methods.py.
from methods import build_gles3_headers

# From our BUILD rule, the first argument is an *.h that build_gles3_headers
# needs to read in.  Since the path is hard-coded in build_gles3_headers, we
# can discard it here.
_ = sys.argv.pop(1)

# The rest of our arguments are the *.glsl sources, and the *.gen.h outputs.
srcs_and_outs = sys.argv[1:]
assert (len(srcs_and_outs) % 2) == 0
num_srcs = len(srcs_and_outs) // 2
srcs = srcs_and_outs[:num_srcs]
outs = srcs_and_outs[num_srcs:]

# Turn the path "external/godotengine/drivers/gles3/shaders/blend_shape.glsl"
# into just the base "external/godotengine".
suffix = "/drivers/gles3/shaders/blend_shape.glsl"
assert srcs[0].endswith(suffix)
base = srcs[0][:-len(suffix)]

# Call the upstream method, from the directory that it expects to see.
paths = [os.path.relpath(src, base) for src in srcs]
old_cwd = os.getcwd()
os.chdir(base)
build_gles3_headers(None, paths, None)
os.chdir(old_cwd)

# Move its outputs back into genfiles.
for src, out in zip(srcs, outs):
    shutil.move(src + ".gen.h", out)
