# This is a tool used by our package.BUILD.bazel file in a genrule().

import collections
import sys

# Load godotengine's SCons implementation of this feature.
from core import make_binders

# Call the upstream method; its first argument needs to quack like
# "a list of things that each have a .path attribute".
Target = collections.namedtuple("Target", ["path"])
targets = [Target(path=x) for x in sys.argv[1:]]
make_binders.run(targets, None, None)
