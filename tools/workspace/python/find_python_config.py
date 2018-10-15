"""Tries to find the appropriate `python-config` binary given the current
Python interpreter.

We can't use the generic `python-config` script from `virtualenv` because
Ubuntu decides to do different things in how it handles things.
"""

from __future__ import print_function

import os
from os.path import isfile, join
import subprocess
import sys
import sysconfig

assert __name__ == "__main__"

version = sys.version_info
candidates = [
    join(sys.prefix, "bin/python{}.{}-config".format(
        version.major, version.minor)),
    join(sys.prefix, "bin/python{}-config".format(version.major)),
    join(sys.prefix, "bin/python-config"),
]

configdir = sysconfig.get_config_var('LIBPL')
# Search until we have a candidate that has a mathcing configdir.
debug_msg = ""
for candidate in candidates:
    if not isfile(candidate):
        debug_msg += "Not a file: {}\n".format(candidate)
        continue
    if not os.access(candidate, os.X_OK):
        debug_msg += "Not executable: {}\n".format(candidate)
    candidate_configdir = subprocess.check_output(
        [candidate, "--configdir"]).decode("utf8").strip()
    if configdir != candidate_configdir:
        debug_msg += "Mismatching configdir:\n  {}: {}\n  {}: {}\n".format(
            sys.executable, configdir, candidate, candidate_configdir)
        continue
    break
else:
    print((
        "Could not find a matching `python-config` candidate for "
        "`{}`!\n\n{}").format(sys.executable, debug_msg))
    sys.exit(1)

print(candidate)
