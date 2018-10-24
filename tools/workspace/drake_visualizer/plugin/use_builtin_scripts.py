"""Amalgam of visualizer scripts."""

from __future__ import print_function

from collections import OrderedDict
import os
import sys

from drake.tools.workspace.drake_visualizer.plugin import (
    contact,
    frame,
    image,
    time,
    scoped_singleton_func,
)


@scoped_singleton_func
def init_visualizer():
    available = OrderedDict((
        ("contact", contact.init_visualizer),
        ("frame", frame.init_visualizer),
        ("time", time.init_visualizer),
        ("image", image.init_visualizer),
    ))
    print("")
    print("Drake Scripts:")
    scripts_raw = os.environ["_DRAKE_VISUALIZER_BUILTIN_SCRIPTS"]
    if scripts_raw == "all":
        scripts = available.keys()
        print("  Specified: <all - using available>")
    else:
        scripts = []
        for script in scripts_raw.split(","):
            script = script.strip()
            if not script:
                continue
            if script not in available:
                print("\nWARNING: Invalid script: {}\n".format(script))
                continue
            scripts.append(script)
        print("  Specified: --use_builtin_scripts={}".format(
            ",".join(scripts)))
    print("  Available: --use_builtin_scripts={}".format(
        ",".join(available.keys())))
    print("")
    results = OrderedDict()
    for script in scripts:
        results[script] = available[script]()
    return results


# Activate the plugin if this script is run directly; store the results to keep
# the plugin objects in scope.
if __name__ == "__main__":
    drake_builtin_scripts = init_visualizer()
