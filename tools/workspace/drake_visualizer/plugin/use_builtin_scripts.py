"""Amalgam of visualizer scripts."""

from __future__ import print_function

from collections import OrderedDict
import os
import sys

from drake.tools.workspace.drake_visualizer.plugin import (
    show_contact,
    show_frame,
    show_image,
    show_time,
    scoped_singleton_func,
)


@scoped_singleton_func
def init_visualizer():
    available = OrderedDict((
        ("contact", show_contact.init_visualizer),
        ("frame", show_frame.init_visualizer),
        ("image", show_image.init_visualizer),
        ("time", show_time.init_visualizer),
    ))
    print("")
    print("Drake Scripts:")
    scripts_raw = os.environ["_DRAKE_VISUALIZER_BUILTIN_SCRIPTS"]
    print("  Specified: --use_builtin_scripts={}".format(scripts_raw))
    if scripts_raw == "all":
        scripts = available.keys()
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
