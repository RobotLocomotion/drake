"""Amalgam of visualizer scripts."""

from collections import OrderedDict
import os
import sys

from drake.visualization import (
    director_contact,
    director_frame,
    director_image,
    director_time,
    singleton_func,
)


@singleton_func
def init_visualizer(argv):
    available = OrderedDict((
        ("contact", director_contact.init_visualizer),
        ("frame", director_frame.init_visualizer),
        ("time", director_time.init_visualizer),
        ("image", lambda: director_image.init_visualizer(argv)),
    ))
    print("")
    print("Drake Scripts:")
    scripts_raw = os.environ.get("DRAKE_SCRIPTS")
    if scripts_raw is None:
        scripts = available.keys()
        print("  Specified: <none - using available>")
    else:
        scripts = []
        for script in scripts_raw.split(","):
            script = script.strip()
            if not script:
                continue
            if script not in available:
                print("\nWARNING: Invalid script: {}\n".format(script))
                scripts.remove(script)
            scripts.append(script)
        print("  Specified: --drake_scripts={}".format(",".join(scripts)))
    print("  Available: --drake_scripts={}".format(",".join(available.keys())))
    print("")
    results = OrderedDict()
    for script in scripts:
        results[script] = available[script]()
    return results


if __name__ == "__main__":
    drake_scripts = init_visualizer(_argv)
