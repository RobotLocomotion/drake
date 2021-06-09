"""Amalgam of visualizer scripts."""

from collections import OrderedDict
from functools import partial
import os
import sys

from _drake_visualizer_builtin_scripts import (
    AVAILABLE_SCRIPTS,
    grid_wireframe,
    limit_clipping_range,
    scoped_singleton_func,
    experimental_show_deformable_mesh,
    show_frame,
    show_hydroelastic_contact,
    show_image,
    show_point_cloud,
    show_point_pair_contact,
    show_time,
)


@scoped_singleton_func
def init_visualizer():
    available = OrderedDict((
        ("experimental_deformable_mesh",
            experimental_show_deformable_mesh.init_visualizer),
        ("frame", show_frame.init_visualizer),
        ("hydroelastic_contact", show_hydroelastic_contact.init_visualizer),
        ("image", show_image.init_visualizer),
        ("point_cloud", show_point_cloud.init_visualizer),
        ("point_pair_contact", show_point_pair_contact.init_visualizer),
        ("time", show_time.init_visualizer),
        ("grid_wireframe", grid_wireframe.activate),
        # N.B. `view` is defined globally for scripts executed directly by
        # `director`.
        (
            "limit_clipping_range",
            partial(limit_clipping_range.activate, view=view),
        ),
    ))
    available_scripts = list(available.keys())
    # N.B. Keep these in sync.
    assert available_scripts == AVAILABLE_SCRIPTS, (
        f"Discrepancy between __init__.py` (AVAILABLE_SCRIPTS) (used for "
        f"--help) and those defined in this file (available_scripts):\n"
        f"  AVAILABE_SCRIPTS: {AVAILABLE_SCRIPTS}\n"
        f"  available_scripts: {available_scripts}\n")
    print("")
    print("Drake Scripts:")
    scripts_raw = os.environ["_DRAKE_VISUALIZER_BUILTIN_SCRIPTS"]
    print("  Specified: --use_builtin_scripts={}".format(scripts_raw))
    if scripts_raw == "all":
        scripts = list(available.keys())
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
