# -*- python -*-

def hermetic_python_env():
    # In general, we do not want to use Python's "user site-packages"
    # (e.g., $HOME/.local) directory because it's not hermetic.  Thus,
    # we set PYTHONNOUSERSITE to disable the user site-packages.
    #
    # However, our macOS setup instructions provide for some dependencies
    # (e.g., PyYAML) to come from pip, and in some reasonable configurations it
    # could be done via `pip install --user` and so be part of $HOME.  Thus, in
    # order to support that configuration, we only set PYTHONNOUSERSITE under
    # linux.  We can revisit this decision if we changes how python packages on
    # macOS are brought into the workspace.
    #
    # If https://github.com/bazelbuild/bazel/issues/4939 gets fixed, we can
    # revisit whether manually specifying a hermetic env is still necessary.
    return select({
        "@drake//tools/skylark:linux": {
            "PYTHONNOUSERSITE": "1",
        },
        "//conditions:default": {},
    })
