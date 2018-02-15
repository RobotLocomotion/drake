# Refer to ../../BUILD.bazel comments for rationale.

import os


_is_stub = True


def getDrakePath():
    # Because //tools:drake_visualizer is a drake_runfiles_binary, the correct
    # answer to getDrakePath is always the runfiles root.  Nice!
    return os.environ["DRAKE_BAZEL_RUNFILES"]
