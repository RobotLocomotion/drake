"""Fail-fast stub for Drake Visualizer."""

import sys

assert __name__ == "__main__"
print("error: drake-visualizer was disabled in this build of Drake; please try running meldis instead:", file=sys.stderr)  # noqa
print("  https://drake.mit.edu/pydrake/pydrake.visualization.meldis.html", file=sys.stderr)  # noqa
