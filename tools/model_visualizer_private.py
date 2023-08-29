"""(Internal use only) This program allows Drake developers to visualize model
files used by tests, e.g.:

  bazel run //tools:model_visualizer_private -- \
      package://drake/geometry/render/test/box.sdf

When using a Drake URI (package://drake), the filegroup with the model to be
visualized must be added to either `//:all_models` or `//:some_test_models`
in the top-level BUILD.bazel file.
"""

from pydrake.visualization.model_visualizer import _main

_main()
