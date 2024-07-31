# Prints a deprecation notice and forwards to the equivalent pydrake script.
# TODO(jwnimmer-tri) Remove this stub on or around 2025-01-01.

from pydrake.visualization.model_visualizer import _main


if __name__ == '__main__':
    print("** Note: This script is deprecated. **")
    print("Please switch to //tools:model_visualizer when convenient.")
    _main()
