# Prints a deprecation notice and forwards to the equivalent pydrake script.

from pydrake.visualization.model_visualizer import _main


if __name__ == '__main__':
    print("** Note: This script is deprecated. **")
    print("Please switch to //tools:model_visualizer when convenient.")
    _main()
