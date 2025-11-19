# Entry point for the wheel build machinery; run with --help for more details.
# On Linux, wheels are build using Docker. On macOS, the host environment is
# used.

import sys

from tools.wheel.wheel_builder.common import do_main as main

if __name__ == "__main__":
    if sys.platform == "linux":
        from tools.wheel.wheel_builder import linux as platform
    elif sys.platform == "darwin":
        from tools.wheel.wheel_builder import macos as platform
    else:
        platform = None

    main(args=sys.argv[1:], platform=platform)
