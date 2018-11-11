# -*- coding: UTF-8 -*-

import os
import sys

os.environ['LANG'] = 'en_US.UTF-8'

try:
    from sphinx.cmd.build import main
    sys.exit(main(sys.argv[1:]))
except ImportError:
    from sphinx import main
    sys.exit(main(sys.argv))
