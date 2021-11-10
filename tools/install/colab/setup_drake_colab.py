"""
This installer shim is deprecated and will be removed from Drake on or after
2022-03-01.

The intention of this file was to support a small preamble that can go into the
first cell of any jupyter notebook that allows users to easily provision the
Colab server with drake iff the notebook is being run on Colab, but which fails
fast when run on a local machine.
"""

import sys


def setup_drake(*, version, build):
    assert 'google.colab' in sys.modules, (
        "This script is intended for use on Google Colab only.")
    raise RuntimeError(
        "The setup_drake_colab installer shim has not worked correctly ever"
        " since Google Colab upgraded to Python 3.7.  Instead of calling this"
        " function, please just use '%pip install drake' in your notebook.")
