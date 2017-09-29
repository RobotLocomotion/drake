import os.path
import pydrake.common


def getDrakePath():
    pydrake.common.AddResourceSearchPath(
        os.path.realpath(os.path.join(os.path.dirname(__file__), os.pardir,
                         os.pardir, os.pardir, os.pardir, "share/drake"))
        )
    return pydrake.common.GetDrakePath()
