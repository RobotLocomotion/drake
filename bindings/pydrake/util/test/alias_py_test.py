import unittest
import warnings

from pydrake.common.deprecation import DrakeDeprecationWarning


class TestAliasPy(unittest.TestCase):
    def test_import(self):
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("once", DrakeDeprecationWarning)
            # Ensure we can import a subset of symbols from old paths.
            from pydrake.util.compatibility import maybe_patch_numpy_formatters
            from pydrake.util.containers import EqualToDict
            from pydrake.util.cpp_const import ConstError
            from pydrake.util.cpp_param import get_param_canonical
            from pydrake.util.cpp_template import TemplateBase
            from pydrake.util.deprecation import deprecated
            from pydrake.util.eigen_geometry import Quaternion
            from pydrake.util.pybind11_version import repository
            from pydrake.util.all import Quaternion
            self.assertEqual(len(w), 8)
