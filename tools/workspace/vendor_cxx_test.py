import unittest

from drake.tools.workspace.vendor_cxx import _rewrite_one_text


class TestVendorCxx(unittest.TestCase):

    def setUp(self):
        # Display all test output.
        self.maxDiff = None

        # These are the include paths that we want to rewrite.
        self._edit_include = {
            'somelib/': 'drake_vendor/somelib/',
        }

        # These are the boilerplate lines that vendor_cxx injects.
        self._open = 'inline namespace drake_vendor __attribute__ ((visibility ("hidden"))) {'  # noqa
        self._close = '}  /* inline namespace drake_vendor */'

    def _check(self, old_lines, expected_new_lines):
        """Tests one call to _rewrite_one_text for expected output."""
        old_text = '\n'.join(old_lines) + '\n'
        new_text = _rewrite_one_text(
            text=old_text, edit_include=self._edit_include.items())
        expected_new_text = '\n'.join(expected_new_lines) + '\n'
        self.assertMultiLineEqual(expected_new_text, new_text)

    def test_simple(self):
        self._check([
            '#include "somelib/somefile.h"',
            '#include "unrelated/thing.h"',
            'int foo();',
        ], [
            # Adjacent pairs of open/close are useless; ideally, we teach
            # vendor_cxx to skip these.
            self._open, self._close,  # TODO(jwnimmer-tri) Useless filler.

            # All include statements must NOT be within an open-namespace.
            '#include "drake_vendor/somelib/somefile.h"',

            self._open, self._close,  # TODO(jwnimmer-tri) Useless filler.
            '#include "unrelated/thing.h"',

            # All code MUST be within an open-namespace.
            self._open,
            'int foo();',
            self._close,
        ])


assert __name__ == '__main__'
unittest.main()
