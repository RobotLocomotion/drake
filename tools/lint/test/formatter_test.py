import unittest

from drake.tools.lint.formatter import FormatterBase, IncludeFormatter


class TestFormatterBase(unittest.TestCase):

    def test_essentials(self):
        original_lines = [
            '// Line 1\n',
            '/* Line 2 */\n',
            '\n',
        ]
        dut = FormatterBase('filename.cc', readlines=original_lines)

        # Everything starts out unchanged.
        self.assertTrue(dut.is_same_as_original())
        self.assertTrue(dut.is_permutation_of_original())
        self.assertEqual(dut.get_all_lines(), original_lines)
        self.assertTrue(dut.get_first_differing_original_index() is None)

        # Basic getters.
        self.assertEqual(dut.get_num_lines(), 3)
        self.assertTrue(dut.is_blank_line(2))
        self.assertEqual(dut.get_line(0), '// Line 1\n')

        # Reverse it and end up with a permutation.
        dut.set_all_lines(reversed(dut.get_all_lines()))
        self.assertFalse(dut.is_same_as_original())
        self.assertTrue(dut.is_permutation_of_original())
        self.assertEqual(dut.get_first_differing_original_index(), 0)

        # Rebuild it using insertion and removal.
        dut.set_all_lines(['\n'] * 3)
        dut.set_line(0, '/* Line 2 */\n')
        dut.insert_lines(0, ['AAA\n', '// Line 1\n'])
        dut.remove_all([0, 3])
        self.assertEqual(dut.get_all_lines(), original_lines)

    def test_format_ranges(self):
        original_lines = [
            '#include "line0"\n',
            '// clang-format off\n',
            '#include "line2"\n',
            '// clang-format on\n',
            '#include "line4"\n',
            '#include "line5"\n',
            '/* clang-format off */\n',
            '#include "line7"\n',
            '#include "line8"\n',
            '/* clang-format on */\n',
            '#include "line10"\n',
        ]
        dut = FormatterBase("filename.cc", readlines=original_lines)

        self.assertEqual(
            dut.get_format_ranges(), [[0], [4, 5], [10]])
        self.assertEqual(
            dut.get_non_format_ranges(), [[1, 2, 3], [6, 7, 8, 9]])

    def test_dos(self):
        original_lines = [
            '#include "line0"\r\n',
        ]
        with self.assertRaisesRegex(Exception, "DOS newline"):
            FormatterBase("filename.cc", readlines=original_lines)

    def test_missing_eof(self):
        original_lines = [
            '#include "line0"',
        ]
        with self.assertRaisesRegex(Exception, "newline.*end of file"):
            FormatterBase("filename.cc", readlines=original_lines)


class TestIncludeFormatter(unittest.TestCase):

    def _split(self, triple_quoted_file_contents):
        lines = triple_quoted_file_contents.split("\n")
        assert len(lines) >= 2
        assert lines[0] == ""  # Detritus from first triple quote.
        assert lines[-1] == ""  # Detritus from last triple quote.
        del lines[0]
        del lines[-1]
        return [line + "\n" for line in lines]

    def _check(self, basename, original, expected, first_differing):
        original_lines = self._split(original)
        expected_lines = self._split(expected)
        dut = IncludeFormatter(
            "drake/dummy/" + basename,
            readlines=original_lines)
        dut.format_includes()
        self.assertEqual(dut.get_all_lines(), expected_lines)
        self.assertEqual(dut.get_first_differing_original_index(),
                         first_differing)

    def test_basic(self):
        # A pile of headers gets sorted per cppguide:
        # - The related header
        # - C system files
        # - C++ system files
        # - Other libraries' .h files
        # - Your project's .h files
        original = """
#include "drake/common/drake_assert.h"
#include "drake/dummy/bar.h"
#include "drake/dummy/dut.h"
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <algorithm>
#include <poll.h>
#include <sys/wait.h>
#include <vector>
"""
        expected = """
#include "drake/dummy/dut.h"

#include <poll.h>
#include <sys/wait.h>

#include <algorithm>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/dummy/bar.h"
"""
        self._check("dut.cc", original, expected, 0)

    def test_nothing(self):
        # A file with _no_ include statements.
        original = """
namespace { }
"""
        self._check("dut.cc", original, original, None)

    def test_regroup(self):
        # Wrongly grouped whitespace.
        original = """
#include "drake/dummy/dut.h"

#include <Eigen/Dense>
#include <algorithm>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/dummy/bar.h"
#include <gtest/gtest.h>
"""
        expected = """
#include "drake/dummy/dut.h"

#include <algorithm>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/dummy/bar.h"
"""
        self._check("dut.cc", original, expected, 2)

    def test_format_off(self):
        # "clang-format off".
        original = """
#include "drake/dummy/dut.h"

// clang-format off
#ifdef FOO
#include <algorithm>
#include <vector>
#else
#include <vector>
#include <algorithm>
#endif
// clang-format on

#include "drake/common/drake_assert.h"
"""
        self._check("dut.cc", original, original, None)

    def test_target_is_header(self):
        # A header file.
        original = """
#include "drake/common/drake_assert.h"
#include <algorithm>

namespace { }
"""
        expected = """
#include <algorithm>

#include "drake/common/drake_assert.h"

namespace { }
"""
        self._check("dut.h", original, expected, 0)

    def test_associated_comment(self):
        # A comment prior to a line.
        original = """
#include "drake/dummy/dut.h"

// Some comment describing the next line.
#include <vector>

namespace { }
"""
        self._check("dut.cc", original, original, None)

    def test_file_opening_comment(self):
        # A comment atop the file with no blank line.
        original = """
/// @file dut.cc
/// Mumble mumble
///
#include <string>
#include <vector>
"""
        self._check("dut.cc", original, original, None)

    def test_internal_related_header(self):
        # Two related headers, guarded by "clang-format off".
        original = """
/* clang-format off (with explanatory comment) */
#include "drake/dummy/dut.h"
#include "drake/dummy/dut_internal.h"
/* clang-format on  (with explanatory comment) */

#include <vector>
#include <string>

#include "drake/dummy/drake_assert.h"
#include "drake/dummy/drake_deprecated.h"
"""
        expected = """
/* clang-format off (with explanatory comment) */
#include "drake/dummy/dut.h"
#include "drake/dummy/dut_internal.h"
/* clang-format on  (with explanatory comment) */

#include <string>
#include <vector>

#include "drake/dummy/drake_assert.h"
#include "drake/dummy/drake_deprecated.h"
"""
        self._check("dut.cc", original, expected, 5)

    def test_resort_solo_groups(self):
        # Groups of one, but sorted incorrectly.
        original = """
#include "drake/dummy/dut.h"

#include "drake/common/drake_assert.h"

#include <vector>
"""
        expected = """
#include "drake/dummy/dut.h"

#include <vector>

#include "drake/common/drake_assert.h"
"""
        self._check("dut.cc", original, expected, 2)

    def test_nontrivial_reformatting(self):
        # If clang-format changes any lines, we want to fail-fast.
        # (Note the two spaces between #include and the double quote.)
        original_lines = ['#include  "nontrivial.h"\n']
        dut = IncludeFormatter("nontrivial.cc", readlines=original_lines)
        dut.format_includes()
        with self.assertRaisesRegex(Exception, 'not just a shuffle'):
            dut.rewrite_file()
