"""Classes to support C++ code formatting tools.
"""

import io
import os
from subprocess import Popen, PIPE, CalledProcessError

import drake.tools.lint.clang_format as clang_format_lib


class FormatterBase:
    """A base class for formatting-related tools, with the ability to load a
    file, add / remove / modify lines, clang-format selected regions, and
    compare the changes to the original file contents.

    This class models a "working list" of the file during reformatting.  Each
    line in the file has an index (starting with index 0), and each line ends
    with a newline (included, not implicit).  Callers can modify the working
    list, and run clang-format on (portions of) the working-list.

    The is_same_as_original and is_permutation_of_original methods compare the
    current working list to the original file contents.  Other methods provide
    access and modification by index.

    The should_format predicate can be overridden in subclasses to change which
    lines are subject to clang-format modifications.
    """

    def __init__(self, filename, readlines=None):
        """Create a new FormatterBase.  The required filename parameter refers
        to the workspace-relative full path, e.g. drake/common/drake_assert.h.

        The readlines parameter is optional and useful mostly for unit testing.
        When readlines is present, readlines becomes the work list and the
        filename is not opened.  When readlines is absent, this constructor
        reads the contents of the filename from disk into the work list (i.e.,
        the default value of readlines is open(filename).readlines()).
        """
        self._filename = filename
        if readlines is None:
            with io.open(filename, "r", encoding="utf8") as opened:
                self._original_lines = opened.readlines()
        else:
            self._original_lines = [str(line) for line in readlines]
        self._working_lines = list(self._original_lines)
        if any(["\r" in line for line in self._working_lines]):
            raise Exception("DOS newlines are not supported")
        if not self._working_lines[-1].endswith("\n"):
            raise Exception("Missing newline character at end of file")
        self._check_rep()

    def _check_rep(self):
        assert self._filename
        for line in self._original_lines:
            assert isinstance(line, str), (type(line), line)
            assert line.endswith("\n"), line
        for line in self._working_lines:
            assert isinstance(line, str), (type(line), line)
            assert line.endswith("\n"), line

    def is_same_as_original(self):
        """Return whether the working list is identical to the original file
        contents read in by (or passed to) the constructor.
        """
        return self._working_lines == self._original_lines

    def get_first_differing_original_index(self):
        """Return the first index from original lines that differs from the
        working lines, or None if no lines differ.  This is a 0-based index,
        not a line number.
        """
        if self.is_same_as_original():
            return None
        min_common_line_count = min(
            len(self._working_lines), len(self._original_lines))
        for i in range(min_common_line_count):
            if self._working_lines[i] != self._original_lines[i]:
                return i
        # One file is a prefix of the other.
        return min_common_line_count

    def is_permutation_of_original(self):
        """Return whether the working list is a permultation of the original
        file lines read in by (or passed to) the constructor, modulo blank
        lines.
        """
        return (
            set(self._working_lines + ["\n"])
            == set(self._original_lines + ["\n"]))

    def is_valid_index(self, index):
        return 0 <= index and index < len(self._working_lines)

    def is_blank_line(self, index):
        return self.get_line(index).strip() == ""

    def get_num_lines(self):
        return len(self._working_lines)

    def get_line(self, index):
        assert self.is_valid_index(index)
        return self._working_lines[index]

    def get_all_lines(self):
        return list(self._working_lines)

    def set_line(self, index, line):
        assert self.is_valid_index(index)
        self._working_lines[index] = str(line)
        self._check_rep()

    def set_all_lines(self, lines):
        self._working_lines = [str(line) for line in lines]
        self._check_rep()

    def insert_lines(self, index, lines):
        assert 0 <= index and index <= len(self._working_lines)
        self._working_lines[index:0] = [str(line) for line in lines]
        self._check_rep()

    def remove_all(self, indices):
        if len(indices) == 0:
            return
        assert len(set(indices)) == len(indices)
        rev_sorted_indices = sorted(indices, reverse=True)
        assert self.is_valid_index(rev_sorted_indices[0])
        assert self.is_valid_index(rev_sorted_indices[-1])
        for index in rev_sorted_indices:
            del self._working_lines[index]

    def should_format(self, clang_format_on, index, line):
        """Subclasses can override to change what's going to be formatted.
        True means the line should be formatted.  The default is the same as
        clang-format -- everything goes except "clang-format off" sections.
        """
        return clang_format_on

    def get_format_indices(self):
        """Return the sorted list of all indices that pass the
        self.should_format predicate.
        """
        result = []
        clang_format_on = True
        for index, line in enumerate(self._working_lines):
            # Ignore lines between clang-format directive comments, and also
            # ignore the lines with the directive comments themselves.
            if '// clang-format off' in line:
                clang_format_on = False
                continue
            if '/* clang-format off' in line:
                clang_format_on = False
                continue
            elif '// clang-format on' in line:
                clang_format_on = True
                continue
            elif '/* clang-format on' in line:
                clang_format_on = True
                continue
            if self.should_format(clang_format_on, index, line):
                result.append(index)
        return result

    def get_non_format_indices(self):
        """Return the complement of get_format_indices().
        """
        all_indices = set(range(len(self._working_lines)))
        return sorted(all_indices - set(self.get_format_indices()))

    @staticmethod
    def indices_to_ranges(indices):
        """Group a list of sorted indices into a sorted list of softed lists of
        adjacent indices.

        For example [1, 2, 5, 7, 8, 9] yields [[1, 2], [5], [7, 8, 9]].
        """
        assert indices == sorted(indices)
        result = []
        for i in indices:
            if len(result) and (result[-1][-1] == (i - 1)):
                # Grow an existing range.
                result[-1] = result[-1] + [i]
            else:
                # Start a new range.
                result.append([i])
        return result

    def get_format_ranges(self):
        return self.indices_to_ranges(self.get_format_indices())

    def get_non_format_ranges(self):
        return self.indices_to_ranges(self.get_non_format_indices())

    def clang_format(self):
        """Reformat the working list using clang-format, passing -lines=...
        groups for only the lines that pass the should_format() predicate.
        """
        # Convert format_ranges to clang's one-based indexing.
        lines_args = ["-lines=%d:%d" % (one_range[0] + 1, one_range[-1] + 1)
                      for one_range in self.get_format_ranges()]
        if not lines_args:
            return

        # Run clang-format.
        command = [
            clang_format_lib.get_clang_format_path(),
            "--style=file",
            "--assume-filename=%s" % self._filename] + \
            lines_args
        formatter = Popen(command, stdin=PIPE, stdout=PIPE)
        text = "".join(self._working_lines)
        stdout, _ = formatter.communicate(input=text.encode("utf8"))
        stdout = stdout.decode("utf8")

        # Handle errors, otherwise reset the working list.
        if formatter.returncode != 0:
            raise CalledProcessError(formatter.returncode, command, stdout)
        self.set_all_lines(stdout.splitlines(True))

    def _pre_rewrite_file(self):
        """Subclasses may override to perform actions prior to rewrite_file."""
        pass

    def rewrite_file(self):
        """Overwrite the contents of the filename passed into the constructor
        with the current working list.
        """
        self._pre_rewrite_file()
        temp_filename = self._filename + ".drake-formatter"
        with io.open(temp_filename, "w", encoding="utf8") as opened:
            for line in self._working_lines:
                opened.write(line)
        os.rename(temp_filename, self._filename)


class IncludeFormatter(FormatterBase):
    """A formatter tool that provides format_includes() helper to sort the
    #include statements to match Google C++ Style Guide, as well as put a
    blank line between each of the #include group sections.

    """

    def __init__(self, filename, *args, **kwargs):
        super(IncludeFormatter, self).__init__(filename, *args, **kwargs)
        self._related_headers = []

        # In most cases, clang-format has a built-in IncludeIsMainRegex that
        # identifies the "related header" correctly, automatically.  For an
        # inl-h file, clang will not realize the .h is "related", so instead we
        # manually compute that the related-header pathname here.
        if filename.endswith('-inl.h'):
            prior_to_ext = filename[:-len('-inl.h')]
            self._related_headers.append(u'#include "%s.h"\n' % prior_to_ext)

    def _pre_rewrite_file(self):
        # Sanity check before writing out again.
        if self.is_permutation_of_original():
            return
        message = ""
        message += "%s: changes were not just a shuffle\n" % self._filename
        message += "=" * 78 + "\n"
        message += "".join(self._original_lines)
        message += "=" * 78 + "\n"
        message += "".join(self._working_lines)
        message += "=" * 78 + "\n"
        raise Exception(message)

    def should_format(self, clang_format_on, index, line):
        # We want all include statements, but until clang-format gets
        # IncludeIsMainRegex support, we need omit the "related header"s.
        return (
            clang_format_on
            and line.startswith("#include")
            and '-inl.h"' not in line
            and line not in self._related_headers)

    def format_includes(self):
        """Reformat the #include statements in the working list (possibly also
        changing around some nearby blank lines).
        """
        # If there are not any includes to format, then we are done.
        if not self.get_format_indices():
            return

        # Remove blank lines that are fully surrounded by include statements
        # (or bracketed by include statements and start- or end-of-file).  We
        # will put back blank lines later, but in the meantime we need all of
        # the includes bunched together so that clang-format will sort them.
        # We want to preserve the comment line layouts near include statements,
        # so here only _completely_ empty ranges are removed.
        blank_indices_to_remove = []
        for one_range in self.get_non_format_ranges():
            if all([self.is_blank_line(index) for index in one_range]):
                blank_indices_to_remove.extend(one_range)
        self.remove_all(blank_indices_to_remove)

        # Add priority spacers after every group of #include statements.  We
        # will turn these spacers into whitespace separators when we're done.
        SPACERS = [u"#include <clang-format-priority-%d>\n" % priority
                   for priority in [15, 25, 35, 45]]
        for one_range in reversed(self.get_format_ranges()):
            last_include_index = one_range[-1]
            self.insert_lines(last_include_index + 1, SPACERS)

        # Run the formatter over only the in-scope #include statements.
        self.clang_format()

        # Undo any internal whitespace buffer that clang-format added around
        # each group of include statements.
        include_indices = self.get_format_indices()
        blank_indices_to_remove = []
        for i, j in zip(include_indices, include_indices[1:]):
            # If a pair of include statements spans a blank line, remove it.
            if i + 2 == j and self.get_line(i + 1) == "\n":
                blank_indices_to_remove.append(i + 1)
        self.remove_all(blank_indices_to_remove)

        # Turn each run of spacers within an include block into a blank line.
        # Remove any runs of spaces at the start or end.
        include_indices = self.get_format_indices()
        spacer_indices_to_remove = []
        spacer_ranges = self.indices_to_ranges([
            i for i in include_indices
            if self.get_line(i) in SPACERS])
        for one_range in spacer_ranges:
            before_spacer = one_range[0] - 1
            after_spacer = one_range[-1] + 1
            if all([x in include_indices
                    for x in [before_spacer, after_spacer]]):
                # Interior group of spacers.  Replace with a blank line.
                self.set_line(one_range[0], "\n")
                one_range = one_range[1:]
            spacer_indices_to_remove.extend(one_range)
        self.remove_all(spacer_indices_to_remove)
