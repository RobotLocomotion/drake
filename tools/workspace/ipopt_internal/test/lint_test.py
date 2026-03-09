from pathlib import Path
import unittest

from python import runfiles


class IpoptLintTest(unittest.TestCase):
    def _read(self, respath):
        """Returns the contents of the given resource path."""
        manifest = runfiles.Create()
        path = Path(manifest.Rlocation(respath))
        return path.read_text(encoding="utf-8")

    def _parse_build(self, varname):
        """Parses a constant list of filenames from a BUILD file.
        The only supported format is like this:

        {varname} = [
            "path/to/file1",
            "path/to/file2",
        ]
        """
        result = []
        contents = self._read(
            "drake/tools/workspace/ipopt_internal/package.BUILD.bazel"
        )
        lines = contents.splitlines()
        start_line = f"{varname} = ["
        end_line = "]"
        start_index = lines.index(start_line)
        end_index = lines.index(end_line, start_index + 1)
        for i in range(start_index + 1, end_index):
            line = lines[i]
            prefix = '    "'
            suffix = '",'
            self.assertTrue(line.startswith(prefix), line)
            self.assertTrue(line.endswith(suffix), line)
            result.append(line[len(prefix) : -len(suffix)])
        return set(result)

    def _parse_make(self, varname, *, guard_line=None):
        """Parses a constant list of filenames from Makefile.am.

        The only supported formats are like this (with no `guard_line`):

        {varname} = \
          path/to/file1 \
          path/to/file2

        Or this (with a `guard_line`):

        {guard_line}
        {varname} += \
          path/to/file1 \
          path/to/file2
        """
        result = []
        contents = self._read("ipopt_internal/src/Makefile.am")
        lines = contents.splitlines()
        if guard_line is None:
            start_line = f"{varname} = \\"
            index = lines.index(start_line) + 1
        else:
            index = lines.index(guard_line) + 1
            self.assertEqual(lines[index], f"{varname} += \\")
            index += 1
        while True:
            line = lines[index]
            has_slash = line.endswith("\\")
            if has_slash:
                line = line[:-1]
            result.append("src/" + line.strip())
            index += 1
            if not has_slash:
                break
        return set(result)

    def test_hdrs_public(self):
        """Checks that _HDRS_PUBLIC matches includeipopt_HEADERS."""
        make_sources = self._parse_make("includeipopt_HEADERS")
        make_sources.remove("src/Interfaces/IpReturnCodes.inc")
        self.assertSetEqual(self._parse_build("_HDRS_PUBLIC"), make_sources)

    def test_srcs_initial(self):
        """Checks that _SRCS_INITIAL matches libipopt_la_SOURCES, except for
        two specific unwanted sources.
        """
        make_sources = self._parse_make("libipopt_la_SOURCES")
        make_sources.remove("src/Interfaces/IpStdCInterface.cpp")
        make_sources.remove("src/Interfaces/IpStdFInterface.c")
        self.assertSetEqual(self._parse_build("_SRCS_INITIAL"), make_sources)

    def test_srcs_solver_int32(self):
        """Checks that _SRCS_SOLVER_INT32 matches !IPOPT_INT64's effect."""
        self.assertSetEqual(
            self._parse_build("_SRCS_SOLVER_INT32"),
            self._parse_make(
                "libipopt_la_SOURCES", guard_line="if !IPOPT_INT64"
            ),
        )
