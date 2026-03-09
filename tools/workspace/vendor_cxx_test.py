import unittest

from tools.workspace.vendor_cxx import _rewrite_one_text


class TestVendorCxx(unittest.TestCase):
    def setUp(self):
        # Display all test output.
        self.maxDiff = None

        # These are the boilerplate lines that vendor_cxx injects.
        self._open = 'inline namespace drake_vendor __attribute__ ((visibility ("hidden"))) {'  # noqa
        self._close = "}  /* inline namespace drake_vendor */"

    def _check(self, old_lines, expected_new_lines, inline_namespace=True):
        """Tests one call to _rewrite_one_text for expected output."""
        old_text = "\n".join(old_lines) + "\n"
        new_text = _rewrite_one_text(
            text=old_text, inline_namespace=inline_namespace
        )

        expected_new_text = "\n".join(expected_new_lines) + "\n"
        self.assertMultiLineEqual(expected_new_text, new_text)

    def test_without_inline_namespace(self):
        self._check(
            [
                "namespace foo",
                "{",
                "}",
                "   namespace bar   {",
                "}",
            ],
            [
                'namespace foo __attribute__ ((visibility ("hidden")))',
                "{",
                "}",
                'namespace bar __attribute__ ((visibility ("hidden")))   {',
                "}",
            ],
            inline_namespace=False,
        )

    def test_comments(self):
        self._check(
            [
                " // file comment",
                " /* block comment",
                "    continues */",
                "int foo();  // eol comment",
                "int bar();  /* eol comment */",
                " /* intro comment */ class Baz;",
                " /* single line c style */",
                "#include <something.h>",
                " /* intro comment that spans multiple lines",
                "    but continues with code after */ class Quux;",
            ],
            [
                " // file comment",
                " /* block comment",
                "    continues */",
                self._open,
                "int foo();  // eol comment",
                "int bar();  /* eol comment */",
                " /* intro comment */ class Baz;",
                self._close,
                " /* single line c style */",
                "#include <something.h>",
                self._open,
                " /* intro comment that spans multiple lines",
                "    but continues with code after */ class Quux;",
                self._close,
            ],
        )

    def test_preprocessor(self):
        self._check(
            [
                "#define FOO \\",
                "   foo",  # This line looks like code, but it isn't.
                "int foo();",
                "// A no-op preprocessor directive:",
                "#",
            ],
            [
                "#define FOO \\",
                "   foo",
                self._open,
                "int foo();",
                self._close,
                "// A no-op preprocessor directive:",
                "#",
            ],
        )

    def test_include_guard(self):
        self._check(
            [
                "#ifndef FOO_HH",
                "#define FOO_HH",
                "#include <something.h>",
                "int foo();",
                "#endif FOO_HH",
            ],
            [
                "#ifndef FOO_HH",
                "#define FOO_HH",
                "#include <something.h>",
                self._open,
                "int foo();",
                self._close,
                "#endif FOO_HH",
            ],
        )

    def test_pragma_once(self):
        self._check(
            [
                "#pragma once",
                "#include <something.h>",
                "int foo();",
            ],
            [
                "#pragma once",
                "#include <something.h>",
                self._open,
                "int foo();",
                self._close,
            ],
        )

    def test_extern_c(self):
        """No namespaces are added for 'extern C' files."""
        content = [
            '#include "somelib/somefile.h"',
            "#include <unrelated/thing.h>",
            'extern "C" {',
            "int foo();",
            "}  // extern C",
        ]
        self._check(content, content)


assert __name__ == "__main__"
unittest.main()
