"""Tests basic docstring reformatting."""

from textwrap import dedent
import unittest

import drake.tools.lint.cpp_docstring_lint as mut


def make_tokens(s, name="file"):
    raw_lines = dedent(s.rstrip()).split("\n")
    return mut.multiline_tokenize(name, raw_lines)


class TestCppDocstringLint(unittest.TestCase):
    def test_abstract_regex(self):
        """Tests basic pattern matching. Used in reorder_multiline_tokens"""
        pattern = mut.AbstractRegex([
            mut.AbstractRegex.Single(lambda c: c == "a"),
            mut.AbstractRegex.Any(lambda c: c == "b"),
            mut.AbstractRegex.Single(lambda c: c == "c"),
        ])
        matches = pattern.find_all("a ab ac abc abbbbc aaaabcccc!")
        self.assertEqual(len(matches), 4)
        self.assertEqual(
            matches[0].groups(),
            [['a'], [], ['c']])
        self.assertEqual(
            matches[1].groups(),
            [['a'], ['b'], ['c']])
        self.assertEqual(
            matches[2].groups(),
            [['a'], list('bbbb'), ['c']])
        self.assertEqual(
            matches[3].groups(),
            [['a'], ['b'], ['c']])

    def test_multiline_tokenize(self):
        # Test all available token types.
        tokens = make_tokens("""\
            /// Hello
            /// world

            void fake_code();

            /** Comment */
            /**
             * Comment
             */

            void more_fake_code();

            // Goodby
            // world

            /* Comment */
        """)
        self.assertEqual(len(tokens), 12)
        self.assertIsInstance(tokens[0], mut.TripleSlashMultilineToken)
        # Briefly test out string representation.
        self.assertEqual(
            str(tokens[0]),
            dedent("""\
                file:1: /// Hello
                file:2: /// world
            """.rstrip()),
        )
        self.assertIsInstance(tokens[1], mut.WhitespaceMultilineToken)
        self.assertIsInstance(tokens[2], mut.GenericMultilineToken)
        self.assertIsInstance(tokens[3], mut.WhitespaceMultilineToken)
        self.assertIsInstance(tokens[4], mut.DoubleStarMultilineToken)
        self.assertIsInstance(tokens[5], mut.DoubleStarMultilineToken)
        self.assertIsInstance(tokens[6], mut.WhitespaceMultilineToken)
        self.assertIsInstance(tokens[7], mut.GenericMultilineToken)
        self.assertIsInstance(tokens[8], mut.WhitespaceMultilineToken)
        self.assertIsInstance(tokens[9], mut.DoubleSlashMultilineToken)
        self.assertIsInstance(tokens[10], mut.WhitespaceMultilineToken)
        self.assertIsInstance(tokens[11], mut.SingleStarMultilineToken)

    def test_user_errors(self):
        with self.assertRaises(mut.UserFormattingError) as cm:
            make_tokens("/* unclosed comment")
        self.assertEqual(
            str(cm.exception),
            "Not closed:\nfile:1: /* unclosed comment")
        with self.assertRaises(mut.UserFormattingError) as cm:
            make_tokens("/** unclosed comment")
        with self.assertRaises(mut.UserFormattingError) as cm:
            make_tokens("""\
                /** needs star
                 *
                 asdlfjkasldkfj
                 */
            """)
        with self.assertRaises(mut.UserFormattingError) as cm:
            token, = make_tokens("""\
                /**
                  ragged indentation
                shown here
                by this line
                 */
            """)
            # TODO(eric.cousineau): Make this a parsing error?
            token.get_docstring_text()

    def test_reformat_docstring(self):
        """Shows different docstrings that get reformatting to the same
        string."""

        # Reformatted.
        expected_text = dedent("""\
            /**
            abc

            def
              ghi

            jkl */
        """.rstrip())

        # EXample inputs.
        tokens = make_tokens("""\
            /** abc

             def
               ghi

             jkl */
            /**     abc

             def
               ghi

             jkl */
            /** abc

                def
                  ghi

                jkl
                    **/
            /// abc
            ///
            /// def
            ///   ghi
            ///
            /// jkl
            /**
             *  abc
             *
             *  def
             *    ghi
             *
             *  jkl
             **/
        """)
        self.assertEqual(len(tokens), 5)

        for token in tokens:
            text = "\n".join(mut.reformat_docstring(token))
            self.assertEqual(expected_text, text, str(token))

        token, = make_tokens("/// This is code /* with a nested comment */")
        expected_text = "/** This is code /+ with a nested comment +/ */"
        actual_text, = mut.reformat_docstring(token)
        self.assertEqual(expected_text, actual_text)

    def test_check_or_apply_lint(self):
        tokens_in = make_tokens("""\
            /// Docstring comment.
            //@{
            // This comment should be above docstring.
            example_code();
            //@}
        """)
        text_out = "\n".join(
            mut.check_or_apply_lint_on_tokens(tokens_in, lint_errors=None))
        text_expected = dedent("""\
            // This comment should be above docstring.
            /**
            Docstring comment.
            @{ */
            example_code();
            /** @} */
        """.rstrip())
        self.assertEqual(text_expected, text_out)

        # Check for lint errors.
        lint_errors = []
        mut.check_or_apply_lint_on_tokens(
            tokens_in, lint_errors=lint_errors, verbose=True)
        # TODO(eric.cousineau): Make lint error show up?
        expected_errors = dedent("""\
            ERROR: Docstring needs reformatting
            file:1: /// Docstring comment.
            file:2: /// @{
              should look like:
            file:1: /**
            file:2: Docstring comment.
            file:3: @{ */

            ERROR: Docstring needs reformatting
            file:5: /// @}
              should look like:
            file:5: /** @} */
        """.rstrip())
        actual_errors = "\n".join(str(x) for x in lint_errors).rstrip()
        self.assertEqual(expected_errors, actual_errors)
