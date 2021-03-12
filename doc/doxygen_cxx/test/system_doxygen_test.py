import re
from textwrap import dedent, indent
import unittest

import drake.doc.doxygen_cxx.system_doxygen as mut


class TestSystemDoxygen(unittest.TestCase):
    def test_strip_cpp_comment_cruft(self):
        self.assertEqual(
            mut.strip_cpp_comment_cruft(dedent("""\
            // This should disappear.
            /// comments.
            /// yay.
            /// @directive
            /// @enddirective
            // This should also disappear.
            """)),
            # N.B. For simplicity, re-indent to show the leading space.
            indent(dedent("""\

             comments.
             yay.
             @directive
             @enddirective
            """), " "),
        )

        self.assertEqual(
            mut.strip_cpp_comment_cruft(dedent("""\
            * Some more comment stuff
            * @directive
            * Something // ignore
            * @enddirective
            """)),
            # N.B. For simplicity, re-indent to show the leading space.
            indent(dedent("""\
             Some more comment stuff
             @directive
             Something
             @enddirective
            """), " "),
        )

    def test_system_yaml_to_html_and_rst(self):
        system_yaml = dedent("""\
        name: Adder
        input_ports:
        - input(0)
        - ...
        - input(N-1)
        - <b>Raw html</b>
        output_ports:
        - sum
        """)
        html = mut.system_yaml_to_html(system_yaml)
        self.assertIn("<table", html)
        self.assertIn("<b>Raw html</b>", html)

        rst = mut.system_yaml_to_pydrake_system_rst_directive(system_yaml)
        self.assertIn(".. pydrake_system::", rst)
        self.assertIn("<b>Raw html</b>", rst)

    def test_process_doxygen(self):
        s = dedent("""\
        /** This is another style of comments

        @system
        name: Alchemist
        input_ports:
        - lead
        output_ports:
        - gold
        @endsystem

        And then we have some more insightful documentation...

        @system
        name: Researcher
        input_ports:
        - ideas
        output_ports:
        - gold
        @endsystem */
        """)
        html_ish = mut.process_doxygen_system_tags(s)
        self.assertEqual(html_ish.count("\n<table"), 2)

        rst_ish = mut.process_doxygen_to_sphinx(s)
        self.assertEqual(rst_ish.count(".. pydrake_system::"), 2)
