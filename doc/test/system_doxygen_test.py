import re
import unittest

from drake.doc.system_doxygen import process_doxygen_system_tags


class TestSystemDoxygen(unittest.TestCase):
    def test_triple_slash_comment(self):
        s = R"""()
/// This is one style of comments
///
/// @system
/// name: Adder // ignore
/// input_ports:
/// - input(0)
/// - ...
/// - input(N-1)
/// output_ports:
/// - sum
/// @endsystem
///
)"""
        t = process_doxygen_system_tags(s)
        self.assertRegex(t, "/// <table")
        self.assertNotRegex(t, "ignore")

    def test_line_asterisk_comment(self):
        s = R"""(
/** This is another style of comments
 *
 * @system
 * name: Alchemist
 * input_ports:  // ignore
 * - lead
 * output_ports:
 * - gold
 * @endsystem
 */
)"""
        t = process_doxygen_system_tags(s)
        self.assertRegex(t, r'\n \* <table')
        self.assertNotRegex(t, "ignore")

    def test_asterisk_comment(self):
        s = R"""(
/** This is another style of comments

@system
name: Alchemist
input_ports:  // ignore
- lead
output_ports:
- gold
@endsystem */
)"""
        t = process_doxygen_system_tags(s)
        self.assertRegex(t, "\n<table")
        self.assertNotRegex(t, "ignore")

    def test_html_works(self):
        s = R"""(
/** This is another style of comments

@system
name: Alchemist<br/><img src="photo.jpg" />
input_ports:
- lead
output_ports:
- <b>gold</b>
@endsystem */
)"""
        t = process_doxygen_system_tags(s)
        self.assertRegex(t, 'Alchemist<br/><img src="photo.jpg" />')
        self.assertRegex(t, '<b>gold</b>')

    def test_multiple_system_tags(self):
        s = R"""(
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
)"""
        t = process_doxygen_system_tags(s)
        self.assertEqual(t.count("\n<table"), 2)
