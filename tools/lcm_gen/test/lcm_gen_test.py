from pathlib import Path
import tempfile
import unittest

from python import runfiles

from tools.lcm_gen import (
    CppGen,
    Parser,
    PrimitiveType,
    Struct,
    UserType,
)


class BaseTest(unittest.TestCase):
    @staticmethod
    def _resource(relative_path: str) -> Path:
        resource_path = f"drake/tools/lcm_gen/test/{relative_path}"
        return Path(runfiles.Create().Rlocation(resource_path))

    def setUp(self):
        self.maxDiff = None
        self._lima_path = self._resource("lima.lcm")
        self._lima_hpp_path = self._resource("goal/papa/lima.hpp")
        self._mike_path = self._resource("mike.lcm")
        self._mike_hpp_path = self._resource("goal/papa/mike.hpp")
        self._november_path = self._resource("november.lcm")
        self._november_hpp_path = self._resource("goal/papa/november.hpp")

        assert self._lima_path.exists()
        assert self._lima_hpp_path.exists()
        assert self._mike_path.exists()
        assert self._mike_hpp_path.exists()
        assert self._november_path.exists()
        assert self._november_hpp_path.exists()


class TestParser(BaseTest):
    """Tests for the Parser class."""

    @staticmethod
    def _join_lines(lines):
        return "\n".join(lines) + "\n"

    def test_remove_c_comments(self):
        """C comments are replaced by whitespace (to preserve line numbers in
        error messages).
        """
        self.assertMultiLineEqual(
            Parser._remove_c_comments(
                self._join_lines(
                    [
                        "foo",
                        "bar /* comment",
                        "still comment ",
                        "comment */ eol",
                        "bar/*quux*/baz",
                    ]
                )
            ),
            self._join_lines(
                [
                    "foo",
                    "bar           ",
                    "              ",
                    "           eol",
                    "bar        baz",
                ]
            ),
        )

    def test_remove_cpp_comments(self):
        """C++ comments are snipped to end-of-line."""

        self.assertMultiLineEqual(
            Parser._remove_cpp_comments(
                self._join_lines(
                    [
                        "foo",
                        "bar // eol",
                        "// line",
                        "next",
                    ]
                )
            ),
            self._join_lines(
                [
                    "foo",
                    "bar ",
                    "",
                    "next",
                ]
            ),
        )

    def test_parse_lima(self):
        """Checks the parse tree for `lima.lcm`."""
        lima = Parser.parse(filename=self._lima_path)
        self.assertEqual(
            str(lima),
            """\
struct papa.lima {
  const double charlie_delta = 3.25e1;
  const float charlie_foxtrot = 4.5e2;
  const int8_t charlie_india8 = -8;
  const int16_t charlie_india16 = 16;
  const int32_t charlie_india32 = 32;
  const int64_t charlie_india64 = 64;
  boolean golf;
  byte bravo;
  double delta;
  float foxtrot;
  int8_t india8;
  int16_t india16;
  int32_t india32;
  int64_t india64;
}
""",
        )
        # Check the value <=> value_str equivalence.
        for c in lima.constants:
            if c.typ in (PrimitiveType.float, PrimitiveType.double):
                self.assertIsInstance(c.value, float)
                self.assertEqual(c.value, float(c.value_str))
            else:
                self.assertIsInstance(c.value, int)
                self.assertEqual(c.value, int(c.value_str))

    def test_parse_mike(self):
        """Checks the parse tree for `mike.lcm`."""
        mike = Parser.parse(filename=self._mike_path)
        self.assertEqual(
            str(mike),
            """\
struct papa.mike {
  double delta[3];
  float foxtrot[4][5];
  papa.lima alpha;
  string sierra;
  int32_t rows;
  int32_t cols;
  byte bravo[rows];
  int8_t india8[rows][cols];
  int16_t india16[7][cols];
  int32_t india32[rows][11];
  papa.lima xray[2];
  papa.lima yankee[rows];
  papa.lima zulu[rows][2];
}
""",
        )
        # Check one field carefully for its exact representation.
        (zulu,) = [f for f in mike.fields if f.name == "zulu"]
        self.assertEqual(zulu.typ.package, "papa")
        self.assertEqual(zulu.typ.name, "lima")
        self.assertEqual(zulu.array_dims[0], "rows")
        self.assertEqual(zulu.array_dims[1], 2)

    def test_parse_november(self):
        """Checks the parse tree for `november.lcm`."""
        november = Parser.parse(filename=self._november_path)
        self.assertEqual(
            str(november),
            """\
struct papa.november {
  papa.lima alpha;
  papa.lima bravo;
  int32_t charlie;
}
""",
        )

    def test_type_str(self):
        """Tests UserType.__str__. The end-to-end tests of the parser wouldn't
        usually hit these corner cases because most types are immediately
        resolved to use fully-qualified names (i.e., with a non-None package).
        """
        dut = UserType(package=None, name="bar")
        self.assertEqual(str(dut), "bar")
        dut = UserType(package="foo", name="bar")
        self.assertEqual(str(dut), "foo.bar")

    def _parse_str(self, content):
        with tempfile.NamedTemporaryFile(
            mode="w", prefix="lcm_gen_test_", encoding="utf-8"
        ) as f:
            f.write(content)
            f.flush()
            return Parser.parse(filename=f.name)

    def test_missing_package_semi(self):
        with self.assertRaisesRegex(SyntaxError, "Expected ';'.*got.*struct"):
            self._parse_str("package foo /*;*/ struct empty { }")

    def test_multiline_const(self):
        foo = self._parse_str("struct foo { const int8_t x = 1, y = 2; }")
        self.assertEqual(foo.constants[0].name, "x")
        self.assertEqual(foo.constants[0].value, 1)
        self.assertEqual(foo.constants[1].name, "y")
        self.assertEqual(foo.constants[1].value, 2)

    def test_bad_const_type(self):
        with self.assertRaisesRegex(SyntaxError, "Expected.*primitive type"):
            self._parse_str('struct bad { const string name = "foo"; }')
        with self.assertRaisesRegex(SyntaxError, "Expected.*primitive type"):
            self._parse_str("struct bad { const bignum x = 2222; }")

    def test_bad_const_value(self):
        with self.assertRaisesRegex(SyntaxError, "Invalid.*value.*0x7f"):
            self._parse_str("struct bad { const int8_t x = 0x7f; }")

    def test_missing_const_name(self):
        with self.assertRaisesRegex(SyntaxError, "Expected.*NAME"):
            self._parse_str("struct bad { const double /* name */ = 1; }")


class TestCppGen(BaseTest):
    """Tests for the CppGen class. For the most part, these merely compare the
    generated code to a checked-in goal file. Testing that the generated code
    works as intended happens in the C++ unit test `functional_test.cc`.
    """

    _HELP = """
===========================================================================
To replace the goal files with newly-regenerated copies, run this command:

bazel run -- //tools/lcm_gen \
  tools/lcm_gen/test/*.lcm --cpp-hpath=tools/lcm_gen/test/goal

===========================================================================
"""

    def test_lima_text(self):
        """The generated text for lima.h exactly matches the goal file."""
        lima = Parser.parse(filename=self._lima_path)
        expected_text = self._lima_hpp_path.read_text(encoding="utf-8")
        actual_text = CppGen(struct=lima).generate()
        self.assertMultiLineEqual(expected_text, actual_text, self._HELP)

    def test_mike_text(self):
        """The generated text for mike.h exactly matches the goal file."""
        mike = Parser.parse(filename=self._mike_path)
        expected_text = self._mike_hpp_path.read_text(encoding="utf-8")
        actual_text = CppGen(struct=mike).generate()
        self.assertMultiLineEqual(expected_text, actual_text, self._HELP)

    def test_november_text(self):
        """The generated text for november.h exactly matches the goal file."""
        november = Parser.parse(filename=self._november_path)
        expected_text = self._november_hpp_path.read_text(encoding="utf-8")
        actual_text = CppGen(struct=november).generate()
        self.assertMultiLineEqual(expected_text, actual_text, self._HELP)

    def test_no_package(self):
        """Sanity test for a message without any LCM package specified."""
        empty = Struct(typ=UserType(package=None, name="empty"))
        actual_text = CppGen(struct=empty).generate()
        lines = actual_text.splitlines()
        while lines[0] == "" or lines[0][0] == "#":
            # Skip over blank lines and preprocessor lines.
            lines.pop(0)
        # The first real line of code should be the class opener.
        self.assertEqual(lines[0], "class empty {")
        # The last real line of code should be the class closer.
        self.assertEqual(lines[-1], "};")
