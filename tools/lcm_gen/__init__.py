"""A portable re-implementation of lcm-gen (see lcm-proj.github.io) using only
the Python 3 standard library.

For the LCM syntax and encoding specification, refer to:
 https://lcm-proj.github.io/lcm/content/lcm-type-ref.html
"""

import dataclasses
import enum
import io
import pathlib
import re
import token
import tokenize
from typing import Optional, List, Union

# A brief summary of LCM's grammar.
#
# Productions:
#  root -> package_decl? struct_decl* ;
#  package_decl -> PACKAGE identifier SEMI ;
#  struct_decl -> STRUCT identifier LCURL struct_statement* RCURL ;
#  struct_statement -> ( const_statement | field_statement ) ;
#  const_statement -> CONST primitive_type const_definition
#                       ( COMMA const_definition )* SEMI ;
#  const_definition -> identifier EQ value
#  field_statement -> qualified_identifier identifier array_dims? SEMI ;
#  array_dims -> ( LSQUARE array_dim RSQUARE )* ;
#  array_dim -> ( integer | identifier );
#  qualified_identifier -> ( identifier DOT )? identifier ;
#
# Where primitive_type, identifier, integer, and value are primitive lexemes.
#
# This tool does not support the deprecated[1] and undocumented "enum" keyword.
#  [1] https://github.com/lcm-proj/lcm/commit/d9dcf8e3


PrimitiveType = enum.Enum("PrimitiveType", " ".join([
    "boolean",
    "byte",
    "double",
    "float",
    "int8_t",
    "int16_t",
    "int32_t",
    "int64_t",
    "string",
]))
PrimitiveType.__str__ = lambda self: self.name


@dataclasses.dataclass(frozen=True)
class UserType:
    """A struct name from an LCM message definition, e.g., "foo" or "foo.bar".
    """
    package: Optional[str]
    name: str

    def __str__(self):
        if self.package is None:
            return self.name
        return f"{self.package}.{self.name}"


@dataclasses.dataclass(frozen=True)
class StructField:
    """A field within an LCM message definition."""
    name: str
    typ: Union[PrimitiveType, UserType]
    array_dims: List[Union[int, str]] = dataclasses.field(default_factory=list)

    def __str__(self):
        result = f"{self.typ} {self.name}"
        for dim in self.array_dims:
            result += f"[{dim}]"
        return result


@dataclasses.dataclass(frozen=True)
class StructConstant:
    """A constant within an LCM message definition."""
    name: str
    typ: PrimitiveType
    value: Union[int, float]
    value_str: str

    def __str__(self):
        return f"const {self.typ} {self.name} = {self.value_str}"


@dataclasses.dataclass(frozen=True)
class Struct:
    """The parse tree for an LCM message definition."""
    typ: UserType
    fields: List[StructField] = dataclasses.field(default_factory=list)
    constants: List[StructConstant] = dataclasses.field(default_factory=list)

    def __str__(self):
        result = f"struct {self.typ} {{\n"
        for c in self.constants:
            result += f"  {c};\n"
        for f in self.fields:
            result += f"  {f};\n"
        result += "}\n"
        return result


class Parser:
    """A basic recursive descent parser for the LCM message grammar.

    This parser only accepts files that have exactly one struct_decl.
    Having no structs (or more than one struct) is a parse error.
    """

    @staticmethod
    def parse(*, filename):
        """Returns a parsed Struct for the given filename."""
        return Parser(filename=filename)._root()

    def __init__(self, *, filename):
        """(Internal use only.)"""
        self._filename = filename
        self._result = None

        # Load the file.
        data = pathlib.Path(filename).read_text(encoding="utf-8")

        # Remove comments.
        data = self._remove_c_comments(data)
        data = self._remove_cpp_comments(data)

        # Tokenize.
        bytes_io = io.BytesIO(bytes(data, encoding="utf-8"))
        self._tokens = list(tokenize.tokenize(bytes_io.readline))
        self._i = 0
        self._consume(token.ENCODING)

    @staticmethod
    def _remove_c_comments(data):
        """Returns data with its C-style comments replaced with whitespace
        (so that column numbers in error messages still make sense).
        """
        while True:
            m = re.search(r"/\*.*?\*/", data, flags=re.DOTALL)
            if not m:
                break
            replacement = "".join([
                ch if ch == "\n" else " "
                for ch in m.group()
            ])
            start, end = m.span()
            data = data[:start] + replacement + data[end:]
        return data

    @staticmethod
    def _remove_cpp_comments(data):
        """Returns data stripped of its C++-style comments."""
        return re.sub(r"//.*$", "", data, flags=re.MULTILINE)

    def _current_type(self):
        """Returns the type of the current token.
        The enumerated types are per the Python `token` module.
        """
        return self._tokens[self._i][0]

    def _current_value(self):
        """Returns string value of the current token."""
        return self._tokens[self._i][1]

    def _syntax_error_details(self):
        """Provides the detail attribute of a SyntaxError."""
        return (
            self._filename,
            self._tokens[self._i][2][0],
            None,
            None)

    def _expect(self, expected_type, expected_value=None):
        """Raises a syntax error unless the current token matches the expected
        type and value (if given).
        """
        actual_value = self._current_value()
        actual_type = self._current_type()
        actual_typename = token.tok_name[actual_type]
        expected_typename = token.tok_name[expected_type]
        if expected_value is not None and actual_value != expected_value:
            raise SyntaxError(
                f"Expected '{expected_value}' "
                f"but got '{actual_value}'",
                self._syntax_error_details())
        if actual_type != expected_type:
            raise SyntaxError(
                f"Expected a token.{expected_typename}"
                f" but got a token.{actual_typename} ('{actual_value}')",
                self._syntax_error_details())

    def _advance(self):
        """Advances the parser to the next token, skipping whitespace."""
        self._i += 1
        while self._current_type() in (token.NEWLINE, token.NL):
            self._i += 1

    def _consume(self, token_type, token_value=None):
        """Does expect() then advance().
        Returns the prior value (i.e., the consumed value) as a string.
        """
        result = self._current_value()
        self._expect(expected_type=token_type, expected_value=token_value)
        if token_type != token.ENDMARKER:
            self._advance()
        return result

    def _root(self):
        """Parses a root production."""
        package = None
        if self._current_value() == "package":
            package = self._package_decl()
        self._struct_decl(package=package)
        self._consume(token.ENDMARKER)
        return self._result

    def _package_decl(self):
        """Parses a package_decl production."""
        self._consume(token.NAME, "package")
        package = self._consume(token.NAME)
        self._consume(token.OP, ";")
        return package

    def _struct_decl(self, package):
        """Parses a struct_decl production."""
        self._consume(token.NAME, "struct")
        name = self._consume(token.NAME)
        self._result = Struct(typ=UserType(package=package, name=name))
        self._consume(token.OP, "{")
        while True:
            if self._current_type() != token.NAME:
                break
            elif self._current_value() == "const":
                self._const_statement()
            else:
                self._field_statement()
        self._consume(token.OP, "}")

    def _const_statement(self):
        """Parses a const_statement production."""
        self._consume(token.NAME, "const")
        typ_str = self._consume(token.NAME)
        typ = None
        try:
            typ = PrimitiveType[typ_str]
        except KeyError:
            pass
        if typ is None or typ == PrimitiveType.string:
            self._i -= 1
            raise SyntaxError(
                f"Expected a primitive type name but got '{typ_str}'",
                self._syntax_error_details())
        self._const_definition(typ=typ)
        while self._current_value() == ",":
            self._consume(token.OP, ",")
            self._const_definition(typ=typ)
        self._consume(token.OP, ";")

    def _const_definition(self, *, typ):
        """Parses a const_definition production."""
        name = self._consume(token.NAME)
        self._consume(token.OP, "=")
        value_sign = ""
        if self._current_value() in ["+", "-"]:
            value_sign = self._consume(token.OP)
        value_str = value_sign + self._consume(token.NUMBER)
        try:
            if typ.name in ["float", "double"]:
                value = float(value_str)
            else:
                value = int(value_str)
        except ValueError:
            value = None
        if value is None:
            self._i -= 1
            raise SyntaxError(
                f"Invalid constant value '{value_str}' for {typ.name}",
                self._syntax_error_details())
        self._result.constants.append(StructConstant(
            name=name, typ=typ, value=value, value_str=value_str))

    def _field_statement(self):
        """Parses a field_statement production."""
        typ = self._qualified_identifier()
        name = self._consume(token.NAME)
        array_dims = []
        while self._current_value() == "[":
            self._consume(token.OP, "[")
            if self._current_type() == token.NAME:
                dim = self._consume(token.NAME)
            else:
                dim = int(self._consume(token.NUMBER))
                assert dim >= 0
            self._consume(token.OP, "]")
            array_dims.append(dim)
        self._consume(token.OP, ";")
        self._result.fields.append(StructField(
            name=name, typ=typ, array_dims=array_dims))

    def _qualified_identifier(self):
        """Parses a qualified_identifier production."""
        name1 = self._consume(token.NAME)
        try:
            return PrimitiveType[name1]
        except KeyError:
            pass
        if self._current_value() == ".":
            self._consume(token.OP, ".")
            name2 = self._consume(token.NAME)
            return UserType(package=name1, name=name2)
        current_package = self._result.typ.package
        return UserType(package=current_package, name=name1)
