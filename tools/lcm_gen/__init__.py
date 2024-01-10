"""A portable re-implementation of lcm-gen (see lcm-proj.github.io) using only
the Python 3 standard library.

For the LCM syntax and encoding specification, refer to:
 https://lcm-proj.github.io/type_specification.html
"""

import argparse
import dataclasses
import enum
import functools
import io
import operator
import re
import sys
import struct
import token
import tokenize
from typing import Optional, List, Union

# A brief summary of LCM's grammar.
#
# Productions:
#  root -> package_decl? struct_decl ;
#  package_decl -> PACKAGE identifier SEMI ;
#  struct_decl -> STRUCT identifier LCURL struct_stmts RCURL ;
#  struct_statements -> struct_statement* ;
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
    package: Optional[str]
    name: str

    def __str__(self):
        if self.package is None:
            return self.name
        return f"{self.package}.{self.name}"


@dataclasses.dataclass
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


@dataclasses.dataclass
class StructConstant:
    """A constant within an LCM message definition."""
    name: str
    typ: PrimitiveType
    value: Union[int, float]
    value_str: str

    def __str__(self):
        return f"const {self.typ} {self.name} = {self.value_str}"


@dataclasses.dataclass
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
    """A simple recursive descent parser for the LCM message grammar."""

    @staticmethod
    def parse(*, filename):
        """Returns a parsed Struct for the given filename."""
        return Parser(filename=filename)._root()

    def __init__(self, *, filename):
        self._filename = filename
        self._result = None

        # Load the file.
        with open(filename, "r", encoding="utf-8") as f:
            data = f.read()

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

    def _consume(self, token_type, allowed_values=None):
        """Does expect() then advance().
        Returns the prior value (i.e., the consumed value) as a string.
        """
        result = self._current_value()
        self._expect(token_type, allowed_values)
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
        try:
            typ = PrimitiveType[typ_str]
            if typ == PrimitiveType.string:
                typ = None
        except KeyError:
            pass
        if typ is None:
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
                if dim < 0:
                    raise SyntaxError(
                        f"Invalid array dimension '{dim}' for '{name}'",
                        self._syntax_error_details())
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


_CPP_TEMPLATE = """\
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

@@NAMESPACE_BEGIN@@

class @@STRUCT_NAME@@ {
 public:
@@MEMBER_CONSTANTS@@
@@MEMBER_FIELDS@@
  // These functions match the expected API from the legacy lcm-gen tool.
  // Note that we use `int64_t` instead of `int` for byte counts.
  int64_t getEncodedSize() const { return 8 + _getEncodedSizeNoHash(); }
  int64_t _getEncodedSizeNoHash() const {
    int64_t _result = 0;
@@GET_ENCODED_SIZE_NO_HASH@@
    return _result;
  }
  template <bool with_hash = true>
  int64_t encode(void* buf, int64_t offset, int64_t maxlen) const {
    uint8_t* const _begin = static_cast<uint8_t*>(buf);
    uint8_t* const _start = _begin + offset;
    uint8_t* const _end = _begin + maxlen;
    uint8_t* _cursor = _start;
    return this->_encode<with_hash>(&_cursor, _end) ? (_cursor - _start) : -1;
  }
  int64_t _encodeNoHash(void* buf, int64_t offset, int64_t maxlen) const {
    return encode<false>(buf, offset, maxlen);
  }
  template <bool with_hash = true>
  int64_t decode(const void* buf, int64_t offset, int64_t maxlen) {
    const uint8_t* const _begin = static_cast<const uint8_t*>(buf);
    const uint8_t* const _start = _begin + offset;
    const uint8_t* const _end = _begin + maxlen;
    const uint8_t* _cursor = _start;
    return this->_decode<with_hash>(&_cursor, _end) ? (_cursor - _start) : -1;
  }
  int64_t _decodeNoHash(const void* buf, int64_t offset, int64_t maxlen) {
    return decode<false>(buf, offset, maxlen);
  }
  static constexpr int64_t getHash() {
    return static_cast<int64_t>(_get_hash_impl());
  }
  template <typename Parents>
  static uint64_t _computeHash(const Parents*) {
    return getHash();
  }

  // New-style (constexpr) hashing.
  template <size_t N = 0>
  static constexpr uint64_t _get_hash_impl(
      const std::array<uint64_t, N>& parents = {}) {
    const uint64_t base_hash = @@BASE_HASH@@;
    std::array<uint64_t, N + 1> new_parents{base_hash};
    for (size_t n = 0; n < N; ++n) {
      if (parents[n] == base_hash) {
        return 0;
      }
      new_parents[n + 1] = parents[n];
    }
@@COMPOSITE_HASH@@
    return (composite_hash << 1) + ((composite_hash >> 63) & 1);
  }

  // New-style encoding.
  template <bool with_hash = true>
  bool _encode(uint8_t** _cursor, uint8_t* _end) const {
    constexpr int64_t _hash = getHash();
    constexpr int64_t _hash_size = with_hash ? 8 : 0;
    return  // true iff success
@@ENCODE@@
  }

  // New-style decoding.
  template <bool with_hash = true>
  bool _decode(const uint8_t** _cursor, const uint8_t* _end) {
    constexpr int64_t _expected_hash = getHash();
    constexpr int64_t _hash_size = with_hash ? 8 : 0;
    int64_t _hash = _expected_hash;
    return  // true iff success
@@DECODE@@
  }

 private:
  // Given N bytes at `_input`, returns a std::array in network byte order.
  template <size_t N>
  static auto _byteswap(const void* _input) {
    // clang-format off
    using result_t = std::conditional_t<
        N == 1, uint8_t, std::conditional_t<
        N == 2, uint16_t, std::conditional_t<
        N == 4, uint32_t, std::conditional_t<
        N == 8, uint64_t, void>>>>;
    // clang-format on
    result_t _result;
    std::memcpy(&_result, _input, N);
    // TODO(jwnimmer-tri) Don't bswap on PowerPC.
    if constexpr (N == 1) {
      return _result;
    } else if constexpr (N == 2) {
      return __builtin_bswap16(_result);
    } else if constexpr (N == 4) {
      return __builtin_bswap32(_result);
    } else if constexpr (N == 8) {
      return __builtin_bswap64(_result);
    }
  }

  // Given a field (or child element within a field), encodes it into the given
  // byte cursor and advances the cursor, returning true on success.
  template <typename T, typename... Ints>
  static bool _encode_field(const T& _input, uint8_t** _cursor, uint8_t* _end,
                            Ints... _dims) {
    static_assert(!std::is_pointer_v<T>);
    if constexpr (sizeof...(Ints) == 0) {
      // With no array dimensions, just decode the field directly.
      if constexpr (std::is_fundamental_v<T>) {
        // POD input.
        constexpr size_t N = sizeof(T);
        auto _swapped = _byteswap<N>(&_input);
        std::memcpy(*_cursor, &_swapped, N);
        *_cursor += N;
        return true;
      } else if constexpr (std::is_same_v<T, std::string>) {
        // String input.
        const int32_t _size = _input->size() + 1;
        const bool ok = (_input->size() < INT32_MAX) &&
                        (*_cursor + 4 + _size <= _end) &&
                        _encode_field(_size, _cursor, _end);
        if (ok) {
          std::memcpy(*_cursor, _input->c_str(), _size);
        }
        *_cursor += _size;
        return ok;
      } else {
        // Struct input.
        return _input.template _encode<false>(_cursor, _end);
      }
    } else {
      // In case of a variable-size dimension, cross-check vs the size field.
      const int64_t _dim = std::get<0>(std::make_tuple(_dims...));
      if (static_cast<int64_t>(_input.size()) != _dim) {
        return false;
      }
      // Encode each sub-item in turn.
      for (const auto& _child : _input) {
        const bool ok = [&](int /* _dim */, auto&&... _child_dims) {
          return _encode_field(_child, _cursor, _end, _child_dims...);
        }(_dims...);
        if (!ok) {
          return false;
        }
      }
      return true;
    }
  }

  // Given a pointer to a field (or child element within a field), decodes it
  // from the given byte cursor and advances the cursor, returning true on
  // success.
  template <typename T, typename... Ints>
  static bool _decode_field(T* _output, const uint8_t** _cursor,
                            const uint8_t* _end, Ints... _dims) {
    static_assert(!std::is_pointer_v<T>);
    if constexpr (sizeof...(Ints) == 0) {
      // With no array dimensions, just decode the field directly.
      if constexpr (std::is_fundamental_v<T>) {
        // POD output.
        constexpr size_t N = sizeof(T);
        auto _swapped = _byteswap<N>(*_cursor);
        std::memcpy(_output, &_swapped, N);
        *_cursor += N;
        // Overflow checking is the responsibility of the top-level decode.
        (void)(_end);
        return true;
      } else if constexpr (std::is_same_v<T, std::string>) {
        // String output.
        int32_t _size{};
        const bool ok = (*_cursor + 4 <= _end) &&
                        _decode_field(&_size, _cursor, _end) &&
                        (_size > 0) && (*_cursor + _size <= _end);
        if (ok) {
          _output->replace(_output->begin(), _output->end(), *_cursor,
                           *_cursor + _size - 1);
        }
        *_cursor += _size;
        return ok;
      } else {
        // Struct output.
        return _output->template _decode<false>(_cursor, _end);
      }
    } else {
      // In case of a variable-size dimension, resize our storage to match.
      if constexpr (std::is_same_v<T, std::vector<typename T::value_type>>) {
        const int64_t _dim = std::get<0>(std::make_tuple(_dims...));
        _output->resize(_dim);
      }
      // Decode each sub-item in turn.
      for (auto& _child : *_output) {
        const bool ok = [&](int /* _dim */, auto&&... _child_dims) {
          return _decode_field(&_child, _cursor, _end, _child_dims...);
        }(_dims...);
        if (!ok) {
          return false;
        }
      }
      return true;
    }
  }
};

@@NAMESPACE_END@@
"""


class CppGen:
    """Produces C++ message code for an LCM message definition."""

    _FIXED_SIZE = {
        PrimitiveType.boolean: 1,
        PrimitiveType.byte: 1,
        PrimitiveType.double: 8,
        PrimitiveType.float: 4,
        PrimitiveType.int8_t: 1,
        PrimitiveType.int16_t: 2,
        PrimitiveType.int32_t: 4,
        PrimitiveType.int64_t: 8,
    }

    def __init__(self, struct):
        self._struct = struct
        self._result = None
        self._size_variables = []
        for field in self._struct.fields:
            for dim in field.array_dims:
                if isinstance(dim, str):
                    if dim not in self._size_variables:
                        self._size_variables.append(dim)

    def generate(self):
        """Returns the C++ text for the message provided in the constructor."""
        self._result = _CPP_TEMPLATE
        self._fill_names()
        self._fill_member_constants()
        self._fill_member_fields()
        self._fill_encoded_size()
        self._fill_encode()
        self._fill_decode()
        self._fill_base_hash()
        self._fill_composite_hash()
        return self._result

    def _replace(self, old, new):
        count = 9999
        updated = self._result.replace(old, new, count)
        assert updated != self._result
        self._result = updated

    def _fill_names(self):
        """Updates the namespace and struct names for this message."""
        typ = self._struct.typ
        if typ.package is None:
            self._replace("@@NAMESPACE_BEGIN@@\n\n", "")
            self._replace("\n@@NAMESPACE_END@@\n", "")
        else:
            self._replace("@@NAMESPACE_BEGIN@@",
                          f"namespace {typ.package} {{")
            self._replace("@@NAMESPACE_END@@",
                          f"}}  // namespace {typ.package}")
        self._replace("@@STRUCT_NAME@@", typ.name)

    def _fill_member_constants(self):
        """Updates member constants for this message."""
        content = "".join([
            "  static constexpr {} {} = {};\n".format(
                self._full_typename(const.typ), const.name, const.value_str)
            for const in self._struct.constants
        ])
        if content:
            content += "\n"
        self._replace("@@MEMBER_CONSTANTS@@\n", content)

    def _full_typename(self, typ):
        """Returns the C++ typename for the given Parser typ."""
        if typ == PrimitiveType.boolean:
            return "bool"
        if typ == PrimitiveType.byte:
            return "uint8_t"
        if typ == PrimitiveType.string:
            return "std::string"
        if isinstance(typ, UserType) and typ.package is not None:
            return f"{typ.package}::{typ.name}"
        return typ.name

    def _fill_member_fields(self):
        """Updates member fields for this message."""
        content = "".join([
            f"  {self._to_member_field_type(field)} {field.name};\n"
            for field in self._struct.fields
        ])
        if content:
            content += "\n"
        self._replace("@@MEMBER_FIELDS@@\n", content)

    def _to_member_field_type(self, field):
        """Returns the C++ type for a member field declaration."""
        result = self._full_typename(field.typ)
        for dim in reversed(field.array_dims):
            if isinstance(dim, int):
                result = f"std::array<{result}, {dim}>"
            else:
                result = f"std::vector<{result}>"
        return result

    def _fill_encoded_size(self):
        """Updates the getEncodedSize() implementation for this message."""
        content = ""
        pad = " " * 4
        for name in self._size_variables:
            content += f"{pad}if ({name} < 0) {{\n"
            content += f"{pad}  return _result;\n"
            content += f"{pad}}}\n"
        for field in self._struct.fields:
            for line in self._fill_one_encoded_size(field).splitlines():
                content += f"{pad}{line}\n"
        self._replace("@@GET_ENCODED_SIZE_NO_HASH@@\n", content)

    def _fill_one_encoded_size(self, field):
        """Returns the getEncodedSize() stanza for one member field."""
        # For fixed-size elements, we can compute the byte size directly.
        known_size = self._known_size(field)
        if known_size is not None:
            return f"_result += {known_size};  // {field.name}\n"

        # For varible-size elements, we need to loop in case of arrays.
        content = ""
        pad = ""
        var = field.name
        for i in range(len(field.array_dims)):
            new_var = f"_{field.name}_{i}"
            content += f"{pad}for (const auto& {new_var} : {var}) {{\n"
            var = new_var
            pad += " " * 2
        if field.typ == PrimitiveType.string:
            content += f"{pad}_result += 4 + {var}.size() + 1;\n"
        else:
            assert isinstance(field.typ, UserType)
            content += f"{pad}_result += {var}._getEncodedSizeNoHash();\n"
        for _ in field.array_dims:
            pad = pad[:-2]
            content += f"{pad}}}\n"
        return content

    def _known_size(self, field):
        """If field will have a known encoded size at runtime, returns a
        string expression for that size; otherwise None.
        """
        primitive_size = self._FIXED_SIZE.get(field.typ)
        if primitive_size is None:
            return None
        result = f"{primitive_size}"
        for dim in field.array_dims:
            result += f" * {dim}"
        return result

    def _fill_encode(self):
        """Updates the encode() implementation for this message."""
        operations = []

        # Check that all variable-length sizes are valid.
        preconditions = [
            f"({dim} >= 0)"
            for dim in self._size_variables
        ]

        # Encode everything. When we have a run of known-size elements, use a
        # single size check for all of them.
        pending = [(
            "_hash_size",
            "(with_hash ? _encode_field(_hash, _cursor, _end) : true)",
        )]
        eof_marker = StructField(name="", typ=UserType(package="", name=""))
        for item in self._struct.fields + [eof_marker]:
            # If the size is known up front, just enqueue it.
            if item is not eof_marker:
                known_size, operation = self._fill_one_encode(item)
                if known_size is not None:
                    pending.append((known_size, operation))
                    continue
            # Otherwise, flush any pending fixed-size operations.
            if pending:
                total_size = " + ".join([one_size for one_size, _ in pending])
                operations.append(f"(*_cursor + {total_size} <= _end)")
                operations.extend([one_op for _, one_op in pending])
                pending = []
            # Finally, add the variable-size operation.
            if item is not eof_marker:
                operations.append(operation)
        assert not pending

        # Format the sequence of operations as a C++ short-circuit expression.
        content = " &&\n".join([
            " " * 8 + item
            for item in preconditions + operations
        ]) + ";\n"
        self._replace("@@ENCODE@@\n", content)

    def _fill_one_encode(self, field):
        """Returns the encode() stanza for one member field."""
        known_size = self._known_size(field)
        if field.array_dims:
            dims = ", " + ", ".join([str(dim) for dim in field.array_dims])
        else:
            dims = ""
        operation = f"_encode_field({field.name}, _cursor, _end{dims})"
        return known_size, operation

    def _fill_decode(self):
        """Updates the decode() implementation for this message."""
        operations = []

        # Decode everything.
        pending_sizes = ["_hash_size"]
        pending_operations = [
            "(with_hash ? _decode_field(&_hash, _cursor, _end) : true)",
            "(_hash == _expected_hash)",
        ]
        pending_field_reads = []
        eof_marker = StructField(name="", typ=UserType(package="", name=""))
        for item in self._struct.fields + [eof_marker]:
            # If our size needs a prior field, flush it now.
            if any([dim in pending_field_reads for dim in item.array_dims]):
                total_size = " + ".join(pending_sizes)
                operations.append(f"(*_cursor + {total_size} <= _end)")
                operations.extend(pending_operations)
                pending_sizes = []
                pending_operations = []
                pending_field_reads = []
            # If the size is known up front, just enqueue it.
            if item is not eof_marker:
                known_size, new_operations = self._fill_one_decode(item)
                if known_size is not None:
                    pending_sizes.append(known_size)
                    pending_operations.extend(new_operations)
                    pending_field_reads.append(item.name)
                    continue
            # Otherwise, flush any pending fixed-size operations.
            if pending_operations:
                if pending_sizes:
                    total_size = " + ".join(pending_sizes)
                    operations.append(f"(*_cursor + {total_size} <= _end)")
                operations.extend(pending_operations)
                pending_sizes = []
                pending_operations = []
                pending_field_reads = []
            else:
                assert not pending_sizes
                assert not pending_field_reads
            # Finally, add the variable-size operation.
            if item is not eof_marker:
                operations.extend(new_operations)
        assert not pending_field_reads

        # Format the sequence of operations as a C++ short-circuit expression.
        content = " &&\n".join([
            " " * 8 + item
            for item in operations
        ]) + ";\n"
        self._replace("@@DECODE@@\n", content)

    def _fill_one_decode(self, field):
        """Returns the decode() stanza for one member field."""
        known_size = self._known_size(field)
        if field.array_dims:
            dims = ", " + ", ".join([str(dim) for dim in field.array_dims])
        else:
            dims = ""
        operations = [
            f"_decode_field(&{field.name}, _cursor, _end{dims})"
        ]
        if field.name in self._size_variables:
            operations.append(f"({field.name} >= 0)")
        return known_size, operations

    def _fill_base_hash(self):
        """Updates the 'base hash' constant for this message."""
        # Collect the list of data to be hashed (int or str).
        data = []
        for item in self._struct.fields:
            data.append(item.name)
            if isinstance(item.typ, PrimitiveType):
                data.append(item.typ.name)
            data.append(len(item.array_dims))
            for dim in item.array_dims:
                data.append(1 if isinstance(dim, str) else 0)
                data.append(str(dim))
        # Consolidate the data to be hashed into a uniform sequence of bytes.
        # Integers are truncated to one byte.
        chars = bytearray()
        for x in data:
            if isinstance(x, int):
                chars.append(x % 256)
            else:
                assert isinstance(x, str)
                chars.append(len(x) % 256)
                chars.extend([ord(ch) for ch in x])
        # Hashify the bytes, interpreting them as an int8_t sequence.
        value = 0x12345678
        for (c,) in struct.iter_unpack("<b", chars):
            # The mixing arithmetic uses signed integers.
            value = ((value << 8) ^ (value >> 55)) + c
            # Truncate as unsigned (i.e., uint64_t).
            value %= 2**64
            # Cast back to signed (i.e., int64_t).
            if value >= 2**63:
                value -= 2**64
        # Cast back to a unsigned (i.e., uint64_t).
        value %= 2**64
        self._replace("@@BASE_HASH@@", f"0x{value:016x}ull")

    def _fill_composite_hash(self):
        """Updates the 'composite hash' expression for this message."""
        pad = " " * 4
        content = f"{pad}const uint64_t composite_hash = base_hash"
        for field in self._struct.fields:
            if isinstance(field.typ, UserType):
                child_type = self._full_typename(field.typ)
                child_hash = f"{child_type}::_get_hash_impl(new_parents)"
                content += f"\n{pad}    + {child_hash}"
        content += ";"
        self._replace("@@COMPOSITE_HASH@@", content)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "src", nargs="+",
        help="*.lcm source file(s)")
    args = parser.parse_args()

    returncode = 0
    for src in args.src:
        try:
            struct = Parser.parse(filename=src)
            generator = CppGen(struct=struct)
            header = generator.generate()
        except Exception as e:
            print(e)
            returncode = 1

    return returncode


if __name__ == "__main__":
    main()
