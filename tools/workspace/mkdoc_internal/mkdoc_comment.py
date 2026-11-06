#
#  Derived from https://github.com/pybind/pybind11/
#
#  Copyright (c) 2016 Wenzel Jakob <wenzel.jakob@epfl.ch>,
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived from this
#     software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#   Converts Doxygen-formatted string to look presentable in a Python
#   docstring.
#

import re
import textwrap
import unicodedata

from doc.doxygen_cxx.system_doxygen import process_doxygen_to_sphinx


def process_comment(comment):
    # Remove C++ comment syntax
    s = remove_cpp_comment_syntax(comment)

    # Remove HTML comments. Must occur before Doxygen commands are parsed
    # since they may be used to workaround limitations related to line breaks
    # in Doxygen.
    # TODO (betsymcphail): Not tested
    s = remove_html_comments(s)

    # MyPy parsing has a particular dislike of double colons.
    # Replace all of them with a different unicode character.
    # Our pydrake_sphinx_extension.py replaces this back to double colons to
    # avoid visual artifacts in the documentation. If this is removed in the
    # future, that part can be removed too.
    s = s.replace("::", "∷")

    # Tables are set aside here and put back at the very end, so that none of
    # the passes below can break up a table.
    tables = []
    s = replace_tables(s, tables)

    # Markdown to reStructuredText (but don't process literal blocks).
    in_code_segment = False
    rst_fragments = []
    for x in re.split(r"(```|@code|@endcode)", s):
        if x == "```" or x == "@code" or x == "@endcode":
            rst_fragments.append(x)
            in_code_segment = not in_code_segment
        elif in_code_segment:
            rst_fragments.append(x)
        else:
            rst_fragments.append(markdown_to_restructuredtext(x))
    s = "".join(rst_fragments)

    # HTML tags. Support both lowercase and uppercase tags.
    # TODO (betsymcphail): Not tested
    s = replace_html_tags(s)

    # TODO (betsymcphail): Not tested
    s = s.replace("``true``", "``True``")
    s = s.replace("``false``", "``False``")

    # Exceptions
    s = replace_exceptions(s)

    s = process_doxygen_commands(s)

    # Reflow text where appropriate.
    s = reflow(s)

    s = restore_tables(s, tables)

    return s


def remove_cpp_comment_syntax(s):
    result = ""
    leading_spaces = float("inf")
    for line in s.expandtabs(tabsize=4).splitlines():
        line = line.strip()
        line = line.removeprefix("/*!")
        if line.startswith("/*"):
            line = line[2:].lstrip("*")
        if line.endswith("*/"):
            line = line[:-2].rstrip("*")
        # http://www.doxygen.nl/manual/docblocks.html#memberdoc
        line = line.removeprefix("///<")
        line = line.removeprefix("///")
        line = line.removeprefix("//!")
        line = line.removeprefix("*")
        if len(line) > 0:
            leading_spaces = min(leading_spaces, len(line) - len(line.lstrip()))
        result += line + "\n"

    if leading_spaces != float("inf"):
        result2 = ""
        for line in result.splitlines():
            result2 += line[leading_spaces:] + "\n"
        result = result2

    return result


def remove_html_comments(s):
    return re.sub(r"<!--(.*?)-->", r"", s, flags=re.DOTALL)


_TABLE_PLACEHOLDER = "MKDOCTABLEPLACEHOLDER{}END"
_TABLE_PLACEHOLDER_LINE = re.compile(
    r"^( *)MKDOCTABLEPLACEHOLDER([0-9]+)END$", re.MULTILINE
)

# Regions of text in which tables must not be converted.
_VERBATIM_REGIONS = re.compile(
    r"(```.*?```"
    r"|[@\\]code.*?[@\\]endcode"
    r"|<pre>.*?</pre>"
    r"|[@\\]verbatim.*?[@\\]endverbatim)",
    re.DOTALL | re.IGNORECASE,
)

_LIST_ITEM = re.compile(r"^( *)([-*+]|[0-9]+[.)]) ")


def _process_table_cell(cell):
    """Converts the text of one table cell to a single line of RST inline
    markup. Cells are processed on their own (instead of as part of the
    comment) so that nothing downstream can split a table row apart.
    """
    # A lone ^ means "merge with the cell above". _replace_tables_in_text()
    # turns those into row spans, so one only reaches here when there is no
    # cell above to merge into (e.g., in the header row); leave it blank.
    if cell.strip() == "^":
        return ""
    cell = " " + cell + " "
    cell = re.sub(r"</?(?:ul|ol)[^>]*>", " ", cell, flags=re.IGNORECASE)
    cell = re.sub(r"<li[^>]*>", " • ", cell, flags=re.IGNORECASE)
    cell = re.sub(r"</li>", " ", cell, flags=re.IGNORECASE)
    cell = markdown_to_restructuredtext(cell)
    cell = replace_html_tags(cell)
    cell = cell.replace("``true``", "``True``")
    cell = cell.replace("``false``", "``False``")
    cell = replace_exceptions(cell)
    cell = process_doxygen_commands(cell)
    cell = re.sub(r"\s+", " ", cell).strip()
    # A bare pipe would start an RST substitution reference. Pipes inside
    # inline literals are left alone, since escapes are not interpreted there.
    parts = re.split(r"(``.*?``)", cell)
    cell = "".join(
        part if i % 2 == 1 else part.replace("|", "\\|")
        for i, part in enumerate(parts)
    )
    # Escape text that would otherwise be read as a list marker or comment.
    if re.match(r"^([-*+]|\.\.|[0-9]+[.)]|[A-Za-z][.)])(\s|$)", cell):
        cell = "\\" + cell
    return cell


def _display_width(text):
    """Returns the number of terminal columns taken by text, which is how
    docutils measures the alignment of a grid table.
    """
    width = 0
    for ch in text:
        if unicodedata.combining(ch):
            continue
        width += 2 if unicodedata.east_asian_width(ch) in ("W", "F") else 1
    return width


def _grid_table_to_rst(cells, nrows, ncols, header_rows):
    """Formats cells as an RST grid table, which (unlike a list-table) can
    express cells that span rows or columns. Each cell is a tuple of
    (row, col, rowspan, colspan, text), where text is already processed.
    """
    # owner[r][c] is the index of the cell that covers position (r, c).
    owner = [[None] * ncols for _ in range(nrows)]
    for index, (r, c, rs, cs, _) in enumerate(cells):
        for dr in range(rs):
            for dc in range(cs):
                owner[r + dr][c + dc] = index

    # Choose column widths wide enough for every cell.
    widths = [3] * ncols
    for _, c, _, cs, text in cells:
        if cs == 1:
            widths[c] = max(widths[c], _display_width(text) + 2)
    for _, c, _, cs, text in cells:
        if cs > 1:
            have = sum(widths[c : c + cs]) + (cs - 1)
            need = _display_width(text) + 2
            if have < need:
                widths[c + cs - 1] += need - have

    # hline[line][c]: is there a horizontal edge above row `line` at column c?
    # vline[r][x]: is there a vertical edge left of column x in row r?
    hline = [
        [
            line in (0, nrows) or owner[line - 1][c] != owner[line][c]
            for c in range(ncols)
        ]
        for line in range(nrows + 1)
    ]
    vline = [
        [
            x in (0, ncols) or owner[r][x - 1] != owner[r][x]
            for x in range(ncols + 1)
        ]
        for r in range(nrows)
    ]

    def border(line):
        fill = "=" if line == header_rows and 0 < header_rows else "-"
        out = ""
        for x in range(ncols + 1):
            up = line > 0 and vline[line - 1][x]
            down = line < nrows and vline[line][x]
            left = x > 0 and hline[line][x - 1]
            right = x < ncols and hline[line][x]
            if (up or down) and (left or right):
                out += "+"
            elif left or right:
                out += fill
            elif up or down:
                out += "|"
            else:
                out += " "
            if x < ncols:
                out += (fill if hline[line][x] else " ") * widths[x]
        return out

    def row_line(r):
        out = ""
        c = 0
        while c < ncols:
            index = owner[r][c]
            top, _, _, cs, text = cells[index]
            width = sum(widths[c : c + cs]) + (cs - 1)
            if r != top:
                text = ""
            padding = " " * (width - 1 - _display_width(text))
            out += "|" + (" " + text + padding if text else " " * width)
            c += cs
        return out + "|"

    lines = []
    for r in range(nrows):
        lines.append(border(r))
        lines.append(row_line(r))
    lines.append(border(nrows))
    return "\n".join(lines)


def _table_to_rst(cells, nrows, ncols, header_rows, caption=None):
    """Formats a table as RST, with no surrounding blank lines. `cells` is a
    list of (row, col, rowspan, colspan, text) tuples with unprocessed text.
    Uses a list-table unless some cell spans rows or columns.
    """
    cells = [
        (r, c, rs, cs, _process_table_cell(text))
        for r, c, rs, cs, text in cells
    ]
    # A span across the header boundary can't be drawn, so drop the header.
    for r, _, rs, _, _ in cells:
        if r < header_rows < r + rs:
            header_rows = 0
    # The directive requires at least one body row.
    if not 0 < header_rows < nrows:
        header_rows = 0

    if all(rs == 1 and cs == 1 for _, _, rs, cs, _ in cells):
        grid = [[""] * ncols for _ in range(nrows)]
        for r, c, _, _, text in cells:
            grid[r][c] = text
        lines = [".. list-table::"]
        if header_rows:
            lines.append(f"   :header-rows: {header_rows}")
        lines.append("")
        for row in grid:
            for i, text in enumerate(row):
                prefix = "   * -" if i == 0 else "     -"
                lines.append(f"{prefix} {text}".rstrip())
        body = "\n".join(lines)
        if caption:
            title = _process_table_cell(caption)
            body = body.replace(
                ".. list-table::", f".. list-table:: {title}", 1
            )
        return body

    grid = _grid_table_to_rst(cells, nrows, ncols, header_rows)
    if not caption:
        return grid
    title = _process_table_cell(caption)
    indented = "\n".join("   " + line for line in grid.split("\n"))
    return f".. table:: {title}\n\n{indented}"


# A separator row must contain a pipe; otherwise any line containing a pipe
# followed by a horizontal rule would be mistaken for a table header.
_MARKDOWN_TABLE_SEPARATOR = re.compile(
    r"^\s*\|?\s*:?-+:?\s*(\|\s*:?-+:?\s*)+\|?\s*$"
    r"|^\s*\|\s*:?-+:?\s*\|?\s*$"
    r"|^\s*:?-+:?\s*\|\s*$"
)


def _split_markdown_row(line):
    line = line.strip()
    line = line.removeprefix("|")
    if line.endswith("|") and not line.endswith("\\|"):
        line = line[:-1]
    cells = re.split(r"(?<!\\)\|", line)
    return [cell.replace("\\|", "|") for cell in cells]


def _replace_tables_in_text(s, tables):
    def stash(rst, extra_indent=0):
        tables.append((rst, extra_indent))
        token = _TABLE_PLACEHOLDER.format(len(tables) - 1)
        return f"\n\n{token}\n\n"

    # HTML tables.
    def html_table(match):
        body = match.group(1)
        if re.search(r"<table", body, re.IGNORECASE):
            return match.group(0)  # Nested tables are not supported.
        caption = None
        caption_match = re.search(
            r"<caption[^>]*>(.*?)</caption\s*>", body, re.DOTALL | re.IGNORECASE
        )
        if caption_match:
            caption = caption_match.group(1)
        cells = []
        occupied = set()
        header_rows = 0
        still_header = True
        r = 0
        for row_match in re.finditer(
            r"<tr(?:\s[^>]*)?>(.*?)</tr\s*>", body, re.DOTALL | re.IGNORECASE
        ):
            found = re.findall(
                r"<(t[hd])((?:\s[^>]*)?)>(.*?)</t[hd]\s*>",
                row_match.group(1),
                re.DOTALL | re.IGNORECASE,
            )
            if not found:
                continue
            c = 0
            for _, attrs, text in found:
                while (r, c) in occupied:
                    c += 1
                spans = []
                for name in ("rowspan", "colspan"):
                    m = re.search(
                        name + r"\s*=\s*[\"']?([0-9]+)", attrs, re.IGNORECASE
                    )
                    spans.append(max(1, int(m.group(1))) if m else 1)
                rs, cs = spans
                cells.append((r, c, rs, cs, text))
                for dr in range(rs):
                    for dc in range(cs):
                        occupied.add((r + dr, c + dc))
                c += cs
            tags = [tag.lower() for tag, _, _ in found]
            if still_header and all(tag == "th" for tag in tags):
                header_rows += 1
            else:
                still_header = False
            r += 1
        if not cells:
            return match.group(0)
        nrows = r
        ncols = max(c + cs for _, c, _, cs, _ in cells)
        # Don't let a rowspan run off the bottom of the table.
        cells = [
            (r, c, min(rs, nrows - r), cs, text) for r, c, rs, cs, text in cells
        ]
        return stash(_table_to_rst(cells, nrows, ncols, header_rows, caption))

    s = re.sub(
        r"<table(?:\s[^>]*)?>(.*?)</table\s*>",
        html_table,
        s,
        flags=re.DOTALL | re.IGNORECASE,
    )

    # Markdown pipe tables: a header row, a separator row, then body rows.
    lines = s.split("\n")
    result = []
    i = 0
    while i < len(lines):
        if (
            "|" in lines[i]
            and i + 1 < len(lines)
            and _MARKDOWN_TABLE_SEPARATOR.match(lines[i + 1])
            and len(_split_markdown_row(lines[i]))
            == len(_split_markdown_row(lines[i + 1]))
        ):
            header = _split_markdown_row(lines[i])
            rows = [header]
            first = i
            i += 2
            # Body rows must have the same number of cells as the header, so
            # that prose which happens to contain a pipe ends the table.
            while (
                i < len(lines)
                and "|" in lines[i]
                and len(_split_markdown_row(lines[i])) == len(header)
            ):
                rows.append(_split_markdown_row(lines[i]))
                i += 1
            # If the table sits inside a list item, indent it to match.
            indent = len(lines[first]) - len(lines[first].lstrip())
            extra_indent = 0
            for j in range(first - 1, -1, -1):
                if not lines[j].strip():
                    break
                item = _LIST_ITEM.match(lines[j])
                if item and len(item.group(1)) < indent:
                    extra_indent = len(item.group(2)) + 1
                    break
            # A lone ^ merges a cell into the one above it.
            cells = []
            owner = {}
            for r, row in enumerate(rows):
                for c, text in enumerate(row):
                    if text.strip() == "^" and (r - 1, c) in owner:
                        index = owner[(r - 1, c)]
                        r0, c0, rs, cs, text0 = cells[index]
                        cells[index] = (r0, c0, rs + 1, cs, text0)
                        owner[(r, c)] = index
                    else:
                        cells.append((r, c, 1, 1, text))
                        owner[(r, c)] = len(cells) - 1
            table = _table_to_rst(cells, len(rows), len(header), header_rows=1)
            result.append(stash(table, extra_indent))
        else:
            result.append(lines[i])
            i += 1
    return "\n".join(result)


def replace_tables(s, tables):
    """Converts Doxygen tables (both Markdown pipe tables and HTML <table>
    elements) into reStructuredText list-table directives, or grid tables
    when a cell spans rows or columns. Each table is stashed in `tables` and
    replaced by a placeholder paragraph, so that the later passes (HTML tags,
    Doxygen commands, reflow) cannot disturb it. Call restore_tables() once
    those passes are done. Tables within verbatim regions (code blocks,
    <pre>, etc.) are left alone.
    """
    parts = _VERBATIM_REGIONS.split(s)
    for i in range(0, len(parts), 2):
        parts[i] = _replace_tables_in_text(parts[i], tables)
    return "".join(parts)


def restore_tables(s, tables):
    """Substitutes the placeholders left by replace_tables()."""

    def restore(match):
        rst, extra_indent = tables[int(match.group(2))]
        indent = " " * (len(match.group(1)) + extra_indent)
        return "\n".join(
            (indent + line) if line else line for line in rst.split("\n")
        )

    return _TABLE_PLACEHOLDER_LINE.sub(restore, s)


def markdown_to_restructuredtext(s):
    # TODO(jamiesnape): Find a third-party library do this?
    # Convert _italics_ to *italics*.
    s = re.sub(
        r"([\s\-,;:!.()]+)_([^\s_]+|[^\s_].*?[^\s_])_([\s\-,;:!.()]+)",
        r"\1*\2*\3",
        s,
        flags=re.DOTALL,
    )
    # Convert __bold__ to **bold**.
    s = re.sub(
        r"([\s\-,;:!.()]+)__([^\s_]+|[^\s_].*?[^\s_])__([\s\-,;:!.()]+)",
        r"\1**\2**\3",
        s,
        flags=re.DOTALL,
    )
    # Convert `typewriter` to ``typewriter``.
    s = re.sub(
        r"([\s\-,;:!.()]+)`([^\s`]|[^\s`].*?[^\s`])`([\s\-,;:!.()]+)",
        r"\1``\2``\3",
        s,
        flags=re.DOTALL,
    )
    # Convert [Link](https://example.org) to `Link <https://example.org>`_.
    s = re.sub(r"\[(.*?)\]\(([\w:.?/#]+)\)", r"`\1 <\2>`_", s, flags=re.DOTALL)

    s = re.sub(r"#### (.*?) ####", r"\n*\1*\n", s)
    s = re.sub(r"#### (.*?)", r"\n*\1*\n", s)
    s = re.sub(r"### (.*?) ###", r"\n**\1**\n", s)
    s = re.sub(r"### (.*?)", r"\n**\1**\n", s)

    s = replace_with_header(r"## (.*?) ##", "-", s)
    s = replace_with_header(r"## (.*?)", "-", s)
    s = replace_with_header(r"# (.*?) #", "=", s)
    s = replace_with_header(r"# (.*?)", "=", s)
    return s


def replace_with_header(pattern, token, s, **kwargs):
    def repl(match):
        return f"\n{match.group(1)}\n{token * len(match.group(1))}\n"

    return re.sub(pattern, repl, s, **kwargs)


def replace_html_tags(s):
    s = re.sub(r"<tt>(.*?)</tt>", r"``\1``", s, flags=re.DOTALL | re.IGNORECASE)
    s = re.sub(
        r"<pre>(.*?)</pre>",
        r"```\n\1\n```\n",
        s,
        flags=re.DOTALL | re.IGNORECASE,
    )
    s = re.sub(r"<em>(.*?)</em>", r"*\1*", s, flags=re.DOTALL | re.IGNORECASE)
    s = re.sub(r"<b>(.*?)</b>", r"**\1**", s, flags=re.DOTALL | re.IGNORECASE)

    s = re.sub(r"<li>", r"\n\n* ", s, flags=re.IGNORECASE)
    s = re.sub(r"</?ol( start=[0-9]+)?>", r"", s, flags=re.IGNORECASE)
    s = re.sub(r"</?ul>", r"", s, flags=re.IGNORECASE)
    s = re.sub(r"</li>", r"\n\n", s, flags=re.IGNORECASE)

    s = re.sub(r"</?dl>", r"", s, flags=re.IGNORECASE)
    s = re.sub(r"<dt>(.*?)</dt>", r"\n**\1**\n", s, flags=re.IGNORECASE)
    s = re.sub(r"</?dd>", r"", s, flags=re.IGNORECASE)

    s = re.sub(
        r'<a href="([\w:.?/#]+)">(.*?)</a>',
        r"`\2 <\1>`_",
        s,
        flags=re.DOTALL | re.IGNORECASE,
    )

    s = re.sub(r"<br/?>", r"\n\n", s, flags=re.IGNORECASE)

    s = replace_with_header(r"<h1>(.*?)</h1>", "=", s, flags=re.IGNORECASE)
    s = replace_with_header(r"<h2>(.*?)</h2>", "-", s, flags=re.IGNORECASE)
    s = re.sub(r"<h3>(.*?)</h3>", r"\n**\1**\n", s, flags=re.IGNORECASE)
    s = re.sub(r"<h4>(.*?)</h4>", r"\n*\1*\n", s, flags=re.IGNORECASE)
    return s


def replace_exceptions(s):
    s = s.replace("std∷bad_alloc", "MemoryError")
    s = s.replace("std∷bad_any_cast", "RuntimeError")
    s = s.replace("std∷bad_array_new_length", "MemoryError")
    s = s.replace("std∷bad_cast", "RuntimeError")
    s = s.replace("std∷bad_exception", "RuntimeError")
    s = s.replace("std∷bad_function_call", "RuntimeError")
    s = s.replace("std∷bad_optional_access", "RuntimeError")
    s = s.replace("std∷bad_typeid", "RuntimeError")
    s = s.replace("std∷bad_variant_access", "RuntimeError")
    s = s.replace("std∷bad_weak_ptr", "RuntimeError")
    s = s.replace("std∷domain_error", "ValueError")
    s = s.replace("std∷exception", "RuntimeError")
    s = s.replace("std∷future_error", "RuntimeError")
    s = s.replace("std∷invalid_argument", "ValueError")
    s = s.replace("std∷length_error", "ValueError")
    s = s.replace("std∷logic_error", "RuntimeError")
    s = s.replace("std∷out_of_range", "ValueError")
    s = s.replace("std∷overflow_error", "RuntimeError")
    s = s.replace("std∷range_error", "ValueError")
    s = s.replace("std∷regex_error", "RuntimeError")
    s = s.replace("std∷runtime_error", "RuntimeError")
    s = s.replace("std∷system_error", "RuntimeError")
    s = s.replace("std∷underflow_error", "RuntimeError")
    return s


def process_doxygen_commands(s):
    # Doxygen tags
    cpp_group = r"([\w:*()]+)"
    param_group = r"([\[\w,\]]+)"

    s = re.sub(rf"[@\\][cp]\s+{cpp_group}", r"``\1``", s)
    s = re.sub(rf"[@\\](?:a|e|em)\s+{cpp_group}", r"*\1*", s)
    s = re.sub(rf"[@\\]b\s+{cpp_group}", r"**\1**", s)
    s = re.sub(
        rf"[@\\]param{param_group}?\s+{cpp_group}",
        r"\n\n$Parameter ``\2``:\n\n",
        s,
    )
    s = re.sub(
        rf"[@\\]tparam{param_group}?\s+{cpp_group}",
        r"\n\n$Template parameter ``\2``:\n\n",
        s,
    )
    # TODO (betsymcphail): Not tested
    s = re.sub(rf"[@\\]retval\s+{cpp_group}", r"\n\n$Returns ``\1``:\n\n", s)

    # Ordering is significant for command names with a common prefix.
    for in_, out_ in (
        ("result", "Returns"),
        ("returns", "Returns"),
        ("return", "Returns"),
        ("attention", "Attention"),
        ("authors", "Authors"),
        ("author", "Authors"),
        ("bug", "Bug report"),
        ("copyright", "Copyright"),
        ("date", "Date"),
        ("deprecated", "Deprecated"),
        ("exception", "Raises"),
        ("invariant", "Invariant"),
        ("note", "Note"),
        ("post", "Postcondition"),
        ("pre", "Precondition"),
        ("remarks", "Remark"),
        ("remark", "Remark"),
        ("sa", "See also"),
        ("see", "See also"),
        ("since", "Since"),
        ("extends", "Extends"),
        ("throws", "Raises"),
        ("throw", "Raises"),
        ("test", "Test case"),
        ("todo", "Todo"),
        ("version", "Version"),
        ("warning", "Warning"),
    ):
        s = re.sub(rf"[@\\]{in_}\s*", rf"\n\n${out_}:\n\n", s)

    s = re.sub(r"[@\\]details\s*", r"\n\n", s)
    s = re.sub(r"[@\\](?:brief|short)\s*", r"", s)
    # TODO (betsymcphail): Not tested
    s = re.sub(r"[@\\]ref\s+", r"", s)

    # Convert from
    #  @code{.foo}
    #  stuff
    #  @endcode
    # to
    #  ```foo
    #  stuff
    #  ```
    for start_, end_ in (("code", "endcode"), ("verbatim", "endverbatim")):
        s = re.sub(
            rf"[@\\]{start_}(?:\{{\.(\w+)\}})?\s?(.*?)\s?[@\\]{end_}",
            r"```\1\n\2\n```\n",
            s,
            flags=re.DOTALL,
        )

    s = process_doxygen_to_sphinx(s)

    # TODO (betsymcphail): Not tested
    s = re.sub(r"[@\\](?:end)?htmlonly\s+", r"", s)

    # These commands are always prefixed with an @ sign.
    # TODO (betsymcphail): Not tested
    s = re.sub(r"@[{}]\s*", r"", s)

    # Doxygen list commands.
    # TODO (betsymcphail): Not tested
    s = re.sub(r"[@\\](?:arg|li)\s+", r"\n\n* ", s)

    # Doxygen sectioning commands.
    s = replace_with_header(r"[@\\]section\s+\w+\s+(.*)", "=", s)
    s = replace_with_header(r"[@\\]subsection\s+\w+\s+(.*)", "-", s)
    s = re.sub(r"[@\\]subsubsection\s+\w+\s+(.*)", r"\n**\1**\n", s)

    # Doxygen LaTeX commands.
    s = re.sub(
        r"[@\\]f\$\s*(.*?)\s*[@\\]f\$", r":math:`\1`", s, flags=re.DOTALL
    )
    s = re.sub(
        r"[@\\]f\[\s*(.*?)\s*[@\\]f\]",
        r"\n\n.. math:: \1\n\n",
        s,
        flags=re.DOTALL,
    )
    s = re.sub(
        r"[@\\]f\{([\w*]+)\}\s*(.*?)\s*[@\\]f\}",
        r"\n\n.. math:: \\begin{\1}\2\\end{\1}\n\n",
        s,
        flags=re.DOTALL,
    )

    # Drake-specific Doxygen aliases.
    # TODO (betsymcphail): Not tested
    s = re.sub(r"[@\\]default\s+", r"\n$*Default:* ", s)
    s = re.sub(
        r"[@\\]experimental\s+",
        "\n\n$Warning:\n\nThis feature is considered to be **experimental** and may change or be removed at any time, without any deprecation notice ahead of time.\n\n",  # noqa
        s,
    )
    # In pydrake docs, "python details" are not actually details; erase the
    # markers for `<detail>` so that it is always shown.
    s = re.sub(r"[@\\]python_details_(?:begin|end)", r"", s)

    # Omit tparam scalar type boilerplate; the python documentation already
    # presents this information in a more useful manner.
    # TODO (betsymcphail): Not tested
    s = re.sub(r"[@\\]tparam_default_scalar\s+", r"", s)
    s = re.sub(r"[@\\]tparam_nonsymbolic_scalar\s+", r"", s)
    s = re.sub(r"[@\\]tparam_double_only\s+", r"", s)

    # Remove these commands that take no argument. Ordering is significant for
    # command names with a common prefix.
    for cmd_ in (
        "~english",
        "~",
        "callergraph",
        "callgraph",
        "hidecallergraph",
        "hidecallgraph",
        "hideinitializer",
        "nosubgrouping",
        "privatesection",
        "private",
        "protectedsection",
        "protected",
        "publicsection",
        "public",
        "pure",
        "showinitializer",
        "static",
        "tableofcontents",
    ):
        s = re.sub(rf"[@\\]{cmd_}\s+", r"", s)

    # Remove these commands and their one optional single-word argument.
    # TODO (betsymcphail): Not tested
    for cmd_ in [
        "dir",
        "file",
    ]:
        s = re.sub(rf"[@\\]{cmd_}( +[\w:./]+)?\s+", r"", s)

    # Remove these commands and their one optional single-line argument.
    # TODO (betsymcphail): Not tested
    for cmd_ in [
        "mainpage",
        "nameoverload",
    ]:
        s = re.sub(rf"[@\\]{cmd_}( +.*)?\s+", r"", s)

    # Remove these commands and their one single-word argument. Ordering is
    # significant for command names with a common prefix.
    for cmd_ in [
        "anchor",
        "copybrief",
        "copydetails",
        "copydoc",
        "def",
        "dontinclude",
        "enum",
        "example",
        "extends",
        "htmlinclude",
        "idlexcept",
        "implements",
        "includedoc",
        "includelineno",
        "include",
        "latexinclude",
        "memberof",
        "namespace",
        "package",
        "relatedalso",
        "related",
        "relatesalso",
        "relates",
        "verbinclude",
    ]:
        s = re.sub(rf"[@\\]{cmd_}\s+[\w:.]+\s+", r"", s)

    # Remove these commands and their one single-line argument. Ordering is
    # significant for command names with a common prefix.
    for cmd_ in [
        "addindex",
        "fn",
        "ingroup",
        "line",
        "property",
        "skipline",
        "skip",
        "typedef",
        "until",
        "var",
    ]:
        s = re.sub(rf"[@\\]{cmd_}\s+.*\s+", r"", s)

    # Remove this command and its one single-word argument and one
    # optional single-word argument.
    # TODO (betsymcphail): Not tested
    s = re.sub(r"[@\\]headerfile\s+[\w:.]+( +[\w:.]+)?\s+", r"", s)

    # Remove these commands and their one single-word argument and one
    # optional single-line argument.
    # TODO (betsymcphail): Not tested
    for cmd_ in [
        "addtogroup",
        "weakgroup",
    ]:
        s = re.sub(rf"[@\\]{cmd_}\s+[\w:.]( +.*)?\s+", r"", s)

    # Remove these commands and their one single-word argument and one
    # single-line argument. Ordering is significant for command names with a
    # common prefix.
    # TODO (betsymcphail): Not tested
    for cmd_ in [
        "snippetdoc",
        "snippetlineno",
        "snippet",
    ]:
        s = re.sub(rf"[@\\]{cmd_}\s+[\w:.]\s+.*\s+", r"", s)

    # Remove these commands and their one single-word argument and two
    # optional single-word arguments.
    for cmd_ in [
        "category",
        "class",
        "interface",
        "protocol",
        "struct",
        "union",
    ]:
        s = re.sub(rf"[@\\]{cmd_}\s+[\w:.]+( +[\w:.]+){{0,2}}\s+", r"", s)

    # Remove these commands and their one single-word argument, one optional
    # quoted argument, and one optional single-word arguments.
    # TODO (betsymcphail): Not tested
    for cmd_ in [
        "diafile",
        "dotfile",
        "mscfile",
    ]:
        s = re.sub(
            rf'[@\\]{cmd_}\s+[\w:.]+(\s+".*?")?(\s+[\w:.]+=[\w:.]+)?s+',
            r"",
            s,
        )

    # Remove these pairs of commands and any text in between.
    for start_, end_ in (
        ("cond", "endcond"),
        ("docbookonly", "enddocbookonly"),
        ("dot", "enddot"),
        ("internal", "endinternal"),
        ("latexonly", "endlatexonly"),
        ("manonly", "endmanonly"),
        ("msc", "endmsc"),
        ("rtfonly", "endrtfonly"),
        ("secreflist", "endsecreflist"),
        ("startuml", "enduml"),
        ("xmlonly", "endxmlonly"),
    ):
        s = re.sub(
            rf"[@\\]{start_}\s?(.*?)\s?[@\\]{end_}",
            r"",
            s,
            flags=re.DOTALL,
        )

        # Some command pairs may bridge multiple comment blocks, so individual
        # start and end commands may appear alone.
        s = re.sub(rf"[@\\]{start_}\s+", r"", s)
        s = re.sub(rf"[@\\]{end_}\s+", r"", s)

    # Remove auto-linking character. Be sure to remove only leading % signs.
    # TODO (betsymcphail): Not tested
    s = re.sub(r"(\s+)%(\S+)", r"\1\2", s)

    # Doxygen escaped characters.
    # TODO (betsymcphail): Not tested
    s = re.sub(r"[@\\]n\s+", r"\n\n", s)

    # Ordering of ---, --, @, and \ is significant.
    # TODO (betsymcphail): Not tested
    for escaped_ in (
        "---",
        "--",
        "::",
        r"\.",
        '"',
        "&",
        "#",
        "%",
        "<",
        ">",
        r"\$",
        "@",
        r"\\",
    ):
        s = re.sub(rf"[@\\]({escaped_})", r"\1", s)

    return s


def reflow(s):
    wrapper = textwrap.TextWrapper()
    wrapper.break_long_words = False
    wrapper.break_on_hyphens = False
    wrapper.drop_whitespace = True
    wrapper.expand_tabs = True
    wrapper.replace_whitespace = True
    wrapper.width = 70
    wrapper.initial_indent = wrapper.subsequent_indent = ""

    # The mapping from the "ext" token in @code{.ext} to the Pygments
    # lexer name (listed at https://pygments.org/docs/lexers/).
    file_ext_to_lang = {
        "": "c++",
        "c++": "c++",
        "cc": "c++",
        "cpp": "c++",
        "python": "python",
        "py": "python",
        "txt": "txt",
        "yaml": "yaml",
    }

    result = ""
    in_code_segment = False
    code_segment_lang = None
    for x in re.split(r"(```[A-Za-z0-9_+]*)", s):
        if x.startswith("```"):
            if not in_code_segment:
                file_ext = x[3:]
                code_segment_lang = file_ext_to_lang[file_ext]
                if code_segment_lang == "c++":
                    # In pydrake docs, C++ code snippets should be collapsed
                    # under an HTML <details> element, to avoid clutter.
                    result += (
                        "\n.. raw:: html\n\n    "
                        "<details>"
                        "<summary>Click to expand C++ code...</summary>"
                        "\n"
                    )
                result += f"\n.. code-block:: {code_segment_lang}\n"
            else:
                result += "\n\n"
                if code_segment_lang == "c++":
                    result += ".. raw:: html\n\n    </details>\n\n"
                code_segment_lang = None
            in_code_segment = not in_code_segment
        elif in_code_segment:
            result += "    ".join(("\n" + x.strip()).splitlines(True))
        else:
            in_rst_directive = False
            for y in re.split(r"(?: *\n){2,}", x):
                # Do not reflow rst directives.
                # These appear, for instance, through our implementation of the
                # custom @system / @endsystem tags.
                if in_rst_directive:
                    if y[:3] != "   ":
                        in_rst_directive = False
                        # and program flow continues in this loop.
                    else:
                        result += y + "\n\n"
                        continue
                elif y[:3] == ".. ":
                    in_rst_directive = True
                    result += y + "\n\n"
                    continue
                lines = re.split(r"(?: *\n *)", y)
                # Do not reflow lists or section headings.
                if re.match(r"^(?:[*+\-]|[0-9]+[.)]) ", lines[0]) or (
                    len(lines) > 1
                    and (
                        lines[1] == "=" * len(lines[0])
                        or lines[1] == "-" * len(lines[0])
                    )
                ):
                    result += y + "\n\n"
                else:
                    wrapped = wrapper.fill(re.sub(r"\s+", " ", y).strip())
                    if len(wrapped) > 0 and wrapped[0] == "$":
                        result += wrapped[1:] + "\n"
                        wrapper.initial_indent = wrapper.subsequent_indent = (
                            " " * 4
                        )
                    else:
                        if len(wrapped) > 0:
                            result += wrapped + "\n\n"
                        wrapper.initial_indent = wrapper.subsequent_indent = ""
    return result.rstrip().lstrip("\n")
