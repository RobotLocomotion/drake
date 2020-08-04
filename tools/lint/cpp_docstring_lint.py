"""
Implements a simple parser which:

- Can report lint errors if a canonical docstring format is not adhered to.
- Can report lint errors if docstrings and comments being intermixed may
  confused `mkdoc.py`.
- Tokenizes a file based on lines, specifically geared towards docstrings.
  This is filled with heuristics, but can lint / reformat the codebase quickly.
- Can fix all docstrings to a standard overall form, while trying to maintain
  the docstring formatting (e.g. indentations, markdown, etc).
"""

import argparse
from functools import partial
import os
from textwrap import dedent
import sys

from drake.tools.lint.util import find_all_sources


def parse_line_tokens(text):
    """Parses the start and end "tokens" for a line. These tokens are meant to
    deal with comments.

    The input `text` should not start or end with any whitespace."""
    start_token = ""
    end_token = ""
    if len(text) > 0:
        # Text must already be stripped..
        assert text[0] != " ", repr(text)
        assert text[-1] != " "
        if text.startswith("///") and not text.startswith("////"):
            start_token = "///"
        elif text.startswith("//"):
            start_token = "//"
        elif text.startswith("/**"):
            start_token = "/**"
        elif text.startswith("/*"):
            start_token = "/*"
        elif text.startswith("* ") or text.strip() == "*":
            start_token = "*"
        has_start_token_for_end = (start_token in ("", "*", "/*", "/**"))
        if text.endswith("*/") and has_start_token_for_end:
            end_token = "*/"
    return start_token, end_token


class FileLine:
    """Indicates a line in a file with a given number."""
    def __init__(self, filename, num, raw_line):
        self.filename = filename
        self.num = num
        assert "\n" not in raw_line, (
            f"Raw line should not have newline: {repr(raw_line)}")
        self.raw_line = raw_line
        without_indent = self.raw_line.lstrip()
        self.indent = self.raw_line[:-len(without_indent)]
        self.start_token, self.end_token = parse_line_tokens(without_indent)
        self._set_text(prefix=f"{self.indent}{self.start_token}")

    def _set_text(self, prefix):
        if self.raw_line.strip() == "":
            self.text = ""
            return
        assert self.raw_line.startswith(prefix), (
            repr(self.raw_line), repr(prefix))
        self.text = self.raw_line[len(prefix):]
        if self.end_token == "*/":
            self.text = self.text[:-len(self.end_token)].rstrip("*").rstrip()

    def reset_indent(self, indent):
        """Readjusts indentation to given string, also adjusting self.text."""
        self.indent = indent
        self._set_text(self.indent)

    def __repr__(self):
        return (
            f"<{self.__class__.__name__} "
            f"{self.filename}:{self.num + 1} "
            f"start_token={repr(self.start_token)} "
            f"end_token={repr(self.end_token)}"
            f">")

    def __str__(self):
        return self.format()

    def format(self, num_width=None):
        """Shows file and line number with given fixed numbering for line
        number."""
        if num_width is None:
            num_width = len(str(self.num))
        return f"{self.filename}:{self.num + 1:<{num_width}}: {self.raw_line}"


def format_lines(lines, abbrev=False):
    """Prints lines with line numbers.
    If abbrev=True and there are more than 5 lines, this will only show the
    first 2 and last 2 lines with an ellipsis between."""
    num_width = len(str(lines[-1].num))
    text_lines = [x.format(num_width) for x in lines]
    if abbrev and len(lines) > 5:
        text_lines = [
            text_lines[0], text_lines[1], "...",
            text_lines[-2], text_lines[-1]]
    return "\n".join(text_lines)


class UserFormattingError(RuntimeError):
    """Indicates a user formatting error."""
    pass


class MultilineToken:
    """Base class for indicating a multiline token."""
    def __init__(self):
        self.lines = []

    def add_line(self, line):
        """Attempts to add a line. If the line cannot be handled by this token
        type, then False is returned."""
        assert isinstance(line, FileLine), line
        if len(self.lines) > 0:
            if line.num != self.lines[-1].num + 1:
                print(self.lines[-1])
                print(line)
                assert False
        self.lines.append(line)
        return True

    def to_raw_lines(self):
        """Convert to raw lines."""
        return [line.raw_line for line in self.lines]

    def assert_finished(self):
        """Can be used to indicate that the given token has not yet finished
        parsing."""
        pass

    def __str__(self):
        return format_lines(self.lines)

    def __len__(self):
        return len(self.lines)

    def __repr__(self):
        return (
            f"<{self.__class__.__name__} {self.lines[0].filename}:"
            f"{self.lines[0].num + 1}-{self.lines[-1].num + 1}>")


def multiline_token_cls_list():
    """Indicates all non-generic multiline tokens that are relevant to this
    script."""
    # N.B. Does not contain `GenericMultilineToken`.
    return (
        # Docstrings.
        TripleSlashMultilineToken,
        DoubleStarMultilineToken,
        # Comments.
        DoubleSlashMultilineToken,
        SingleStarMultilineToken,
        # Whitespace.
        WhitespaceMultilineToken,
    )


class GenericMultilineToken(MultilineToken):
    """Indicates a set of lines that do not fit any other token type."""
    def add_line(self, line):
        for cls in multiline_token_cls_list():
            if cls().add_line(line):
                return False
        return super().add_line(line)


class WhitespaceMultilineToken(MultilineToken):
    """Indicates a set of lines that are purely whitespace."""
    def add_line(self, line):
        if line.raw_line.strip() == "":
            return super().add_line(line)
        else:
            return False


def dedent_lines(lines, token, require_nonragged):
    """
    Dedents all lines.

    token: For debugging purposes. Will print out an error with debug string.
    require_nonragged: If specified, then the first line must be the
    leftmost indented line.
    """
    text = dedent("\n".join(lines))
    if require_nonragged and len(text) > 0:
        # Ensure that first nonempty line does not start with whitespace
        # (ragged indentation?).
        if text.lstrip("\n")[0] == " ":
            raise UserFormattingError(
                f"Must not have ragged indentation:\n{token}")
    return text


def remove_empty_leading_and_trailing_lines(raw_lines):
    """Removes empty leading and trailing lines."""
    while raw_lines[0].strip() == "":
        del raw_lines[0]
    while raw_lines[-1].strip() == "":
        del raw_lines[-1]


class CommentMultilineToken(MultilineToken):
    """Abstract base class for indicating docstring multiline tokens."""
    def get_docstring_text(self):
        """Gets the docstring text with all preceding "cruft" stripped out.
        Preserves indicated whitespace."""
        text_lines = [line.text for line in self.lines]
        remove_empty_leading_and_trailing_lines(text_lines)
        return dedent_lines(text_lines, self, require_nonragged=True).strip()


class PrivateCommentMultilineToken(CommentMultilineToken):
    """Indicates a comment (non-docstring) multiline token.
    This is an abstract class."""
    pass


class DoubleSlashMultilineToken(PrivateCommentMultilineToken):
    """Indicates a block of comments starting with "//"."""
    def add_line(self, line):
        if line.start_token != "//":
            return False
        return super().add_line(line)


class SingleStarMultilineToken(PrivateCommentMultilineToken):
    """Indicates a comments of the form "/* ... */"."""
    def __init__(self):
        self._finished = False
        super().__init__()

    def add_line(self, line):
        if len(self.lines) == 0 and (
                line.start_token != "/*" or line.raw_line.endswith("\\")):
            # Don't even try.
            return False
        if self._finished:
            return False
        if line.end_token == "*/":
            self._finished = True
        return super().add_line(line)

    def assert_finished(self):
        if not self._finished:
            raise UserFormattingError(f"Not closed:\n{self}")


class DocstringMultilineToken(CommentMultilineToken):
    pass


class TripleSlashMultilineToken(DocstringMultilineToken):
    """Indicates a block of docstring comments starting with "///"."""
    def add_line(self, line):
        if line.start_token == "///":
            if line.text in ("@{", "@}"):
                # "///@{" triggers the "ragged indent" error. Fake it out.
                new = f"/// {line.text}"
                line = FileLine(line.filename, line.num, f"{line.indent}{new}")
            return super().add_line(line)
        else:
            raw = line.raw_line.strip()
            if raw in ("//@{", "//@}"):
                # Not really triple slash, but meh.
                new = f"/// {raw[2:]}"
                line = FileLine(line.filename, line.num, f"{line.indent}{new}")
                return super().add_line(line)
            else:
                return False


class DoubleStarMultilineToken(DocstringMultilineToken):
    """Indicates a multiline docstring comment of the form "/** ... */".
    This may have "*" in intermediate lines."""
    def __init__(self):
        self.secondary_type = None
        self._finished = False
        super().__init__()

    def assert_finished(self):
        if not self._finished:
            raise UserFormattingError(f"Not closed:\n{self}")

    def get_docstring_text(self):
        if self.lines[0].text == "":
            return super().get_docstring_text()
        else:
            text_lines = [line.text for line in self.lines]
            remove_empty_leading_and_trailing_lines(text_lines)
            text = text_lines[0].lstrip() + "\n"
            text += dedent_lines(text_lines[1:], self, require_nonragged=False)
            return text.strip()

    def add_line(self, line):
        if len(self.lines) == 0 and line.start_token != "/**":
            return False
        if self._finished:
            return False

        do_add_line = False

        if line.end_token == "*/":
            self._finished = True
            do_add_line = True

        if len(self.lines) == 0:
            do_add_line = True
            if line.text and line.text[0] == "*":
                text = line.text.lstrip("*")
                line = FileLine(
                    line.filename, line.num, f"{line.indent}/**{text}")
        else:
            if self.secondary_type is None:
                if line.start_token == "*":
                    self.secondary_type = "*"
                else:
                    self.secondary_type = ""
            if self.secondary_type == "":
                do_add_line = True
                # Reset indentation to match first line.
                line.reset_indent(self.lines[0].indent)
            else:
                if not do_add_line and line.start_token != "*":
                    lines_str = format_lines(self.lines + [line])
                    raise UserFormattingError(
                        f"Must continue with single star:\n"
                        f"{repr(line)}\n"
                        f"{lines_str}")
                do_add_line = True
        if not do_add_line:
            lines_str = format_lines(self.lines + [line])
            assert self._finished, (
                f"Needs termination:\n{lines_str}")
        if do_add_line:
            return super().add_line(line)


def make_new_multiline_token(line):
    """Creates a new MultilineToken class according to the first class that can
    consume the provided line."""
    for cls in multiline_token_cls_list():
        token = cls()
        if token.add_line(line):
            return token
    token = GenericMultilineToken()
    assert token.add_line(line), line
    return token


def multiline_tokenize(filename, raw_lines):
    """Tokenizes a set of raw lines, which are labeled as part of `filename`,
    into a set of multiline tokens."""
    tokens = []
    active_token = None
    for num, raw_line in enumerate(raw_lines):
        line = FileLine(filename, num, raw_line)
        if active_token is None:
            active_token = make_new_multiline_token(line)
        elif not active_token.add_line(line):
            active_token.assert_finished()
            tokens.append(active_token)
            active_token = make_new_multiline_token(line)
        assert active_token is not None
    active_token.assert_finished()
    tokens.append(active_token)
    return tokens


def reformat_docstring(docstring, *, public):
    """Reformats a DocstringMultilineToken into a canonical form."""
    MAX_LEN = 80
    indent = docstring.lines[0].indent
    spacing = ""
    text = docstring.get_docstring_text()
    # Can't have nested comments :(
    # Replace with magical D-style stuff?
    text = text.replace("*/", "+/").replace("/*", "/+")
    text_lines = text.split("\n")
    first_line = text_lines[0]

    def maybe_wrap(text, suffix):
        new_line = f"{indent}{text}"
        too_long = len(new_line) + len(suffix) > MAX_LEN
        should_extend = ("@endcode" in text or "</pre>" in text)
        if too_long or should_extend:
            return [
                new_line,
                f"{indent}{suffix}",
            ]
        else:
            return [f"{new_line}{suffix}"]

    if public:
        new_start = "/**"
        new_start_single = "///"
    else:
        new_start = "/*"
        new_start_single = "//"

    if len(text_lines) == 1:
        if "://" in first_line:
            # Weird behavior with bogus lint?
            return [f"{indent}{new_start_single} {first_line}"]
        new_lines = maybe_wrap(f"{new_start} {first_line}", " */")
    else:
        new_lines = [f"{indent}{new_start} {text_lines[0]}"]
        for line in text_lines[1:-1]:
            new_line = f"{indent}{spacing}{line}".rstrip()
            new_lines.append(new_line)
        last_line = text_lines[-1]
        new_lines += maybe_wrap(f"{spacing}{last_line}", " */")
    return new_lines


def reformat_docstring_token(token, *, public):
    """Reformats a multiline token for applying / checking lint."""
    if public:
        cls = DocstringMultilineToken
    else:
        cls = PrivateCommentMultilineToken
    if isinstance(token, cls):
        # Multi-pass for idempotent.
        # TODO(eric): Fix this.
        tokens = [token]
        first_line = token.lines[0]
        prev_lines = None
        for i in range(3):
            new_lines = reformat_docstring(token, public=public)
            if prev_lines is not None:
                if new_lines == prev_lines:
                    break
                else:
                    pass
            token = parse_single_multiline_token(
                new_lines, first_line.filename, first_line.num)
            tokens.append(token)
            prev_lines = new_lines
        else:
            for i, token in enumerate(tokens):
                print(f"i = {i}")
                print(token)
                print("---")
            assert False, "Bug in indempotent check"
        return new_lines
    else:
        return token.to_raw_lines()


class AbstractRegex:
    """Simple pattern matcher for a sequence of objects (of any time)."""

    class PatternGroup:
        """Base class for a part within given sequence."""
        def try_match(self, xs, index):
            """Takes a sequence and an index in the sequence, and returns the
            number of elements consume, or None if the part did not match."""
            raise NotImplemented

    class Single(PatternGroup):
        """Matches a single token that is indicated by the predicate."""
        def __init__(self, predicate):
            self._predicate = predicate

        def __repr__(self):
            return f"<Single {self._predicate}>"

        def try_match(self, xs, index):
            x = xs[index]
            if self._predicate(x):
                return [x]
            else:
                return None

    class Any(PatternGroup):
        """Matches zero or more continuous tokens that are indicated by the
        predicate."""
        def __init__(self, predicate):
            self._predicate = predicate

        def __repr__(self):
            return f"<Any {self._predicate}>"

        def try_match(self, xs, index):
            group = []
            while self._predicate(xs[index]) and index < len(xs):
                group.append(xs[index])
                index += 1
            return group

    class Match:
        """Indicates a match with a set of pattern groups."""
        def __init__(self):
            self._groups = []

        def add_group(self, group):
            self._groups.append(group)

        def groups(self):
            return list(self._groups)

    def __init__(self, parts):
        self._pattern_groups = list(parts)

    def __repr__(self):
        return f"<AbstractRegex {self._pattern_groups}>"

    def find_all(self, xs):
        """Finds all mathches and returns List[Match]."""
        matches = []
        index = 0
        while index < len(xs):
            start_index = index
            match = self.Match()
            for pattern_group in self._pattern_groups:
                if index == len(xs):
                    match = None
                    break
                group = pattern_group.try_match(xs, index)
                if group is not None:
                    index += len(group)
                    match.add_group(group)
                else:
                    match = None
                    break
            if match is not None:
                assert index > start_index
                matches.append(match)
            else:
                # Matching failed. Restart.
                index = start_index + 1
        return matches


def reorder_multiline_tokens(tokens, lint_errors):
    """Reorders tokens to a given order (or reports lint errors if linting).
    Used to report errors that make mkdoc.py choke."""
    mkdoc_issue = AbstractRegex([
        AbstractRegex.Single(is_meaningful_docstring_token),
        AbstractRegex.Any(is_whitespace_token),
        AbstractRegex.Single(is_comment_token_but_not_nolint),
        AbstractRegex.Single(is_generic_token_but_not_macro),
    ])
    matches = mkdoc_issue.find_all(tokens)
    for match in matches:
        (doc,), ws, (comment,), (generic,) = match.groups()
        if lint_errors is not None:
            lint_errors.append(LintError(
                text=(
                    "ERROR: Docstring must be placed directly next to symbol "
                    "for mkdoc.py"),
                error_lines=[
                    doc.lines[-1], comment.lines[0], generic.lines[0]],
            ))
        else:
            original = [doc] + ws + [comment, generic]
            start = tokens.index(original[0])
            for x in original:
                tokens.remove(x)
            comment.lines = [
                x for x in comment.lines if x.text.strip() != ""]
            new = [comment] + ws + [doc, generic]
            for i, x in enumerate(new):
                tokens.insert(start + i, x)
    return tokens


def reformat_private_docstring_tokens(tokens, lint_errors):
    maybe_private_docstring = AbstractRegex([
        AbstractRegex.Single(partial(is_meaningful_docstring_token, public=False)),
        AbstractRegex.Single(is_generic_token_but_not_macro),
    ])
    matches = maybe_private_docstring.find_all(tokens)
    for match in matches:
        (comment,), (generic,) = match.groups()
        original = [comment, generic]
        start = tokens.index(original[0])
        for x in original:
            tokens.remove(x)
        old_first_line = comment.lines[0]
        new_lines = reformat_docstring_token(comment, public=False)
        new_comment = parse_single_multiline_token(
            new_lines, old_first_line.filename, old_first_line.num)
        new = [new_comment, generic]
        for i, x in enumerate(new):
            tokens.insert(start + i, x)
    return tokens


def is_whitespace_token(token):
    """Predicate for a whitespace token."""
    return isinstance(token, WhitespaceMultilineToken)


def is_meaningful_docstring_token(token, *, public=True):
    """Predicate for a docstring token that isn't simply a doxygen
    directive."""
    if public:
        cls = DocstringMultilineToken
    else:
        cls = PrivateCommentMultilineToken
    if isinstance(token, cls):
        if len(token.lines) == 1:
            if token.lines[0].text.strip().startswith("@"):
                return False
        return True
    return False


def is_generic_token_but_not_macro(token):
    """Predicate for a generic token (possibly code) that is not a
    macro definition."""
    if isinstance(token, GenericMultilineToken):
        if not token.lines[0].text.startswith("#"):
            return True
    return False


def is_comment_token_but_not_nolint(token):
    """Predicate for a comment token that is not a "nolint" directive."""
    if isinstance(token, PrivateCommentMultilineToken):
        # TODO(eric.cousineau): Figure out how to make cpplint play nicely with
        # mkdoc.py?
        if token.lines[-1].text.strip().startswith("NOLINTNEXTLINE"):
            return False
        return True
    return False


class LintError:
    def __init__(self, text, error_lines=None, fix_lines=None):
        self.text = text
        self.error_lines = error_lines
        self.fix_lines = fix_lines

    def __str__(self):
        s = f"{self.text}\n"
        if self.error_lines is not None:
            s += f"{format_lines(self.error_lines)}\n"
        if self.fix_lines is not None:
            s += f"  should look like:\n{format_lines(self.fix_lines)}\n"
        return s

    def sorting_key(self):
        if self.error_lines is not None:
            line_num = self.error_lines[0].num
        else:
            line_num = float('inf')
        return (line_num, self.text)


def print_multiline_tokens(tokens):
    """Prints a list of tokens with newlines in between each."""
    for token in tokens:
        print(token)


def parse_single_multiline_token(raw_lines, filename, start_num):
    """Parse a set of isolated lines into a new token.

    All lines must be consumed in the parsing."""
    token = None
    for i, raw_line in enumerate(raw_lines):
        line = FileLine(filename, start_num + i, raw_line)
        if token is None:
            token = make_new_multiline_token(line)
        else:
            assert token.add_line(line), line
    token.assert_finished()
    return token


def lint_multiline_token(lint_errors, token, new_lines, verbose):
    """Detects if there are any changes between `token` and the re-processed
    lines from `new_lines."""
    assert lint_errors is not None
    first_line = token.lines[0]
    # Reparse to ensure that our new token is still valid.
    new_token = parse_single_multiline_token(
        new_lines, first_line.filename, first_line.num)
    # Compare.
    if token.to_raw_lines() != new_token.to_raw_lines():
        if verbose:
            error_lines = token.lines
            fix_lines = new_token.lines
        else:
            error_lines = token.lines[:3]
            fix_lines = new_token.lines[:3]
        lint_errors.append(LintError(
            text="ERROR: Docstring needs reformatting",
            error_lines=error_lines,
            fix_lines=fix_lines,
        ))


def is_ignored_file(relpath):
    # TODO(eric.cousineau): Figure out better heuristic for this.
    if relpath.startswith(("attic/", "tools/")):
        return True
    if relpath.startswith("gen/") or "/gen/" in relpath:
        return True
    if not relpath.endswith(".h"):
        return True
    return False


def check_or_apply_lint_on_tokens(
        tokens, lint_errors, verbose=False, maybe_private=True):
    tokens = reorder_multiline_tokens(tokens, lint_errors)
    # Replace docstrings with "re-rendered" version.
    new_lines = []
    for token in tokens:
        new_lines_i = reformat_docstring_token(token, public=True)
        if lint_errors is not None:
            lint_multiline_token(
                lint_errors, token, new_lines_i, verbose=verbose)
        new_lines += new_lines_i
    if maybe_private:
        tokens = reformat_private_docstring_tokens(tokens, lint_errors)
        new_lines_fin = []
        for token in tokens:
            new_lines_fin += token.to_raw_lines()
        return new_lines_fin
    else:
        return new_lines


def check_or_apply_lint_on_file(filename, check_lint, verbose=False):
    """Operates on a single file.

    If check_lint is True, will simply print a set of errors if any formatting
    is needed.
    Otherwise, this will mutate the file in place to fix all lint errors.
    """
    with open(filename, "r", encoding="utf8") as f:
        raw_lines = [x.rstrip() for x in f.readlines()]
    tokens = multiline_tokenize(filename, raw_lines)
    if check_lint:
        lint_errors = []
    else:
        lint_errors = None
    new_lines = check_or_apply_lint_on_tokens(
        tokens, lint_errors, verbose=False)
    if check_lint:
        if len(lint_errors) == 0:
            return []
        lint_errors = sorted(lint_errors, key=LintError.sorting_key)
        lint_errors_out = []
        for i, lint_error in enumerate(lint_errors):
            if i == 3 and not verbose:
                remaining = len(lint_errors) - 3
                lint_errors_out.append(LintError(
                    f"ERROR: There are {remaining} more errors for: "
                    f"{filename}"))
                break
            lint_errors_out.append(lint_error)
        if lint_errors_out:
            lint_errors_out.append(LintError(
                f"note: to fix, please run one of the following:\n"
                f"   bazel-bin/tools/lint/cpp_docstring_lint {filename}\n"
                f"   bazel-bin/tools/lint/cpp_docstring_lint --all"))
        return lint_errors_out
    else:
        with open(filename, "w", encoding="utf8") as f:
            f.write("\n".join(new_lines))
            f.write("\n")
        return []


def main(workspace_name="drake"):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "filenames", type=str, nargs="*",
        help="Files to lint / fix")
    parser.add_argument(
        "--all", action="store_true",
        help="Use all available files from the source tree.")
    parser.add_argument(
        "--lint", action="store_true",
        help="Only lint the files; do not try to fix them.")
    parser.add_argument(
        "-v", "--verbose", action="store_true",
        help="Print all linting errors")
    args = parser.parse_args()

    filenames = args.filenames

    if args.all:
        assert filenames == []
        workspace_dir, relpaths = find_all_sources(workspace_name)
        if len(relpaths) == 0:
            print("ERROR: '--all' could not find anything")
            return 1
        os.chdir(workspace_dir)
        for relpath in sorted(relpaths):
            if not is_ignored_file(relpath):
                filenames.append(relpath)
        if not args.lint:
            print(
                f"This will reformat {len(filenames)} files within "
                f"{workspace_dir}")
            if input("Are you sure [y/N]? ") not in ["y", "Y"]:
                print("... canceled")
                sys.exit(1)

    if filenames == ["<test>"]:
        test()
        return

    good = True
    for filename in filenames:
        lint_errors = check_or_apply_lint_on_file(
            filename, check_lint=args.lint, verbose=args.verbose)
        if lint_errors:
            print("\n".join(str(x) for x in lint_errors))
            good = False
    if not good:
        sys.exit(1)


if __name__ == "__main__":
    main()
