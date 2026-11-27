import unittest

from tools.workspace.mkdoc_internal.mkdoc_comment import process_comment


class TestDocstring(unittest.TestCase):
    # TODO(m-chaturvedi, eric.cousineau): Completely test `mkdoc_comment.py`
    # and `mkdoc.py`.
    # Meanwhile, please refer to: https://git.io/JexDb
    def test_process_comment_issue_12445(self):
        self.assertEqual(
            process_comment("// `one` and `two`"), "// ``one`` and ``two``"
        )
        self.assertEqual(process_comment("/// this is foo"), "this is foo")
        self.assertEqual(
            process_comment("///< this is foo\n/// this is bar"),
            "this is foo this is bar",
        )

    def test_markdown_to_restructuredtext(self):
        input = """\
/// Quisque sagittis purus sit amet volutpat.
/// # First level heading #
/// Aliquet nec ullamcorper sit amet risus nullam eget felis. __Bold__. Hac
/// habitasse platea dictumst quisque sagittis purus sit.
///
/// ## Second level heading ##
/// Donec massa sapien faucibus et molestie ac feugiat sed lectus. _Italics_.
/// Amet justo donec enim diam vulputate ut pharetra sit.
///
/// ### Third level heading ###
/// Orci eu lobortis elementum nibh. `Typewriter`. Luctus venenatis lectus
/// magna fringilla urna porttitor rhoncus dolor.
///
/// #### Fourth level heading ####
/// Orci eu lobortis elementum nibh. Luctus venenatis lectus magna fringilla
///  urna porttitor rhoncus dolor.

/// Tortor id aliquet lectus proin nibh. [Link](https://example.org). Cras
/// semper auctor neque vitae tempus quam pellentesque nec.
""".rstrip()
        output = """\
Quisque sagittis purus sit amet volutpat.

First level heading
===================

Aliquet nec ullamcorper sit amet risus nullam eget felis. **Bold**.
Hac habitasse platea dictumst quisque sagittis purus sit.

Second level heading
--------------------

Donec massa sapien faucibus et molestie ac feugiat sed lectus.
*Italics*. Amet justo donec enim diam vulputate ut pharetra sit.

**Third level heading**

Orci eu lobortis elementum nibh. ``Typewriter``. Luctus venenatis
lectus magna fringilla urna porttitor rhoncus dolor.

*Fourth level heading*

Orci eu lobortis elementum nibh. Luctus venenatis lectus magna
fringilla urna porttitor rhoncus dolor.

Tortor id aliquet lectus proin nibh. `Link <https://example.org>`_.
Cras semper auctor neque vitae tempus quam pellentesque nec.
""".rstrip()
        self.assertEqual(process_comment(input), output)

    def test_markdown_table(self):
        input = """\
/// Before.
///
/// | Name | `Value` |
/// |:-----|--------:|
/// | a    | 1       |
/// | b    | x \\| y   |
///
/// After.
""".rstrip()
        output = """\
Before.

.. list-table::
   :header-rows: 1

   * - Name
     - ``Value``
   * - a
     - 1
   * - b
     - x \\| y

After."""
        self.assertEqual(process_comment(input), output)

    def test_html_table(self):
        input = """\
/// Before.
///
/// <table>
/// <tr><th>Name</th><th>Value</th></tr>
/// <tr><td>a</td><td>1</td></tr>
/// <tr><td>b</td>
///     <td>two
///     lines</td></tr>
/// </table>
///
/// After.
""".rstrip()
        output = """\
Before.

.. list-table::
   :header-rows: 1

   * - Name
     - Value
   * - a
     - 1
   * - b
     - two lines

After."""
        self.assertEqual(process_comment(input), output)

    def test_table_in_verbatim_regions_untouched(self):
        # Fenced code, @code, <pre>, and @verbatim (including the backslash
        # spellings) must not have their contents converted to tables.
        for open_tag, close_tag in [
            ("```", "```"),
            ("@code", "@endcode"),
            ("\\code", "\\endcode"),
            ("<pre>", "</pre>"),
            ("@verbatim", "@endverbatim"),
            ("\\verbatim", "\\endverbatim"),
        ]:
            with self.subTest(open_tag=open_tag):
                input = (
                    f"/// {open_tag}\n"
                    "/// 0    |\n"
                    "/// 1/2  | 1/2\n"
                    "/// -----|----\n"
                    "/// 1    | 2/9\n"
                    f"/// {close_tag}"
                )
                output = process_comment(input)
                self.assertNotIn("list-table", output)
                self.assertIn("-----|----", output)
                self.assertIn("1    | 2/9", output)

    def test_markdown_table_requires_pipe_separator(self):
        # A line with a pipe followed by a horizontal rule is not a table.
        output = process_comment("/// Compute |x|\n/// -----\n/// more")
        self.assertNotIn("list-table", output)
        # The separator must have as many columns as the header.
        input = "/// | a | b | c |\n/// |---|---|\n/// | 1 | 2 |"
        self.assertNotIn("list-table", process_comment(input))

    def test_markdown_table_ends_at_mismatched_row(self):
        input = """\
/// | a | b | c |
/// |---|---|---|
/// | 1 | 2 | 3 |
/// where a | b is meant
""".rstrip()
        output = process_comment(input)
        self.assertTrue(
            output.endswith("     - 3\n\nwhere a | b is meant"), output
        )

    def test_table_cell_escaping(self):
        input = """\
/// | a | b | c |
/// |---|---|---|
/// | - | \\|x\\| | .. |
/// |   | `p\\|q` | 1. |
""".rstrip()
        output = process_comment(input)
        self.assertEqual(
            output,
            """\
.. list-table::
   :header-rows: 1

   * - a
     - b
     - c
   * - \\-
     - \\|x\\|
     - \\..
   * -
     - ``p|q``
     - \\1.""",
        )

    def test_table_cell_markup(self):
        # Line breaks and lists in cells must not split the row.
        input = """\
/// | a | b |
/// |---|---|
/// | x<br>y | @p z and _it_ |
///
/// <table><tr><td><ul><li>one</li><li>two</li></ul></td><td>x</td></tr>
/// </table>
""".rstrip()
        output = process_comment(input)
        self.assertEqual(
            output,
            """\
.. list-table::
   :header-rows: 1

   * - a
     - b
   * - x y
     - ``z`` and *it*

.. list-table::

   * - • one • two
     - x""",
        )

    def test_header_only_table(self):
        # list-table rejects :header-rows: when there is no body.
        output = process_comment("/// | a | b |\n/// |---|---|")
        self.assertEqual(output, ".. list-table::\n\n   * - a\n     - b")
        output = process_comment(
            "/// <table><tr><th>A</th><th>B</th></tr></table>"
        )
        self.assertEqual(output, ".. list-table::\n\n   * - A\n     - B")

    def test_html_table_with_pipe_and_empty_cell(self):
        output = process_comment(
            "/// <table><tr><td>a|b</td><td></td></tr></table>"
        )
        self.assertEqual(output, ".. list-table::\n\n   * - a\\|b\n     -")

    def test_html_table_caption(self):
        input = (
            "/// <table><caption>Cap</caption>"
            "<tr><th>H</th><th>I</th></tr>"
            "<tr><td>1</td><td>2</td></tr></table>"
        )
        output = process_comment(input)
        self.assertEqual(
            output,
            """\
.. list-table:: Cap
   :header-rows: 1

   * - H
     - I
   * - 1
     - 2""",
        )

    def test_markdown_table_row_spans(self):
        # Doxygen merges a cell into the one above it with a lone ^. A
        # list-table can't express that, so a grid table is used.
        input = """\
/// | Time   | Event   | Detail   |
/// |--------|---------|----------|
/// | 200 ms | capture | first    |
/// | 400 ms | ^       | ^        |
/// | 600 ms | send    | last `x` |
""".rstrip()
        output = process_comment(input)
        self.assertEqual(
            output,
            """\
+--------+---------+------------+
| Time   | Event   | Detail     |
+========+=========+============+
| 200 ms | capture | first      |
+--------+         |            |
| 400 ms |         |            |
+--------+---------+------------+
| 600 ms | send    | last ``x`` |
+--------+---------+------------+""",
        )

    def test_html_table_row_and_column_spans(self):
        input = (
            "/// <table>"
            "<tr><th>A</th><th>B</th><th>C</th></tr>"
            "<tr><td rowspan=2>r</td><td colspan=2>wide cell here</td></tr>"
            "<tr><td>p</td><td>q</td></tr>"
            "</table>"
        )
        output = process_comment(input)
        self.assertEqual(
            output,
            """\
+---+---+------------+
| A | B | C          |
+===+===+============+
| r | wide cell here |
|   +---+------------+
|   | p | q          |
+---+---+------------+""",
        )

    def test_html_table_colspan_in_header_and_caption(self):
        input = (
            "/// <table><caption>Cap</caption>"
            "<tr><th colspan=2>H</th></tr>"
            "<tr><td>1</td><td>2</td></tr></table>"
        )
        output = process_comment(input)
        self.assertEqual(
            output,
            """\
.. table:: Cap

   +-------+
   | H     |
   +===+===+
   | 1 | 2 |
   +---+---+""",
        )

    def test_grid_table_wide_characters(self):
        # Wide (East Asian) characters take two columns in a grid table.
        input = (
            "/// <table><tr><td>ωₙ ✔</td><td rowspan=2>漢字</td></tr>"
            "<tr><td>b</td></tr></table>"
        )
        output = process_comment(input)
        self.assertEqual(
            output,
            """\
+------+------+
| ωₙ ✔ | 漢字 |
+------+      |
| b    |      |
+------+------+""",
        )

    def test_span_across_header_drops_header(self):
        # A span can't cross the header boundary of a grid table.
        input = "/// | a | b |\n/// |---|---|\n/// | ^ | y |"
        output = process_comment(input)
        self.assertEqual(
            output,
            """\
+---+---+
| a | b |
|   +---+
|   | y |
+---+---+""",
        )

    def test_table_in_list_item(self):
        input = """\
///  - When x
///    cont.
///    A | B
///    - | -
///    1 | 2
///    See http://x
///  - Next
""".rstrip()
        output = process_comment(input)
        self.assertEqual(output.split("\n\n")[0], "- When x\n  cont.")
        # The table is indented to belong to the list item, and the text that
        # follows it is not swallowed into the directive.
        self.assertIn(
            "\n\n  .. list-table::\n     :header-rows: 1\n\n     * - A\n"
            "       - B\n     * - 1\n       - 2\n\nSee http://x",
            output,
        )

    def test_table_after_return(self):
        input = """\
/// @return
/// | a | b |
/// |---|---|
/// | 1 | 2 |
///
/// After.
""".rstrip()
        output = process_comment(input)
        self.assertEqual(
            output,
            """\
Returns:
    .. list-table::
       :header-rows: 1

       * - a
         - b
       * - 1
         - 2

After.""",
        )

    def test_replace_exceptions(self):
        input = """\
void func();
/// Function, overload 1. Velit ut tortor pretium viverra suspendisse potenti
/// nullam ac tortor.
/// @throws std::exception Begin raises. Morbi tincidunt augue interdum velit
/// euismod. Justo nec ultrices dui sapien eget mi proin sed libero. End
/// raises.
""".rstrip()
        output = """\
void func(); Function, overload 1. Velit ut tortor pretium viverra
suspendisse potenti nullam ac tortor.

Raises:
    RuntimeError Begin raises. Morbi tincidunt augue interdum velit
    euismod. Justo nec ultrices dui sapien eget mi proin sed libero.
    End raises.
""".rstrip()
        self.assertEqual(process_comment(input), output)

    # Doxygen commands

    # Doxygen tags
    def test_font(self):
        self.assertEqual(process_comment("@c Typewriter"), "``Typewriter``")
        self.assertEqual(process_comment("@p Typewriter"), "``Typewriter``")
        self.assertEqual(process_comment("@e Italics"), "*Italics*")
        self.assertEqual(process_comment("@a Italics"), "*Italics*")
        self.assertEqual(process_comment("@em Italics"), "*Italics*")
        self.assertEqual(
            process_comment("Begin line *Italics*"), "Begin line *Italics*"
        )
        self.assertEqual(process_comment("@b Bold"), "**Bold**")
        self.assertEqual(
            process_comment("Begin line **Bold**"), "Begin line **Bold**"
        )

    def test_parameter(self):
        input = """\
/// @param param Begin parameter. Dignissim diam quis enim lobortis
/// scelerisque fermentum dui faucibus. End parameter.
""".rstrip()
        output = """\
Parameter ``param``:
    Begin parameter. Dignissim diam quis enim lobortis scelerisque
    fermentum dui faucibus. End parameter.
""".rstrip()
        self.assertEqual(process_comment(input), output)

        input = """\
/// @param[in] param1 Begin first input parameter. Mollis nunc sed id semper
/// risus in hendrerit gravida rutrum. End first input parameter.
/// @param[in] param2 Begin second input parameter. Tristique senectus et
/// netus et malesuada fames ac turpis. End second input parameter.
""".rstrip()
        output = """\
Parameter ``param1``:
    Begin first input parameter. Mollis nunc sed id semper risus in
    hendrerit gravida rutrum. End first input parameter.

Parameter ``param2``:
    Begin second input parameter. Tristique senectus et netus et
    malesuada fames ac turpis. End second input parameter.
""".rstrip()
        self.assertEqual(process_comment(input), output)

        input = """\
/// @param[in,out] param Begin input/output parameter. Morbi enim nunc faucibus
/// a pellentesque sit. End input/output parameter.
""".rstrip()  # noqa
        output = """\
Parameter ``param``:
    Begin input/output parameter. Morbi enim nunc faucibus a
    pellentesque sit. End input/output parameter.
""".rstrip()
        self.assertEqual(process_comment(input), output)

    def test_template_parameter(self):
        input = """\
/// @tparam T Begin template parameter. acilisi etiam dignissim diam quis. Ut
/// pharetra sit amet aliquam. End template parameter.
""".rstrip()
        output = """\
Template parameter ``T``:
    Begin template parameter. acilisi etiam dignissim diam quis. Ut
    pharetra sit amet aliquam. End template parameter.
""".rstrip()
        self.assertEqual(process_comment(input), output)

    # Command names
    def test_returns(self):
        input = """\
/// @returns Begin returns. Faucibus interdum posuere lorem ipsum dolor.
/// Malesuada proin libero nunc consequat interdum varius sit amet. End
/// returns.
""".rstrip()
        output = """\
Returns:
    Begin returns. Faucibus interdum posuere lorem ipsum dolor.
    Malesuada proin libero nunc consequat interdum varius sit amet.
    End returns.
""".rstrip()
        self.assertEqual(process_comment(input), output)

    def test_bug(self):
        input = """\
/// @bug Begin bug report. Cras pulvinar mattis nunc sed blandit libero. Eget
/// est lorem ipsum dolor sit amet consectetur. End bug report.
""".rstrip()
        output = """\
Bug report:
    Begin bug report. Cras pulvinar mattis nunc sed blandit libero.
    Eget est lorem ipsum dolor sit amet consectetur. End bug report.
""".rstrip()
        self.assertEqual(process_comment(input), output)

    def test_attention(self):
        input = """\
/// @attention Begin attention. Ultricies lacus sed turpis tincidunt id
/// aliquet risus feugiat. End attention.
""".rstrip()
        output = """\
Attention:
    Begin attention. Ultricies lacus sed turpis tincidunt id aliquet
    risus feugiat. End attention.
""".rstrip()
        self.assertEqual(process_comment(input), output)

    def test_deprecated(self):
        input = """\
/// @deprecated Begin deprecated. Est pellentesque elit ullamcorper dignissim
/// cras tincidunt lobortis. End deprecated.
""".rstrip()
        output = """\
Deprecated:
    Begin deprecated. Est pellentesque elit ullamcorper dignissim cras
    tincidunt lobortis. End deprecated.
""".rstrip()
        self.assertEqual(process_comment(input), output)

    def test_invariant(self):
        input = """\
/// @invariant Begin invariant. Odio euismod lacinia at quis risus sed
/// vulputate odio. End invariant.
""".rstrip()
        output = """\
Invariant:
    Begin invariant. Odio euismod lacinia at quis risus sed vulputate
    odio. End invariant.
""".rstrip()
        self.assertEqual(process_comment(input), output)

    def test_note(self):
        input = """\
/// @note Begin note. Consectetur a erat nam at lectus. Consequat ac felis
/// donec et odio pellentesque diam. End note.
""".rstrip()
        output = """\
Note:
    Begin note. Consectetur a erat nam at lectus. Consequat ac felis
    donec et odio pellentesque diam. End note.
""".rstrip()
        self.assertEqual(process_comment(input), output)

    def test_remark(self):
        input = """\
/// @remarks Begin remarks. Ante metus dictum at tempor commodo. Nec feugiat
/// in fermentum posuere urna nec. End remarks.
""".rstrip()
        output = """\
Remark:
    Begin remarks. Ante metus dictum at tempor commodo. Nec feugiat in
    fermentum posuere urna nec. End remarks.
""".rstrip()
        self.assertEqual(process_comment(input), output)

    def test_sa(self):
        output = """\
See also:
    Struct
""".rstrip()
        self.assertEqual(process_comment("@sa Struct"), output)

    def test_see(self):
        output = """\
See also:
    Class
""".rstrip()
        self.assertEqual(process_comment("/// @see Class"), output)

    def test_since(self):
        input = """\
/// @since Begin since. Nullam eget felis eget nunc lobortis mattis aliquam
/// faucibus. End since.
""".rstrip()
        output = """\
Since:
    Begin since. Nullam eget felis eget nunc lobortis mattis aliquam
    faucibus. End since.
""".rstrip()
        self.assertEqual(process_comment(input), output)

    def test_test(self):
        input = """\
/// @test Begin test case. Sem integer vitae justo eget magna fermentum.
/// Convallis posuere morbi leo urna molestie at elementum eu facilisis. End
/// test case.
""".rstrip()
        output = """\
Test case:
    Begin test case. Sem integer vitae justo eget magna fermentum.
    Convallis posuere morbi leo urna molestie at elementum eu
    facilisis. End test case.
""".rstrip()
        self.assertEqual(process_comment(input), output)

    def test_todo(self):
        input = """\
/// @todo Begin TODO. Ut tellus elementum sagittis vitae et leo duis ut diam.
/// Et malesuada fames ac turpis. End TODO.
""".rstrip()
        output = """\
Todo:
    Begin TODO. Ut tellus elementum sagittis vitae et leo duis ut
    diam. Et malesuada fames ac turpis. End TODO.
""".rstrip()
        self.assertEqual(process_comment(input), output)

    def test_version(self):
        output = """\
Version:
    1.0
""".rstrip()
        self.assertEqual(process_comment("@version 1.0"), output)

    def test_details(self):
        input = """\
/// @details Vitae sapien pellentesque habitant morbi tristique senectus.
/// Iaculis eu non diam phasellus vestibulum.
""".rstrip()
        output = """\
Vitae sapien pellentesque habitant morbi tristique senectus. Iaculis
eu non diam phasellus vestibulum.
""".rstrip()
        self.assertEqual(process_comment(input), output)

    def test_brief(self):
        self.assertEqual(
            process_comment("/// @brief Static method"), "Static method"
        )

    def test_code_cpp(self):
        input = """\
/// @code{.cpp}
/// Class class();
/// class.PublicMethod();
/// @endcode
""".rstrip()
        output = """\
.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    Class class();
    class.PublicMethod();

.. raw:: html

    </details>
""".rstrip()
        self.assertEqual(process_comment(input), output)

    def test_code_py(self):
        input = """\
/// @python_details_begin
/// @code{.py}
/// # This is a Python comment.
/// foo = dict()
/// print(foo)
/// @endcode
/// @python_details_end
""".rstrip()
        output = """\
.. code-block:: python

    # This is a Python comment.
    foo = dict()
    print(foo)
""".rstrip()
        self.assertEqual(process_comment(input), output)

    def test_doxygen_system_tags(self):
        # First line is required to parse system tags correctly.
        # This line must be non-empty, or reflow logic does not work as
        # intended.
        input = """\
/// Some text.
///
/// @system
/// name: Alchemist
/// input_ports:
/// - lead
/// output_ports:
/// - gold
/// @endsystem
""".rstrip()
        output = """\
Some text.

.. pydrake_system::

    name: Alchemist
    input_ports:
    - lead
    output_ports:
    - gold
""".rstrip()
        self.assertEqual(process_comment(input), output)

    # Sectioning commands
    def test_sections(self):
        input = """\
/**
 * @section first_level_heading First level heading
 * Cursus in hac habitasse platea dictumst quisque sagittis purus sit. Et
 * malesuada fames ac turpis.
 * @subsection second_level_heading Second level heading
 * Adipiscing diam donec adipiscing tristique risus nec feugiat. Condimentum
 * vitae sapien pellentesque habitant.
 * @subsubsection third_level_heading Third level heading
 * Fermentum odio eu feugiat pretium nibh. Sed nisi lacus sed viverra.  Ut
 * ornare lectus sit amet est.
*/
""".rstrip()
        output = """\
First level heading
===================

Cursus in hac habitasse platea dictumst quisque sagittis purus sit. Et
malesuada fames ac turpis.

Second level heading
--------------------

Adipiscing diam donec adipiscing tristique risus nec feugiat.
Condimentum vitae sapien pellentesque habitant.

**Third level heading**

Fermentum odio eu feugiat pretium nibh. Sed nisi lacus sed viverra. Ut
ornare lectus sit amet est.
""".rstrip()
        self.assertEqual(process_comment(input), output)

    # LaTeX commands
    def test_latex_commands(self):
        input = """\
/// Public method. @f$ A = \\pi r^2 @f$. Condimentum id venenatis a
/// condimentum vitae sapien pellentesque habitant morbi.
""".rstrip()
        output = """\
Public method. :math:`A = \\pi r^2`. Condimentum id venenatis a
condimentum vitae sapien pellentesque habitant morbi.
""".rstrip()
        self.assertEqual(process_comment(input), output)

        input = """\
/// Public template method. @f[ a^2 + b^2 = c^2. @f] Sagittis id consectetur
/// purus ut faucibus pulvinar.
""".rstrip()
        output = """\
Public template method.

.. math:: a^2 + b^2 = c^2.

Sagittis id consectetur purus ut faucibus pulvinar.
""".rstrip()
        self.assertEqual(process_comment(input), output)

    # No argument commands
    def test_callgraph(self):
        self.assertEqual(process_comment("/// @hidecallgraph"), "")
        self.assertEqual(process_comment("/// @hidecallergraph"), "")

    def test_hide_initializer(self):
        self.assertEqual(process_comment("/// @hideinitializer"), "")

    def test_private(self):
        self.assertEqual(process_comment("/// @private"), "")

    def test_protected(self):
        self.assertEqual(process_comment("/// @protected"), "")

    def test_public(self):
        self.assertEqual(process_comment("/// @public"), "")

    def test_showinitializer(self):
        self.assertEqual(process_comment("/// @showinitializer"), "")

    def test_static(self):
        self.assertEqual(process_comment("/// @static"), "")

    # Commands with single-word argument
    def test_anchor(self):
        self.assertEqual(process_comment("/// @anchor anchor"), "")

    def test_enum(self):
        self.assertEqual(process_comment("/// @enum Enum"), "")

    def test_remove_relatesalso(self):
        self.assertEqual(process_comment("/// @relatesalso Struct"), "")

    def test_remove_relates(self):
        self.assertEqual(process_comment("/// @relates Class"), "")

    # Commands with single-line argument
    def test_fn(self):
        input = """\
/// @fn void PublicMethod()
void PublicMethod() {}
""".rstrip()
        self.assertEqual(process_comment(input), "void PublicMethod() {}")

    def test_ingroup(self):
        input = "/// @ingroup first_group second_group"
        self.assertEqual(process_comment(input), "")

    def test_remove_var_command(self):
        input = """\
/// @var int protected_member_
/// Protected member. ``Typewriter``. Porttitor eget dolor morbi non arcu
/// risus quis varius quam.
""".rstrip()
        output = """\
Protected member. ``Typewriter``. Porttitor eget dolor morbi non arcu
risus quis varius quam.
""".rstrip()
        self.assertEqual(process_comment(input), output)

    def test_class(self):
        input = """\
/// @class Class
/// Class. Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do
/// eiusmod tempor incididunt ut labore et dolore magna aliqua.
""".rstrip()
        output = """\
Class. Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do
eiusmod tempor incididunt ut labore et dolore magna aliqua.
""".rstrip()
        self.assertEqual(process_comment(input), output)

    def test_struct(self):
        input = """\
/**
 * @struct MidLevelSymbol
 * Mid-level symbol. Ut enim ad minim veniam, quis nostrud exercitation
 * ullamco laboris nisi ut aliquip ex ea commodo consequat.
 *
*/
""".rstrip()
        output = """\
Mid-level symbol. Ut enim ad minim veniam, quis nostrud exercitation
ullamco laboris nisi ut aliquip ex ea commodo consequat.
""".rstrip()
        self.assertEqual(process_comment(input), output)

    # pairs of commands
    def test_cond(self):
        input = """\
/// @cond
/// Begin ignored conditional section. Tortor vitae purus faucibus ornare
/// suspendisse. Orci dapibus ultrices in iaculis. End ignored conditional
/// section.
/// @endcond
""".rstrip()
        self.assertEqual(process_comment(input), "")

    def test_internal(self):
        input = """\
/**
 * @internal
 * Begin ignored internal section. Est ante in nibh mauris cursus mattis
 * molestie. Morbi tristique senectus et netus et malesuada. Magnis dis
 * parturient montes nascetur ridiculus mus mauris. End ignored internal
 * section.
 * @endinternal
*/""".rstrip()
        self.assertEqual(process_comment(input), "")
