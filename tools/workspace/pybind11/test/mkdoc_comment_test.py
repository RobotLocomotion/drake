import unittest

from drake.tools.workspace.pybind11.mkdoc_comment import process_comment


class TestDocstring(unittest.TestCase):

    # TODO(m-chaturvedi, eric.cousineau): Completely test `mkdoc_comment.py`
    # and `mkdoc.py`.
    # Meanwhile, please refer to: https://git.io/JexDb
    def test_process_comment_issue_12445(self):
        self.assertEqual(process_comment("// `one` and `two`"),
                         "// ``one`` and ``two``")
        self.assertEqual(process_comment("/// this is foo"), "this is foo")
        self.assertEqual(process_comment(
            "///< this is foo\n/// this is bar"), "this is foo this is bar")

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
        self.assertEqual(process_comment("Begin line *Italics*"),
                         "Begin line *Italics*")
        self.assertEqual(process_comment("@b Bold"), "**Bold**")
        self.assertEqual(process_comment("Begin line **Bold**"),
                         "Begin line **Bold**")

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

    #
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
        self.assertEqual(process_comment("/// @brief Static method"),
                         "Static method")

    def test_code(self):
        input = """\
/// @code{.cpp}
/// Class class();
/// class.PublicMethod();
/// @endcode
""".rstrip()
        output = """\
::

    Class class();
    class.PublicMethod();
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

    def test_protected(self):
        self.assertEqual(process_comment("/// @protected"), "")

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
