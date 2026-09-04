"""Provides Sphinx extensions / monkey patches to:
 - Remove excessive bases when documenting inheritance
 - Document parameterized bindings of templated methods / classes

For guidance, see:
 - http://www.sphinx-doc.org/en/master/extdev/appapi.html#sphinx.application.Sphinx.add_autodocumenter  # noqa
"""

# TODO(eric.cousineau): How to document only protected methods?
# e.g. `LeafSystem` only consists of private things to overload, but it's
# important to be user-visible.

import re
from textwrap import indent
import warnings

from docutils import nodes
from docutils.parsers.rst import Directive
from docutils.statemachine import ViewList
import sphinx.domains.python as pydoc
from sphinx.ext import autodoc
from sphinx.util.nodes import nested_parse_with_titles

from doc.doxygen_cxx.system_doxygen import system_yaml_to_html
from pydrake.common.cpp_template import TemplateBase
from pydrake.common.deprecation import DrakeDeprecationWarning


def generate_sig_re(extended=False):
    """Returns a regular expression suitable for extracting signatures.

    These are based on the regular expressions from sphinx 7.2.6, but have been
    modified to accept type-parameterized class names and to treat member
    type parameters as part of the name, rather than extracting them as a
    separate group.
    """
    if extended:
        expression = r"""^ ([\w.]+::)?       # explicit module name
                           ([\w.]+           # module and/or class name(s)
                       (?: \[\s*[^(]*\s*])?  # optional: type parameters list
                      \.)?                   # end of module/class name(s)
                     """
    else:
        expression = r"""^ ([\w.]*           # class name(s)
                       (?: \[\s*[^(]*\s*])?  # optional: type parameters list
                      \.)?                   # end of class name(s)
                     """

    expression += r"""(\w+  \s*              # thing name
                       (?: \[\s*.*\s*])?     # optional: type parameters list
                      )                      # end of thing name
                  """

    # Sphinx captures an optional type parameters list as its own group, but
    # we want to capture that as part of the name(s). Provide a dummy
    # group here so the number of capture groups still matches what Sphinx
    # expects when it unpacks the match.
    expression += r"(!!dummy_tp_list!!)?"

    expression += r"""(?: \((.*)\)           # optional: arguments
                       (?:\s* -> \s* (.*))?  #           return annotation
                      )? $                   # and nothing more
                  """

    return re.compile(expression, re.VERBOSE)


def patch(obj, name, f):
    """Patch the method of a class."""
    original = getattr(obj, name)

    def method(*args, **kwargs):
        return f(original, *args, **kwargs)

    setattr(obj, name, method)


def repair_naive_name_split(objpath):
    """Rejoins any strings with braces that were naively split across '.'."""
    num_open = 0
    out = []
    cur = ""
    for p in objpath:
        num_open += p.count("[") - p.count("]")
        assert num_open >= 0
        if cur:
            cur += "."
        cur += p
        if num_open == 0:
            out.append(cur)
            cur = ""
    assert len(cur) == 0, (objpath, cur, out)
    return out


class TemplateDocumenter(autodoc.ModuleLevelDocumenter):
    """Specializes `Documenter` for templates from `cpp_template`."""

    objtype = "template"
    member_order = autodoc.ClassDocumenter.member_order
    directivetype = "template"

    # Take priority over attributes.
    priority = 1 + autodoc.AttributeDocumenter.priority

    @classmethod
    def can_document_member(cls, member, membername, isattr, parent):
        """Overrides base to check for template objects."""
        return isinstance(member, TemplateBase)

    def get_object_members(self, want_all):
        """Overrides base; we shouldn't show any details beyond the list of
        instantiations.
        """
        return False, []

    def check_module(self):
        """Overrides base to show template objects given the correct module."""
        if self.options.imported_members:
            return True
        scope = self.object._scope
        if isinstance(scope, type):
            module_name = scope.__module__
        else:
            module_name = scope.__name__
        return module_name == self.modname

    def add_directive_header(self, sig):
        """Overrides base to add a line to indicate instantiations."""
        autodoc.ModuleLevelDocumenter.add_directive_header(self, sig)
        sourcename = self.get_sourcename()
        self.add_line("", sourcename)
        names = []
        for param in self.object.param_list:
            # TODO(eric.cousineau): Use attribute aliasing already present in
            # autodoc.
            rst = f":class:`{self.object._instantiation_name(param)}`"
            names.append(rst)
        self.add_line(
            "   Instantiations: {}".format(", ".join(names)), sourcename
        )


def tpl_attrgetter(obj, name, *defargs):
    """Attribute getter hook for autodoc to permit accessing instantiations via
    instantiation names.

    In ideal world, we'd be able to override instance names easily; however,
    since Sphinx aims to permit either sweeping automation (`automodule`) or
    specific instances (`autoclass`), we have to try and get it to play nice
    with string retrieval.

    Note:
        We cannot call `.. autoclass:: obj.MyTemplate[param]`, because this
    getter is constrained to `TemplateBase` instances.
    """
    # N.B. Rather than try to evaluate parameters from the string, we instead
    # match based on instantiation name.
    if isinstance(obj, TemplateBase) and name[0] != "_":
        for param in obj.param_list:
            inst = obj[param]
            if inst.__name__ == name:
                return inst
        assert False, (
            "Not a template?",
            param,
            obj.param_list,
            inst.__name__,
            name,
        )
    return autodoc.safe_getattr(obj, name, *defargs)


def patch_resolve_name(original, self, *args, **kwargs):
    """Patches implementations of `resolve_name` to handle splitting across
    braces.
    """
    modname, objpath = original(self, *args, **kwargs)
    return modname, repair_naive_name_split(objpath)


def _is_hidden_base(base) -> bool:
    """Returns True iff pybind11_object, the implementation base class that
    pybind11 injects into every bound class, is the name of `base`"""
    return base.__name__ == "pybind11_object"


def autodoc_process_bases(app, name, obj, options, bases):
    """Hides base classes from `bases`."""
    bases[:] = [b for b in bases if not _is_hidden_base(b)]


def patch_class_hide_empty_bases(original, self, sig):
    """Wraps `ClassDocumenter.add_directive_header` to omit the "Bases: ..."
    line entirely when every real base is hidden.

    The `autodoc-process-bases` event only lets us edit the *contents* of the
    bases list; Sphinx's own `add_directive_header` unconditionally emits the
    "Bases:" line once `show_inheritance` is on. We temporarily disable the
    latter for this instance once we know there are no bases left to show.
    """
    bases = self.object.__bases__
    if bases and all(_is_hidden_base(b) for b in bases):
        show_inheritance = self.options.show_inheritance
        self.options.show_inheritance = False
        try:
            original(self, sig)
        finally:
            self.options.show_inheritance = show_inheritance
    else:
        original(self, sig)


def autodoc_skip_member(app, what, name, obj, skip, options):
    """Skips undesirable members."""
    # N.B. This should be registered before `napoleon`s event.
    # N.B. For some reason, `:exclude-members` via `autodoc_default_options`
    # did not work. Revisit this at some point.
    if "__del__" in name:
        return True
    # In order to work around #11954.
    if "__init__" in name:
        return False
    return None


def patch_sort_members(original, self, documenters, order):
    """Adds a `bycustomfunction` member-order strategy, which sorts members
    alphabetically by case-insensitive full name.
    """
    if order != "bycustomfunction":
        return original(self, documenters, order)
    # N.B. This follows suit with the following 3.x code: https://git.io/Jv1CH
    documenters.sort(key=lambda e: e[0].name.split("::")[1].lower())
    return documenters


class PydrakeSystemDirective(Directive):
    """
    Translates `pydrake_system` directives (with YAML) to `raw` HTML
    directives.

    See also:
    - https://www.sphinx-doc.org/en/1.6.7/extdev/tutorial.html#the-directive-classes
    - https://docutils.sourceforge.io/docs/howto/rst-directives.html#error-handling
    - https://github.com/sphinx-contrib/autoprogram/blob/0.1.5/sphinxcontrib/autoprogram.py
    """  # noqa

    has_content = True

    def run(self):
        system_yaml = "\n".join(self.content)
        try:
            system_html = system_yaml_to_html(system_yaml)
        except TypeError as e:
            raise self.severe(f"pydrake_system error: {e}")
        raw_content = indent(system_html.strip(), "   ")
        raw_rst = f".. raw:: html\n\n{raw_content}"
        node = _parse_rst(self.state, raw_rst)
        return node.children


def _parse_rst(state, rst_text):
    # Adapted from `autoprogram` source.
    result = ViewList()
    for line in rst_text.splitlines():
        result.append(line, "<parsed>")
    node = nodes.section()
    node.document = state.document
    nested_parse_with_titles(state, result, node)
    return node


def setup(app):
    """Installs Drake-specific extensions and patches."""
    app.add_css_file("css/custom.css")
    # Add directive to process system doxygen.
    app.add_directive("pydrake_system", PydrakeSystemDirective)
    # Do not warn on Drake deprecations.
    # TODO(eric.cousineau): See if there is a way to intercept this.
    warnings.simplefilter("ignore", DrakeDeprecationWarning)
    # Ignore `pybind11_object` as a base.
    app.connect("autodoc-process-bases", autodoc_process_bases)
    patch(
        autodoc.ClassDocumenter,
        "add_directive_header",
        patch_class_hide_empty_bases,
    )
    # Skip specific members.
    app.connect("autodoc-skip-member", autodoc_skip_member)
    # Register directive so we can pretty-print template declarations.
    pydoc.PythonDomain.directives["template"] = pydoc.PyClasslike
    # Register autodocumentation for templates.
    app.add_autodoc_attrgetter(TemplateBase, tpl_attrgetter)
    app.add_autodocumenter(TemplateDocumenter)
    # Hack regular expressions to match type-parameterized names.
    autodoc.py_ext_sig_re = generate_sig_re(extended=True)
    pydoc.py_sig_re = generate_sig_re(extended=False)
    patch(autodoc.ClassLevelDocumenter, "resolve_name", patch_resolve_name)
    patch(autodoc.ModuleLevelDocumenter, "resolve_name", patch_resolve_name)
    patch(autodoc.Documenter, "sort_members", patch_sort_members)
    return dict(parallel_read_safe=True)
