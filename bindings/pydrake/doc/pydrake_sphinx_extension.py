"""Provides Sphinx extensions / monkey patches to:
 - Remove excessive bases when documenting inheritance
 - Document parameterized bindings of templated methods / classes

For guidance, see:
 - http://www.sphinx-doc.org/en/master/extdev/appapi.html#sphinx.application.Sphinx.add_autodocumenter  # noqa
"""

# TODO(eric.cousineau): Fix for Sphinx >= 2.0.0 per comment in
# `mac/.../requirements.txt`, most likely due to `IrregularExpression` hack.
# TODO(eric.cousineau): How to document only protected methods?
# e.g. `LeafSystem` only consists of private things to overload, but it's
# important to be user-visible.

from __future__ import print_function

from collections import namedtuple
import re
import warnings

from sphinx.locale import _
import sphinx.domains.python as pydoc
from sphinx.ext import autodoc

from pydrake.common.cpp_template import TemplateBase
from pydrake.common.deprecation import DrakeDeprecationWarning


def rindex(s, sub):
    """Reverse index of a substring."""
    return len(s) - s[::-1].index(sub) - len(sub)


def patch(obj, name, f):
    """Patch the method of a class."""
    original = getattr(obj, name)

    def method(*args, **kwargs):
        return f(original, *args, **kwargs)

    setattr(obj, name, method)


def repair_naive_name_split(objpath):
    """Rejoins any strings with braces that were naively split across '.'.
    """
    num_open = 0
    out = []
    cur = ''
    for p in objpath:
        num_open += p.count('[') - p.count(']')
        assert num_open >= 0
        if cur:
            cur += '.'
        cur += p
        if num_open == 0:
            out.append(cur)
            cur = ''
    assert len(cur) == 0, (objpath, cur, out)
    return out


class IrregularExpression(object):
    """Provides analogous parsing to `autodoc.py_ext_sig_re` and
    `pydoc.py_sig_re`, but permits nested parsing for class-like directives to
    work with the munged names.

    These are meant to be used to monkey-patch existing compiled regular
    expressions.
    """

    FakeMatch = namedtuple('FakeMatch', 'groups')

    py_sig_old = autodoc.py_ext_sig_re
    py_sig = re.compile(
        r'''^     (\w.*?) \s*               # symbol
                  (?:
                      \((.*)\)              # optional: arguments
                      (?:\s* -> \s* (.*))?  # return annotation
                  )? $
              ''', re.VERBOSE)

    def __init__(self, extended):
        """
        Args:
            extended: For use in `autodoc` (returns explicit reST module name
                scope).
        """
        self.extended = extended

    def match(self, full):
        """Tests if a string matches `full`. If not, returns None."""
        m = self.py_sig.match(full)
        if not m:
            return None
        symbol, arg, retann = m.groups()
        # Heuristic to not try and match for docstring phrases. Any space
        # should be balanced with a comma for the symbol.
        if symbol.count(' ') > symbol.count(','):
            return None
        # Extract module name using a greedy match.
        explicit_modname = None
        if "::" in symbol:
            pos = rindex(symbol, "::") + 2
            explicit_modname = symbol[:pos]
            symbol = symbol[pos:].strip()
        # Extract {path...}.{base}, accounting for brackets.
        if not symbol:
            return
        pieces = repair_naive_name_split(symbol.split('.'))
        assert len(pieces) > 0, (symbol, pieces)
        base = pieces[-1]
        if len(pieces) > 1:
            path = '.'.join(pieces[:-1]) + '.'
        else:
            path = None
        if self.extended:
            groups = (explicit_modname, path, base, arg, retann)
        else:
            assert explicit_modname is None
            groups = (path, base, arg, retann)
        return self.FakeMatch(lambda: groups)


class TemplateDocumenter(autodoc.ModuleLevelDocumenter):
    """Specializes `Documenter` for templates from `cpp_template`.
    """
    objtype = 'template'
    member_order = autodoc.ClassDocumenter.member_order
    directivetype = 'template'

    # Take priority over attributes.
    priority = 1 + autodoc.AttributeDocumenter.priority

    option_spec = {
        'show-all-instantiations': autodoc.bool_option,
    }
    # Permit propagation of class-specific propreties.
    option_spec.update(autodoc.ClassDocumenter.option_spec)

    @classmethod
    def can_document_member(cls, member, membername, isattr, parent):
        """Overrides base to check for template objects."""
        return isinstance(member, TemplateBase)

    def get_object_members(self, want_all):
        """Overrides base to return instantiations from templates."""
        members = []
        for param in self.object.param_list:
            instantiation = self.object[param]
            members.append((instantiation.__name__, instantiation))
            if not self.options.show_all_instantiations:
                break
        return False, members

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
        self.add_line(u'', sourcename)
        names = []
        for param in self.object.param_list:
            # TODO(eric.cousineau): Use attribute aliasing already present in
            # autodoc.
            rst = ":class:`{}`".format(self.object._instantiation_name(param))
            names.append(rst)
        self.add_line(
            u"   Instantiations: {}".format(", ".join(names)), sourcename)


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
    if "[" in name:
        assert name.endswith(']'), name
        for param in obj.param_list:
            inst = obj[param]
            if inst.__name__ == name:
                return inst
        assert False, (
            "Not a template?",
            param, obj.param_list,
            inst.__name__, name)
    return autodoc.safe_getattr(obj, name, *defargs)


def patch_resolve_name(original, self, *args, **kwargs):
    """Patches implementations of `resolve_name` to handle split across braces.
    """
    modname, objpath = original(self, *args, **kwargs)
    return modname, repair_naive_name_split(objpath)


def patch_class_add_directive_header(original, self, sig):
    """Patches display of bases for classes to strip out pybind11 meta classes
    from bases.
    """
    if self.doc_as_attr:
        self.directivetype = 'attribute'
    autodoc.Documenter.add_directive_header(self, sig)
    # add inheritance info, if wanted
    if not self.doc_as_attr and self.options.show_inheritance:
        sourcename = self.get_sourcename()
        self.add_line(u'', sourcename)
        bases = getattr(self.object, '__bases__', None)
        if not bases:
            return
        bases = [b.__module__ in ('__builtin__', 'builtins') and
                 u':class:`%s`' % b.__name__ or
                 u':class:`%s.%s`' % (b.__module__, b.__name__)
                 for b in bases
                 if b.__name__ != "pybind11_object"]
        if not bases:
            return
        self.add_line(_(u'   Bases: %s') % ', '.join(bases),
                      sourcename)


def autodoc_skip_member(app, what, name, obj, skip, options):
    """Skips undesirable members.
    """
    # N.B. This should be registerd before `napoleon`s event.
    # N.B. For some reason, `:exclude-members` via `autodoc_default_options`
    # did not work. Revisit this at some point.
    if "__del__" in name:
        return True
    return None


def setup(app):
    """Installs Drake-specific extensions and patches.
    """
    app.add_stylesheet('css/custom.css')
    # Do not warn on Drake deprecations.
    # TODO(eric.cousineau): See if there is a way to intercept this.
    warnings.simplefilter('ignore', DrakeDeprecationWarning)
    # Ignore `pybind11_object` as a base.
    patch(
        autodoc.ClassDocumenter, 'add_directive_header',
        patch_class_add_directive_header)
    # Skip specific members.
    app.connect('autodoc-skip-member', autodoc_skip_member)
    # Register directive so we can pretty-print template declarations.
    pydoc.PythonDomain.directives['template'] = pydoc.PyClasslike
    # Register autodocumentation for templates.
    app.add_autodoc_attrgetter(TemplateBase, tpl_attrgetter)
    app.add_autodocumenter(TemplateDocumenter)
    # Hack regular expressions to make them irregular (nested).
    autodoc.py_ext_sig_re = IrregularExpression(extended=True)
    pydoc.py_sig_re = IrregularExpression(extended=False)
    patch(autodoc.ClassLevelDocumenter, 'resolve_name', patch_resolve_name)
    patch(autodoc.ModuleLevelDocumenter, 'resolve_name', patch_resolve_name)
    return dict(parallel_read_safe=True)
