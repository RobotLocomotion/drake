import sys
import traceback

# TODO(eric.cousineau): Make autocomplete ignore `ModuleShim` attributes
# (e.g. `install`).


class ModuleShim(object):
    """ Provide a shim for automatically resolving extra variables.
    This can be used to deprecate import alias in modules, to simplify
    dependencies.

    @see https://stackoverflow.com/a/7668273/7829525 """

    def __init__(self, orig_module, handler):
        assert hasattr(orig_module, "__all__"), (
            "Please define `__all__` for this module.")
        # https://stackoverflow.com/a/16237698/7829525
        object.__setattr__(self, '_orig_module', orig_module)
        object.__setattr__(self, '_handler', handler)

    def __getattr__(self, name):
        # Use the original module if possible.
        m = self._orig_module
        if hasattr(m, name):
            return getattr(m, name)
        else:
            # Otherwise, use the handler, and store the result.
            try:
                import_type = self._get_import_type()
                value = self._handler(name, import_type)
            except AttributeError as e:
                if e.message:
                    raise e
                else:
                    raise AttributeError(
                        "'module' object has no attribute '{}'".format(name))
            setattr(m, name, value)
            return value

    def __setattr__(self, name, value):
        # Redirect writes to the original module.
        setattr(self._orig_module, name, value)

    def _get_import_type(self):
        # Check what kind of import type this came from.
        sub = traceback.extract_stack()[-3:-1]
        assert sub[1][2] == "__getattr__"
        caller_text = sub[0][3].strip()
        if caller_text.startswith("from "):
            if caller_text.endswith("*"):
                return "from_all"
            else:
                return "from_direct"
        else:
            return "direct"

    def __repr__(self):
        return repr(self._orig_module)

    @classmethod
    def install(cls, name, handler):
        """ Hook into module's attribute accessors and mutators.
        @param name
            Module name. Generally should be __name__.
        @param handler
            Function of the form `handler(var, import_type)`, where `var` is
            the variable name and `import_type` is the type of import:
              "direct" implies this is called via `import mod; mod.{var}`
              "from_direct" implies this is called via `from mod import {var}`
              "from_all" implies this is called via `from mod import *`
        """
        old_module = sys.modules[name]
        new_module = cls(old_module, handler)
        sys.modules[name] = new_module
