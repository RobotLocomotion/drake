# -*- coding: utf-8 -*-
#
# Configuration file for the Sphinx documentation builder.
#
# This file does only contain a selection of the most common options. For a
# full list see the documentation:
# http://www.sphinx-doc.org/en/master/config


# -- Project information -----------------------------------------------------

project = u'pydrake'
copyright = u''
author = u''

# The short X.Y version
version = u''
# The full version, including alpha/beta/rc tags
release = u''


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.mathjax',
    # Precede napoleon, since it's greedy on `autodoc-skip-member`
    'drake.doc.pydrake.pydrake_sphinx_extension',
    'sphinx.ext.napoleon',
]

# Option available in Sphinx 1.5+.
napoleon_include_init_with_doc = True

# The suffix(es) of source filenames.
source_suffix = '.rst'

# The master toctree document.
master_doc = 'index'

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'sphinx_rtd_theme'

html_static_path = ['_static']

html_copy_source = False

html_show_copyright = False

html_show_sphinx = False
autodoc_member_order = 'bycustomfunction'
