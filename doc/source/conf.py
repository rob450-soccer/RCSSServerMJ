# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html
import sys
from pathlib import Path

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here.
# sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]) + '/src')

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

from rcsssmj import __VERSION__ as RCSSSMJ_VERSION

project = 'RCSSServerMJ'
copyright = f'2025, {project} Developers'
author = 'Stefan Glaser'
release = RCSSSMJ_VERSION

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
   'sphinx.ext.autodoc',
   'sphinx.ext.autosummary',
]

templates_path = ['_templates']
exclude_patterns = []


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'pydata_sphinx_theme'
html_static_path = ['_static']

html_theme_options = {
    "logo": {
        "alt_text": f"{project} - Home",
        "text": project,
      #   "image_light": "_static/logo-light.png",
      #   "image_dark": "_static/logo-dark.png",
    },
    "icon_links": [
        {
            "name": "GitLab",
            "url": "https://gitlab.com/robocup-sim/rcssservermj",
            "icon": "fa-brands fa-square-gitlab",
            "type": "fontawesome",
        },
    ],
}



# -- RCSSServerMJ-specific options -----------------------------------------------

rst_prolog = f"""
.. |ndash|   unicode:: U+02013 .. En dash
.. |mdash|   unicode:: U+02014 .. Em dash
.. |larrow|  unicode:: U+02190 .. Left arrow
.. |uarrow|  unicode:: U+02191 .. Up arrow
.. |rarrow|  unicode:: U+02192 .. Right arrow
.. |darrow|  unicode:: U+02193 .. Down arrow
.. |lrarrow| unicode:: U+02194 .. Left-Right arrow
.. |udarrow| unicode:: U+02195 .. Up-Down arrow

.. |br| raw:: html

   <br />

"""