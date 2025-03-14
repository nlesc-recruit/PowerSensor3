# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information
from datetime import datetime

project = 'PowerSensor 3'
current_year = datetime.now().year
author = 'Leon Oostrum, John Romein, Ben van Werkhoven, Quinten Twisk, Gijs Schoonderbeek, Steven van der Vlugt'
copyright = f'{current_year}, {author}'
release = '1.6.1'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = ['breathe', 'exhale', 'myst_parser']

source_suffix = {'.rst': 'restructuredtext',
                 '.md': 'markdown'}

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

breathe_projects = {'powersensor3': './_doxygen/xml'}
breathe_default_project = 'powersensor3'

exhale_args = {'containmentFolder': './api',
               'rootFileName': 'library_root.rst',
               'doxygenStripFromPath': '..',
               'rootFileTitle': '',
               'createTreeView': True,
               'exhaleExecutesDoxygen': True,
               'exhaleDoxygenStdin': 'INPUT = ../host/include,../host/src/PowerSensor.cc,../host/src/sensors.cc'}

primary_domain = 'cpp'
highlight_language = 'cpp'
