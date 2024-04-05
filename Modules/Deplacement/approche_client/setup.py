#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

distutils_setup = generate_distutils_setup(
    packages=['approche_client'],
    package_dir={'':'python'}
)

setup(**distutils_setup)