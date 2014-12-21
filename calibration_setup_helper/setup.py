#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['calibration_setup_helper'],
    package_dir={'': 'src'},
    scripts=['scripts/calibration_setup_helper.py']
)

setup(**d)
