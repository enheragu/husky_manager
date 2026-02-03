#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Setup file for husky_manager Python modules.
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['web_manager'],
    package_dir={'': 'src'},
    requires=['rospy', 'flask', 'werkzeug']
)

setup(**setup_args)
