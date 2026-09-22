#!/usr/bin/env python3
"""catkin setup for the fr3_teleop_support Python package."""

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup(
    **generate_distutils_setup(
        packages=["fr3_teleop_support"],
        package_dir={"": "src"},
    )
)
