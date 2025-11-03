from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rh6_can_py'],
    package_dir={'': ''},  # package is in rh6_can_py/
)

setup(**d)