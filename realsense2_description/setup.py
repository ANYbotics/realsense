from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['any_realsense2_description'],
    package_dir={'': 'tests'},
)

setup(**setup_args)
