from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
     packages=['py_ext_control_template'],
     package_dir={'': 'src'}
)

setup(**setup_args)