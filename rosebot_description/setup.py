## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
import os

os.environ['DISTUTILS_DEBUG'] = "1"

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=[ 'rosebotnav', 'pygapp'],
 	package_dir={'':'src'},
	#include_package_data=True,
    package_data={'rosebotnav': ['sauvegardes/*', 'media/images/robomovies.jpg', 'media/fonts/*'],},
)

setup(**setup_args)
