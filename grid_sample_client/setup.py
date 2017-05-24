## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()

d['name'] = "grid_sample_client"
d['description'] = "Python interface for GraspIt! grid sampler"
d['packages'] = ['grid_sample_client']
d['package_dir'] = {'': 'src'}

setup(**d)

