# Note: use setuptools.glob rather than the built-in glob; see
# https://bugs.python.org/issue37578.
import os
import setuptools
from setuptools import setup, find_packages, glob
from setuptools.dist import Distribution

DRAKE_VERSION = '0.32.0'

# Required python packages that will be pip installed along with pydrake
# TODO Can we remove any of these?
python_required = [
    'matplotlib',
    'meshcat',
    'numpy',
    'pydot',
    'pygame',
    'PyYAML',
    'scipy',
]


# Distribution which always forces a binary package with platform name.
class BinaryDistribution(Distribution):
    def is_pure(self):
        return False

    def has_ext_modules(self):
        return True


def find_data_files(*patterns):
    result = []
    for pattern in patterns:
        result += [f'../{f}' for f in glob.iglob(pattern, recursive=True)]
    return result


def make_alias(name):
    if name is None:
        f = '__init__.py'
        i = 'pydrake'
    else:
        f = f'{name}.py'
        i = f'pydrake.{name}'

    with open(os.path.join('drake', f), 'w') as m:
        m.write(f'from {i} import *\n')


os.makedirs('drake', exist_ok=True)

make_alias(None)
make_alias('all')
make_alias('common')
make_alias('examples')
make_alias('forwarddiff')
make_alias('manipulation')
make_alias('multibody')
make_alias('solvers')
make_alias('systems')
make_alias('visualization')

setup(name='drake',
      version=DRAKE_VERSION,
      description='Model-based design and verification for robotics',
      long_description='''
Drake ("dragon" in Middle English) is a toolbox maintained by the Robot
Locomotion Group at the MIT Computer Science and Artificial Intelligence
Lab (CSAIL). It is a collection of tools for analyzing the dynamics of
robots and building control systems for them in C++ and Python, with a
heavy emphasis on optimization-based design/analysis.'''.strip(),
      url='https://drake.mit.edu',
      author='Drake Development Team',
      author_email='drake-users@mit.edu',
      classifiers=[
        'Development Status :: 4 - Beta',
        'Environment :: Console',
        'Intended Audience :: Developers',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: BSD License',
        'Operating System :: MacOS',
        'Operating System :: POSIX :: Linux',
        'Programming Language :: Python :: 3 :: Only',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: Implementation :: CPython',
        'Topic :: Scientific/Engineering',
        'Topic :: Software Development :: Libraries :: Python Modules'],
      distclass=BinaryDistribution,
      # TODO Check this: do we need to add third-party licenses?
      license='BSD 3-Clause License',
      platforms=['linux_x86_64'],
      packages=find_packages(),
      # Add in any packaged data.
      include_package_data=True,
      package_data={
          '': find_data_files(
              'pydrake/**/*.so',
              'pydrake/lib/**',
              'pydrake/doc/**',
              'pydrake/share/**',
          )
      },
      python_requires='>=3.6',
      install_requires=python_required,
      ext_modules=[
           setuptools.Extension(name='drake',
                                sources=[])],
      zip_safe=False
      )
