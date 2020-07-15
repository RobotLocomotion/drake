import os
import subprocess
import sys
from urllib.request import urlretrieve

def setup_drake(version, build='nightly'):
    """Install drake on Google's Colaboratory and set up the path.  This will
    take approximately two minutes (to provision the machine), but should only
    need to reinstall once every 12 hours. Colab may ask you to "Reset all
    runtimes"; say no to save yourself the reinstall.

    Args: 
        version: A string to identify which revision of drake to install (if
            needed).  Note that if drake is already installed, this *will*
            replace the installation.
        build: An optional string to specify the hosted directory on to
            https://drake-packages.csail.mit.edu/drake/ of the build identified 
            by version.  Current options are 'nightly', 'continuous', or
            'experimental'.  Default is 'nightly', which is recommended.

    Note: Possible version names vary depending on the build.  
        - Nightly builds are versioned by date, e.g., '20200725', and have the
          date of the prior evening (not the morning after).  You can also use
          'latest'. - Continuous builds are only available with the version
          'latest'. - Experimental builds use the version name
          '<timestamp>-<commit>'. See
          https://drake.mit.edu/jenkins#building-binary-packages-on-demand for
          information on obtaining a binary from an experimental branch.

    See https://drake.mit.edu/from_binary.html for more information.
    """

    assert 'google.colab' in sys.modules, "This script is intended for use on Google Colab only."

    if os.path.isdir('/opt/drake'):
        subprocess.call(['rm', '-rf', '/opt/drake'])

    base_url='https://drake-packages.csail.mit.edu/drake/'
    urlretrieve("{}{}/drake-{}-bionic.tar.gz".format(
        base_url, build, version), 'drake.tar.gz')
    subprocess.call(['tar', '-xzf', 'drake.tar.gz', '-C', '/opt'])
    subprocess.call(['apt-get', 'update', '-o',
        'APT::Acquire::Retries=4', '-qq'])
    with open("/opt/drake/share/drake/setup/packages-bionic.txt",
                "r") as f:
        packages = f.read().splitlines()
    subprocess.call(['apt-get', 'install', '-o',
        'APT::Acquire::Retries=4', '-o', 'Dpkg::Use-Pty=0', '-qy',
        '--no-install-recommends'] + packages)

    if 'pydrake' not in sys.modules:
        v = sys.version_info
        path = "/opt/drake/lib/python{}.{}/site-packages".format(
            v.major, v.minor)
        sys.path.append(path)

    # Note: Checking pydrake in sys.modules here was flakey.  That update seems
    # to happen asynchronously.
    try:
      import pydrake
    except ImportError:
      raise(Exception("pydrake installation failed!"))
