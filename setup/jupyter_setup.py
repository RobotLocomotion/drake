import sys
import platform
from IPython import get_ipython


def setup_drake():
    """Install drake (if necessary) and set up the path.

    On Google's Colaboratory, this will take approximately two minutes on the
    first time it runs (to provision the machine), but should only need to
    reinstall once every 12 hours. Colab may ask you to "Reset all runtimes";
    say no to save yourself the reinstall.
    """
    try:
        import pydrake
    except ImportError:
        if platform.system() == "Darwin":
            get_ipython().system(
                u"if [ ! -d '/opt/drake' ]; then curl -o drake.tar.gz https://drake-packages.csail.mit.edu/drake/continuous/drake-latest-mac.tar.gz && tar -xzf drake.tar.gz -C /opt && export HOMEBREW_CURL_RETRIES=4 && brew update && brew bundle --file=/opt/drake/share/drake/setup/Brewfile --no-lock; fi"  # noqa
            )
        elif platform.linux_distribution() == ("Ubuntu", "18.04", "bionic"):
            get_ipython().system(
                u"if [ ! -d '/opt/drake' ]; then curl -o drake.tar.gz https://drake-packages.csail.mit.edu/drake/continuous/drake-latest-bionic.tar.gz && tar -xzf drake.tar.gz -C /opt &&apt-get update -o APT::Acquire::Retries=4 -qq && apt-get install -o APT::Acquire::Retries=4 -o Dpkg::Use-Pty=0 -qy --no-install-recommends $(cat /opt/drake/share/drake/setup/packages-bionic.txt); fi"  # noqa
            )
        else:
            assert False, "Unsupported platform"
        v = sys.version_info
        sys.path.append("/opt/drake/lib/python{}.{}/site-packages".format(
            v.major, v.minor))
