# This script re-packs Drake's *.tar.gz release binaries into *.deb file(s).
#
# It currently under development and is not yet used in production.
# Known deficiencies:
#
# (*) Missing dependencies. We need to scrape the list of required packages
# from within the tgz (e.g., drake/share/drake/setup/packages-focal.txt)
# and mix that into our debian/control file. Possibly we'd want to split it
# into Depends vs Recommends vs Suggests:
# https://www.debian.org/doc/debian-policy/ch-relationships.html
#
# (*) Lots of shlibdebs spam during build. We should at least nerf the stuff
# from drake-visualizer, for starters. Possibly we shouldn't be calling
# shlibdeps at all, assuming that packages-focal.txt is authoritative.

import argparse
import email.utils
import itertools
import lsb_release
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile

from bazel_tools.tools.python.runfiles import runfiles


def _rlocation(relative_path):
    manifest = runfiles.Create()
    resource_path = f'drake/tools/release_engineering/dev/{relative_path}'
    resolved_path = manifest.Rlocation(resource_path)
    assert resolved_path, f'Missing {resource_path}'
    return os.path.realpath(resolved_path)


def _run(args):
    """Implements all steps for repacking tgz => deb.
    """
    # Find our runfiles.
    deb_compat = _rlocation('debian/compat')
    deb_control_in = _rlocation('debian/control.in')
    deb_copyright = _rlocation('debian/copyright')
    deb_changelog_in = _rlocation('debian/changelog.in')

    # Discern the version badging to use, get the dependencies for drake.
    codename = lsb_release.get_os_release()['CODENAME']
    with tarfile.open(args.tgz) as archive:
        version = archive.getmember('drake/share/doc/drake/VERSION.TXT')
        version_txt = archive.extractfile(version).read().decode('utf-8')
        version_mtime = version.mtime

        packages = archive.getmember(
            f'drake/share/drake/setup/packages-{codename}.txt')
        packages_txt = archive.extractfile(packages).read().decode('utf-8')

    version_tokens = version_txt.split()
    assert len(version_tokens) == 2, version_txt
    yyyymmddhhmmss, git_sha = version_tokens
    debian_version = f'0.0.{yyyymmddhhmmss}'

    # Compute the new control.  The `packages_txt` will have one package-name
    # per line, we add a couple more dependencies and then transform this to
    # be an indented and comma separated list.
    packages_txt += '${shlibs:Depends}\n${misc:Depends}'
    packages_txt = packages_txt.replace('\n', ',\n         ')
    with open(deb_control_in, encoding='utf-8') as f:
        deb_control_contents = f.read().format(
            depends=packages_txt)

    # Compute the new changelog.
    with open(deb_changelog_in, encoding='utf-8') as f:
        deb_changelog_contents = f.read().format(
            debian_version=debian_version,
            git_sha=git_sha,
            date=email.utils.formatdate(version_mtime),
        )

    # Use `alien` to prepare the debian packaging rules.
    cwd = f'{args.tempdir}/alien'
    os.mkdir(cwd)
    alien = [
        'fakeroot',
        '/usr/bin/alien',
        '--to-deb',
        '--single',
        f'--version={debian_version}',
        '--keep-version',
        args.tgz,
    ]
    env = dict(os.environ)
    env['EMAIL'] = 'drake-users@mit.edu'
    subprocess.check_call(alien, cwd=cwd, env=env)
    package_dir = f'{cwd}/drake-{debian_version}'

    # Install into /opt/drake, not /drake.
    os.mkdir(f'{package_dir}/opt')
    os.rename(f'{package_dir}/drake',
              f'{package_dir}/opt/drake')

    # Overwrite some metadata.
    shutil.copy(deb_compat, f'{package_dir}/debian/')
    with open(f'{package_dir}/debian/control', 'w', encoding='utf-8') as f:
        f.write(deb_control_contents)
    shutil.copy(deb_copyright, f'{package_dir}/debian/')
    with open(f'{package_dir}/debian/changelog', 'w', encoding='utf-8') as f:
        f.write(deb_changelog_contents)

    # Patch RPATH entries for drake-visualizer.
    lib_dir = Path(package_dir) / 'opt' / 'drake' / 'lib'
    # VTK C++ libraries need to be able to find their C++ counterparts.
    for vtk_cxx_lib in lib_dir.glob('libvtk*-8.2.so*'):
        subprocess.check_call([
            'patchelf',
            '--set-rpath',
            '$ORIGIN',
            str(vtk_cxx_lib),
        ])
    # VTK Python libraries and director/vtkDRCFiltersPython.so need to be able
    # to find their C++ counterparts.  They live in equitable directory
    # locations to be able to patch the same rpath.
    py_ver = f'python{".".join([str(d) for d in sys.version_info[0:2]])}'
    site_packages = lib_dir / py_ver / 'site-packages'
    vtk_py_libs = list((site_packages / 'vtkmodules').glob('vtk*.so'))
    director_lib = [site_packages / 'director' / 'vtkDRCFiltersPython.so']
    # The director director/vtkDRCFiltersPython.so liv
    for py_lib in itertools.chain(vtk_py_libs, director_lib):
        subprocess.check_call([
            'patchelf',
            '--set-rpath',
            '$ORIGIN/../../..',
            str(py_lib),
        ])

    # Remove shebang for scripts in director (drake-visualizer).
    for script in {'min_bounding_rect.py', 'qhull_2d.py'}:
        path = site_packages / 'director' / 'thirdparty' / script
        with open(path, 'r', encoding='utf-8') as f:
            contents = f.read()
            contents = contents.replace('#!/usr/bin/python', '#', 1)
        with open(path, 'w', encoding='utf-8') as f:
            f.write(contents)

    # Fix directory permissions for all of drake.
    for root, dirs, files in os.walk(package_dir + '/opt'):
        for d in dirs:
            os.chmod(os.path.join(root, d), 0o755)

    # Fix file permissions for e.g., header files, license files, many others
    # where typically 664 but needs 644.  Also fixes a handful of .so files
    # that have their executable bits set (e.g., VTK).
    for d in {'/opt/drake/include', '/opt/drake/lib', '/opt/drake/share'}:
        for root, dirs, files in os.walk(package_dir + d):
            for f in files:
                os.chmod(os.path.join(root, f), 0o644)

    # non-standard-executable-perm: 0775 != 0755
    non_standard_perm = ['/opt/drake/bin/optitrack_client']
    script_not_executable = [
        '/opt/drake/share/drake/setup/deepnote/install_nginx',
        '/opt/drake/share/drake/setup/install_prereqs',
    ]
    for script in itertools.chain(non_standard_perm, script_not_executable):
        os.chmod(package_dir + script, 0o755)

    # Create the deb.
    subprocess.check_call(['fakeroot', 'debian/rules', 'binary'],
                          cwd=package_dir)
    shutil.move(f'{cwd}/drake_{debian_version}-1_amd64.deb',
                f'{args.output_dir}/')


def main():
    parser = argparse.ArgumentParser(
        description='Re-package Drake into Debian files.')
    parser.add_argument(
        '--tgz', type=str, required=True,
        help='the foo.tar.gz filename to be re-packaged')
    parser.add_argument(
        '--output-dir', metavar='DIR', default=os.path.realpath('.'),
        help='directory to place *.deb output (default: .)')
    args = parser.parse_args()
    args.tgz = os.path.realpath(args.tgz)

    with tempfile.TemporaryDirectory(prefix='drake-repack-') as tempdir:
        args.tempdir = tempdir
        _run(args)


if __name__ == '__main__':
    main()
