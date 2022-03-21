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
import os
import shutil
import subprocess
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
    deb_control = _rlocation('debian/control')
    deb_copyright = _rlocation('debian/copyright')
    deb_changelog_in = _rlocation('debian/changelog.in')

    # Discern the version badging to use.
    with tarfile.open(args.tgz) as archive:
        version = archive.getmember('drake/share/doc/drake/VERSION.TXT')
        version_txt = archive.extractfile(version).read().decode('utf-8')
        version_mtime = version.mtime
    version_tokens = version_txt.split()
    assert len(version_tokens) == 2, version_txt
    yyyymmddhhmmss, git_sha = version_tokens
    debian_version = f'0.0.{yyyymmddhhmmss}'

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
    shutil.copy(deb_control, f'{package_dir}/debian/')
    shutil.copy(deb_copyright, f'{package_dir}/debian/')
    with open(f'{package_dir}/debian/changelog', 'w', encoding='utf-8') as f:
        f.write(deb_changelog_contents)

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
