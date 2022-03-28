# This file contains Linux-specific logic used to build the PyPI wheels. See
# build-wheels for the user interface.

import atexit
import os
import subprocess
import sys
import tarfile

from collections import namedtuple
from datetime import datetime, timezone

from .common import die, gripe, wheel_name

resource_root = None

files_to_remove = []
images_to_remove = []

tag_base = 'pip-drake'

# This is the complete set of defined targets (i.e. potential wheels). By
# default, all targets are built, but the user may down-select from this set.
# The platform alias is used for Docker tag names, and, when combined with the
# Python version, must be unique.
Target = namedtuple('Target', [
    'python_version',
    'platform_name',
    'platform_version',
    'platform_alias',
])
targets = (
    Target('36', 'ubuntu', '18.04', 'bionic'),
    Target('37', 'ubuntu', '18.04', 'bionic'),
    Target('38', 'ubuntu', '18.04', 'bionic'),
    Target('39', 'ubuntu', '20.04', 'focal'),
)
glibc_versions = {
    'bionic': '2_27',
    'focal': '2_31',
}

# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


def docker(*args, pipe=False):
    """
    Runs a docker command.

    If pipe is false, blocks until completion and returns a CompletedProcess
    instance; raises an exception on failure.

    If pipe is true, returns the process instance with its stdout captured.
    """
    command = ['docker'] + list(args)
    environment = os.environ.copy()
    environment['DOCKER_BUILDKIT'] = '1'
    if pipe:
        return subprocess.Popen(command, stdout=subprocess.PIPE,
                                cwd=resource_root, env=environment)
    else:
        return subprocess.run(command, check=True,
                              cwd=resource_root, env=environment)


def cleanup():
    """
    Removes temporary artifacts on exit.
    """
    for f in files_to_remove:
        try:
            os.unlink(f)
        except FileNotFoundError:
            gripe(f'Warning: failed to remove \'{f}\'?')
    if len(images_to_remove):
        docker('image', 'rm', *images_to_remove)


def git_root(path):
    command = ['git', 'rev-parse', '--show-toplevel']
    raw = subprocess.check_output(command, cwd=path)
    return raw.decode(sys.stdout.encoding).rsplit('\n', maxsplit=1)[0]


def strip_tar_metadata(info):
    """
    Removes some metadata (owner, timestamp) from a TarInfo.
    """
    info.uid = info.gid = 0
    info.uname = info.gname = 'root'
    info.mtime = 0
    info.pax_headers = {}
    return info


def add_to_tar(tar, name, parent_path, root_path, exclude=[]):
    """
    Adds files or directories to the specified tar file.
    """
    tar_path = os.path.join(parent_path, name)
    full_path = os.path.join(root_path, parent_path, name)

    if os.path.isdir(full_path):
        for f in sorted(os.listdir(full_path)):
            if f in exclude:
                continue

            add_to_tar(tar, f, os.path.join(parent_path, name), root_path)
    else:
        tar.add(full_path, tar_path, recursive=False,
                filter=strip_tar_metadata)


def create_source_tar(path):
    """
    Creates a tarball of the repository working tree.
    """
    out = tarfile.open(path, "w:xz")

    repo_dir = git_root(resource_root)

    print('[-] Creating source archive', end='', flush=True)
    for f in sorted(os.listdir(repo_dir)):
        # Exclude build and VCS directories.
        if f == '.git' or f == 'user.bazelrc' or f.startswith('bazel-'):
            continue

        print('.', end='', flush=True)
        exclude = ['wheel'] if f == 'tools' else []
        add_to_tar(out, f, '', repo_dir, exclude=exclude)

    print(' done')
    out.close()


def build_stage(target, args, tag_prefix, stage=None):
    """
    Runs a Docker build and return the build tag.
    """

    # Generate canonical tag from target.
    platform = target.platform_alias
    tag = f'{tag_base}:{tag_prefix}-{platform}-py{target.python_version}'

    # Generate extra arguments to specify what stage to build.
    if stage is not None:
        extra = ['--target', stage]
    else:
        extra = []

    # Run the build.
    print('[-] Build', tag, extra + args)
    docker('build', '--tag', tag, *extra, *args, resource_root)

    return tag


def target_args(target):
    """
    Returns the docker build arguments for the specified platform target.
    """
    platform_name = target.platform_name
    platform_version = target.platform_version
    python_version = target.python_version
    return [
        '--build-arg', f'PYTHON={python_version[0]}.{python_version[1:]}',
        '--build-arg', f'PLATFORM={platform_name}:{platform_version}',
    ]


def build_image(target, identifier, options):
    """
    Runs the build for a target and (optionally) extract the wheel.
    """
    args = [
        '--ssh', 'default',
        '--build-arg', f'DRAKE_VERSION={options.version}',
    ] + target_args(target)
    if not options.keep_containers:
        args.append('--force-rm')

    # Build the image.
    if options.tag_stages:
        # Inspect Dockerfile, find stages, and build them.
        for line in open(os.path.join(resource_root, 'Dockerfile')):
            if line.startswith('FROM'):
                stage = line.strip().split()[-1]
                tag = build_stage(target, args, tag_prefix=stage, stage=stage)
    else:
        tag = build_stage(target, args, tag_prefix=identifier)
        images_to_remove.append(tag)

    # Extract the wheel (if requested).
    if options.extract:
        print('[-] Extracting wheel(s) from', tag)

        command = 'tar -cf - /wheel/wheelhouse/*.whl'
        extractor = docker(
            'run', '--rm', tag, 'bash', '-c', command, pipe=True)
        subprocess.check_call(
            ['tar', '--strip-components=2', '-xf', '-'],
            stdin=extractor.stdout, cwd=options.output_dir)

        extractor_result = extractor.wait()
        if extractor_result:
            raise subprocess.CalledProcessError(extractor_result,
                                                extractor.args, None, None)


def test_wheel(target, options):
    """
    Runs the test script for the wheel matching the specified target.
    """
    glibc = glibc_versions[target.platform_alias]
    wheel = wheel_name(python_version=target.python_version,
                       wheel_version=options.version,
                       wheel_platform=f'manylinux_{glibc}_x86_64')

    platform = target.platform_alias
    container = f'{tag_base}:test-{platform}-py{target.python_version}'
    test_dir = os.path.join(resource_root, 'test')

    docker('build', '-t', container, *target_args(target), test_dir)
    images_to_remove.append(container)

    docker('run', '--rm', '-t',
           '-v' f'{test_dir}:/test',
           '-v' f'{options.output_dir}:/wheel',
           container, '/test/test-wheel.sh', f'/wheel/{wheel}')


def build(options):
    # Collect set of wheels to be built.
    targets_to_build = []
    for t in targets:
        if t.platform_name in options.platforms:
            if t.python_version in options.python_versions:
                targets_to_build.append(t)

    # Check if there is anything to do.
    if not len(targets_to_build):
        die('Nothing to do! (Platform and/or Python version selection '
            'resulted in an empty set of wheels)')

    # Generate a unique identifier for this build, if needed.
    if options.tag_stages:
        identifier = None
    else:
        salt = os.urandom(8).hex()
        time = datetime.now(timezone.utc).strftime('%Y%m%d%H%M%S')
        identifier = f'{time}-{salt}'

    # Generate the repository source archive.
    source_tar = os.path.join(resource_root, 'image', 'drake-src.tar.xz')
    files_to_remove.append(source_tar)
    create_source_tar(source_tar)

    # Build the requested wheels.
    for t in targets_to_build:
        build_image(t, identifier, options)

        if options.test:
            test_wheel(t, options)


def add_build_arguments(parser):
    parser.add_argument(
        '-k', '--keep-containers', action='store_true',
        help='do not delete intermediate containers')
    parser.add_argument(
        '-s', '--tag-stages', action='store_true',
        help='permanently tag individual stages')


def add_selection_arguments(parser):
    parser.add_argument(
        '--platform', dest='platforms',
        default=','.join(set([t.platform_name for t in targets])),
        help='platform(s) to build; separate with \',\''
             ' (default: %(default)s)')
    parser.add_argument(
        '--python', dest='python_versions', metavar='VERSIONS',
        default=','.join(sorted(set([t.python_version for t in targets]))),
        help='python version(s) to build; separate with \',\''
             ' (default: %(default)s)')


def fixup_options(options):
    options.python_versions = set(options.python_versions.split(','))
    options.platforms = set(options.platforms.split(','))

    if options.test and not options.extract:
        die('Testing wheels requires wheels to be extracted')


# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

atexit.register(cleanup)
