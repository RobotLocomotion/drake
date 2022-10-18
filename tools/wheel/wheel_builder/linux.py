# This file contains Linux-specific logic used to build the PyPI wheels. See
# //tools/wheel:builder for the user interface.

import atexit
import os
import pathlib
import subprocess
import sys
import tarfile

from datetime import datetime, timezone

from .common import die, gripe, wheel_name
from .common import resource_root, wheelhouse
from .common import find_tests

from .linux_types import Platform, Role, Target, BUILD, TEST

# Artifacts that need to be cleaned up. DO NOT MODIFY outside of this file.
_files_to_remove = []
_images_to_remove = []

tag_base = 'pip-drake'

# This is the complete set of defined targets (i.e. potential wheels). By
# default, all targets are built, but the user may down-select from this set.
# The platform alias is used for Docker tag names, and, when combined with the
# Python version, must be unique.
targets = (
    Target(
        build_platform=Platform('ubuntu', '20.04', 'focal'),
        test_platform=None,
        python_version_tuple=(3, 8)),
    Target(
        build_platform=Platform('ubuntu', '20.04', 'focal'),
        test_platform=None,
        python_version_tuple=(3, 9)),
    Target(
        build_platform=Platform('ubuntu', '20.04', 'focal'),
        test_platform=Platform('ubuntu', '22.04', 'jammy'),
        python_version_tuple=(3, 10, 6),
        python_sha='f795ff87d11d4b0c7c33bc8851b0c28648d8a4583aa2100a98c22b4326b6d3f3'),  # noqa
)
glibc_versions = {
    'focal': '2_31',
}


def _path_depth(path):
    """
    Return the number of components (i.e. the "depth") of `path`.
    """
    offset = 1 if os.path.isabs(path) else 0  # Strip leading '/'.
    return len(pathlib.Path(path).parts[offset:])


def _docker(*args, pipe=False):
    """
    Runs a docker command.

    If `pipe` is False, blocks until completion and returns a
    CompletedProcess instance; raises an exception on failure.

    If `pipe` is True, returns the process instance with its stdout captured.
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


@atexit.register
def _cleanup():
    """
    Removes temporary artifacts on exit.
    """
    for f in _files_to_remove:
        try:
            os.unlink(f)
        except FileNotFoundError:
            gripe(f'Warning: failed to remove \'{f}\'?')
    if len(_images_to_remove):
        _docker('image', 'rm', *_images_to_remove)


def _git_root(path):
    """
    Determines the canonical repository root of the working tree which includes
    `path`.
    """
    command = ['git', 'rev-parse', '--show-toplevel']
    raw = subprocess.check_output(command, cwd=path)
    return raw.decode(sys.stdout.encoding).rsplit('\n', maxsplit=1)[0]


def _strip_tar_metadata(info):
    """
    Removes some metadata (owner, timestamp) from a TarInfo.
    """
    info.uid = info.gid = 0
    info.uname = info.gname = 'root'
    info.mtime = 0
    info.pax_headers = {}
    return info


def _add_to_tar(tar, name, parent_path, root_path, exclude=[]):
    """
    Adds files or directories to the specified tar file.
    """
    tar_path = os.path.join(parent_path, name)
    full_path = os.path.join(root_path, parent_path, name)

    if os.path.isdir(full_path):
        for f in sorted(os.listdir(full_path)):
            if f in exclude:
                continue

            _add_to_tar(tar, f, os.path.join(parent_path, name), root_path)
    else:
        tar.add(full_path, tar_path, recursive=False,
                filter=_strip_tar_metadata)


def _create_source_tar(path):
    """
    Creates a tarball of the repository working tree.
    """
    out = tarfile.open(path, "w:xz")

    repo_dir = _git_root(resource_root)

    print('[-] Creating source archive', end='', flush=True)
    for f in sorted(os.listdir(repo_dir)):
        # Exclude build and VCS directories.
        if f == '.git' or f == 'user.bazelrc' or f.startswith('bazel-'):
            continue

        print('.', end='', flush=True)
        exclude = ['wheel'] if f == 'tools' else []
        _add_to_tar(out, f, '', repo_dir, exclude=exclude)

    print(' done')
    out.close()


def _tagname(target: Target, role: Role, tag_prefix: str):
    """
    Generates a Docker tag name for a target and tag prefix.
    """
    platform = target.platform(role).alias
    return f'{tag_base}:{tag_prefix}-{platform}-py{target.python_tag}'


def _build_stage(target, args, tag_prefix, stage=None):
    """
    Runs a Docker build and return the build tag.
    """

    # Generate canonical tag from target.
    tag = _tagname(target, BUILD, tag_prefix)

    # Generate extra arguments to specify what stage to build.
    if stage is not None:
        extra = ['--target', stage]
    else:
        extra = []

    # Run the build.
    print('[-] Build', tag, extra + args)
    _docker('build', '--tag', tag, *extra, *args, resource_root)

    return tag


def _target_args(target: Target, role: Role):
    """
    Returns the docker build arguments for the specified platform target.
    """
    platform_name = target.platform(role).name
    platform_version = target.platform(role).version
    python_version = target.python_version

    if role == BUILD and target.python_sha is not None:
        python_args = [
            '--build-arg', f'PYTHON=build:{target.python_version_full}',
            '--build-arg', f'PYTHON_SHA={target.python_sha}',
        ]
    else:
        python_args = [
            '--build-arg', f'PYTHON={python_version}',
        ]

    return [
        '--build-arg', f'PLATFORM={platform_name}:{platform_version}',
    ] + python_args


def _build_image(target, identifier, options):
    """
    Runs the build for a target and (optionally) extract the wheel.
    """
    args = [
        '--ssh', 'default',
        '--build-arg', f'DRAKE_VERSION={options.version}',
    ] + _target_args(target, BUILD)
    if not options.keep_containers:
        args.append('--force-rm')

    # Build the image.
    if options.tag_stages:
        # Inspect Dockerfile, find stages, and build them.
        for line in open(os.path.join(resource_root, 'Dockerfile')):
            if line.startswith('FROM'):
                stage = line.strip().split()[-1]
                tag = _build_stage(target, args, tag_prefix=stage, stage=stage)
    else:
        tag = _build_stage(target, args, tag_prefix=identifier)
        _images_to_remove.append(tag)

    # Extract the wheel (if requested).
    if options.extract:
        print('[-] Extracting wheel(s) from', tag)

        wheelhouse_parts = _path_depth(wheelhouse)
        wheel_glob = os.path.join(wheelhouse, '*.whl')

        command = f'tar -cf - {wheel_glob}'
        extractor = _docker(
            'run', '--rm', tag, 'bash', '-c', command, pipe=True)
        subprocess.check_call(
            ['tar', f'--strip-components={wheelhouse_parts}', '-xvf', '-'],
            stdin=extractor.stdout, cwd=options.output_dir)

        extractor_result = extractor.wait()
        if extractor_result:
            raise subprocess.CalledProcessError(extractor_result,
                                                extractor.args, None, None)


def _test_wheel(target, identifier, options):
    """
    Runs the test script for the wheel matching the specified target.
    """
    glibc = glibc_versions[target.platform(BUILD).alias]
    wheel = wheel_name(python_version=target.python_tag,
                       wheel_version=options.version,
                       wheel_platform=f'manylinux_{glibc}_x86_64')

    test_image = _tagname(target, TEST, f'test-{identifier}')
    test_container = test_image.replace(':', '__')
    if options.tag_stages:
        base_image = _tagname(target, TEST, 'test')
    else:
        base_image = test_container
    test_dir = os.path.join(resource_root, 'test')

    # Build the test base image.
    _docker('build', '-t', base_image, *_target_args(target, TEST), test_dir)
    if not options.tag_stages:
        _images_to_remove.append(base_image)

    # Install the wheel.
    install_script = '/test/install-wheel.sh'
    _docker('run', '-t', f'--name={test_container}',
            '-v' f'{test_dir}:/test',
            '-v' f'{options.output_dir}:{wheelhouse}',
            base_image, install_script, os.path.join(wheelhouse, wheel))

    # Tag the container with the wheel installed.
    _docker('commit', test_container, test_image)
    _docker('container', 'rm', test_container)
    if options.tag_stages:
        _images_to_remove.append(test_image)

    # Run individual tests.
    test_script = '/test/test-wheel.sh'
    for test in find_tests():
        print(f'[-] Executing test {test}')
        _docker('run', '--rm', '-t',
                '-v' f'{test_dir}:/test',
                '-v' f'{options.output_dir}:{wheelhouse}',
                test_image, test_script, f'/test/{test}',
                os.path.join(wheelhouse, wheel))
        print(f'[-] Executing test {test} - PASSED')


def build(options):
    """
    Builds wheel(s) with the provided options.
    """

    # Collect set of wheels to be built.
    targets_to_build = []
    for t in targets:
        if t.platform(BUILD).name in options.platforms:
            if t.python_tag in options.python_versions:
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
    _files_to_remove.append(source_tar)
    _create_source_tar(source_tar)

    # Build the requested wheels.
    for t in targets_to_build:
        _build_image(t, identifier, options)

        if options.test:
            _test_wheel(t, identifier, options)


def add_build_arguments(parser):
    """
    Adds arguments that control the build.
    """
    parser.add_argument(
        '-k', '--keep-containers', action='store_true',
        help='do not delete intermediate containers')
    parser.add_argument(
        '-s', '--tag-stages', action='store_true',
        help='permanently tag individual stages')


def add_selection_arguments(parser):
    """
    Adds arguments that control which wheel(s) to build.
    """
    parser.add_argument(
        '--platform', dest='platforms',
        default=','.join(set([t.platform(BUILD).name for t in targets])),
        help='platform(s) to build; separate with \',\''
             ' (default: %(default)s)')
    parser.add_argument(
        '--python', dest='python_versions', metavar='VERSIONS',
        default=','.join(sorted(set([t.python_tag for t in targets]))),
        help='python version(s) to build; separate with \',\''
             ' (default: %(default)s)')


def fixup_options(options):
    """
    Validates options and applies any necessary transformations.
    (Converts comma-separated strings to sets.)
    """
    options.python_versions = set(options.python_versions.split(','))
    options.platforms = set(options.platforms.split(','))

    if options.test and not options.extract:
        die('Testing wheels requires wheels to be extracted')
