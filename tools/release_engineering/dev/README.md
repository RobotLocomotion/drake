# Docker / Apt / S3 Release Process

## Initial Setup

This process only needs to be done once per system.

Note that these scripts are tested on Ubuntu Noble;
other platforms are not supported.

### Install required packages

Install the maintainer-required prerequisites:

  setup/install_prereqs --with-maintainer-only

Follow instructions to install Docker
https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository

Follow instructions to install awscli
https://docs.aws.amazon.com/cli/latest/userguide/getting-started-install.html

### Configure aptly

Download `.aptly.conf` from S3 to your home directory

On AWS, in IAM / Users / \<username\> / Security Credentials, select
`Create Access Key`.

Replace all (3) instances of "awsAccessKeyID" and "awsSecretAccessKey" in
`.aptly.conf` with the created values.

Set `rootDir` in `.aptly.conf` to your local home directory
(e.g. `/Users/<username>/.aptly`)

### Import the gpg key

Download the public key from S3.

Using the passphrase from the AWS Secrets Manager, run:

    gpg --import <key.asc>

### Log into Docker

Log into Docker to ensure that you will be able to push the Docker images:

    docker login

The Docker ID and password may be found in the AWS Secrets Manager.

### Create GitHub API Token

To create the required `~/.config/readonly_github_api_token.txt` file, open a
browser to https://github.com/settings/tokens and create a new token, making
sure that `repo` is selected. Save the plaintext hexadecimal token to that
file.

### Get the push_release scripts

Clone the drake repository:

    git clone https://github.com/RobotLocomotion/drake.git
    cd drake

## Run script for GitHub / S3

The next step is to mirror the GitHub release artifacts to S3.

Once your machine is set up, run the `push_release` script as described below:

    bazel run //tools/release_engineering/dev:push_release -- <version>

The release creator will provide the version. Throughout this process, don’t
use `v` on the version string. For example:

    bazel run //tools/release_engineering/dev:push_release -- 1.0.0

## Run script for Docker

The next step is to push Docker images. Run the `push_docker` script as
described below:

    bazel run //tools/release_engineering/dev:push_docker -- <version>

### Verification

Verify that:

1. [s3://drake-packages/drake/release](https://s3.console.aws.amazon.com/s3/buckets/drake-packages?region=us-east-1&prefix=drake/release/&showversions=false)
contains:

    * Binaries: A set of `drake-<version>-[...].tar.gz` files for each supported
    configuration (e.g. jammy, noble, and mac). In addition, there should be
    `.sha256` and `.sha512` files for each `.tar.gz` file.
    * Wheels: A set of `drake-<version>-[...].whl` files for each supported
    configuration (Python version and platform; e.g., cp3{10-14}-manylinux and
    cp3{13-14}-macosx). In addition, there should be `.sha256` and `.sha512`
    files for each `.whl` file.
    * Source: A `drake-<version>-src.tar.gz` file and corresponding `.sha256`
    and `.sha512` files.

2. The `*.deb` and corresponding `.sha256` and `.sha512`  files are in AWS
[s3://drake-packages/drake/release](https://s3.console.aws.amazon.com/s3/buckets/drake-packages?region=us-east-1&prefix=drake/release/&showversions=false)
for each supported configuration (e.g. jammy and noble) as
`/drake-dev_<version>-1_amd64-<configuration>.deb`.

3. [Dockerhub](https://hub.docker.com/r/robotlocomotion/drake/tags?ordering=last_updated&page=1)
has a plain `<version>` tag as well as a `<version>` tag for each supported
configuration (e.g. jammy and noble).


## Run script for apt

Once your machine is set-up, run the `push_release` script as described below:

    cd tools/release_engineering/dev
    ./push_release <version>

The release creator will provide the version. Again, don’t use `v` on the
version string. For example:

    ./push_release 1.0.0

### Verification

Verify that you can install drake via APT, see https://drake.mit.edu/apt.html
for instructions.
