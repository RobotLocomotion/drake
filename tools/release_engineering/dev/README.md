# Docker / Apt / S3 Release Process

## Initial Setup

This process only needs to be done once per system.

Note that these scripts are tested on Ubuntu Jammy;
other platforms are not supported.

### Install required packages

Linux:

    apt install gnupg1
    apt install aptly
    apt install docker
    apt install awscli


**Note:** There are compatibility issues with `gnupg2` and `aptly`. See
[this issue](https://github.com/aptly-dev/aptly/issues/657), for example.

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

    gpg[1] --import <key.asc>

**Note:** It is not clear if `gpg` or `gpg1` is correct to use on Ubuntu

### Log into Docker

Log into Docker to ensure that you will be able to push the Docker images:

    docker login

The Docker ID and password may be found in the AWS Secrets Manager.

### Create GitHub API Token

To create the required `~/.config/readonly_github_api_token.txt` file, open a
browser to https://github.com/settings/tokens and create a new token (it does
not need any extra permissions; the default "no checkboxes are set" is good),
and save the plaintext hexadecimal token to that file.

### Get the push_release scripts

Clone the drake repository:

    git clone https://github.com/RobotLocomotion/drake.git
    cd drake

## Run script to push docker images and mirror the .tar/.deb artifacts to S3

Once your machine is set-up, run the `push_release` script as described below:

    bazel run //tools/release_engineering/dev:push_release -- <version>


The release creator will provide the version. Throughout this process, don’t
use `v` on the version string. For example:

    bazel run //tools/release_engineering/dev:push_release -- 1.0.0

### Verification

Verify that:

1. [s3://drake-packages/drake/release](https://s3.console.aws.amazon.com/s3/buckets/drake-packages?region=us-east-1&prefix=drake/release/&showversions=false)
contains a set of `drake-<version>-[...].tar.gz[...]` files for each supported
configuration (e.g. jammy, noble, and mac).

1. [Dockerhub](https://hub.docker.com/r/robotlocomotion/drake/tags?ordering=last_updated&page=1)
has `<version>` tags for each supported configuration (e.g. jammy and noble).

1. The `*.deb` files are in AWS
[s3://drake-packages/drake/release](https://s3.console.aws.amazon.com/s3/buckets/drake-packages?region=us-east-1&prefix=drake/release/&showversions=false)
`/<configuration>/drake-dev_<version>-1_amd64.deb` for each supported configuration (e.g. jammy and noble)

## Run script for apt

(Before proceeding, refer to the sections below if you need to add a new
configuration or package.)

Once your machine is set-up, run the `push_release` script as described below:

    cd tools/release_engineering/dev
    ./push_release <version>

The release creator will provide the version. Again, don’t use `v` on the
version string. For example:

    ./push_release 0.32.0

The script will prompt for the GPG passphrase, which may be found in the AWS
Secrets Manager. The script may prompt for this multiple times.

### [Optional] Add a new configuration

For example, to add Jammy:

1. Edit `~/.aptly.conf` to add a `jammy` section
1. Add a `jammy` folder to
[drake-apt](https://s3.console.aws.amazon.com/s3/buckets/drake-apt?region=us-east-1&tab=objects)
s3 bucket
1. Edit the `push_release` script:

    1. After downloading the aptly database from S3, create the `drake-jammy`
    repo with the command
    ``aptly repo create -distribution=jammy drake-jammy``. For example:

            # Download the current version of the aptly database from S3.
            aws s3 sync --delete s3://drake-infrastructure/aptly/.aptly "${HOME}/.aptly"

            aptly repo create -distribution=jammy drake-jammy

    1. The first time a repo is published we must use
    ``aptly publish snapshot`` instead of ``aptly publish switch``:

            aptly publish snapshot -gpg-key="${gpg_key: -8}" -distribution="jammy" "drake-${platform}-${binary_version}" \
                  "s3:drake-apt.csail.mit.edu/${platform}:"

Follow instructions below as normal. Don’t forget to revert the changes to
the `push_release` script!

### [Optional] Add a package

First, check which packages are already included in the repo. From the command
line:

    aptly repo show -with-packages drake-<distro>

For example:

    aptly repo show -with-packages drake-jammy

    Name: drake-jammy
    Comment:
    Default Distribution: jammy
    Default Component: main
    Number of packages: 1
    Packages:
      Drake-dev_1.9.0-1_amd64


Edit the `push_release` script to add the package. Add the following line for
each package after `.aptly` directory is cloned from AWS but before the `for()`
loop so it’s only done once. Use the full path to the .deb file.

    aptly repo add [-force-replace] drake-<distro> <package_name>

For example:

    # Download the current version of the aptly database from S3.
    aws s3 sync --delete s3://drake-infrastructure/aptly/.aptly "${HOME}/.aptly"

    # Add new packages
    aptly repo add drake-jammy ~/drake_release/lcm_1.4.0-gabdd8a2_amd64.deb
    aptly repo add drake-jammy ~/drake_release/libbot2_0.0.1.20221116-1_amd64.deb

Follow instructions below as normal. Don’t forget to revert the changes to
the `push_release` script!

Verify packages have been added:

    aptly repo show -with-packages drake-jammy

    Name: drake-jammy
    Comment:
    Default Distribution: jammy
    Default Component: main
    Number of packages: 4
    Packages:
      drake-dev_1.10.0-1_amd64
      drake-dev_1.9.0-1_amd64
      lcm_1.4.0-gabdd8a2_amd64
      libbot2_0.0.1.20221116-1_amd64
