# Docker / Apt / S3 Release Process

## Initial Setup

This process only needs to be done once per system. The scripts may be run on
Linux (tested on Ubuntu Focal) or Mac (at your own risk, not tested recently).

### Install required packages

Mac:

    brew install gnupg brew install gnupg@1.4
    brew install aptly
    brew install docker
    [brew install docker-credential-helper]
    brew install awscli

Linux:

    apt install gnupg1
    apt install aptly
    apt install docker
    apt install awscli


**Warning:** When running this process on on Focal, `aptly` will need to be
updated to 1.5.0 to handle the zst compression used in Jammy packages
(https://github.com/aptly-dev/aptly/pull/1050). To update, follow the
instructions here: https://www.aptly.info/download.

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


### Get the push_release script

Clone the drake repository:

    git clone https://github.com/RobotLocomotion/drake.git

In `tools/release_engineering/dev/push_release`, replace the placeholder
`gpg_key` with the value from the AWS Secrets Manager.


## Run script for docker and tar

Once your machine is set-up, run the `push_release` script as described below:

    push_release <version> <date>


The release creator will provide the version and date. Throughout this process,
don’t use `v` on the version string. For example:

    ./push_release 0.32.0 20210714

**Note:** If running the script while ssh’ed into Mac, you may need to
run:

    security unlock keychain

If prompted, use the docker ID and password from AWS Secrets Manager.

When prompted, enter the passphrase from AWS Secrets Manager.

Verify that:

1. [s3://drake-packages/drake/release](https://s3.console.aws.amazon.com/s3/buckets/drake-packages?region=us-east-1&prefix=drake/release/&showversions=false)
contains a set of `drake-<version>-[...].tar.gz[...]` files for each supported
configuration (e.g. Focal, Jammy and Mac).

1. [Dockerhub](https://hub.docker.com/r/robotlocomotion/drake/tags?ordering=last_updated&page=1)
has `<version>` tags for each supported configuration (e.g. Focal and Jammy).

## Copy apt packages to S3

1. The deb files are created by Jenkins jobs run by the release manager and are
uploaded to GitHub as release artifacts. Download the .deb files for Focal and
Jammy from: `https://github.com/RobotLocomotion/drake/releases/tag/v<version>`


1. Copy the .deb files to S3:

        aws s3 cp drake-dev_<version>-1_amd64-<focal|jammy>.deb \
        s3://drake-packages/drake/release/<focal|jammy>/drake-dev_<version>-1_amd64.deb

**Note:** This requires the aws client, appropriate permissions, etc. If it’s easier, use the web interface to upload the deb files.

**Note:** The configuration name is not include in the deb filename uploaded
to S3

Verify that:

1. The `*.deb` files are in AWS
`S3/Buckets/drake-packages/drake/release/<configuration>/drake-dev_<version>-1_amd64.deb` for each supported configuration (e.g. focal and jammy)


## Run script to push apt release

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


### Run the script

    ./push_release <version> <date> --apt --no-docker --no-tar

When prompted, enter the passphrase from AWS Secrets Manager (the script will
prompt for the same passphrase multiple times).
