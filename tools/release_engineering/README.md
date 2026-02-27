This directory contains tools that are only intended for use by Drake
maintainers, not Drake users. These tools are only supported on our primary
developer platform, which is currently Ubuntu Noble 24.04.

Most tools are used as part of the https://drake.mit.edu/release_playbook.html
instructions, or for other maintenance activities.

These tools require additional dependencies from the Ubuntu host. Before using
any of these tools, you must first install the prereqs:

```
$ sudo ./setup/ubuntu/install_prereqs.sh --with-maintainer-only
```

Additionally, `awscli` is a required dependency, but is unavailable via `apt`
on Ubuntu Noble. Follow these instructions to install it:
https://docs.aws.amazon.com/cli/latest/userguide/getting-started-install.html

# apt Repository Mirroring

This describes the process of running the `push_release` script to update
Drake's apt repository with artifacts from a new release, or create a new
repository if this is the first release with support for a new distribution.

## Configure aptly

1. Download `.aptly.conf` from S3 to your home directory.
2. On AWS, in IAM / Users / \<username\> / Security Credentials, select
   `Create Access Key`.
3. Replace all (3) instances of "awsAccessKeyID" and "awsSecretAccessKey" in
   `.aptly.conf` with the created values.
4. Set `rootDir` in `.aptly.conf` to your local home directory (e.g.
   `/Users/<username>/.aptly`).

## Import the gpg key

5. Download the public key from S3.
6. Using the passphrase from the AWS Secrets Manager, run:

     gpg --import <key.asc>

## Run the script

7. Run the `push_release` script as described below:

     cd tools/release_engineering
     ./push_release <version>

  The release creator will provide the version. Don’t use `v` on the version
  string; for example:

     ./push_release 1.0.0

## Verification

Verify that you can install drake via apt; see https://drake.mit.edu/apt.html
for instructions.
