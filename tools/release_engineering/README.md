This directory contains tools that are only intended for use by Drake
maintainers, not Drake users. These tools are only supported on our primary
developer platform, which is currently Ubuntu Noble 24.04.

Most tools are used as part of the https://drake.mit.edu/release_playbook.html
instructions, or for other maintenance activities.

These tools require additional dependencies from the Ubuntu host. Before using
any of these tools, you must first install the prereqs:

```bash
setup/install_prereqs --developer
```

Additionally, `awscli` is a required dependency, but is unavailable via `apt`
on Ubuntu Noble. Follow these instructions to install it:
https://docs.aws.amazon.com/cli/latest/userguide/getting-started-install.html

# apt Repository Mirroring

This describes the process of running the `push_apt` script to update
Drake's apt repository with artifacts from a new release, or create a new
repository if this is the first release with support for a new distribution.

## Configure aptly

1. Create the `.aptly.conf` config file in your home directory from the
   following template:

   ```json
   {
      "rootDir": <dir>,
      "downloadConcurrency": 4,
      "downloadSpeedLimit": 0,
      "architectures": ["amd64"],
      "dependencyFollowSuggests": false,
      "dependencyFollowRecommends": false,
      "dependencyFollowAllVariants": false,
      "dependencyFollowSource": false,
      "dependencyVerboseResolve": false,
      "gpgDisableSign": false,
      "gpgDisableVerify": false,
      "gpgProvider": "internal",
      "downloadSourcePackages": false,
      "skipLegacyPool": true,
      "ppaDistributorID": "ubuntu",
      "ppaCodename": "",
      "skipContentsPublishing": false,
      "FileSystemPublishEndpoints": {},
      "SwiftPublishEndpoints": {}
   }
   ```

   Additionally, there should be a key called `"S3PublishEndpoints"` whose
   value is an object containing one key-value pair for each currently
   supported Ubuntu distribution. For example, for Ubuntu Noble,

   ```json
   "S3PublishEndpoints": {
      "drake-apt.csail.mit.edu/noble": {
         "region": "us-east-1",
         "bucket": "drake-apt",
         "endpoint": "",
         "awsAccessKeyID": <key-id>,
         "awsSecretAccessKey": <access-key>,
         "prefix": "noble",
         "acl": "public-read",
         "storageClass": "",
         "encryptionMethod": "",
         "plusWorkaround": false,
         "disableMultiDel": false,
         "forceSigV2": false,
         "debug": false
      }
   }
   ```

   See the section below for more details when adding a new distribution to
   this list.

2. On AWS, in IAM / Users / \<username\> / Security Credentials, select
   "Create Access Key."
3. Replace all instances of "awsAccessKeyID" and "awsSecretAccessKey" in
   `.aptly.conf` with the created values.
4. Set `rootDir` in `.aptly.conf` to your local home directory (e.g.
   `/Users/<username>/.aptly`).

## Import the gpg key

5. Download the public key from S3.
6. Using the passphrase from the AWS Secrets Manager, run:

   ```bash
   gpg --import <key.asc>
   ```

## Run the script

7. Run the `push_apt` script as described below:

   ```bash
   cd tools/release_engineering
    ./push_apt <version>
   ```

   The release creator will provide the version. Don’t use `v` on the version
   string; for example:

   ```bash
   ./push_apt 1.0.0
   ```

## Verification

Verify that you can install drake via apt; see https://drake.mit.edu/apt.html
for instructions.

## Adding a new distribution

The `push_apt` script should be configured to perform most of the necessary
steps when adding a new distribution.

Before running the script for the first time for a new distribution, edit the
`.aptly.conf` file from above to add a new object under "S3PublishEndpoints"
corresponding to the new distribution.

Since Drake's `apt` site is deprecated and maintained for compatibility only on
Ubuntu Noble, adding a new distribution should not be needed.
