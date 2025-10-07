# Jenkins Pipelines

This directory contains the Jenkinsfiles used for Drake's continuous
integration, including flavors for experimental, continuous, nightly, and
staging. See https://drake.mit.edu/jenkins.html for details on how to
request builds on pull requests.

## Layout

The pipelines are:

* [Jenkinsfile-experimental](./Jenkinsfile-experimental): used for on-demand
and pre-merge (i.e., pull request) jobs
* [Jenkinsfile-external-examples](./Jenkinsfile-external-examples): used for
the experimental job(s) which build
[drake-external-examples](https://github.com/RobotLocomotion/drake-external-examples)
* [Jenkinsfile-production](./Jenkinsfile-production): used for continuous and
nightly jobs
* [Jenkinsfile-staging](./Jenkinsfile-staging): used for staging (release) jobs

Corresponding templates for each of these files are contained in
[templates](./templates/). Functions shared across the pipelines are stored
in [utils.groovy](./utils/utils.groovy).

Note that Drake's Jenkins jobs are configured to obtain the Jenkinsfiles from
their current locations, so moving these files requires a simultaneous job
reconfiguration. Contact `@BetsyMcPhail` for details on how to proceed with
these changes.

## Making Changes

The Jenkinsfiles adjacent to this file should not be modified by hand. They
are generated automatically from the [template files](templates/) using
[utils/generate-Jenkinsfiles.cmake](utils/generate-Jenkinsfiles.cmake). In general,
use the following procedure to update the Jenkinsfiles:

1. Make the necessary changes. This could be to either the
[template file(s)](templates/), or the [shared utilities](utils/utils.groovy).
2. Run [utils/generate-Jenkinsfiles.cmake](utils/generate-Jenkinsfiles.cmake)
as directed in the script to regenerate the Jenkinsfiles. This will insert
the content of utilities into the Jenkinsfiles, so that changes to
these file(s) does not require a rebase of every open pull request in order to
obtain the updates.
3. Push your changes to the remote in order to test them through Jenkins.

## Testing Changes

Drake's experimental Jenkins jobs are configured to build and test a merge of
the master branch onto a given feature branch with an open pull request. This
merge includes the Jenkinsfile itself, meaning that changes to the experimental
pipeline can be tested directly using the existing CI on pull requests.

The Jenkins jobs for production (i.e., continuous and nightly) and staging
should never be run on pull request, so there are a few alternative options
for testing changes to these pipelines depending on the nature of the change.
Contact `@BetsyMcPhail` for how to proceed. (If the changes are concurrently
being made to the experimental pipelines, then they can be tested there with a
pull request and replicated in the production pipelines.)
