This branch contains files listing experimental CI jobs that correspond to
production jobs, in the form of requests to execute those jobs on a pull
request. "Provisioned" jobs use an already-set-up build environment, while
"unprovisioned" jobs run from a clean environment. 

In particular, there are currently two files:

* `request-jobs-experimental.txt`: Contains a one-to-one mapping of all
production jobs to their experimental equivalents, separated by provisioned
and unprovisioned.
* `request-jobs-unprovisioned.txt`: Contains a mapping of all production jobs
to their experimental unprovisioned equivalents (i.e., including both
already-unprovisioned jobs and provisioned-as-unprovisioned jobs). This file
is especially useful for testing changes to installation prerequisites, before
those changes are built into the provisioned CI images.
