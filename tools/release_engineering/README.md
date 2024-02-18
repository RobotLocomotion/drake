This directory contains tools that are only intended for use by Drake
maintainers, not Drake users. These tools are only supported on our primary
developer platform, which is currently Ubuntu Jammy 22.04.

Most tools are used as part of the https://drake.mit.edu/release_playbook.html
instructions, or for other maintenance activities.

These tools require additional dependencies from the Ubuntu host. Before using
any of these tools, you must first install the prereqs:

```
$ sudo ./setup/ubuntu/install_prereqs.sh --with-maintainer-only
```
