# Valkyrie

This experimental directory does not adhere to the development practices that
apply elsewhere in Drake. In particular, for PRs that affect only this
directory, one feature review is sufficient; platform review is not required.

If build or test targets in this directory break, and the PR author or oncall
buildcop cannot trivially resolve them, a GitHub issue will be assigned to
the Valkyrie team. If the issue is not resolved within 24 hours, the author
or buildcop may disable the offending targets.
