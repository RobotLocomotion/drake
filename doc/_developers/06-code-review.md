---
title: Code Review
slug: code-review
links: 
    - title: Review Process 
      slug: review-process
    - title: Review Processing Tools
      slug: review-processing-tools
---

#### {{page.links[0].title}} {#{{page.links[0].slug}}}

For complex changes, especially those that will span multiple PRs, please
open a GitHub issue and solicit design feedback before you invest a lot of
time in code.

Before your submit a pull request, please consult the
[Code Review Checklist](/code_review_checklist.html),
where a list of the most frequent problems are collected.

Be prepared to engage in active code review on your pull requests.  The Drake
code review process has two phases: feature review and platform review. You
are responsible for finding reviewers, and for providing them the information
they need to review your change effectively. If a reviewer asks you for more
information, that is a sign you should add more documentation to your PR.

A PR generally *should not* include more than 750 added or changed lines (the
green ``+###`` number as reported by github), and *must not* include more than
1500 lines, with the following exemptions:

* Data files do not count towards the line limit.
* Machine-generated changes do not count towards the line limit.
* Files in [Special Directories](/directory_structure.html) do not count towards the line limit.
* This rule may be overridden by agreement of at least two platform reviewers (listed below).

The utility ``tools/prstat`` will report the total added or changed
lines, excluding files that are easily identified to meet the exemptions above.

We use [https://reviewable.io](https://reviewable.io/) for code reviews. You can sign in for free with
your GitHub identity. Before your first code review, please take a look at
[Tips for Participating In Drake Code Reviews using reviewable.io](/reviewable.html).

If you have an expected pace for your review, please add a ``priority`` label
(which have different meanings for PRs and
[for issues](/issues.html#priority)). The response expectations, for both the author and reviewer:

* `priority: emergency` - Very quick response time, nominally reserved for build cop.
* `priority: high` - Some urgency, quick response time.
* `priority: medium` - (Default) Normal response time.
* `priority: low` - No rush.
* `priority: backlog` - Give priority to all other PRs on your plate.

If you are an external contributor, you will need to request that a priority be
added by a Drake Developer.

**Feature Review.** After creating your pull request, assign it to someone
else on your team for feature review. Choose the person most familiar
with the context of your pull request. This reviewer is responsible for
protecting your team by inspecting for bugs, for test coverage, and for
alignment with the team's goals. During this review, you and your reviewer
should also strive to minimize the number of changes that will be necessary
in platform review.

**Platform Review.** After your feature reviewer has signed off on your change,
reassign it to a Drake owner for platform review. The owner will inspect for
architectural compatibility, stability, performance, test coverage, and style.

The following GitHub users are Drake owners. If possible, seek platform review
from an owner who has previously reviewed related changes. Shared context will
make the review faster.

* @EricCousineau-TRI (Toyota Research Institute)
* @ggould-tri (Toyota Research Institute)
* @jwnimmer-tri (Toyota Research Institute)
* @rpoyner-tri (Toyota Research Institute)
* @sammy-tri (Toyota Research Institute)
* @SeanCurtis-TRI (Toyota Research Institute)
* @sherm1 (Toyota Research Institute)
* @soonho-tri (Toyota Research Institute)
* @RussTedrake (MIT / Toyota Research Institute)

**Merge.** Once the PR is fully reviewed and passes CI, the assigned platform
reviewer will merge it to master.  If time is of the essence, you may post a
reminder to the PR to get the reviewer's attention.  If the PR should not be
merged yet, or if you prefer to merge it yourself, apply the label "status:
do not merge" to disable the merge.

If you are a frequent contributor who has been granted write access to
RobotLocomotion/drake, a green "Merge Pull Request" button will appear when
your change is fully reviewed and passes CI. You may click it to merge your PR.
Choose the "Squash and merge option" unless otherwise instructed (see
[Curated Commits](/reviewable.html#curated-commits)).

**After Merge.** If your PR breaks continuous integration, the [Build Cop](/buildcop.html)
will contact you to work out a resolution.


#### {{page.links[1].title}} {#{{page.links[1].slug}}}

[Tips for Participating In Drake Code Reviews using reviewable.io](/reviewable.html)  