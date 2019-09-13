Within Drake, QDLDL is used by OSQP.

When changing the release of either QDLDL or OSQP used by Drake, ideally try to
keep Drake's pin of QDLDL aligned with what Drake's pin of OSQP desires.

For example, if Drake pinned OSQP to commit = "v0.4.1" then look at
https://github.com/oxfordcontrol/osqp/tree/v0.4.1/lin_sys/direct/qdldl
which shows that OSQP 0.4.1 prefers a pin of 7ab0fca for QDLDL.

Looking at https://github.com/oxfordcontrol/qdldl/commit/7ab0fca we
see that it was approximately the 0.1.3 release.  (The actual v0.1.3
release tag on QDLDL is on the merge commit of 7ab0fca onto master.)
So, Drake should ideally pin QDLDL commit = "v0.1.3".
