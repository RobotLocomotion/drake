Within Drake, QDLDL is used by OSQP and SCS.

When changing the release of either QDLDL or OSQP used by Drake, ideally try to
keep Drake's pin of QDLDL aligned with what Drake's pin of OSQP desires.  (SCS
tends to use stale copies of QDLDL and we can't really be expected to stick
with those old versions just for the sake of SCS.)

For example, if Drake pinned OSQP to commit = "v1.0.0" then look at OSQP
1.0.0's
[qdldl.cmake](https://github.com/osqp/osqp/blob/v1.0.0/algebra/_common/lin_sys/qdldl/qdldl.cmake).
recipe, where you should encounter a stanza that resembles the following...

```cmake
FetchContent_Declare(
  qdldl
  GIT_REPOSITORY https://github.com/osqp/qdldl.git
  GIT_TAG v0.1.8
  )
```

...which shows that OSQP 1.0.0 fetches v0.1.8 for QDLDL. So, Drake should
ideally pin QDLDL commit = "v0.1.8".

Given the potential interactions of QDLDL, OSQP, and SCS it may not be possible
to keep all of the versions aligned perfectly.  When in doubt, prefer to use
the newest tagged release of all three, patching them as necessary to get
everything working.
