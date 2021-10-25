The file SuiteSparse_config.mk in this directory is copied from

  https://github.com/Kitware/fletch/blob/master/Patches/SuiteSparse/SuiteSparse_config.mk

If we ever need to upgrade SuiteSparse, we might need to refresh our copy
to track any changes in SuiteSparse's configuration.

Alternatively, we might just be able to make a smaller patch file against
SuiteSparse to obtain the configuration that we want, rather than replace
this file wholesale.
