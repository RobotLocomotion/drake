gym
===

This imports the OpenAI Gym RL tool into our build.


Note on Upgrading
=================

`stable_baselines3` lags `gym` by more than `gym`'s deprecation window and
does not document its compatibility limit correctly (claims to support `gym`
<0.24 but in fact uses classes removed in 0.22, for example).

Because of this, it is unlikely that upgrading this package beyond 0.21 will
succeed, or is even desirable.

 * Check https://github.com/DLR-RM/stable-baselines3/pull/780 for status.
 * `stable_baselines3` allegedly will fix all of this version shear when gym
   hits 1.0.
 * `gym` will allegedly remove all the deprecated APIs that SB3 uses in 0.26.

A full upgrade of `gym` to 1.0 and correspondingly of `stable_baselines3` to
HEAD will be necessary in the future, but will require substantial changes to
the `stable_baselines3_internal` stub and to any corresponding unit tests.
