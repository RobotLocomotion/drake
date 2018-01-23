#pragma once

namespace drake {

/// Documents the argument(s) as unused, placating GCC's -Wunused-parameter
/// warning.  This can be called within function bodies to mark that certain
/// parameters are unused.
///
/// When possible, removing the unused parameter is better than placating the
/// warning.  However, in some cases the parameter is part of a virtual API or
/// template concept that is used elsewhere, so we can't remove it.  In those
/// cases, this function might be an appropriate work-around.
///
/// Here's rough advice on how to fix Wunused-parameter warnings:
///
/// (1) If the parameter can be removed entirely, prefer that as the first
///     choice.  (This may not be possible if, e.g., a method must match some
///     virtual API or template concept.)
///
/// (2) Unless the parameter name has acute value, prefer to omit the name of
///     the parameter, leaving only the type, e.g.
/// @code
/// void Print(const State& state) override { /* No state to print. */ }
/// @endcode
///     changes to
/// @code
/// void Print(const State&) override { /* No state to print. */}
/// @endcode
///     This no longer triggers the warning and further makes it clear that a
///     parameter required by the API is definitively unused in the function.
///
///     This is an especially good solution in the context of method
///     definitions (vs declarations); the parameter name used in a definition
///     is entirely irrelevant to Doxygen and most readers.
///
/// (3) When leaving the parameter name intact has acute value, it is
///     acceptable to keep the name and mark it `unused`.  For example, when
///     the name appears as part of a virtual method's base class declaration,
///     the name is used by Doxygen to document the method, e.g.,
/// @code
/// /** Sets the default State of a System.  This default implementation is to
///     set all zeros.  Subclasses may override to use non-zero defaults.  The
///     custom defaults may be based on the given @p context, when relevant.  */
/// virtual void SetDefault(const Context<T>& context, State<T>* state) const {
///   unused(context);
///   state->SetZero();
/// }
/// @endcode
///
template <typename ... Args>
void unused(const Args& ...) {}

}  // namespace drake
