#pragma once

// GENERATED FILE DO NOT EDIT
// This file contains docstrings for the Python bindings that were
// automatically extracted by mkdoc.py.

#include <array>
#include <utility>

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

// #include "drake/systems/optimization/system_constraint_adapter.h"
// #include "drake/systems/optimization/system_constraint_wrapper.h"

// Symbol: pydrake_doc_systems_optimization
constexpr struct /* pydrake_doc_systems_optimization */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::systems
    struct /* systems */ {
      // Symbol: drake::systems::SystemConstraintAdapter
      struct /* SystemConstraintAdapter */ {
        // Source: drake/systems/optimization/system_constraint_adapter.h
        const char* doc =
R"""(This class is a factory class to generate SystemConstraintWrapper.
Namely this class helps to convert a SystemConstraint to a
solvers∷Constraint. Internally this class will convert a
System<double> to System<AutoDiffXd> (and System<symbolic∷Expression>
if possible), and store these systems (of different scalar types)
inside this class. Using this class with a system that cannot be
converted to System<AutoDiffXd> will cause a runtime error.)""";
        // Symbol: drake::systems::SystemConstraintAdapter::Create
        struct /* Create */ {
          // Source: drake/systems/optimization/system_constraint_adapter.h
          const char* doc =
R"""(This method creates a solvers∷Constraint from a SystemConstraint. The
newly created constraint represents lower <=
system_constraint.Calc(UpdateContextFromDecisionVariablesGeneric(x))
<= upper, where lower and upper are obtained from
SystemConstraint∷lower_bound() and SystemConstraint∷upper_bound().

Parameter ``index``:
    The index of the constraint stored inside ``system`` in the class
    constructor.

Parameter ``context``:
    SystemConstraint∷Calc function requires a context as the input. On
    the other hand, the generated constraint might be imposed on a
    partial subset of variables (state, time, input and parameters)
    inside the context. Hence we use
    ``UpdateContextFromDecisionVariablesGeneric`` to select the
    decision variables inside ``context``. The unselected variables
    will remain to its values stored in ``context``.)""";
        } Create;
        // Symbol: drake::systems::SystemConstraintAdapter::MaybeCreateConstraintSymbolically
        struct /* MaybeCreateConstraintSymbolically */ {
          // Source: drake/systems/optimization/system_constraint_adapter.h
          const char* doc =
R"""(Given a SystemConstraint and the Context to evaluate this
SystemConstraint, parse the constraint in the symbolic forms.
Currently we support parsing the following forms:

1. bounding box ( lower <= x <= upper ) 2. linear equality ( aᵀx = b )
3. linear inequality ( lower <= aᵀx <= upper )

If the SystemConstraint cannot be parsed to the forms above, then
returns nullopt; otherwise returns a vector containing the parsed
constraint.

Parameter ``index``:
    The index of the constraint in the System object.

Parameter ``context``:
    The context used to evaluate the SystemConstraint.

Returns ``constraints``:
    If the SystemConstraint can be parsed to the constraint in the
    above forms, then constraints.value()[i] is the i'th row of the
    SystemConstraint evaluation result; if the SystemConstraint cannot
    be parsed in the above forms (either due to the System is not
    instantiated with symbolic∷Expression, or the constraint is not
    linear), then constraints.has_value() = false.)""";
        } MaybeCreateConstraintSymbolically;
        // Symbol: drake::systems::SystemConstraintAdapter::MaybeCreateGenericConstraintSymbolically
        struct /* MaybeCreateGenericConstraintSymbolically */ {
          // Source: drake/systems/optimization/system_constraint_adapter.h
          const char* doc =
R"""(Given a SystemConstraint and the Context to evaluate this
SystemConstraint, parses the constraint to a generic nonlinear
constraint lower <= SystemConstraint.Calc(context) <= upper. If the
SystemConstraint cannot be parsed to the form above, then returns
empty; otherwise returns a parsed constraint, together with the bound
variables. We currently only support systems without abstract state or
abstract parameters.

Parameter ``index``:
    The index of the constraint in the System object.

Parameter ``context``:
    The context used to evaluate the SystemConstraint.

Note:
    each expression in ``context`` (like state, parameter, etc) should
    be either a single symbolic variable, or a constant. Currently we
    do not support complicated symbolic expressions.

Returns ``constraint``:
    A generic nonlinear constraint parsed from SystemConstraint. If
    the SystemConstraint cannot be parsed to the generic constraint
    using ``context`` instantiated with symbolic∷Expression, then
    constraint.has_value() = false.

Raises:
    RuntimeError if the system contains abstract state or abstract
    parameters.)""";
        } MaybeCreateGenericConstraintSymbolically;
        // Symbol: drake::systems::SystemConstraintAdapter::SystemConstraintAdapter
        struct /* ctor */ {
          // Source: drake/systems/optimization/system_constraint_adapter.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::systems::SystemConstraintAdapter::system_autodiff
        struct /* system_autodiff */ {
          // Source: drake/systems/optimization/system_constraint_adapter.h
          const char* doc =
R"""(Getters for the system instantiated with AutoDiffXd.)""";
        } system_autodiff;
        // Symbol: drake::systems::SystemConstraintAdapter::system_symbolic
        struct /* system_symbolic */ {
          // Source: drake/systems/optimization/system_constraint_adapter.h
          const char* doc =
R"""(Returns the symbolic system. Throws a runtime error if the system
cannot be instantiated with symbolic∷Expression.)""";
        } system_symbolic;
      } SystemConstraintAdapter;
      // Symbol: drake::systems::SystemConstraintWrapper
      struct /* SystemConstraintWrapper */ {
        // Source: drake/systems/optimization/system_constraint_wrapper.h
        const char* doc =
R"""(This wrapper class wraps a SystemConstraint object to the format of
solvers∷Constraint. The constraint is lower <=
SystemConstraint.Calc(UpdateContextFromDecisionVaraibles(x)) <= upper
where lower/upper are the lower and upper bounds of the
SystemConstraint object. When the lower and upper are equal, this
represents an equality constraint.)""";
        // Symbol: drake::systems::SystemConstraintWrapper::SystemConstraintWrapper
        struct /* ctor */ {
          // Source: drake/systems/optimization/system_constraint_wrapper.h
          const char* doc =
R"""(Wraps a single SystemConstraint of the given system into a
solvers∷Constraint. Note that this constraint doesn't require the
System to support symbolic expressions. The wrapped solvers∷Constraint
is a generic nonlinear constraint.

Parameter ``system_double``:
    The System whose SystemConstraint is converted to
    solvers∷Constraint.

Parameter ``system_autodiff``:
    This system should be converted from system_double by converting
    the scalar type. If this is pointer is null, then the AutoDiffXd
    version of the system will be created internally inside this
    wrapper class.

Parameter ``index``:
    The index of the SystemConstraint in ``system_double`` (and also
    ``system_autodiff)``.

Parameter ``context``:
    The value stored in this context will be used in
    SystemConstraintWrapper∷Eval. If ``updater_double`` (and
    ``updater_autodiff)`` doesn't update everything in the context
    (such as state, input, params, etc), then the un-updated part in
    the context will keep its value to those stored in ``context``.

Parameter ``updater_double``:
    Maps x in SystemConstraintWrapper∷Eval(x, &y) to a context. The
    context is then used in SystemConstraint.Calc(context).

Parameter ``updater_autodiff``:
    Same as ``updater_double``, but works for autodiff type.

Parameter ``x_size``:
    The number of variables bound with this constraint. Namely, the
    size of x in SystemConstraintWrapper.Eval(x, &y).)""";
        } ctor;
        // Symbol: drake::systems::SystemConstraintWrapper::constraint_index
        struct /* constraint_index */ {
          // Source: drake/systems/optimization/system_constraint_wrapper.h
          const char* doc =
R"""(Getter for the index of the constraint in the system.)""";
        } constraint_index;
        // Symbol: drake::systems::SystemConstraintWrapper::system_autodiff
        struct /* system_autodiff */ {
          // Source: drake/systems/optimization/system_constraint_wrapper.h
          const char* doc =
R"""(Gets the AutoDiffXd type System stored in this constraint.)""";
        } system_autodiff;
      } SystemConstraintWrapper;
    } systems;
  } drake;
} pydrake_doc_systems_optimization;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
