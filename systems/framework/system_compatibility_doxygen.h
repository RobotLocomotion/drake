/** @file
 Doxygen-only documentation for @ref system_compatibility.  */

//------------------------------------------------------------------------------
/** @defgroup system_compatibility System Compatibility
    @ingroup technical_notes

System compatibility refers to the correspondence between a System and data
structures used to hold the results of computation. Examples include Context,
State, Parameters, etc. To avoid hard-to-debug errors when the wrong object is
used with a System, public methods of System and System-derived classes check
compatibility with objects by using unique system IDs. These IDs are applied to
objects when System-provided methods are used to construct them:
AllocateContext(), AllocateOutput(), etc.

In most cases users should be able to ignore this compatibility checking
mechanism; setting and checking of IDs should happen naturally in most
cases. However, when copying data values between similar systems, some case
must be taken. for example, a Clone() of a checked object will not work with a
system different from its source, but constructing a destination object using
methods of the destination System and using SetFrom() or lower level value
accessors will work.

<h2>Details</h2>

A system ID is required for checking the correspondence between a
system-specific data structure like a Context or DiscreteValues and the
(unique) System which can accept it as a value. A data structure participates
in the system ID hinting family by implementing the following concept:
@code
    internal::SystemId get_system_id() const;
    void set_system_id(internal::SystemId id);
@endcode
get_system_id of an object whose set_system_id has not been called will return
an invalid (default-constructed, zero-valued) ID.

A System method participates by calling ValidateCreatedForThisSystem(object)
on an object that implements the concept.

An invalid system ID represents the absence of information about the associated
System; an object carrying an invalid system ID cannot be used in any
participating System method, under penalty of std::exception.

Likewise passing an object with valid system ID into a method of a System with
different ID will result in a std::exception.

An ID-bearing structure may contain ID-bearing substructures which have
different (or invalid) IDs from their parent. Using such a structure may, but
is not guaranteed to, result in a std::exception, depending on the semantics of
the method called.

Where an ID-bearing object implements Clone(), it MUST clone the system ID and,
recursively, those of its members, whether valid or invalid.

Where an ID-bearing object implements the potentially-scalar-converting
SetFrom(), if the scalar types differ, it MUST NOT set the system IDs, nor may
it set the system IDs of its copied members (because being of the wrong type it
is no longer valid data for the original System). This prohibition does not
apply when the scalar types are the same.

Data types that currently implement the above-described scheme include Context,
ContinuousState, DiscreteValues, CompositeEventCollection, Parameters, State,
and SystemOutput.

The SystemConstraint class uses a custom mechanism to extend checking of
Context compatibility to its public methods.

<h2>Effects on Public API</h2>

This change does not declare a deprecation because virtually all of the
affected use cases were invalid to begin with and would have resulted in errors
later in the affected code. The exception is hand-constructed objects (i.e.,
objects with public constructors that were not constructed through methods of
System); most such objects have not worked reliably in the past anyway.

*/
