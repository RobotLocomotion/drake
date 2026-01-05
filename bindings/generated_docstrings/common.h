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

// #include "drake/common/autodiff.h"
// #include "drake/common/autodiff_config.h"
// #include "drake/common/cond.h"
// #include "drake/common/constants.h"
// #include "drake/common/copyable_unique_ptr.h"
// #include "drake/common/cpu_capabilities.h"
// #include "drake/common/default_scalars.h"
// #include "drake/common/diagnostic_policy.h"
// #include "drake/common/double_overloads.h"
// #include "drake/common/drake_assert.h"
// #include "drake/common/drake_assertion_error.h"
// #include "drake/common/drake_bool.h"
// #include "drake/common/drake_copyable.h"
// #include "drake/common/drake_deprecated.h"
// #include "drake/common/drake_export.h"
// #include "drake/common/drake_marker.h"
// #include "drake/common/drake_path.h"
// #include "drake/common/drake_throw.h"
// #include "drake/common/dummy_value.h"
// #include "drake/common/eigen_types.h"
// #include "drake/common/extract_double.h"
// #include "drake/common/file_source.h"
// #include "drake/common/find_loaded_library.h"
// #include "drake/common/find_resource.h"
// #include "drake/common/find_runfiles.h"
// #include "drake/common/fmt.h"
// #include "drake/common/fmt_eigen.h"
// #include "drake/common/fmt_ostream.h"
// #include "drake/common/hash.h"
// #include "drake/common/identifier.h"
// #include "drake/common/is_approx_equal_abstol.h"
// #include "drake/common/is_cloneable.h"
// #include "drake/common/is_less_than_comparable.h"
// #include "drake/common/memory_file.h"
// #include "drake/common/name_value.h"
// #include "drake/common/network_policy.h"
// #include "drake/common/never_destroyed.h"
// #include "drake/common/nice_type_name.h"
// #include "drake/common/parallelism.h"
// #include "drake/common/pointer_cast.h"
// #include "drake/common/polynomial.h"
// #include "drake/common/random.h"
// #include "drake/common/reset_after_move.h"
// #include "drake/common/reset_on_copy.h"
// #include "drake/common/scope_exit.h"
// #include "drake/common/scoped_singleton.h"
// #include "drake/common/sha256.h"
// #include "drake/common/sorted_pair.h"
// #include "drake/common/ssize.h"
// #include "drake/common/string_hash.h"
// #include "drake/common/string_map.h"
// #include "drake/common/string_set.h"
// #include "drake/common/string_unordered_map.h"
// #include "drake/common/string_unordered_set.h"
// #include "drake/common/temp_directory.h"
// #include "drake/common/text_logging.h"
// #include "drake/common/text_logging_spdlog.h"
// #include "drake/common/timer.h"
// #include "drake/common/type_safe_index.h"
// #include "drake/common/unused.h"
// #include "drake/common/value.h"

// Symbol: pydrake_doc_common
constexpr struct /* pydrake_doc_common */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::AbstractValue
    struct /* AbstractValue */ {
      // Source: drake/common/value.h
      const char* doc =
R"""(A fully type-erased container class. An AbstractValue stores an object
of some type T (where T is declared during at construction time) that
at runtime can be passed between functions without mentioning T. Only
when the stored T must be accessed does the user need to mention T
again.

(Advanced.) Note that AbstractValue's getters and setters method
declare that "If T does not match, a RuntimeError will be thrown with
a helpful error message". The code that implements this check uses
hashing, so in the extraordinarily unlikely case of a 64-bit hash
collision, the error may go undetected in Release builds. (Debug
builds have extra checks that will trigger.)

(Advanced.) Only Value should inherit directly from AbstractValue.
User-defined classes with additional features may inherit from Value.)""";
      // Symbol: drake::AbstractValue::AbstractValue
      struct /* ctor */ {
        // Source: drake/common/value.h
        const char* doc = R"""()""";
      } ctor;
      // Symbol: drake::AbstractValue::Clone
      struct /* Clone */ {
        // Source: drake/common/value.h
        const char* doc = R"""(Returns a copy of this AbstractValue.)""";
      } Clone;
      // Symbol: drake::AbstractValue::GetNiceTypeName
      struct /* GetNiceTypeName */ {
        // Source: drake/common/value.h
        const char* doc =
R"""(Returns a human-readable name for the underlying type T. This may be
slow but is useful for error messages. If T is polymorphic, this
returns the typeid of the most-derived type of the contained object.)""";
      } GetNiceTypeName;
      // Symbol: drake::AbstractValue::Make
      struct /* Make */ {
        // Source: drake/common/value.h
        const char* doc =
R"""(Returns an AbstractValue containing the given ``value``.)""";
      } Make;
      // Symbol: drake::AbstractValue::SetFrom
      struct /* SetFrom */ {
        // Source: drake/common/value.h
        const char* doc =
R"""(Copies the value in ``other`` to this value. If other is not
compatible with this object, a RuntimeError will be thrown with a
helpful error message.)""";
      } SetFrom;
      // Symbol: drake::AbstractValue::Wrap
      struct /* Wrap */ {
        // Source: drake/common/value.h
        const char* doc = R"""()""";
        // Symbol: drake::AbstractValue::Wrap::value
        struct /* value */ {
          // Source: drake/common/value.h
          const char* doc = R"""()""";
        } value;
      } Wrap;
      // Symbol: drake::AbstractValue::get_mutable_value
      struct /* get_mutable_value */ {
        // Source: drake/common/value.h
        const char* doc =
R"""(Returns the value wrapped in this AbstractValue as mutable reference.
The reference remains valid only until this object is set or
destroyed.

Template parameter ``T``:
    The originally declared type of this AbstractValue, e.g., from
    AbstractValue::Make<T>() or Value<T>::Value(). If T does not
    match, a RuntimeError will be thrown with a helpful error message.)""";
      } get_mutable_value;
      // Symbol: drake::AbstractValue::get_value
      struct /* get_value */ {
        // Source: drake/common/value.h
        const char* doc =
R"""(Returns the value wrapped in this AbstractValue as a const reference.
The reference remains valid only until this object is set or
destroyed.

Template parameter ``T``:
    The originally declared type of this AbstractValue, e.g., from
    AbstractValue::Make<T>() or Value<T>::Value(). If T does not
    match, a RuntimeError will be thrown with a helpful error message.)""";
      } get_value;
      // Symbol: drake::AbstractValue::maybe_get_mutable_value
      struct /* maybe_get_mutable_value */ {
        // Source: drake/common/value.h
        const char* doc =
R"""(Returns the mutable value wrapped in this AbstractValue, if T matches
the originally declared type of this AbstractValue.

Template parameter ``T``:
    The originally declared type of this AbstractValue, e.g., from
    AbstractValue::Make<T>() or Value<T>::Value(). If T does not
    match, returns nullptr.)""";
      } maybe_get_mutable_value;
      // Symbol: drake::AbstractValue::maybe_get_value
      struct /* maybe_get_value */ {
        // Source: drake/common/value.h
        const char* doc =
R"""(Returns the value wrapped in this AbstractValue, if T matches the
originally declared type of this AbstractValue.

Template parameter ``T``:
    The originally declared type of this AbstractValue, e.g., from
    AbstractValue::Make<T>() or Value<T>::Value(). If T does not
    match, returns nullptr.)""";
      } maybe_get_value;
      // Symbol: drake::AbstractValue::set_value
      struct /* set_value */ {
        // Source: drake/common/value.h
        const char* doc =
R"""(Sets the value wrapped in this AbstractValue.

Template parameter ``T``:
    The originally declared type of this AbstractValue, e.g., from
    AbstractValue::Make<T>() or Value<T>::Value(). If T does not
    match, a RuntimeError will be thrown with a helpful error message.)""";
      } set_value;
      // Symbol: drake::AbstractValue::static_type_info
      struct /* static_type_info */ {
        // Source: drake/common/value.h
        const char* doc =
R"""(Returns typeid(T) for this Value<T> object. If T is polymorphic, this
does NOT reflect the typeid of the most-derived type of the contained
object; the result is always the base type T.)""";
      } static_type_info;
      // Symbol: drake::AbstractValue::type_info
      struct /* type_info */ {
        // Source: drake/common/value.h
        const char* doc =
R"""(Returns typeid of the contained object of type T. If T is polymorphic,
this returns the typeid of the most-derived type of the contained
object.)""";
      } type_info;
    } AbstractValue;
    // Symbol: drake::AutoDiffVecXd
    struct /* AutoDiffVecXd */ {
      // Source: drake/common/autodiff.h
      const char* doc =
R"""(A dynamic-sized vector of autodiff variables.)""";
    } AutoDiffVecXd;
    // Symbol: drake::AutoDiffXd
    struct /* AutoDiffXd */ {
      // Source: drake/common/autodiff.h
      const char* doc =
R"""(A scalar type that performs automatic differentiation. Always use this
``AutoDiffXd`` alias when referring to the scalar type.)""";
    } AutoDiffXd;
    // Symbol: drake::CalcProbabilityDensity
    struct /* CalcProbabilityDensity */ {
      // Source: drake/common/random.h
      const char* doc =
R"""(Calculates the density (probability density function) of the
multivariate distribution.

Parameter ``distribution``:
    The distribution type.

Parameter ``x``:
    The value of the sampled vector. $Note:

When instantiating this function, the user needs to explicitly pass in
the scalar type, for example CalcProbabilityDensity<double>(...), the
compiler might have problem to deduce the scalar type automatically.)""";
    } CalcProbabilityDensity;
    // Symbol: drake::DefaultHash
    struct /* DefaultHash */ {
      // Source: drake/common/hash.h
      const char* doc =
R"""(The default hashing functor, akin to std::hash.)""";
    } DefaultHash;
    // Symbol: drake::DefaultHasher
    struct /* DefaultHasher */ {
      // Source: drake/common/hash.h
      const char* doc =
R"""(The default HashAlgorithm concept implementation across Drake. This is
guaranteed to have a result_type of size_t to be compatible with
std::hash.)""";
    } DefaultHasher;
    // Symbol: drake::DelegatingHasher
    struct /* DelegatingHasher */ {
      // Source: drake/common/hash.h
      const char* doc =
R"""(An adapter that forwards the HashAlgorithm::operator(data, length)
function concept into a runtime-provided std::function of the same
signature. This is useful for passing a concrete HashAlgorithm
implementation through into non-templated code, such as with an Impl
or Cell pattern.)""";
      // Symbol: drake::DelegatingHasher::DelegatingHasher
      struct /* ctor */ {
        // Source: drake/common/hash.h
        const char* doc =
R"""(Create a delegating hasher that calls the given ``func``.)""";
      } ctor;
      // Symbol: drake::DelegatingHasher::Func
      struct /* Func */ {
        // Source: drake/common/hash.h
        const char* doc =
R"""(A std::function whose signature matches HashAlgorithm::operator().)""";
      } Func;
      // Symbol: drake::DelegatingHasher::operator()
      struct /* operator_call */ {
        // Source: drake/common/hash.h
        const char* doc =
R"""(Append [data, data + length) bytes into the wrapped algorithm.)""";
      } operator_call;
    } DelegatingHasher;
    // Symbol: drake::EigenMapView
    struct /* EigenMapView */ {
      // Source: drake/common/eigen_types.h
      const char* doc =
R"""(Given a random access container (like std::vector, std::array, or C
array), returns an Eigen::Map view into that container. Because this
effectively forms a reference to borrowed memory, you must be be
careful using the return value as anything other than a temporary. The
Map return value currently uses Eigen::Dynamic size at compile time
even when the container is fixed-size (e.g., std::array); if that ever
turns into a performance bottleneck in practice, it would be plausible
to interrogate the size and return a fixed-size Map, instead.)""";
    } EigenMapView;
    // Symbol: drake::EigenPtr
    struct /* EigenPtr */ {
      // Source: drake/common/eigen_types.h
      const char* doc =
R"""(This wrapper class provides a way to write non-template functions
taking raw pointers to Eigen objects as parameters while limiting the
number of copies, similar to ``Eigen::Ref``. Internally, it keeps an
instance of ``Eigen::Ref<T>`` and provides access to it via
``operator*`` and ``operator->``. As with ordinary pointers, these
operators do not perform nullptr checks in Release builds. User-facing
APIs should check for nullptr explicitly.

The primary motivation of this class is to follow <a
href="https://google.github.io/styleguide/cppguide.html#Reference_Arguments">GSG's
"output arguments should be pointers" convention</a> while taking
advantage of using ``Eigen::Ref``. It can also be used to pass
optional Eigen objects since EigenPtr, unlike ``Eigen::Ref``, can be
null.

Some examples:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    // This function is taking an Eigen::Ref of a matrix and modifies it in
    // the body. This violates GSG's pointer convention for output parameters.
    void foo(Eigen::Ref<Eigen::MatrixXd> M) {
       M(0, 0) = 0;
    }
    // At Call-site, we have:
    foo(M);
    foo(M.block(0, 0, 2, 2));
    
    // We can rewrite the above function into the following using EigenPtr.
    void foo(EigenPtr<Eigen::MatrixXd> M) {
       DRAKE_THROW_UNLESS(M != nullptr);  // If you want a Release-build check.
       (*M)(0, 0) = 0;
    }
    // Note that, call sites should be changed to:
    foo(&M);
    
    // We need tmp to avoid taking the address of a temporary object such as the
    // return value of .block().
    auto tmp = M.block(0, 0, 2, 2);
    foo(&tmp);

.. raw:: html

    </details>

Notice that methods taking an EigenPtr can mutate the entries of a
matrix as in method ``foo()`` in the example code above, but cannot
change its size. This is because ``operator*`` and ``operator->``
return an ``Eigen::Ref<T>`` object and only plain matrices/arrays can
be resized and not expressions. This **is** the desired behavior,
since resizing the block of a matrix or even a more general expression
should not be allowed. If you do want to be able to resize a mutable
matrix argument, then you must pass it as a ``Matrix<T>*``, like so:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    void bar(Eigen::MatrixXd* M) {
      DRAKE_THROW_UNLESS(M != nullptr);
      // In this case this method only works with 4x3 matrices.
      if (M->rows() != 4 && M->cols() != 3) {
        M->resize(4, 3);
      }
      (*M)(0, 0) = 0;
    }

.. raw:: html

    </details>

Note:
    This class provides a way to avoid the ``const_cast`` hack
    introduced in <a
    href="https://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html#TopicPlainFunctionsFailing">Eigen's
    documentation</a>.)""";
      // Symbol: drake::EigenPtr::EigenPtr<PlainObjectType>
      struct /* ctor */ {
        // Source: drake/common/eigen_types.h
        const char* doc_0args = R"""()""";
        // Source: drake/common/eigen_types.h
        const char* doc_1args_stdnullptrt = R"""(Overload for ``nullptr``.)""";
        // Source: drake/common/eigen_types.h
        const char* doc_copy =
R"""(Copy constructor results in a *reference* to the given matrix type.)""";
        // Source: drake/common/eigen_types.h
        const char* doc_1args_PlainObjectTypeIn =
R"""(Constructs with a reference to another matrix type. May be
``nullptr``.)""";
        // Source: drake/common/eigen_types.h
        const char* doc_1args_constEigenPtr = R"""(Constructs from another EigenPtr.)""";
      } ctor;
      // Symbol: drake::EigenPtr::RefType
      struct /* RefType */ {
        // Source: drake/common/eigen_types.h
        const char* doc = R"""()""";
      } RefType;
      // Symbol: drake::EigenPtr::operator bool
      struct /* operator_bool */ {
        // Source: drake/common/eigen_types.h
        const char* doc =
R"""(Returns whether or not this contains a valid reference.)""";
      } operator_bool;
      // Symbol: drake::EigenPtr::operator!=
      struct /* operator_ne */ {
        // Source: drake/common/eigen_types.h
        const char* doc = R"""()""";
      } operator_ne;
      // Symbol: drake::EigenPtr::operator*
      struct /* operator_mul */ {
        // Source: drake/common/eigen_types.h
        const char* doc =
R"""(Precondition:
    The pointer is not null (enforced in Debug builds only).)""";
      } operator_mul;
    } EigenPtr;
    // Symbol: drake::ExtractDoubleOrThrow
    struct /* ExtractDoubleOrThrow */ {
      // Source: drake/common/extract_double.h
      const char* doc_1args_scalar =
R"""(Returns ``scalar`` as a double. Never throws.)""";
      // Source: drake/common/extract_double.h
      const char* doc_1args_constEigenMatrixBase =
R"""(Returns ``matrix`` as an Eigen::Matrix<double, ...> with the same size
allocation as ``matrix``. Calls ExtractDoubleOrThrow on each element
of the matrix, and therefore throws if any one of the extractions
fail.)""";
    } ExtractDoubleOrThrow;
    // Symbol: drake::FileSource
    struct /* FileSource */ {
      // Source: drake/common/file_source.h
      const char* doc =
R"""(Represents a file. The file can be on-disk or in-memory.)""";
    } FileSource;
    // Symbol: drake::FindResource
    struct /* FindResource */ {
      // Source: drake/common/find_resource.h
      const char* doc =
R"""((Advanced) Attempts to locate a Drake resource named by the given
``resource_path``. The ``resource_path`` refers to the relative path
within the Drake source repository, prepended with ``drake/``. For
example, to find the source file ``examples/pendulum/Pendulum.urdf``,
the ``resource_path`` would be
``drake/examples/pendulum/Pendulum.urdf``. Paths that do not start
with ``drake/`` will return an error result. The ``resource_path``
must refer to a file (not a directory).

The search scans for the resource in the following resource roots and
in the following order:

1. In the DRAKE_RESOURCE_ROOT environment variable.
2. In the Bazel runfiles for a bazel-bin/pkg/program.
3. In the Drake CMake install directory.

The first resource root from the list that exists is used to find any
and all Drake resources. If the resource root does not contain the
resource, the result is an error even (if a resource root lower on the
list happens to have the resource). If all three roots are
unavailable, then returns an error result.)""";
    } FindResource;
    // Symbol: drake::FindResourceOrThrow
    struct /* FindResourceOrThrow */ {
      // Source: drake/common/find_resource.h
      const char* doc =
R"""((Advanced) Convenient wrapper for querying FindResource(resource_path)
followed by FindResourceResult::get_absolute_path_or_throw().

The primary purpose of this function is for Drake's software internals
to locate Drake resources (e.g., config files) within Drake's build
system. In most cases, end users should not need to use it.

Do NOT use this function to feed into a
drake::multibody::parsing::Parser. Instead, use
parser.AddModelsFromUrl() in coordination with the parser's
PackageMap.)""";
    } FindResourceOrThrow;
    // Symbol: drake::FindResourceResult
    struct /* FindResourceResult */ {
      // Source: drake/common/find_resource.h
      const char* doc =
R"""(Models the outcome of drake::FindResource. After a call to
FindResource, typical calling code would use
get_absolute_path_or_throw(). Alternatively, get_absolute_path() will
return an ``optional<string>``, which can be manually checked to
contain a value before using the path. If the resource was not found,
get_error_message() will contain an error message.

For a given FindResourceResult instance, exactly one of
get_absolute_path() or get_error_message() will contain a value.
(Similarly, exactly one of them will not contain a value.))""";
      // Symbol: drake::FindResourceResult::FindResourceResult
      struct /* ctor */ {
        // Source: drake/common/find_resource.h
        const char* doc = R"""()""";
      } ctor;
      // Symbol: drake::FindResourceResult::get_absolute_path
      struct /* get_absolute_path */ {
        // Source: drake/common/find_resource.h
        const char* doc =
R"""(Returns the absolute path to the resource, iff the resource was found.)""";
      } get_absolute_path;
      // Symbol: drake::FindResourceResult::get_absolute_path_or_throw
      struct /* get_absolute_path_or_throw */ {
        // Source: drake/common/find_resource.h
        const char* doc =
R"""(Either returns the get_absolute_path() iff the resource was found, or
else throws RuntimeError.)""";
      } get_absolute_path_or_throw;
      // Symbol: drake::FindResourceResult::get_error_message
      struct /* get_error_message */ {
        // Source: drake/common/find_resource.h
        const char* doc =
R"""(Returns the error message, iff the resource was not found. The string
will never be empty; only the optional can be empty.)""";
      } get_error_message;
      // Symbol: drake::FindResourceResult::get_resource_path
      struct /* get_resource_path */ {
        // Source: drake/common/find_resource.h
        const char* doc =
R"""(Returns the resource_path asked of FindResource. (This may be empty
only in the make_empty() case.))""";
      } get_resource_path;
      // Symbol: drake::FindResourceResult::make_empty
      struct /* make_empty */ {
        // Source: drake/common/find_resource.h
        const char* doc =
R"""(Returns an empty error result (no requested resource).)""";
      } make_empty;
      // Symbol: drake::FindResourceResult::make_error
      struct /* make_error */ {
        // Source: drake/common/find_resource.h
        const char* doc =
R"""(Returns an error result (the requested resource was NOT found).

Precondition:
    neither string parameter is empty

Parameter ``resource_path``:
    the value passed to FindResource)""";
      } make_error;
      // Symbol: drake::FindResourceResult::make_success
      struct /* make_success */ {
        // Source: drake/common/find_resource.h
        const char* doc =
R"""(Returns a success result (the requested resource was found).

Precondition:
    neither string parameter is empty

Parameter ``resource_path``:
    the value passed to FindResource

Parameter ``absolute_path``:
    an absolute base path that precedes resource_path)""";
      } make_success;
    } FindResourceResult;
    // Symbol: drake::FindRunfile
    struct /* FindRunfile */ {
      // Source: drake/common/find_runfiles.h
      const char* doc =
R"""((Advanced.) Returns the absolute path to the given ``resource_path``
from Bazel runfiles, or else an error message when not found. When
HasRunfiles() is false, returns an error.

Note:
    For Drake Developers, note that in an installed copy of Drake
    (e.g., in a binary release) the HasRunfiles() check will return
    ``False``, so this function will return an error. That means it's
    generally ill-advised to call this function from library code
    inside Drake, or if you do you'll need to have a fallback plan in
    case of an error. Typically our library code should be calling
    FindResource() not FindRunfile(). On the other hand, for our
    private code like unit tests and benchmarks, it's fine to call
    this function and in those programs it's also fine to leave the
    source_repository defaulted (empty).

Parameter ``resource_path``:
    The path to find, formulated as the repository name followed by
    package and filename, e.g., "repository/topdir/subdir/file.ext".
    Drake resource paths look like "drake/common/foo.txt".

Parameter ``source_repository``:
    When looking up a Drake runfile, this value is ignored (and
    therefore may be set to anything). Otherwise, it should be set to
    the value of the preprocessor definition
    ``BAZEL_CURRENT_REPOSITORY`` or else when running with bzlmod
    enabled you risk using an incorrect repo_mapping. That
    preprocessor definition is Bazel magic that takes on *different
    values* depending on which translation unit is being compiled (and
    is undefined unless you add Bazel's runfiles library to your own
    library's ``deps = ...`` directly), so cannot be used as the
    default value here.)""";
    } FindRunfile;
    // Symbol: drake::GetScopedSingleton
    struct /* GetScopedSingleton */ {
      // Source: drake/common/scoped_singleton.h
      const char* doc =
R"""(Provides thread-safe, global-safe access to a shared resource. When
all references are gone, the resource will be freed due to using a
weak_ptr.

Template parameter ``T``:
    Class of the resource. Must be default-constructible.

Template parameter ``Unique``:
    Optional class, meant to make a unique specialization, such that
    you can have multiple singletons of T if necessary.

Note:
    An example application is obtaining license for multiple disjoint
    solver objects, where acquiring a license requires network
    communication, and the solver is under an interface where you
    cannot explicitly pass the license resource to the solver without
    violating encapsulation.)""";
    } GetScopedSingleton;
    // Symbol: drake::HasRunfiles
    struct /* HasRunfiles */ {
      // Source: drake/common/find_runfiles.h
      const char* doc =
R"""((Advanced.) Returns true iff this process has Bazel runfiles
available. For both C++ and Python programs, and no matter what
workspace a program resides in (``@drake`` or otherwise), this will be
true when running ``bazel-bin/pkg/program`` or ``bazel test
//pkg:program`` or ``bazel run //pkg:program``.)""";
    } HasRunfiles;
    // Symbol: drake::Identifier
    struct /* Identifier */ {
      // Source: drake/common/identifier.h
      const char* doc =
R"""(A simple identifier class.

Note:
    This is *purposely* a separate class from TypeSafeIndex. For more
    explanation, see TypeSafeIndexVsIdentifier "this section".

This class serves as an upgrade to the standard practice of passing
`int`s around as unique identifiers (or, as in this case, `int64_t`s).
In the common practice, a method that takes identifiers to different
types of objects would have an interface like:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    void foo(int64_t bar_id, int64_t thing_id);

.. raw:: html

    </details>

It is possible for a programmer to accidentally switch the two ids in
an invocation. This mistake would still be *syntactically* correct; it
will successfully compile but lead to inscrutable run-time errors.
This identifier class provides the same speed and efficiency of
passing `int64_t`s, but enforces unique types and limits the valid
operations, providing compile-time checking. The function would now
look like:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    void foo(BarId bar_id, ThingId thing_id)

.. raw:: html

    </details>

and the compiler will catch instances where the order is reversed.

The identifier is a *stripped down* 64-bit int. Each uniquely declared
identifier type has the following properties:

- The identifier's default constructor produces *invalid* identifiers.
- Valid identifiers must be constructed via the copy constructor or through
Identifier::get_new_id().
- The identifier is immutable.
- The identifier can only be tested for equality/inequality with other
identifiers of the *same* type.
- Identifiers of different types are *not* interconvertible.
- The identifier can be queried for its underlying ``int64_t`` value.
- The identifier can be written to an output stream; its underlying ``int64_t``
value gets written.
- Identifiers are not guaranteed to possess *meaningful* ordering. I.e.,
identifiers for two objects created sequentially may not have sequential
identifier values.
- Identifiers can only be generated from the static method get_new_id().

While there *is* the concept of an invalid identifier, this only
exists to facilitate use with STL containers that require default
constructors. Using an invalid identifier is generally considered to
be an error. In Debug build, attempts to compare, get the value of, or
write an invalid identifier to a stream will throw an exception.

Functions that query for identifiers should not return invalid
identifiers. We prefer the practice of returning
std::optional<Identifier> instead.

It is the designed intent of this class, that ids derived from this
class can be passed and returned by value. (Drake's typical calling
convention requires passing input arguments by const reference, or by
value when moved from. That convention does not apply to this class.)

The following alias will create a unique identifier type for class
``Foo``:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    using FooId = Identifier<class FooTag>;

.. raw:: html

    </details>

**Examples of valid and invalid operations**

The Identifier guarantees that id instances of different types can't
be compared or combined. Efforts to do so will cause a compile-time
failure. For example:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    using AId = Identifier<class ATag>;
    using BId = Identifier<class BTag>;
    AId a1;                              // Compiler error; there is no
    //   default constructor.
    AId a2 = AId::get_new_id();          // Ok.
    AId a3(a2);                          // Ok.
    AId a4 = AId::get_new_id();          // Ok.
    BId b = BId::get_new_id();           // Ok.
    if ( a2 == 1 ) { ... }               // Compiler error.
    if ( a2 == a4 ) { ... }              // Ok, evaluates to false.
    if ( a2 == a3 ) { ... }              // Ok, evaluates to true.
    if ( a2 == b ) { ... }               // Compiler error.
    a4 = a2;                             // Ok.
    a3 = 7;                              // Compiler error.

.. raw:: html

    </details>

**TypeSafeIndex vs Identifier**

In principle, the *identifier* is related to the TypeSafeIndex. In
some sense, both are "type-safe" ``int`s. They differ in their
semantics. We can consider `ints``, indices, and identifiers as a list
of ``int`` types with *decreasing* functionality.

- The int, obviously, has the full range of C++ ints.
- The TypeSafeIndex can be implicitly cast *to* an int, but there are a
limited number of operations *on* the index that produce other instances
of the index (e.g., increment, in-place addition, etc.) They can be
compared with ``int`` and other indices of the same type. This behavior
arises from the intention of having them serve as an *index* in an
ordered set (e.g., ``std::vector``).
- The Identifier is the most restricted. They exist solely to serve as a
unique identifier. They are immutable when created. Very few operations
exist on them (comparison for *equality* with other identifiers of the same
type, hashing, writing to output stream). These *cannot* be used as
indices.

Ultimately, indices *can* serve as identifiers (within the scope of
the object they index into). Although, their mutability could make
this a dangerous practice for a public API. Identifiers are more
general in that they don't reflect an object's position in memory
(hence the inability to transform to or compare with an ``int``). This
decouples details of implementation from the idea of the object.
Combined with its immutability, it would serve well as a element of a
public API.

See also:
    TypeSafeIndex

Template parameter ``Tag``:
    The name of the tag that uniquely segregates one instantiation
    from another.)""";
      // Symbol: drake::Identifier::Identifier<Tag>
      struct /* ctor */ {
        // Source: drake/common/identifier.h
        const char* doc =
R"""(Default constructor; the result is an *invalid* identifier. This only
exists to satisfy demands of working with various container classes.)""";
      } ctor;
      // Symbol: drake::Identifier::get_new_id
      struct /* get_new_id */ {
        // Source: drake/common/identifier.h
        const char* doc =
R"""(Generates a new identifier for this id type. This new identifier will
be different from all previous identifiers created. This method does
*not* make any guarantees about the values of ids from successive
invocations. This method is guaranteed to be thread safe.)""";
      } get_new_id;
      // Symbol: drake::Identifier::get_value
      struct /* get_value */ {
        // Source: drake/common/identifier.h
        const char* doc =
R"""(Extracts the underlying representation from the identifier. This is
considered invalid for invalid ids and is strictly enforced in Debug
builds.)""";
      } get_value;
      // Symbol: drake::Identifier::is_same_as_valid_id
      struct /* is_same_as_valid_id */ {
        // Source: drake/common/identifier.h
        const char* doc =
R"""((Internal use only) Compares this possibly-invalid Identifier with one
that is known to be valid and returns ``False`` if they don't match.
It is an error if ``valid_id`` is not actually valid, and that is
strictly enforced in Debug builds. However, it is not an error if
``this`` id is invalid; that results in a ``False`` return. This
method can be faster than testing separately for validity and
equality.)""";
      } is_same_as_valid_id;
      // Symbol: drake::Identifier::is_valid
      struct /* is_valid */ {
        // Source: drake/common/identifier.h
        const char* doc = R"""(Reports if the id is valid.)""";
      } is_valid;
      // Symbol: drake::Identifier::operator!=
      struct /* operator_ne */ {
        // Source: drake/common/identifier.h
        const char* doc =
R"""(Compares one identifier with another of the same type for inequality.
This is considered invalid for invalid ids and is strictly enforced in
Debug builds.)""";
      } operator_ne;
      // Symbol: drake::Identifier::operator<
      struct /* operator_lt */ {
        // Source: drake/common/identifier.h
        const char* doc =
R"""(Compare two identifiers in order to define a total ordering among
identifiers. This makes identifiers compatible with data structures
which require total ordering (e.g., std::set).)""";
      } operator_lt;
    } Identifier;
    // Symbol: drake::IsApproxEqualAbsTolWithPermutedColumns
    struct /* IsApproxEqualAbsTolWithPermutedColumns */ {
      // Source: drake/common/is_approx_equal_abstol.h
      const char* doc =
R"""(Returns true if and only if a simple greedy search reveals a
permutation of the columns of m2 to make the matrix equal to m1 to
within a certain absolute elementwise ``tolerance``. E.g., there
exists a P such that


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    forall i,j,  |m1 - m2*P|_{i,j} <= tolerance
       where P is a permutation matrix:
          P(i,j)={0,1}, sum_i P(i,j)=1, sum_j P(i,j)=1.

.. raw:: html

    </details>

Note: Returns false for matrices of different sizes. Note: The current
implementation is O(n^2) in the number of columns. Note: In marginal
cases (with similar but not identical columns) this algorithm can fail
to find a permutation P even if it exists because it accepts the first
column match (m1(i),m2(j)) and removes m2(j) from the pool. It is
possible that other columns of m2 would also match m1(i) but that
m2(j) is the only match possible for a later column of m1.)""";
    } IsApproxEqualAbsTolWithPermutedColumns;
    // Symbol: drake::LoadedLibraryPath
    struct /* LoadedLibraryPath */ {
      // Source: drake/common/find_loaded_library.h
      const char* doc =
R"""(This function returns the absolute path of the library with the name
``library_name`` if that library was loaded in the current running
process. Otherwise it returns an empty optional.)""";
    } LoadedLibraryPath;
    // Symbol: drake::MakeNameValue
    struct /* MakeNameValue */ {
      // Source: drake/common/name_value.h
      const char* doc =
R"""((Advanced) Creates a NameValue. The conventional method for calling
this function is the DRAKE_NVP sugar macro below.

Both pointers are aliased for the lifetime of the return value.)""";
    } MakeNameValue;
    // Symbol: drake::MakeSortedPair
    struct /* MakeSortedPair */ {
      // Source: drake/common/sorted_pair.h
      const char* doc =
R"""(A convenience wrapper for creating a sorted pair from two objects.

Parameter ``x``:
    The first_ object.

Parameter ``y``:
    The second_ object.

Returns:
    A newly-constructed SortedPair object.)""";
    } MakeSortedPair;
    // Symbol: drake::ManualTimer
    struct /* ManualTimer */ {
      // Source: drake/common/timer.h
      const char* doc =
R"""(Implementation of timing for use with unit tests that control time
manually.)""";
      // Symbol: drake::ManualTimer::ManualTimer
      struct /* ctor */ {
        // Source: drake/common/timer.h
        const char* doc = R"""()""";
      } ctor;
      // Symbol: drake::ManualTimer::Start
      struct /* Start */ {
        // Source: drake/common/timer.h
        const char* doc = R"""()""";
      } Start;
      // Symbol: drake::ManualTimer::Tick
      struct /* Tick */ {
        // Source: drake/common/timer.h
        const char* doc = R"""()""";
      } Tick;
      // Symbol: drake::ManualTimer::set_tick
      struct /* set_tick */ {
        // Source: drake/common/timer.h
        const char* doc = R"""(Sets the return value of Tick().)""";
      } set_tick;
    } ManualTimer;
    // Symbol: drake::MaybeGetDrakePath
    struct /* MaybeGetDrakePath */ {
      // Source: drake/common/drake_path.h
      const char* doc =
R"""((Advanced) Returns the fully-qualified path to the first folder
containing Drake resources as located by FindResource, or nullopt if
none is found. For example
``${result}/examples/pendulum/Pendulum.urdf`` would be the path to the
Pendulum example's URDF resource.

Most users should prefer FindResource() or FindResourceOrThrow() to
locate Drake resources for a specific resource filename. This method
only exists for legacy compatibility reasons, and might eventually be
removed.)""";
    } MaybeGetDrakePath;
    // Symbol: drake::MemoryFile
    struct /* MemoryFile */ {
      // Source: drake/common/memory_file.h
      const char* doc = R"""(A virtual file, stored in memory.)""";
      // Symbol: drake::MemoryFile::Make
      struct /* Make */ {
        // Source: drake/common/memory_file.h
        const char* doc =
R"""(Creates an instance of MemoryFile from the file located at the given
``path``. The filename_hint() will be the stringified path. Making a
MemoryFile computes the hash of its contents. If all you want is the
contents, use drake::ReadFile() or drake::ReadFileOrThrow() instead.

Raises:
    RuntimeError if the file at ``path`` cannot be read.)""";
      } Make;
      // Symbol: drake::MemoryFile::MemoryFile
      struct /* ctor */ {
        // Source: drake/common/memory_file.h
        const char* doc_0args =
R"""(Default constructor with no contents, checksum, or filename hint. In
this case, the ``checksum`` will be the checksum of the empty
contents.)""";
        // Source: drake/common/memory_file.h
        const char* doc_3args =
R"""(Constructs a new file from the given ``contents``.

Parameter ``contents``:
    The contents of a file.

Parameter ``extension``:
    The extension typically associated with the file contents. The
    case is unimportant, but it must either be empty or of the form
    ``.foo``.

Parameter ``filename_hint``:
    A label for the file. The label is used for warning and error
    messages. Otherwise, the label has no other functional purpose. It
    need not be a valid file name, but must consist of a single line
    (no newlines).

Warning:
    An empty extension may be problematic. Many consumers of
    MemoryFile key on the extension to determine if the file is
    suitable for a purpose. Always provide an accurate, representative
    extension when possible.

Raises:
    RuntimeError if ``filename_hint`` contains newlines.

Raises:
    RuntimeError if ``extension`` is not empty and the first character
    isn't '.'.)""";
      } ctor;
      // Symbol: drake::MemoryFile::Serialize
      struct /* Serialize */ {
        // Source: drake/common/memory_file.h
        const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.

When used in yaml, it is important to specify *all* fields.
Applications may depend on the ``extension`` value to determine what
to do with the file contents. Omitting ``extension`` would make it
unusable in those cases.

Omitting ``filename_hint`` is less dangerous; error messages would
lack a helpful identifier, but things would otherwise function.

The value of contents should be a base64-encoded string of the file
contents. Yaml's ``!!binary`` tag is required to declare the value is
such a string. Serializing the MemoryFile will produce such a string.
Writing a yaml file by hand will be more challenging.

For this yaml:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    {yaml}
    contents: !!binary VGhpcyBpcyBhbiBleGFtcGxlIG9mIG1
    lbW9yeSBmaWxlIHRlc3QgY29udGVudHMu
    extension: .txt
    filename_hint: payload.txt

.. raw:: html

    </details>

we would produce a MemoryFile with contents equal to:

This is an example of memory file test contents.)""";
      } Serialize;
      // Symbol: drake::MemoryFile::contents
      struct /* contents */ {
        // Source: drake/common/memory_file.h
        const char* doc = R"""(Returns the file's contents.)""";
      } contents;
      // Symbol: drake::MemoryFile::extension
      struct /* extension */ {
        // Source: drake/common/memory_file.h
        const char* doc =
R"""(Returns the extension (as passed to the constructor). When not empty,
it will always be reported with a leading period and all lower case
characters.)""";
      } extension;
      // Symbol: drake::MemoryFile::filename_hint
      struct /* filename_hint */ {
        // Source: drake/common/memory_file.h
        const char* doc =
R"""(Returns the notional "filename" for this ``file``.)""";
      } filename_hint;
      // Symbol: drake::MemoryFile::sha256
      struct /* sha256 */ {
        // Source: drake/common/memory_file.h
        const char* doc =
R"""(Returns the checksum of ``this`` instance's ``contents()``.)""";
      } sha256;
      // Symbol: drake::MemoryFile::to_string
      struct /* to_string */ {
        // Source: drake/common/memory_file.h
        const char* doc =
R"""(Returns a string representation. Note: the file contents will be
limited to ``contents_limit`` number of characters. To include the
full contents, pass any number less than or equal to zero.)""";
      } to_string;
    } MemoryFile;
    // Symbol: drake::NameValue
    struct /* NameValue */ {
      // Source: drake/common/name_value.h
      const char* doc =
R"""((Advanced) A basic implementation of the Name-Value Pair concept as
used in the Serialize / Archive pattern.

NameValue stores a pointer to a const ``name`` and a pointer to a
mutable ``value``. Both pointers must remain valid throughout the
lifetime of an object. NameValue objects are typically short-lived,
existing only for a transient moment while an Archive is visiting some
Serializable field.

For more information, refer to Drake's yaml_serialization "YAML
Serialization" and especially the implementing_serialize "Implementing
Serialize" section, or also the <a
href="https://www.boost.org/doc/libs/release/libs/serialization/doc/wrappers.html#nvp">Boost
Name-Value Pairs</a> documentation for background.)""";
      // Symbol: drake::NameValue::NameValue<T>
      struct /* ctor */ {
        // Source: drake/common/name_value.h
        const char* doc =
R"""((Advanced) Constructs a NameValue. Prefer DRAKE_NVP instead of this
constructor. Both pointers are aliased and must remain valid for the
lifetime of this object. Neither pointer can be nullptr.)""";
      } ctor;
      // Symbol: drake::NameValue::name
      struct /* name */ {
        // Source: drake/common/name_value.h
        const char* doc = R"""()""";
      } name;
      // Symbol: drake::NameValue::value
      struct /* value */ {
        // Source: drake/common/name_value.h
        const char* doc = R"""()""";
      } value;
      // Symbol: drake::NameValue::value_type
      struct /* value_type */ {
        // Source: drake/common/name_value.h
        const char* doc = R"""(Type of the referenced value.)""";
      } value_type;
    } NameValue;
    // Symbol: drake::NiceTypeName
    struct /* NiceTypeName */ {
      // Source: drake/common/nice_type_name.h
      const char* doc =
R"""(Obtains canonicalized, platform-independent, human-readable names for
arbitrarily-complicated C++ types.

Usage:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    // For types:
    using std::pair; using std::string;
    using MyVectorType = pair<int,string>;
    std::cout << "Type MyVectorType was: "
    << drake::NiceTypeName::Get<MyVectorType>() << std::endl;
    // Output: std::pair<int,std::string>
    
    // For expressions:
    std::unique_ptr<AbstractThing> thing;  // Assume actual type is ConcreteThing.
    std::cout << "Actual type of 'thing' was: "
    << drake::NiceTypeName::Get(*thing) << std::endl;
    // Output: ConcreteThing

.. raw:: html

    </details>

We demangle and attempt to canonicalize the compiler-generated type
names as reported by ``typeid(T).name()`` so that the same string is
returned by all supported compilers and platforms. The output of
NiceTypeName::Get<T>() is useful in error and log messages and
testing. It also provides a persistent, platform-independent
identifier for types; ``std::type_info`` cannot provide that.

Warning:
    Don't expect usable names for types that are defined in an
    anonymous namespace or for function-local types. Names will still
    be produced but they won't be unique, pretty, or
    compiler-independent.

This class exists only to group type name-related static methods;
don't try to construct an object of this type.)""";
      // Symbol: drake::NiceTypeName::Canonicalize
      struct /* Canonicalize */ {
        // Source: drake/common/nice_type_name.h
        const char* doc =
R"""(Given a compiler-dependent demangled type name string as returned by
Demangle(), attempts to form a canonicalized representation that will
be the same for any compiler. Unnecessary spaces and superfluous
keywords like "class" and "struct" are removed. The
NiceTypeName::Get<T>() method uses this function to produce a
human-friendly type name that is the same on any platform.)""";
      } Canonicalize;
      // Symbol: drake::NiceTypeName::Demangle
      struct /* Demangle */ {
        // Source: drake/common/nice_type_name.h
        const char* doc =
R"""(Using the algorithm appropriate to the current compiler, demangles a
type name as returned by ``typeid(T).name()``, with the result
hopefully suitable for meaningful display to a human. The result is
compiler-dependent.

See also:
    Canonicalize())""";
      } Demangle;
      // Symbol: drake::NiceTypeName::Get
      struct /* Get */ {
        // Source: drake/common/nice_type_name.h
        const char* doc_0args =
R"""(Returns a nicely demangled and canonicalized type name that is the
same on all platforms, using Canonicalize(). This is calculated on the
fly so is expensive whenever called, though very reasonable for use in
error messages. For repeated or performance-sensitive uses, see
GetFromStorage().)""";
        // Source: drake/common/nice_type_name.h
        const char* doc_1args_constT =
R"""(Returns the type name of the most-derived type of an object of type T,
typically but not necessarily polymorphic. This must be calculated on
the fly so is expensive whenever called, though very reasonable for
use in error messages. For non-polymorphic types this produces the
same result as would ``Get<decltype(thing)>()`` but for polymorphic
types the results will differ.)""";
        // Source: drake/common/nice_type_name.h
        const char* doc_1args_info =
R"""(Returns the nicely demangled and canonicalized type name of ``info``.
This must be calculated on the fly so is expensive whenever called,
though very reasonable for use in error messages.)""";
      } Get;
      // Symbol: drake::NiceTypeName::GetFromStorage
      struct /* GetFromStorage */ {
        // Source: drake/common/nice_type_name.h
        const char* doc =
R"""(Like Get<T>() but only computed once per process for a given type
``T``. The returned reference will not be deleted even at program
termination, so feel free to use it in error messages even in
destructors that may be invoked during program tear-down.)""";
      } GetFromStorage;
      // Symbol: drake::NiceTypeName::RemoveNamespaces
      struct /* RemoveNamespaces */ {
        // Source: drake/common/nice_type_name.h
        const char* doc =
R"""(Given a canonical type name that may include leading namespaces,
attempts to remove those namespaces. For example,
``drake::systems::MyThing<internal::type>`` becomes
``MyThing<internal::type>``. If the last segment ends in ``::``, the
original string is returned unprocessed. Note that this is just string
processing -- a segment that looks like a namespace textually will be
treated as one, even if it is really a class. So
``drake::MyClass::Impl`` will be reduced to ``Impl`` while
``drake::MyClass<T>::Impl`` is reduced to ``MyClass<T>::Impl``.)""";
      } RemoveNamespaces;
    } NiceTypeName;
    // Symbol: drake::Parallelism
    struct /* Parallelism */ {
      // Source: drake/common/parallelism.h
      const char* doc =
R"""(Specifies a desired degree of parallelism for a parallelized
operation.

This class denotes a specific number of threads; either 1 (no
parallelism), a user-specified value (any number >= 1), or the maximum
number. For convenience, conversion from ``bool`` is provided so that
``False`` is no parallelism and ``True`` is maximum parallelism.

Drake's API uses this class to allow users to control the degree of
parallelism, regardless of how the parallelization is implemented
(e.g., ``std::async``, OpenMP, TBB, etc).

Configuring the process-wide maximum parallelism
================================================

The number of threads denoted by Parallelism::Max() is configurable
with environment variables, but will be invariant within a single
process. The first time it's accessed, the configured value will be
latched into a global variable indefinitely. To ensure your
configuration is obeyed, any changes to the environment variables that
govern the max parallelism should be made prior to importing or
calling any Drake code.

The following recipe determines the value that
Parallelism::Max().num_threads() will report:

1. The default is the hardware limit from ``std::thread::hardware_concurrency()``;
this will be used when none of the special cases below are in effect.

2. If the environment variable ``DRAKE_NUM_THREADS`` is set to a positive integer
less than ``hardware_concurrency()``, the ``num_threads`` will be that value.

3. If the environment variable ``DRAKE_NUM_THREADS`` is not set but the
environment variable ``OMP_NUM_THREADS`` is set to a positive integer less than
``hardware_concurrency()``, the ``num_threads`` will be that value. (Note in
particular that a comma-separated ``OMP_NUM_THREADS`` value will be ignored, even
though that syntax might be valid for an OpenMP library).

The configuration recipe above does not require Drake to be built with
OpenMP enabled. The inspection of ``OMP_NUM_THREADS`` as a
configuration value is provided for convenience, regardless of whether
OpenMP is enabled.

**A note for Drake developers**

In Drake's unit tests, ``DRAKE_NUM_THREADS`` is set to "1" by default.
If your unit test requires actual parallelism, use the ``num_threads =
N`` attribute in the ``BUILD.bazel`` file to declare a different
value.)""";
      // Symbol: drake::Parallelism::Max
      struct /* Max */ {
        // Source: drake/common/parallelism.h
        const char* doc =
R"""(Constructs a Parallelism with the maximum number of threads. Refer to
the class overview documentation for how to configure the maximum.)""";
      } Max;
      // Symbol: drake::Parallelism::None
      struct /* None */ {
        // Source: drake/common/parallelism.h
        const char* doc =
R"""(Constructs a Parallelism with no parallelism (i.e., num_threads=1).
Python note: This function is not bound in pydrake (due to its name);
instead, use the default constructor (or pass either ``1`` or
``False`` to the 1-argument constructor).)""";
      } None;
      // Symbol: drake::Parallelism::Parallelism
      struct /* ctor */ {
        // Source: drake/common/parallelism.h
        const char* doc_0args =
R"""(Default constructs with no parallelism (i.e., num_threads=1).)""";
        // Source: drake/common/parallelism.h
        const char* doc_1args_parallelize =
R"""(Constructs a Parallelism with either no parallelism (i.e.,
num_threads=1) or the maximum number of threads (Max()), as selected
by ``parallelize``. This constructor allows for implicit conversion,
for convenience.)""";
        // Source: drake/common/parallelism.h
        const char* doc_1args_num_threads =
R"""(Constructs with the provided number of threads ``num_threads``.

Precondition:
    num_threads >= 1.

Note:
    Constructing and using a Parallelism with num_threads greater than
    the actual hardware concurrency may result in fewer than the
    specified number of threads actually being launched or poor
    performance due to CPU contention.)""";
      } ctor;
      // Symbol: drake::Parallelism::num_threads
      struct /* num_threads */ {
        // Source: drake/common/parallelism.h
        const char* doc =
R"""(Returns the degree of parallelism. The result will always be >= 1.)""";
      } num_threads;
    } Parallelism;
    // Symbol: drake::Polynomial
    struct /* Polynomial */ {
      // Source: drake/common/polynomial.h
      const char* doc =
R"""(A scalar multi-variate polynomial, modeled after the msspoly in
spotless.

Polynomial represents a list of additive Monomials, each one of which
is a product of a constant coefficient (of T, which by default is
double) and any number of distinct Terms (variables raised to positive
integer powers).

Variables are identified by integer indices rather than symbolic
names, but an automatic facility is provided to convert variable names
up to four characters into unique integers, provided those variables
are named using only lowercase letters and the "#_." characters
followed by a number. For example, valid names include "dx4" and
"m_x".

Monomials which have the same variables and powers may be constructed
but will be automatically combined: (3 * a * b * a) + (1.5 * b * a**2)
will be reduced to (4.5 * b * a**2) internally after construction.

Polynomials can be added, subtracted, and multiplied. They may only be
divided by scalars (of T) because Polynomials are not closed under
division.)""";
      // Symbol: drake::Polynomial::CoefficientsAlmostEqual
      struct /* CoefficientsAlmostEqual */ {
        // Source: drake/common/polynomial.h
        const char* doc =
R"""(Checks if a Polynomial is approximately equal to this one.

Checks that every coefficient of ``other`` is within ``tol`` of the
corresponding coefficient of this Polynomial.

Note: When ``tol_type`` is kRelative, if any monomials appear in
``this`` or ``other`` but not both, then the method returns false
(since the comparison is relative to a missing zero coefficient). Use
kAbsolute if you want to ignore non-matching monomials with
coefficients less than ``tol``.)""";
      } CoefficientsAlmostEqual;
      // Symbol: drake::Polynomial::Derivative
      struct /* Derivative */ {
        // Source: drake/common/polynomial.h
        const char* doc =
R"""(Takes the derivative of this (univariate) Polynomial.

Returns a new Polynomial that is the derivative of this one in its
sole variable.

Raises:
    RuntimeError if this Polynomial is not univariate.

If derivative_order is given, takes the nth derivative of this
Polynomial.)""";
      } Derivative;
      // Symbol: drake::Polynomial::EvaluateMultivariate
      struct /* EvaluateMultivariate */ {
        // Source: drake/common/polynomial.h
        const char* doc =
R"""(Evaluate a multivariate Polynomial at a specific point.

Evaluates a Polynomial with the given values for each variable.

Raises:
    RuntimeError if the Polynomial contains variables for which values
    were not provided.

The provided values may be of any type which is std::is_arithmetic
(supporting the std::pow, *, and + operations) and need not be
CoefficientsType or RealScalar))""";
      } EvaluateMultivariate;
      // Symbol: drake::Polynomial::EvaluatePartial
      struct /* EvaluatePartial */ {
        // Source: drake/common/polynomial.h
        const char* doc =
R"""(Substitute values for some but not necessarily all variables of a
Polynomial.

Analogous to EvaluateMultivariate, but: (1) Restricted to T, and (2)
Need not map every variable in var_values.

Returns a Polynomial in which each variable in var_values has been
replaced with its value and constants appropriately combined.)""";
      } EvaluatePartial;
      // Symbol: drake::Polynomial::EvaluateUnivariate
      struct /* EvaluateUnivariate */ {
        // Source: drake/common/polynomial.h
        const char* doc =
R"""(Evaluate a univariate Polynomial at a specific point.

Evaluates a univariate Polynomial at the given x.

Raises:
    RuntimeError if this Polynomial is not univariate.

``x`` may be of any type supporting the ** and + operations (which can
be different from both CoefficientsType and RealScalar).

This method may also be used for efficient evaluation of the
derivatives of the univariate polynomial, evaluated at ``x``.
``derivative_order`` = 0 (the default) returns the polynomial value
without differentiation. ``derivative_order`` = 1 returns the first
derivative, etc.

Precondition:
    derivative_order must be non-negative.)""";
      } EvaluateUnivariate;
      // Symbol: drake::Polynomial::FromExpression
      struct /* FromExpression */ {
        // Source: drake/common/polynomial.h
        const char* doc =
R"""(Constructs a Polynomial representing the symbolic expression ``e``.
Note that the ID of a variable is preserved in this translation.

Raises:
    RuntimeError if ``e`` is not polynomial-convertible.

Precondition:
    e.is_polynomial() is true.)""";
      } FromExpression;
      // Symbol: drake::Polynomial::GetCoefficients
      struct /* GetCoefficients */ {
        // Source: drake/common/polynomial.h
        const char* doc =
R"""(Returns the vector of the coefficients of the polynomial in the order
of powers of the variable - the [i]th element is the coefficient of
the variable raised to the ith power.

Raises:
    RuntimeError if this Polynomial is not univariate.)""";
      } GetCoefficients;
      // Symbol: drake::Polynomial::GetDegree
      struct /* GetDegree */ {
        // Source: drake/common/polynomial.h
        const char* doc =
R"""(Returns the highest degree of any Monomial in this Polynomial.

The degree of a multivariate Monomial is the product of the degrees of
each of its terms.)""";
      } GetDegree;
      // Symbol: drake::Polynomial::GetMonomials
      struct /* GetMonomials */ {
        // Source: drake/common/polynomial.h
        const char* doc = R"""()""";
      } GetMonomials;
      // Symbol: drake::Polynomial::GetNumberOfCoefficients
      struct /* GetNumberOfCoefficients */ {
        // Source: drake/common/polynomial.h
        const char* doc =
R"""(Returns the number of unique Monomials (and thus the number of
coefficients) in this Polynomial.)""";
      } GetNumberOfCoefficients;
      // Symbol: drake::Polynomial::GetSimpleVariable
      struct /* GetSimpleVariable */ {
        // Source: drake/common/polynomial.h
        const char* doc =
R"""(If the polynomial is "simple" -- e.g. just a single term with
coefficient 1 -- then returns that variable; otherwise returns 0.)""";
      } GetSimpleVariable;
      // Symbol: drake::Polynomial::GetVariables
      struct /* GetVariables */ {
        // Source: drake/common/polynomial.h
        const char* doc =
R"""(Returns a set of all of the variables present in this Polynomial.)""";
      } GetVariables;
      // Symbol: drake::Polynomial::IdToVariableName
      struct /* IdToVariableName */ {
        // Source: drake/common/polynomial.h
        const char* doc = R"""()""";
      } IdToVariableName;
      // Symbol: drake::Polynomial::Integral
      struct /* Integral */ {
        // Source: drake/common/polynomial.h
        const char* doc =
R"""(Takes the integral of this (univariate, non-constant) Polynomial.

Returns a new Polynomial that is the indefinite integral of this one
in its sole variable.

Raises:
    RuntimeError if this Polynomial is not univariate, or if it has no
    variables.

If integration_constant is given, adds that constant as the constant
term (zeroth-order coefficient) of the resulting Polynomial.)""";
      } Integral;
      // Symbol: drake::Polynomial::IsAffine
      struct /* IsAffine */ {
        // Source: drake/common/polynomial.h
        const char* doc =
R"""(Returns true iff this is a sum of terms of degree 1, plus a constant.)""";
      } IsAffine;
      // Symbol: drake::Polynomial::IsValidVariableName
      struct /* IsValidVariableName */ {
        // Source: drake/common/polynomial.h
        const char* doc = R"""(Variable name/ID conversion facility.)""";
      } IsValidVariableName;
      // Symbol: drake::Polynomial::Monomial
      struct /* Monomial */ {
        // Source: drake/common/polynomial.h
        const char* doc =
R"""(An additive atom of a Polynomial: The product of any number of Terms
and a coefficient.)""";
        // Symbol: drake::Polynomial::Monomial::Factor
        struct /* Factor */ {
          // Source: drake/common/polynomial.h
          const char* doc =
R"""(Factors this by other; returns 0 iff other does not divide this.)""";
        } Factor;
        // Symbol: drake::Polynomial::Monomial::GetDegree
        struct /* GetDegree */ {
          // Source: drake/common/polynomial.h
          const char* doc = R"""()""";
        } GetDegree;
        // Symbol: drake::Polynomial::Monomial::GetDegreeOf
        struct /* GetDegreeOf */ {
          // Source: drake/common/polynomial.h
          const char* doc = R"""()""";
        } GetDegreeOf;
        // Symbol: drake::Polynomial::Monomial::HasSameExponents
        struct /* HasSameExponents */ {
          // Source: drake/common/polynomial.h
          const char* doc = R"""()""";
        } HasSameExponents;
        // Symbol: drake::Polynomial::Monomial::HasVariable
        struct /* HasVariable */ {
          // Source: drake/common/polynomial.h
          const char* doc = R"""()""";
        } HasVariable;
        // Symbol: drake::Polynomial::Monomial::coefficient
        struct /* coefficient */ {
          // Source: drake/common/polynomial.h
          const char* doc = R"""()""";
        } coefficient;
        // Symbol: drake::Polynomial::Monomial::operator<
        struct /* operator_lt */ {
          // Source: drake/common/polynomial.h
          const char* doc =
R"""(A comparison to allow std::lexicographical_compare on this class; does
not reflect any sort of mathematical total order.)""";
        } operator_lt;
        // Symbol: drake::Polynomial::Monomial::terms
        struct /* terms */ {
          // Source: drake/common/polynomial.h
          const char* doc = R"""()""";
        } terms;
        // Symbol: drake::Polynomial::Monomial::to_string
        struct /* to_string */ {
          // Source: drake/common/polynomial.h
          const char* doc = R"""()""";
        } to_string;
      } Monomial;
      // Symbol: drake::Polynomial::Polynomial<T>
      struct /* ctor */ {
        // Source: drake/common/polynomial.h
        const char* doc_0args = R"""(Construct the vacuous polynomial, "0".)""";
        // Source: drake/common/polynomial.h
        const char* doc_1args_scalar =
R"""(Construct a Polynomial of a single constant. e.g. "5".)""";
        // Source: drake/common/polynomial.h
        const char* doc_2args_coeff_terms =
R"""(Construct a Polynomial consisting of a single Monomial, e.g. "5xy**3".)""";
        // Source: drake/common/polynomial.h
        const char* doc_2args_start_finish =
R"""(Construct a Polynomial from a sequence of Monomials.)""";
        // Source: drake/common/polynomial.h
        const char* doc_1args_conststdenableift =
R"""(Constructs a polynomial consisting of a single Monomial of the
variable named ``varname1``.

Note:
    : This constructor is only provided for T = double. For the other
    cases, a user should use the constructor with two arguments below
    (taking std::string and unsigned int). If we provided this
    constructor for T = AutoDiffXd and T = symbolic::Expression, there
    would be compiler errors for ``Polynomial<T>(0)`` as the following
    candidates are ambiguous: - Polynomial(const T& scalar) -
    Polynomial(const std::string& varname, const unsigned int num = 1))""";
        // Source: drake/common/polynomial.h
        const char* doc_2args_varname_num =
R"""(Construct a polynomial consisting of a single Monomial of the variable
named varname + num.)""";
        // Source: drake/common/polynomial.h
        const char* doc_2args_coeff_v =
R"""(Construct a single Monomial of the given coefficient and variable.)""";
        // Source: drake/common/polynomial.h
        const char* doc_1args_constEigenMatrixBase =
R"""(A constructor for univariate polynomials: takes a vector of
coefficients for the x**0, x**1, x**2, x**3... Monomials. All terms
are always added, even if a coefficient is zero.)""";
      } ctor;
      // Symbol: drake::Polynomial::PowerType
      struct /* PowerType */ {
        // Source: drake/common/polynomial.h
        const char* doc =
R"""(This should be 'unsigned int' but MSVC considers a call to
std::pow(..., unsigned int) ambiguous because it won't cast unsigned
int to int.)""";
      } PowerType;
      // Symbol: drake::Polynomial::Product
      struct /* Product */ {
        // Source: drake/common/polynomial.h
        const char* doc = R"""()""";
        // Symbol: drake::Polynomial::Product::type
        struct /* type */ {
          // Source: drake/common/polynomial.h
          const char* doc = R"""()""";
        } type;
      } Product;
      // Symbol: drake::Polynomial::RealScalar
      struct /* RealScalar */ {
        // Source: drake/common/polynomial.h
        const char* doc = R"""()""";
      } RealScalar;
      // Symbol: drake::Polynomial::RootType
      struct /* RootType */ {
        // Source: drake/common/polynomial.h
        const char* doc = R"""()""";
      } RootType;
      // Symbol: drake::Polynomial::Roots
      struct /* Roots */ {
        // Source: drake/common/polynomial.h
        const char* doc =
R"""(Returns the roots of this (univariate) Polynomial.

Returns the roots of a univariate Polynomial as an Eigen column vector
of complex numbers whose components are of the RealScalar type.

Raises:
    RuntimeError of this Polynomial is not univariate.)""";
      } Roots;
      // Symbol: drake::Polynomial::RootsType
      struct /* RootsType */ {
        // Source: drake/common/polynomial.h
        const char* doc = R"""()""";
      } RootsType;
      // Symbol: drake::Polynomial::Subs
      struct /* Subs */ {
        // Source: drake/common/polynomial.h
        const char* doc =
R"""(Replaces all instances of variable orig with replacement.)""";
      } Subs;
      // Symbol: drake::Polynomial::Substitute
      struct /* Substitute */ {
        // Source: drake/common/polynomial.h
        const char* doc =
R"""(Replaces all instances of variable orig with replacement.)""";
      } Substitute;
      // Symbol: drake::Polynomial::Term
      struct /* Term */ {
        // Source: drake/common/polynomial.h
        const char* doc =
R"""(An individual variable raised to an integer power; e.g. x**2.)""";
        // Symbol: drake::Polynomial::Term::operator<
        struct /* operator_lt */ {
          // Source: drake/common/polynomial.h
          const char* doc =
R"""(A comparison to allow std::lexicographical_compare on this class; does
not reflect any sort of mathematical total order.)""";
        } operator_lt;
        // Symbol: drake::Polynomial::Term::power
        struct /* power */ {
          // Source: drake/common/polynomial.h
          const char* doc = R"""()""";
        } power;
        // Symbol: drake::Polynomial::Term::var
        struct /* var */ {
          // Source: drake/common/polynomial.h
          const char* doc = R"""()""";
        } var;
      } Term;
      // Symbol: drake::Polynomial::VarType
      struct /* VarType */ {
        // Source: drake/common/polynomial.h
        const char* doc = R"""()""";
      } VarType;
      // Symbol: drake::Polynomial::VariableNameToId
      struct /* VariableNameToId */ {
        // Source: drake/common/polynomial.h
        const char* doc = R"""()""";
      } VariableNameToId;
      // Symbol: drake::Polynomial::is_univariate
      struct /* is_univariate */ {
        // Source: drake/common/polynomial.h
        const char* doc =
R"""(Returns true if this is a univariate polynomial)""";
      } is_univariate;
      // Symbol: drake::Polynomial::operator*
      struct /* operator_mul */ {
        // Source: drake/common/polynomial.h
        const char* doc = R"""()""";
      } operator_mul;
      // Symbol: drake::Polynomial::operator*=
      struct /* operator_imul */ {
        // Source: drake/common/polynomial.h
        const char* doc = R"""()""";
      } operator_imul;
      // Symbol: drake::Polynomial::operator+
      struct /* operator_add */ {
        // Source: drake/common/polynomial.h
        const char* doc = R"""()""";
      } operator_add;
      // Symbol: drake::Polynomial::operator+=
      struct /* operator_iadd */ {
        // Source: drake/common/polynomial.h
        const char* doc = R"""()""";
      } operator_iadd;
      // Symbol: drake::Polynomial::operator-
      struct /* operator_sub */ {
        // Source: drake/common/polynomial.h
        const char* doc = R"""()""";
      } operator_sub;
      // Symbol: drake::Polynomial::operator-=
      struct /* operator_isub */ {
        // Source: drake/common/polynomial.h
        const char* doc = R"""()""";
      } operator_isub;
      // Symbol: drake::Polynomial::operator/
      struct /* operator_div */ {
        // Source: drake/common/polynomial.h
        const char* doc = R"""()""";
      } operator_div;
      // Symbol: drake::Polynomial::operator/=
      struct /* operator_idiv */ {
        // Source: drake/common/polynomial.h
        const char* doc = R"""()""";
      } operator_idiv;
      // Symbol: drake::Polynomial::operator<
      struct /* operator_lt */ {
        // Source: drake/common/polynomial.h
        const char* doc =
R"""(A comparison to allow std::lexicographical_compare on this class; does
not reflect any sort of mathematical total order.)""";
      } operator_lt;
      // Symbol: drake::Polynomial::to_string
      struct /* to_string */ {
        // Source: drake/common/polynomial.h
        const char* doc = R"""()""";
      } to_string;
    } Polynomial;
    // Symbol: drake::Polynomiald
    struct /* Polynomiald */ {
      // Source: drake/common/polynomial.h
      const char* doc = R"""()""";
    } Polynomiald;
    // Symbol: drake::RandomDistribution
    struct /* RandomDistribution */ {
      // Source: drake/common/random.h
      const char* doc =
R"""(Drake supports explicit reasoning about a few carefully chosen random
distributions.)""";
      // Symbol: drake::RandomDistribution::kExponential
      struct /* kExponential */ {
        // Source: drake/common/random.h
        const char* doc =
R"""(Vector elements are independent and drawn from an)""";
      } kExponential;
      // Symbol: drake::RandomDistribution::kGaussian
      struct /* kGaussian */ {
        // Source: drake/common/random.h
        const char* doc =
R"""(Vector elements are independent and drawn from a)""";
      } kGaussian;
      // Symbol: drake::RandomDistribution::kUniform
      struct /* kUniform */ {
        // Source: drake/common/random.h
        const char* doc =
R"""(Vector elements are independent and uniformly distributed)""";
      } kUniform;
    } RandomDistribution;
    // Symbol: drake::RandomGenerator
    struct /* RandomGenerator */ {
      // Source: drake/common/random.h
      const char* doc =
R"""(Defines Drake's canonical implementation of the
UniformRandomBitGenerator C++ concept (as well as a few conventional
extras beyond the concept, e.g., seeds). This uses the 32-bit Mersenne
Twister mt19937 by Matsumoto and Nishimura, 1998. For more
information, see
https://en.cppreference.com/w/cpp/numeric/random/mersenne_twister_engine)""";
      // Symbol: drake::RandomGenerator::RandomGenerator
      struct /* ctor */ {
        // Source: drake/common/random.h
        const char* doc_0args =
R"""(Creates a generator using the ``default_seed``. Actual creation of the
generator is deferred until first use so this constructor is fast and
does not allocate any heap memory.)""";
        // Source: drake/common/random.h
        const char* doc_1args =
R"""(Creates a generator using given ``seed``.)""";
      } ctor;
      // Symbol: drake::RandomGenerator::max
      struct /* max */ {
        // Source: drake/common/random.h
        const char* doc = R"""()""";
      } max;
      // Symbol: drake::RandomGenerator::min
      struct /* min */ {
        // Source: drake/common/random.h
        const char* doc = R"""()""";
      } min;
      // Symbol: drake::RandomGenerator::operator()
      struct /* operator_call */ {
        // Source: drake/common/random.h
        const char* doc = R"""(Generates a pseudo-random value.)""";
      } operator_call;
      // Symbol: drake::RandomGenerator::result_type
      struct /* result_type */ {
        // Source: drake/common/random.h
        const char* doc = R"""()""";
      } result_type;
    } RandomGenerator;
    // Symbol: drake::ReadFile
    struct /* ReadFile */ {
      // Source: drake/common/find_resource.h
      const char* doc =
R"""(Returns the content of the file at the given path, or nullopt if it
cannot be read. Note that the path is a filesystem path, not a
``resource_path``.)""";
    } ReadFile;
    // Symbol: drake::ReadFileOrThrow
    struct /* ReadFileOrThrow */ {
      // Source: drake/common/find_resource.h
      const char* doc =
R"""(Returns the content of the file at the given path, or throws if it
cannot be read. Note that the path is a filesystem path, not a
``resource_path``.)""";
    } ReadFileOrThrow;
    // Symbol: drake::RlocationOrError
    struct /* RlocationOrError */ {
      // Source: drake/common/find_runfiles.h
      const char* doc =
R"""((Advanced.) The return type of FindRunfile(). Exactly one of the two
strings is non-empty.)""";
      // Symbol: drake::RlocationOrError::abspath
      struct /* abspath */ {
        // Source: drake/common/find_runfiles.h
        const char* doc =
R"""(The absolute path to the resource_path runfile.)""";
      } abspath;
      // Symbol: drake::RlocationOrError::error
      struct /* error */ {
        // Source: drake/common/find_runfiles.h
        const char* doc = R"""(The error message.)""";
      } error;
    } RlocationOrError;
    // Symbol: drake::ScopeExit
    struct /* ScopeExit */ {
      // Source: drake/common/scope_exit.h
      const char* doc =
R"""(Helper class to create a scope exit guard -- an object that when
destroyed runs ``func``. This is useful to apply RAII to third-party
code that only supports manual acquire and release operations.

Example:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    void some_function() {
      void* foo = ::malloc(10);
      ScopeExit guard([foo]() {
        ::free(foo);
      });
    
      // ...
      if (condition) { throw RuntimeError("..."); }
      // ...
    }

.. raw:: html

    </details>

Here, the allocation of ``foo`` will always be free'd no matter
whether ``some_function`` returns normally or via an exception.)""";
      // Symbol: drake::ScopeExit::Disarm
      struct /* Disarm */ {
        // Source: drake/common/scope_exit.h
        const char* doc =
R"""(Disarms this guard, so that the destructor has no effect.)""";
      } Disarm;
      // Symbol: drake::ScopeExit::ScopeExit
      struct /* ctor */ {
        // Source: drake/common/scope_exit.h
        const char* doc =
R"""(Creates a resource that will call ``func`` when destroyed. Note that
``func()`` should not throw an exception, since it will typically be
invoked during stack unwinding.)""";
      } ctor;
    } ScopeExit;
    // Symbol: drake::Sha256
    struct /* Sha256 */ {
      // Source: drake/common/sha256.h
      const char* doc =
R"""(Represents a SHA-256 cryptographic checksum. See also
https://en.wikipedia.org/wiki/SHA-2.

This class is not bound in pydrake, because Python programmers should
prefer using https://docs.python.org/3/library/hashlib.html instead.)""";
      // Symbol: drake::Sha256::Checksum
      struct /* Checksum */ {
        // Source: drake/common/sha256.h
        const char* doc_1args_data =
R"""(Computes the checksum of the given ``data`` buffer.)""";
        // Source: drake/common/sha256.h
        const char* doc_1args_stream =
R"""(Computes the checksum of the given ``stream``. Does not check for
istream errors; that is the responsibility of the caller.

Precondition:
    stream != nullptr.)""";
      } Checksum;
      // Symbol: drake::Sha256::Parse
      struct /* Parse */ {
        // Source: drake/common/sha256.h
        const char* doc =
R"""(Parses the 64-character ASCII hex representation of a SHA-256
checksum. Returns std::nullopt if the argument is an invalid checksum.)""";
      } Parse;
      // Symbol: drake::Sha256::Sha256
      struct /* ctor */ {
        // Source: drake/common/sha256.h
        const char* doc =
R"""(Constructs an all-zero checksum. Note that this is NOT the same as the
checksum of empty (zero-sized) data.)""";
      } ctor;
      // Symbol: drake::Sha256::operator!=
      struct /* operator_ne */ {
        // Source: drake/common/sha256.h
        const char* doc = R"""()""";
      } operator_ne;
      // Symbol: drake::Sha256::operator<
      struct /* operator_lt */ {
        // Source: drake/common/sha256.h
        const char* doc = R"""()""";
      } operator_lt;
      // Symbol: drake::Sha256::to_string
      struct /* to_string */ {
        // Source: drake/common/sha256.h
        const char* doc =
R"""(Returns the 64-character ASCII hex representation of this checksum.)""";
      } to_string;
    } Sha256;
    // Symbol: drake::SortedPair
    struct /* SortedPair */ {
      // Source: drake/common/sorted_pair.h
      const char* doc =
R"""(This class is similar to the std::pair class. However, this class uses
a pair of homogeneous types (std::pair can use heterogeneous types)
and sorts the first and second values such that the first value is
less than or equal to the second one). Note that the sort is a stable
one. Thus the SortedPair class is able to be used to generate keys
(e.g., for std::map, etc.) from pairs of objects.

The availability of construction and assignment operations (i.e.,
default constructor, copy constructor, copy assignment, move
constructor, move assignment) is the same as whatever T provides . All
comparison operations (including equality, etc.) are always available.

To format this class for logging, include ``<fmt/ranges.h>`` (exactly
the same as for ``std::pair``).

Template parameter ``T``:
    A template type that provides ``operator<``.)""";
      // Symbol: drake::SortedPair::SortedPair<T>
      struct /* SortedPair */ {
        // Source: drake/common/sorted_pair.h
        const char* doc_was_unable_to_choose_unambiguous_names = R"""()""";
      } SortedPair;
      // Symbol: drake::SortedPair::Swap
      struct /* Swap */ {
        // Source: drake/common/sorted_pair.h
        const char* doc = R"""(Swaps ``this`` and ``t``.)""";
      } Swap;
      // Symbol: drake::SortedPair::first
      struct /* first */ {
        // Source: drake/common/sorted_pair.h
        const char* doc =
R"""(Gets the first (according to ``operator<``) of the objects.)""";
      } first;
      // Symbol: drake::SortedPair::get
      struct /* get */ {
        // Source: drake/common/sorted_pair.h
        const char* doc = R"""()""";
      } get;
      // Symbol: drake::SortedPair::second
      struct /* second */ {
        // Source: drake/common/sorted_pair.h
        const char* doc =
R"""(Gets the second (according to ``operator<``) of the objects.)""";
      } second;
      // Symbol: drake::SortedPair::set
      struct /* set */ {
        // Source: drake/common/sorted_pair.h
        const char* doc = R"""(Resets the stored objects.)""";
      } set;
    } SortedPair;
    // Symbol: drake::SteadyTimer
    struct /* SteadyTimer */ {
      // Source: drake/common/timer.h
      const char* doc =
R"""(Implementation of timing utility that uses monotonic
std::chrono::steady_clock.)""";
      // Symbol: drake::SteadyTimer::Start
      struct /* Start */ {
        // Source: drake/common/timer.h
        const char* doc = R"""()""";
      } Start;
      // Symbol: drake::SteadyTimer::SteadyTimer
      struct /* ctor */ {
        // Source: drake/common/timer.h
        const char* doc = R"""()""";
      } ctor;
      // Symbol: drake::SteadyTimer::Tick
      struct /* Tick */ {
        // Source: drake/common/timer.h
        const char* doc = R"""()""";
      } Tick;
    } SteadyTimer;
    // Symbol: drake::StrideX
    struct /* StrideX */ {
      // Source: drake/common/eigen_types.h
      const char* doc = R"""(A fully dynamic Eigen stride.)""";
    } StrideX;
    // Symbol: drake::Timer
    struct /* Timer */ {
      // Source: drake/common/timer.h
      const char* doc = R"""(Abstract base class for timing utility.)""";
      // Symbol: drake::Timer::Start
      struct /* Start */ {
        // Source: drake/common/timer.h
        const char* doc =
R"""(Begins timing. Call Start every time you want to reset the timer to
zero.)""";
      } Start;
      // Symbol: drake::Timer::Tick
      struct /* Tick */ {
        // Source: drake/common/timer.h
        const char* doc =
R"""(Obtains a timer measurement in seconds. Call this repeatedly to get
multiple measurements.

Returns:
    the amount of time since the timer started.)""";
      } Tick;
      // Symbol: drake::Timer::Timer
      struct /* ctor */ {
        // Source: drake/common/timer.h
        const char* doc =
R"""(Properly implemented Timers must start timing upon construction.)""";
      } ctor;
    } Timer;
    // Symbol: drake::ToleranceType
    struct /* ToleranceType */ {
      // Source: drake/common/constants.h
      const char* doc = R"""()""";
      // Symbol: drake::ToleranceType::kAbsolute
      struct /* kAbsolute */ {
        // Source: drake/common/constants.h
        const char* doc = R"""()""";
      } kAbsolute;
      // Symbol: drake::ToleranceType::kRelative
      struct /* kRelative */ {
        // Source: drake/common/constants.h
        const char* doc = R"""()""";
      } kRelative;
    } ToleranceType;
    // Symbol: drake::TypeSafeIndex
    struct /* TypeSafeIndex */ {
      // Source: drake/common/type_safe_index.h
      const char* doc =
R"""(A type-safe non-negative index class.

Note:
    This is *purposely* a separate class from Identifier. For more
    information, see TypeSafeIndexVsIdentifier "this section".

This class serves as an upgrade to the standard practice of passing
`int`s around as indices. In the common practice, a method that takes
indices into multiple collections would have an interface like:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    void foo(int bar_index, int thing_index);

.. raw:: html

    </details>

It is possible for a programmer to accidentally switch the two index
values in an invocation. This mistake would still be *syntactically*
correct; it will successfully compile but lead to inscrutable run-time
errors. The type-safe index provides the same speed and efficiency of
passing `int`s, but provides compile-time checking. The function would
now look like:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    void foo(BarIndex bar_index, ThingIndex thing_index);

.. raw:: html

    </details>

and the compiler will catch instances where the order is reversed.

The type-safe index is a *stripped down* ``int``. Each uniquely
declared index type has the following properties:

- Valid index values are *explicitly* constructed from ``int`` values.
- The index is implicitly convertible to an ``int`` (to serve as an index).
- The index supports increment, decrement, and in-place addition and
  subtraction to support standard index-like operations.
- An index *cannot* be constructed or compared to an index of another
  type.
- In general, indices of different types are *not* interconvertible.
- Binary integer operators (e.g., +, -, |, *, etc.) *always* produce ``int``
  return values. One can even use operands of different index types in
  such a binary expression. It is the *programmer's* responsibility to
  confirm that the resultant ``int`` value has meaning.

While there *is* the concept of an "invalid" index, this only exists
to support default construction *where appropriate* (e.g., using
indices in STL containers). Using an invalid index in *any* operation
is considered an error. In Debug build, attempts to compare,
increment, decrement, etc. an invalid index will throw an exception.

A function that returns TypeSafeIndex values which need to communicate
failure should *not* use an invalid index. It should return an
``std::optional<Index>`` instead.

It is the designed intent of this class, that indices derived from
this class can be passed and returned by value. (Drake's typical
calling convention requires passing input arguments by const
reference, or by value when moved from. That convention does not apply
to this class.)

This is the recommended method to create a unique index type
associated with class ``Foo``:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    using FooIndex = TypeSafeIndex<class FooTag>;

.. raw:: html

    </details>

This references a non-existent, and ultimately anonymous, class
``FooTag``. This is sufficient to create a unique index type. It is
certainly possible to use an existing class (e.g., ``Foo``). But this
provides no functional benefit.

**Construction from integral types**

C++ will do `implicit integer conversions
<https://en.cppreference.com/w/cpp/language/implicit_conversion#Integral_conversions>`_.
This allows construction of TypeSafeIndex values with arbitrary
integral types. Index values must lie in the range of [0, 2³¹). The
constructor will validate the input value (in Debug mode). Ultimately,
the caller is responsible for confirming that the values provided lie
in the valid range.

**Examples of valid and invalid operations**

The TypeSafeIndex guarantees that index instances of different types
can't be compared or combined. Efforts to do so will cause a
compile-time failure. However, comparisons or operations on *other*
types that are convertible to an int will succeed. For example:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    using AIndex = TypeSafeIndex<class A>;
       using BIndex = TypeSafeIndex<class B>;
       AIndex a(1);
       BIndex b(1);
       if (a == 2) { ... }      // Ok.
       size_t sz = 7;
       if (a == sz) { ... }     // Ok.
       if (a == b) { ... }      // <-- Compiler error.
       AIndex invalid;          // Creates an invalid index.
       ++invalid;               // Runtime error in Debug build.

.. raw:: html

    </details>

As previously stated, the intent of this class is to seamlessly serve
as an index into indexed objects (e.g., vector, array, etc.). At the
same time, we want to avoid implicit conversions *from* int to an
index. These two design constraints combined lead to a limitation in
how TypeSafeIndex instances can be used. Specifically, we've lost a
common index pattern:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    for (MyIndex a = 0; a < N; ++a) { ... }

.. raw:: html

    </details>

This pattern no longer works because it requires implicit conversion
of int to TypeSafeIndex. Instead, the following pattern needs to be
used:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    for (MyIndex a(0); a < N; ++a) { ... }

.. raw:: html

    </details>

**Use with Eigen**

At the time of this writing when using the latest Eigen 3.4 preview
branch, a TypeSafeIndex cannot be directly used to index into an
Eigen::Matrix; the developer must explicitly introduce the ``int``
conversion:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    VectorXd some_vector = ...;
       FooIndex foo_index = ...;
       some_vector(foo_index) = 0.0;       // Fails to compile.
       some_vector(int{foo_index}) = 0.0;  // Compiles OK.

.. raw:: html

    </details>

TODO(#15354) We hope to fix this irregularity in the future.

See also:
    drake::geometry::Identifier

Template parameter ``Tag``:
    The name of the tag associated with a class type. The class need
    not be a defined class.)""";
      // Symbol: drake::TypeSafeIndex::TypeSafeIndex<Tag>
      struct /* ctor */ {
        // Source: drake/common/type_safe_index.h
        const char* doc_0args =
R"""(Default constructor; the result is an *invalid* index. This only
exists to serve applications which require a default constructor.)""";
        // Source: drake/common/type_safe_index.h
        const char* doc_1args_index =
R"""(Construction from a non-negative ``int`` value. The value must lie in
the range of [0, 2³¹). Constructor only promises to test validity in
Debug build.)""";
        // Source: drake/common/type_safe_index.h
        const char* doc_1args_constTypeSafeIndex =
R"""(Disallow construction from another index type.)""";
      } ctor;
      // Symbol: drake::TypeSafeIndex::is_valid
      struct /* is_valid */ {
        // Source: drake/common/type_safe_index.h
        const char* doc =
R"""(Reports if the index is valid--the only operation on an invalid index
that doesn't throw an exception in Debug builds.)""";
      } is_valid;
      // Symbol: drake::TypeSafeIndex::operator int
      struct /* operator_int */ {
        // Source: drake/common/type_safe_index.h
        const char* doc = R"""(Implicit conversion-to-int operator.)""";
      } operator_int;
      // Symbol: drake::TypeSafeIndex::operator!=
      struct /* operator_ne */ {
        // Source: drake/common/type_safe_index.h
        const char* doc_1args_other =
R"""(Allow inequality test with indices of this tag.)""";
        // Source: drake/common/type_safe_index.h
        const char* doc_1args_constU =
R"""(Allow inequality test with unsigned integers.)""";
        // Source: drake/common/type_safe_index.h
        const char* doc_1args_constTypeSafeIndex =
R"""(Prevent inequality test with indices of other tags.)""";
      } operator_ne;
      // Symbol: drake::TypeSafeIndex::operator++
      struct /* operator_inc */ {
        // Source: drake/common/type_safe_index.h
        const char* doc_0args = R"""(Prefix increment operator.)""";
        // Source: drake/common/type_safe_index.h
        const char* doc_1args = R"""(Postfix increment operator.)""";
      } operator_inc;
      // Symbol: drake::TypeSafeIndex::operator+=
      struct /* operator_iadd */ {
        // Source: drake/common/type_safe_index.h
        const char* doc_1args_i =
R"""(Addition assignment operator. In Debug builds, this method asserts
that the resulting index is non-negative.)""";
        // Source: drake/common/type_safe_index.h
        const char* doc_1args_other =
R"""(Allow addition for indices with the same tag.)""";
        // Source: drake/common/type_safe_index.h
        const char* doc_1args_constTypeSafeIndex =
R"""(Prevent addition for indices of different tags.)""";
      } operator_iadd;
      // Symbol: drake::TypeSafeIndex::operator--
      struct /* operator_dec */ {
        // Source: drake/common/type_safe_index.h
        const char* doc_0args =
R"""(Prefix decrement operator. In Debug builds, this method asserts that
the resulting index is non-negative.)""";
        // Source: drake/common/type_safe_index.h
        const char* doc_1args =
R"""(Postfix decrement operator. In Debug builds, this method asserts that
the resulting index is non-negative.)""";
      } operator_dec;
      // Symbol: drake::TypeSafeIndex::operator-=
      struct /* operator_isub */ {
        // Source: drake/common/type_safe_index.h
        const char* doc_1args_i =
R"""(Subtraction assignment operator. In Debug builds, this method asserts
that the resulting index is non-negative.)""";
        // Source: drake/common/type_safe_index.h
        const char* doc_1args_other =
R"""(Allow subtraction for indices with the same tag.)""";
        // Source: drake/common/type_safe_index.h
        const char* doc_1args_constTypeSafeIndex =
R"""(Prevent subtraction for indices of different tags.)""";
      } operator_isub;
      // Symbol: drake::TypeSafeIndex::operator<
      struct /* operator_lt */ {
        // Source: drake/common/type_safe_index.h
        const char* doc_1args_other =
R"""(Allow less than test with indices of this tag.)""";
        // Source: drake/common/type_safe_index.h
        const char* doc_1args_constU =
R"""(Allow less than test with unsigned integers.)""";
        // Source: drake/common/type_safe_index.h
        const char* doc_1args_constTypeSafeIndex =
R"""(Prevent less than test with indices of other tags.)""";
      } operator_lt;
      // Symbol: drake::TypeSafeIndex::operator<=
      struct /* operator_le */ {
        // Source: drake/common/type_safe_index.h
        const char* doc_1args_other =
R"""(Allow less than or equals test with indices of this tag.)""";
        // Source: drake/common/type_safe_index.h
        const char* doc_1args_constU =
R"""(Allow less than or equals test with unsigned integers.)""";
        // Source: drake/common/type_safe_index.h
        const char* doc_1args_constTypeSafeIndex =
R"""(Prevent less than or equals test with indices of other tags.)""";
      } operator_le;
      // Symbol: drake::TypeSafeIndex::operator>
      struct /* operator_gt */ {
        // Source: drake/common/type_safe_index.h
        const char* doc_1args_other =
R"""(Allow greater than test with indices of this tag.)""";
        // Source: drake/common/type_safe_index.h
        const char* doc_1args_constU =
R"""(Allow greater than test with unsigned integers.)""";
        // Source: drake/common/type_safe_index.h
        const char* doc_1args_constTypeSafeIndex =
R"""(Prevent greater than test with indices of other tags.)""";
      } operator_gt;
      // Symbol: drake::TypeSafeIndex::operator>=
      struct /* operator_ge */ {
        // Source: drake/common/type_safe_index.h
        const char* doc_1args_other =
R"""(Allow greater than or equals test with indices of this tag.)""";
        // Source: drake/common/type_safe_index.h
        const char* doc_1args_constU =
R"""(Allow greater than or equals test with unsigned integers.)""";
        // Source: drake/common/type_safe_index.h
        const char* doc_1args_constTypeSafeIndex =
R"""(Prevent greater than or equals test with indices of other tags.)""";
      } operator_ge;
    } TypeSafeIndex;
    // Symbol: drake::Value
    struct /* Value */ {
      // Source: drake/common/value.h
      const char* doc =
R"""(A container class for an arbitrary type T (with some restrictions).
This class inherits from AbstractValue and therefore at runtime can be
passed between functions without mentioning T.

Example:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    void print_string(const AbstractValue& arg) {
      const std::string& message = arg.get_value<std::string>();
      std::cerr << message;
    }
    void meow() {
      const Value<std::string> value("meow");
      print_string(value);
    }

.. raw:: html

    </details>

(Advanced.) User-defined classes with additional features may subclass
Value, but should take care to override Clone().

Template parameter ``T``:
    Must be copy-constructible or cloneable. Must not be a pointer,
    array, nor have const, volatile, or reference qualifiers.)""";
      // Symbol: drake::Value::Clone
      struct /* Clone */ {
        // Source: drake/common/value.h
        const char* doc = R"""()""";
      } Clone;
      // Symbol: drake::Value::SetFrom
      struct /* SetFrom */ {
        // Source: drake/common/value.h
        const char* doc = R"""()""";
      } SetFrom;
      // Symbol: drake::Value::Value<T>
      struct /* ctor */ {
        // Source: drake/common/value.h
        const char* doc_was_unable_to_choose_unambiguous_names = R"""()""";
      } ctor;
      // Symbol: drake::Value::get_mutable_value
      struct /* get_mutable_value */ {
        // Source: drake/common/value.h
        const char* doc =
R"""(Returns a mutable reference to the stored value. The reference remains
valid only until this object is set or destroyed.)""";
      } get_mutable_value;
      // Symbol: drake::Value::get_value
      struct /* get_value */ {
        // Source: drake/common/value.h
        const char* doc =
R"""(Returns a const reference to the stored value. The reference remains
valid only until this object is set or destroyed.)""";
      } get_value;
      // Symbol: drake::Value::set_value
      struct /* set_value */ {
        // Source: drake/common/value.h
        const char* doc =
R"""(Replaces the stored value with a new one.)""";
      } set_value;
      // Symbol: drake::Value::static_type_info
      struct /* static_type_info */ {
        // Source: drake/common/value.h
        const char* doc = R"""()""";
      } static_type_info;
      // Symbol: drake::Value::type_info
      struct /* type_info */ {
        // Source: drake/common/value.h
        const char* doc = R"""()""";
      } type_info;
    } Value;
    // Symbol: drake::Vector1d
    struct /* Vector1d */ {
      // Source: drake/common/eigen_types.h
      const char* doc = R"""(A column vector of size 1 of doubles.)""";
    } Vector1d;
    // Symbol: drake::Vector6d
    struct /* Vector6d */ {
      // Source: drake/common/eigen_types.h
      const char* doc = R"""(A column vector of size 6 of doubles.)""";
    } Vector6d;
    // Symbol: drake::VectorXPoly
    struct /* VectorXPoly */ {
      // Source: drake/common/polynomial.h
      const char* doc =
R"""(A column vector of polynomials; used in several optimization classes.)""";
    } VectorXPoly;
    // Symbol: drake::all
    struct /* all */ {
      // Source: drake/common/drake_bool.h
      const char* doc =
R"""(Checks truth for all elements in matrix ``m``. This is identical to
``Eigen::DenseBase::all()``, except this function allows for lazy
evaluation, so works even when scalar_predicate<>::is_bool does not
hold. An empty matrix returns true.)""";
    } all;
    // Symbol: drake::all_of
    struct /* all_of */ {
      // Source: drake/common/drake_bool.h
      const char* doc =
R"""(Checks if unary predicate ``pred`` holds for all elements in the
matrix ``m``. An empty matrix returns true.)""";
    } all_of;
    // Symbol: drake::any
    struct /* any */ {
      // Source: drake/common/drake_bool.h
      const char* doc =
R"""(Checks truth for at least one element in matrix ``m``. This is
identical to ``Eigen::DenseBase::any()``, except this function allows
for lazy evaluation, so works even when scalar_predicate<>::is_bool
does not hold. An empty matrix returns false.)""";
    } any;
    // Symbol: drake::any_of
    struct /* any_of */ {
      // Source: drake/common/drake_bool.h
      const char* doc =
R"""(Checks if unary predicate ``pred`` holds for at least one element in
the matrix ``m``. An empty matrix returns false.)""";
    } any_of;
    // Symbol: drake::assert
    struct /* assert */ {
      // Symbol: drake::assert::ConditionTraits
      struct /* ConditionTraits */ {
        // Source: drake/common/drake_assert.h
        const char* doc = R"""()""";
        // Symbol: drake::assert::ConditionTraits::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/drake_assert.h
          const char* doc = R"""()""";
        } Evaluate;
      } ConditionTraits;
    } assert;
    // Symbol: drake::cond
    struct /* cond */ {
      // Source: drake/common/cond.h
      const char* doc =
R"""(@name cond Constructs conditional expression (similar to Lisp's cond).


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    cond(cond_1, expr_1,
    cond_2, expr_2,
    ...,   ...,
    cond_n, expr_n,
    expr_{n+1})

.. raw:: html

    </details>

The value returned by the above cond expression is ``expr_1`` if
``cond_1`` is true; else if ``cond_2`` is true then ``expr_2``; ... ;
else if ``cond_n`` is true then ``expr_n``. If none of the conditions
are true, it returns ``expr_``{n+1}.

Note:
    This functions assumes that ``ScalarType`` provides ``operator``<
    and the type of ``f_cond`` is the type of the return type of
    ``operator<(ScalarType, ScalarType)``. For example,
    ``symbolic::Expression`` can be used as a ``ScalarType`` because
    it provides ``symbolic::Formula operator<(symbolic::Expression,
    symbolic::Expression)``.)""";
    } cond;
    // Symbol: drake::copyable_unique_ptr
    struct /* copyable_unique_ptr */ {
      // Source: drake/common/copyable_unique_ptr.h
      const char* doc =
R"""(A smart pointer with deep copy semantics.

This is *similar* to ``std::unique_ptr`` in that it does not permit
shared ownership of the contained object. However, unlike
``std::unique_ptr``, copyable_unique_ptr supports copy and assignment
operations, by insisting that the contained object be "copyable". To
be copyable, the class must have either an accessible copy
constructor, or it must have an accessible clone method with signature


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    std::unique_ptr<Foo> Clone() const;

.. raw:: html

    </details>

where Foo is the type of the managed object. By "accessible" we mean
either that the copy constructor or clone method is public, or
``friend copyable_unique_ptr<Foo>;`` appears in Foo's class
declaration.

Generally, the API is modeled as closely as possible on the C++
standard ``std::unique_ptr`` API and copyable_unique_ptr is
interoperable with ``unique_ptr<T>`` wherever that makes sense.
However, there are some differences:

1. It always uses a default deleter.
2. There is no array version.
3. To allow for future copy-on-write optimizations, there is a distinction
between writable and const access, the get() method is modified to return
only a const pointer, with get_mutable() added to return a writable pointer.
Furthermore, dereferencing (operator*()) a mutable pointer will give a
mutable reference (in so far as T is not declared const), and dereferencing
a const pointer will give a const reference.

This class is entirely inline and has no computational or space
overhead except when copying is required; it contains just a single
pointer and does no reference counting.

**Usage**

In the simplest use case, the instantiation type will match the type
of object it references, e.g.:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    copyable_unique_ptr<Foo> ptr = make_unique<Foo>(...);

.. raw:: html

    </details>

In this case, as long ``Foo`` is deemed compatible, the behavior will
be as expected, i.e., when ``ptr`` copies, it will contain a reference
to a new instance of ``Foo``.

copyable_unique_ptr can also be used with polymorphic classes -- a
copyable_unique_ptr, instantiated on a *base* class, references an
instance of a *derived* class. When copying the object, we would want
the copy to likewise contain an instance of the derived class. For
example:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    copyable_unique_ptr<Base> cu_ptr = make_unique<Derived>();
    copyable_unique_ptr<Base> other_cu_ptr = cu_ptr;           // Triggers a copy.
    is_dynamic_castable<Derived>(other_cu_ptr.get());          // Should be true.

.. raw:: html

    </details>

This works for well-designed polymorphic classes.

Warning:
    Ill-formed polymorphic classes can lead to fatal type slicing of
    the referenced object, such that the new copy contains an instance
    of ``Base`` instead of ``Derived``. Some mistakes that would lead
    to this degenerate behavior:

- The ``Base`` class's Clone() implementation does not invoke the ``Derived``
class's implementation of a suitable virtual method.

Template parameter ``T``:
    The type of the contained object, which *must* be copyable as
    defined above. May be an abstract or concrete type.)""";
      // Symbol: drake::copyable_unique_ptr::copyable_unique_ptr<T>
      struct /* ctor */ {
        // Source: drake/common/copyable_unique_ptr.h
        const char* doc_0args =
R"""(Default constructor stores a ``nullptr``. No heap allocation is
performed. The empty() method will return true when called on a
default-constructed copyable_unique_ptr.)""";
        // Source: drake/common/copyable_unique_ptr.h
        const char* doc_1args_raw =
R"""(Given a raw pointer to a writable heap-allocated object, take over
ownership of that object. No copying occurs.)""";
        // Source: drake/common/copyable_unique_ptr.h
        const char* doc_1args_value =
R"""(Constructs a unique instance of T as a copy of the provided model
value.)""";
        // Source: drake/common/copyable_unique_ptr.h
        const char* doc_copy =
R"""(Copy constructor is deep; the new copyable_unique_ptr object contains
a new copy of the object in the source, created via the source
object's copy constructor or ``Clone()`` method. If the source
container is empty this one will be empty also.)""";
        // Source: drake/common/copyable_unique_ptr.h
        const char* doc_1args_conststduniqueptr =
R"""(Copy constructor from a standard ``unique_ptr`` of *compatible* type.
The copy is deep; the new copyable_unique_ptr object contains a new
copy of the object in the source, created via the source object's copy
constructor or ``Clone()`` method. If the source container is empty
this one will be empty also.)""";
        // Source: drake/common/copyable_unique_ptr.h
        const char* doc_move =
R"""(Move constructor is very fast and leaves the source empty. Ownership
is transferred from the source to the new copyable_unique_ptr. If the
source was empty this one will be empty also. No heap activity occurs.)""";
        // Source: drake/common/copyable_unique_ptr.h
        const char* doc_1args_u_ptr =
R"""(Move constructor from a standard ``unique_ptr``. The move is very fast
and leaves the source empty. Ownership is transferred from the source
to the new copyable_unique_ptr. If the source was empty this one will
be empty also. No heap activity occurs.)""";
        // Source: drake/common/copyable_unique_ptr.h
        const char* doc_1args_stduniqueptr =
R"""(Move construction from a compatible standard ``unique_ptr``. Type
``U*`` must be implicitly convertible to type ``T*``. Ownership is
transferred from the source to the new copyable_unique_ptr. If the
source was empty this one will be empty also. No heap activity occurs.)""";
      } ctor;
      // Symbol: drake::copyable_unique_ptr::empty
      struct /* empty */ {
        // Source: drake/common/copyable_unique_ptr.h
        const char* doc =
R"""(Return true if this container is empty, which is the state the
container is in immediately after default construction and various
other operations.)""";
      } empty;
      // Symbol: drake::copyable_unique_ptr::get
      struct /* get */ {
        // Source: drake/common/copyable_unique_ptr.h
        const char* doc =
R"""(Return a const pointer to the contained object if any, or ``nullptr``.
Note that this is different than ``%get()`` for the standard smart
pointers like ``std::unique_ptr`` which return a writable pointer. Use
get_mutable() here for that purpose.)""";
      } get;
      // Symbol: drake::copyable_unique_ptr::get_mutable
      struct /* get_mutable */ {
        // Source: drake/common/copyable_unique_ptr.h
        const char* doc =
R"""(Return a writable pointer to the contained object if any, or
``nullptr``. Note that you need write access to this container in
order to get write access to the object it contains.

Warning:
    If copyable_unique_ptr is instantiated on a const template
    parameter (e.g., ``copyable_unique_ptr<const Foo>``), then
    get_mutable() returns a const pointer.)""";
      } get_mutable;
      // Symbol: drake::copyable_unique_ptr::operator*
      struct /* operator_mul */ {
        // Source: drake/common/copyable_unique_ptr.h
        const char* doc_0args_const =
R"""(Return a const reference to the contained object. Note that this is
different from ``std::unique_ptr::operator*()`` which would return a
non-const reference (if ``T`` is non-const), even if the container
itself is const. For a const copyable_unique_ptr will always return a
const reference to its contained value.

Warning:
    Currently copyable_unique_ptr is a std::unique_ptr. As such, a
    const copyable_unique_ptr<Foo> can be upcast to a const
    unique_ptr<Foo> and the parent's behavior will provide a mutable
    reference. This is strongly discouraged and will break as the
    implementation of this class changes to shore up this gap in the
    const correctness protection.

Precondition:
    ``this != nullptr`` reports ``True``.)""";
        // Source: drake/common/copyable_unique_ptr.h
        const char* doc_0args_nonconst =
R"""(Return a writable reference to the contained object (if T is itself
not const). Note that you need write access to this container in order
to get write access to the object it contains.

We *strongly* recommend, that, if dereferencing a copyable_unique_ptr
without the intention of mutating the underlying value, prefer to
dereference a *const* copyable_unique_ptr (or use *my_ptr.get()) and
not a mutable copyable_unique_ptr. As "copy-on-write" behavior is
introduced in the future, this recommended practice will prevent
unwanted copies of the underlying value.

If copyable_unique_ptr is instantiated on a const template parameter
(e.g., ``copyable_unique_ptr<const Foo>``), then operator*() must
return a const reference.

Precondition:
    ``this != nullptr`` reports ``True``.)""";
      } operator_mul;
    } copyable_unique_ptr;
    // Symbol: drake::dummy_value
    struct /* dummy_value */ {
      // Source: drake/common/dummy_value.h
      const char* doc =
R"""(Provides a "dummy" value for a ScalarType -- a value that is unlikely
to be mistaken for a purposefully-computed value, useful for
initializing a value before the true result is available.

Defaults to using std::numeric_limits::quiet_NaN when available; it is
a compile-time error to call the unspecialized dummy_value::get() when
quiet_NaN is unavailable.

See autodiff_overloads.h to use this with Eigen's AutoDiffScalar.)""";
      // Symbol: drake::dummy_value::get
      struct /* get */ {
        // Source: drake/common/dummy_value.h
        const char* doc = R"""()""";
      } get;
    } dummy_value;
    // Symbol: drake::dynamic_pointer_cast
    struct /* dynamic_pointer_cast */ {
      // Source: drake/common/pointer_cast.h
      const char* doc =
R"""(Casts the object owned by the std::unique_ptr ``other`` from type
``U`` to ``T``; if the cast fails, returns nullptr. Casting is
performed using ``dynamic_cast`` on the managed value (i.e., the
result of ``other.get()``). On success, ``other`'s managed value is
transferred to the result and `other`` is empty; on failure, ``other``
will retain its original managed value and the result is empty. As
with ``dynamic_cast``, casting nullptr to anything always succeeds, so
a nullptr result could indicate either that the argument was nullptr
or that the cast failed.

This method is analogous to the built-in std::dynamic_pointer_cast
that operates on a std::shared_ptr.

Note that this function only supports default deleters.)""";
    } dynamic_pointer_cast;
    // Symbol: drake::dynamic_pointer_cast_or_throw
    struct /* dynamic_pointer_cast_or_throw */ {
      // Source: drake/common/pointer_cast.h
      const char* doc_1args_stduniqueptr =
R"""(Casts the object owned by the std::unique_ptr ``other`` from type
``U`` to ``T``; if ``other`` is nullptr or the cast fails, throws a
RuntimeError. Casting is performed using ``dynamic_cast`` on the
managed value (i.e., the result of ``other.get()``). On success,
``other`'s managed value is transferred to the result and `other`` is
empty; on failure, ``other`` will retain its original managed value.

Raises:
    RuntimeError if the cast fails.

Note that this function only supports default deleters.)""";
      // Source: drake/common/pointer_cast.h
      const char* doc_1args_U =
R"""(Casts the pointer ``other`` from type ``U`` to ``T`` using
``dynamic_cast``. The result is never nullptr.

This differs from the C++ built-in dynamic_cast by providing a nicer
exception message, and always throwing on any failure.

Raises:
    RuntimeError if ``other`` is nullptr or the cast fails.)""";
    } dynamic_pointer_cast_or_throw;
    // Symbol: drake::fmt_debug_string
    struct /* fmt_debug_string */ {
      // Source: drake/common/fmt.h
      const char* doc =
R"""(Returns ``fmt::("{:?}", x)``, i.e, using fmt's "debug string format";
see https://fmt.dev docs for the '?' presentation type for details. We
provide this wrapper because not all of our supported platforms have a
new-enough fmt to rely on it. On platforms with older fmt, we use a
Drake re-implementation of the feature that does NOT handle unicode
correctly.)""";
    } fmt_debug_string;
    // Symbol: drake::fmt_eigen
    struct /* fmt_eigen */ {
      // Source: drake/common/fmt_eigen.h
      const char* doc =
R"""(When passing an Eigen::Matrix to fmt, use this wrapper function to
instruct fmt to use Drake's custom formatter for Eigen types.

Within Drake, when formatting an Eigen matrix into a string you must
wrap the Eigen object as ``fmt_eigen(M)``. This holds true whether it
be for logging, error messages, debugging, or etc.

For example:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    if (!CheckValid(M)) {
    throw RuntimeError(fmt::format("Invalid M = {}", fmt_eigen(M)));
    }

.. raw:: html

    </details>

Warning:
    The return value of this function should only ever be used as a
    temporary object, i.e., in a fmt argument list or a logging
    statement argument list. Never store it as a local variable,
    member field, etc.

Note:
    To ensure floating-point data is formatted without losing any
    digits, Drake's code is compiled using -DEIGEN_NO_IO, which
    enforces that nothing within Drake is allowed to use Eigen's
    ``operator<<``. Downstream code that calls into Drake is not
    required to use that option; it is only enforced by Drake's build
    system, not by Drake's headers.)""";
    } fmt_eigen;
    // Symbol: drake::fmt_floating_point
    struct /* fmt_floating_point */ {
      // Source: drake/common/fmt.h
      const char* doc =
R"""(Returns ``fmt::to_string(x)`` but always with at least one digit after
the decimal point. Different versions of fmt disagree on whether to
omit the trailing ".0" when formatting integer-valued floating-point
numbers.

Template parameter ``T``:
    must be either ``float`` or ``double``.)""";
    } fmt_floating_point;
    // Symbol: drake::fmt_runtime
    struct /* fmt_runtime */ {
      // Source: drake/common/fmt.h
      const char* doc =
R"""(When using fmt >= 8, this is an alias for <a
href="https://fmt.dev/latest/api.html#compile-time-format-string-checks">fmt::runtime</a>.
When using fmt < 8, this is a no-op.)""";
    } fmt_runtime;
    // Symbol: drake::fmt_streamed
    struct /* fmt_streamed */ {
      // Source: drake/common/fmt_ostream.h
      const char* doc =
R"""(When using fmt >= 9, this is an alias for <a
href="https://fmt.dev/latest/api.html#ostream-api">fmt::streamed</a>.
When using fmt < 9, this uses a polyfill instead.

Within Drake, the nominal use for ``fmt::streamed`` is when formatting
third-party types that provide ``operator<<`` support but not
``fmt::formatter<T>`` support. Once we stop using
``FMT_DEPRECATED_OSTREAM=1``, compilation errors will help you
understand where you are required to use this wrapper.)""";
    } fmt_streamed;
    // Symbol: drake::hash_append
    struct /* hash_append */ {
      // Source: drake/common/hash.h
      const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Provides hash_append for integral constants.)""";
    } hash_append;
    // Symbol: drake::hash_append_range
    struct /* hash_append_range */ {
      // Source: drake/common/hash.h
      const char* doc =
R"""(Provides hash_append for a range, as given by two iterators.)""";
    } hash_append_range;
    // Symbol: drake::if_then_else
    struct /* if_then_else */ {
      // Source: drake/common/double_overloads.h
      const char* doc_3args_f_cond_v_then_v_else =
R"""(The semantics is similar but not exactly the same as C++'s conditional
expression constructed by its ternary operator, @c ?:. In
``if_then_else(f_cond, v_then, v_else)``, both of ``v_then`` and
``v_else`` are evaluated regardless of the evaluation of ``f_cond``.
In contrast, only one of ``v_then`` or ``v_else`` is evaluated in
C++'s conditional expression ``f_cond ? v_then : v_else``.)""";
      // Source: drake/common/drake_bool.h
      const char* doc_3args_constboolean_constEigenMatrix_constEigenMatrix =
R"""(Overloads if_then_else for Eigen vectors of ``m_then`` and ``m_else``
values with with a single ``f_cond`` condition to toggle them all at
once.)""";
    } if_then_else;
    // Symbol: drake::is_approx_equal_abstol
    struct /* is_approx_equal_abstol */ {
      // Source: drake/common/is_approx_equal_abstol.h
      const char* doc =
R"""(Returns true if and only if the two matrices are equal to within a
certain absolute elementwise ``tolerance``. Special values
(infinities, NaN, etc.) do not compare as equal elements.)""";
    } is_approx_equal_abstol;
    // Symbol: drake::is_cloneable_internal
    struct /* is_cloneable_internal */ {
      // Symbol: drake::is_cloneable_internal::is_cloneable_helper
      struct /* is_cloneable_helper */ {
        // Source: drake/common/is_cloneable.h
        const char* doc = R"""()""";
      } is_cloneable_helper;
    } is_cloneable_internal;
    // Symbol: drake::is_eigen_nonvector_of
    struct /* is_eigen_nonvector_of */ {
      // Source: drake/common/eigen_types.h
      const char* doc = R"""()""";
    } is_eigen_nonvector_of;
    // Symbol: drake::is_eigen_scalar_same
    struct /* is_eigen_scalar_same */ {
      // Source: drake/common/eigen_types.h
      const char* doc = R"""()""";
    } is_eigen_scalar_same;
    // Symbol: drake::is_eigen_type
    struct /* is_eigen_type */ {
      // Source: drake/common/eigen_types.h
      const char* doc = R"""()""";
    } is_eigen_type;
    // Symbol: drake::is_eigen_vector
    struct /* is_eigen_vector */ {
      // Source: drake/common/eigen_types.h
      const char* doc = R"""()""";
    } is_eigen_vector;
    // Symbol: drake::is_eigen_vector_of
    struct /* is_eigen_vector_of */ {
      // Source: drake/common/eigen_types.h
      const char* doc = R"""()""";
    } is_eigen_vector_of;
    // Symbol: drake::is_less_than_comparable_internal
    struct /* is_less_than_comparable_internal */ {
      // Symbol: drake::is_less_than_comparable_internal::is_less_than_comparable_helper
      struct /* is_less_than_comparable_helper */ {
        // Source: drake/common/is_less_than_comparable.h
        const char* doc = R"""()""";
      } is_less_than_comparable_helper;
    } is_less_than_comparable_internal;
    // Symbol: drake::log
    struct /* log */ {
      // Source: drake/common/text_logging.h
      const char* doc =
R"""(Retrieve an instance of a logger to use for logging; for example:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    drake::log()->info("potato!")

.. raw:: html

    </details>

See the text_logging.h documentation for a short tutorial.)""";
    } log;
    // Symbol: drake::logging
    struct /* logging */ {
      // Symbol: drake::logging::Warn
      struct /* Warn */ {
        // Source: drake/common/text_logging.h
        const char* doc =
R"""(When constructed, logs a message (at "warn" severity); the destructor
is guaranteed to be trivial. This is useful for declaring an instance
of this class as a function-static global, so that a warning is logged
the first time the program encounters some code, but does not repeat
the warning on subsequent encounters within the same process.

For example:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    double* SanityCheck(double* data) {
    if (!data) {
    static const logging::Warn log_once("Bad data!");
    return alternative_data();
    }
    return data;
    }

.. raw:: html

    </details>)""";
        // Symbol: drake::logging::Warn::Warn
        struct /* Warn */ {
          // Source: drake/common/text_logging.h
          const char* doc = R"""()""";
        } Warn;
      } Warn;
      // Symbol: drake::logging::get_dist_sink
      struct /* get_dist_sink */ {
        // Source: drake/common/text_logging_spdlog.h
        const char* doc =
R"""((Advanced) Retrieves the default sink for all Drake logs. When spdlog
is enabled, the return value can be cast to
spdlog::sinks::dist_sink_mt and thus allows consumers of Drake to
redirect Drake's text logs to locations other than the default of
stderr. When spdlog is disabled, the return value is an empty class.)""";
      } get_dist_sink;
      // Symbol: drake::logging::logger
      struct /* logger */ {
        // Source: drake/common/text_logging.h
        const char* doc =
R"""(The drake::logging::logger class provides text logging methods. See
the text_logging.h documentation for a short tutorial.)""";
      } logger;
      // Symbol: drake::logging::set_log_level
      struct /* set_log_level */ {
        // Source: drake/common/text_logging.h
        const char* doc =
R"""(Sets the log threshold used by Drake's C++ code.

Parameter ``level``:
    Must be a string from spdlog enumerations: ``trace``, `debug`,
    ``info``, `warn`, ``err``, `critical`, ``off``, or ``unchanged``
    (not an enum, but useful for command-line).

Returns:
    The string value of the previous log level. If SPDLOG is disabled,
    then this returns an empty string.)""";
      } set_log_level;
      // Symbol: drake::logging::set_log_pattern
      struct /* set_log_pattern */ {
        // Source: drake/common/text_logging.h
        const char* doc =
R"""(Invokes ``drake::log()->set_pattern(pattern)``.

Parameter ``pattern``:
    Formatting for message. For more information, see:
    https://github.com/gabime/spdlog/wiki/3.-Custom-formatting)""";
      } set_log_pattern;
    } logging;
    // Symbol: drake::never_destroyed
    struct /* never_destroyed */ {
      // Source: drake/common/never_destroyed.h
      const char* doc = R"""()""";
      // Symbol: drake::never_destroyed::access
      struct /* access */ {
        // Source: drake/common/never_destroyed.h
        const char* doc = R"""(Returns the underlying T reference.)""";
      } access;
      // Symbol: drake::never_destroyed::never_destroyed<T>
      struct /* ctor */ {
        // Source: drake/common/never_destroyed.h
        const char* doc =
R"""(Passes the constructor arguments along to T using perfect forwarding.)""";
      } ctor;
    } never_destroyed;
    // Symbol: drake::none
    struct /* none */ {
      // Source: drake/common/drake_bool.h
      const char* doc =
R"""(Checks that no elements of ``m`` are true. An empty matrix returns
true.)""";
    } none;
    // Symbol: drake::none_of
    struct /* none_of */ {
      // Source: drake/common/drake_bool.h
      const char* doc =
R"""(Checks if unary predicate ``pred`` holds for no elements in the matrix
``m``. An empty matrix returns true.)""";
    } none_of;
    // Symbol: drake::operator!=
    struct /* operator_ne */ {
      // Source: drake/common/sorted_pair.h
      const char* doc =
R"""(Determine whether two SortedPair objects are not equal using
``operator==``.)""";
    } operator_ne;
    // Symbol: drake::operator<
    struct /* operator_lt */ {
      // Source: drake/common/sorted_pair.h
      const char* doc =
R"""(Compares two pairs using lexicographic ordering.)""";
    } operator_lt;
    // Symbol: drake::operator<=
    struct /* operator_le */ {
      // Source: drake/common/sorted_pair.h
      const char* doc =
R"""(Determines whether ``x <= y`` using ``operator<``.)""";
    } operator_le;
    // Symbol: drake::operator>
    struct /* operator_gt */ {
      // Source: drake/common/sorted_pair.h
      const char* doc =
R"""(Determines whether ``x > y`` using ``operator<``.)""";
    } operator_gt;
    // Symbol: drake::operator>=
    struct /* operator_ge */ {
      // Source: drake/common/sorted_pair.h
      const char* doc =
R"""(Determines whether ``x >= y`` using ``operator<``.)""";
    } operator_ge;
    // Symbol: drake::pow
    struct /* pow */ {
      // Source: drake/common/polynomial.h
      const char* doc = R"""(Provides power function for Polynomial.)""";
    } pow;
    // Symbol: drake::reset_after_move
    struct /* reset_after_move */ {
      // Source: drake/common/reset_after_move.h
      const char* doc =
R"""(Type wrapper that performs value-initialization on the wrapped type,
and guarantees that when moving from this type that the donor object
is reset to its value-initialized value.

Background:

For performance reasons, we often like to provide overloaded move
functions on our types, instead of relying on the copy functions. When
doing so, it is more robust to rely on the compiler's ``= default``
implementation using member-wise move, instead of writing out the
operations manually. In general, move functions should reset the donor
object of the move to its default-constructed (empty) resource state.
Inductively, the member fields' existing move implementations do this
already, except in the case of non-class (primitive) members, where
the donor object's primitives will not be zeroed. By wrapping
primitive members fields with this type, they are guaranteed to be
zeroed on construction and after being moved from.

Example:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    class Foo {
     public:
      DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Foo);
      Foo() = default;
    
     private:
      std::vector<int> items_;
      reset_after_move<int> sum_;
    };

.. raw:: html

    </details>

When moving from ``Foo``, the donor object will reset to its default
state: ``items_`` will be empty and ``sum_`` will be zero. If ``Foo``
had not used the ``reset_after_move`` wrapper, the ``sum_`` would
remain intact (be copied) while moving, even though ``items_`` was
cleared.

Template parameter ``T``:
    must support CopyConstructible, CopyAssignable, MoveConstructible,
    and MoveAssignable and must not throw exceptions during
    construction or assignment.

See also:
    reset_on_copy)""";
      // Symbol: drake::reset_after_move::operator const type-parameter-0-0 &
      struct /* operator_const_T0_ */ {
        // Source: drake/common/reset_after_move.h
        const char* doc = R"""()""";
      } operator_const_T0_;
      // Symbol: drake::reset_after_move::operator type-parameter-0-0 &
      struct /* operator_T0_ */ {
        // Source: drake/common/reset_after_move.h
        const char* doc = R"""()""";
      } operator_T0_;
      // Symbol: drake::reset_after_move::operator*
      struct /* operator_mul */ {
        // Source: drake/common/reset_after_move.h
        const char* doc = R"""()""";
      } operator_mul;
      // Symbol: drake::reset_after_move::reset_after_move<T>
      struct /* ctor */ {
        // Source: drake/common/reset_after_move.h
        const char* doc_0args =
R"""(Constructs a reset_after_move<T> with a value-initialized wrapped
value. See
http://en.cppreference.com/w/cpp/language/value_initialization.)""";
        // Source: drake/common/reset_after_move.h
        const char* doc_1args =
R"""(Constructs a reset_after_move<T> with the given wrapped value. This is
an implicit conversion, so that reset_after_move<T> behaves more like
the unwrapped type.)""";
      } ctor;
      // Symbol: drake::reset_after_move::value
      struct /* value */ {
        // Source: drake/common/reset_after_move.h
        const char* doc = R"""()""";
      } value;
    } reset_after_move;
    // Symbol: drake::reset_on_copy
    struct /* reset_on_copy */ {
      // Source: drake/common/reset_on_copy.h
      const char* doc =
R"""(Type wrapper that performs value-initialization on copy construction
or assignment.

Rather than copying the source supplied for copy construction or copy
assignment, this wrapper instead value-initializes the destination
object. Move assignment and construction preserve contents in the
destination as usual, but reset the source to its value-initialized
value.

Only types T that satisfy ``std::is_scalar<T>`` are currently
permitted: integral and floating point types, enums, and pointers.
Value initialization means the initialization performed when a
variable is constructed with an empty initializer ``{}``. For the
restricted set of types we support, that just means that numeric types
are set to zero and pointer types are set to nullptr. Also, all the
methods here are noexcept due to the ``std::is_scalar<T>``
restriction. See
http://en.cppreference.com/w/cpp/language/value_initialization.

Background:

It is preferable to use default copy construction for classes whenever
possible because it avoids difficult-to-maintain enumeration of member
fields in bespoke copy constructors. The presence of fields that must
be reset to zero in the copy (counters, for example) prevents use of
default copy construction. Similarly, pointers that would be invalid
in the copy need to be set to null to avoid stale references. By
wrapping those problematic data members in this adapter, default copy
construction can continue to be used, with all data members copied
properly except the designated ones, which are value-initialized
instead. The resetting of the source on move doesn't change semantics
since the condition of the source after a move is generally undefined.
It is instead opportunistic good hygiene for early detection of bugs,
taking advantage of the fact that we know type T can be
value-initialized. See reset_after_move for more discussion.

Example:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    class Foo {
     public:
      DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Foo);
      Foo() = default;
    
     private:
      std::vector<int> items_;
      reset_on_copy<int> use_count_;
    };

.. raw:: html

    </details>

When copying from ``Foo``, the new object will contain a copy of
``items_`` but ``use_count_`` will be zero. If ``Foo`` had not used
the ``reset_on_copy`` wrapper, ``use_count_`` would have been copied
also, which we're assuming is not the desired behavior here.

Warning:
    Even if you initialize a reset_on_copy member to a non-zero value
    using an initializer like ``reset_on_copy<int> some_member_{5}``
    it will be *reset* to zero, not *reinitialized* to 5 when copied.

Note:
    Enum types T are permitted, but be aware that they will be reset
    to zero, regardless of whether 0 is one of the specified
    enumeration values.

Template parameter ``T``:
    must satisfy ``std::is_scalar<T>``.

See also:
    reset_after_move)""";
      // Symbol: drake::reset_on_copy::operator const type-parameter-0-0 &
      struct /* operator_const_T0_ */ {
        // Source: drake/common/reset_on_copy.h
        const char* doc = R"""()""";
      } operator_const_T0_;
      // Symbol: drake::reset_on_copy::operator type-parameter-0-0 &
      struct /* operator_T0_ */ {
        // Source: drake/common/reset_on_copy.h
        const char* doc = R"""()""";
      } operator_T0_;
      // Symbol: drake::reset_on_copy::operator*
      struct /* operator_mul */ {
        // Source: drake/common/reset_on_copy.h
        const char* doc = R"""()""";
      } operator_mul;
      // Symbol: drake::reset_on_copy::reset_on_copy<T>
      struct /* ctor */ {
        // Source: drake/common/reset_on_copy.h
        const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Constructs a reset_on_copy with a value-initialized wrapped value.)""";
      } ctor;
    } reset_on_copy;
    // Symbol: drake::scalar_predicate
    struct /* scalar_predicate */ {
      // Source: drake/common/drake_bool.h
      const char* doc =
R"""(A traits struct that describes the return type of predicates over a
scalar type (named ``T``). For example, a predicate that evaluates
``double`s will return a `bool``, but a predicate that evaluates
symbolic::Expression will return a symbolic::Formula. By default, the
return type is inferred from the type's comparison operator, but
scalar types are permitted to specialize this template for their
needs.)""";
      // Symbol: drake::scalar_predicate::type
      struct /* type */ {
        // Source: drake/common/drake_bool.h
        const char* doc = R"""(The return type of predicates over T.)""";
      } type;
    } scalar_predicate;
    // Symbol: drake::static_pointer_cast
    struct /* static_pointer_cast */ {
      // Source: drake/common/pointer_cast.h
      const char* doc =
R"""(Casts the object owned by the std::unique_ptr ``other`` from type
``U`` to ``T``; no runtime type checking is performed.

This method is analogous to the built-in std::static_pointer_cast that
operates on a std::shared_ptr.

Note that this function only supports default deleters.)""";
    } static_pointer_cast;
    // Symbol: drake::string_multiset
    struct /* string_multiset */ {
      // Source: drake/common/string_set.h
      const char* doc =
R"""(Like ``std::multiset<std::string>``, but with better defaults than the
plain ``std::multiset<std::string>`` spelling. We need
``std::less<void>`` as the comparison function so that
``std::string_view`` and ``const char*`` can be used as lookup keys
without copying them to a ``std::string``.)""";
    } string_multiset;
    // Symbol: drake::string_set
    struct /* string_set */ {
      // Source: drake/common/string_set.h
      const char* doc =
R"""(Like ``std::set<std::string>``, but with better defaults than the
plain ``std::set<std::string>`` spelling. We need ``std::less<void>``
as the comparison function so that ``std::string_view`` and ``const
char*`` can be used as lookup keys without copying them to a
``std::string``.)""";
    } string_set;
    // Symbol: drake::string_unordered_multiset
    struct /* string_unordered_multiset */ {
      // Source: drake/common/string_unordered_set.h
      const char* doc =
R"""(Like ``std::unordered_multiset<std::string>``, but with better
defaults than the plain ``std::unordered_multiset<std::string>``
spelling. We need the custom hash and comparison functions so that
``std::string_view`` and ``const char*`` can be used as lookup keys
without copying them to a ``std::string``.)""";
    } string_unordered_multiset;
    // Symbol: drake::string_unordered_set
    struct /* string_unordered_set */ {
      // Source: drake/common/string_unordered_set.h
      const char* doc =
R"""(Like ``std::unordered_set<std::string>``, but with better defaults
than the plain ``std::unordered_set<std::string>`` spelling. We need
the custom hash and comparison functions so that ``std::string_view``
and ``const char*`` can be used as lookup keys without copying them to
a ``std::string``.)""";
    } string_unordered_set;
    // Symbol: drake::temp_directory
    struct /* temp_directory */ {
      // Source: drake/common/temp_directory.h
      const char* doc =
R"""(Returns a directory location suitable for temporary files. The
directory will be called ${parent}/robotlocomotion_drake_XXXXXX where
each X is replaced by a character from the portable filename character
set. The path ${parent} is defined as one of the following (in
decreasing priority):

- ${TEST_TMPDIR} - ${TMPDIR} - /tmp

If successful, this will always create a new directory. While the
caller is not obliged to delete the directory, it has full power to do
so based on specific context and need.

Returns:
    The path representing a newly created directory There will be no
    trailing ``/``.

Raises:
    if the directory ${parent}/robotlocomotion_drake_XXXXXX cannot be
    created, or is not a directory.)""";
    } temp_directory;
    // Symbol: drake::to_string
    struct /* to_string */ {
      // Source: drake/common/file_source.h
      const char* doc_1args_source = R"""(Returns a string representation.)""";
      // Source: drake/common/identifier.h
      const char* doc_1args_constdrakeIdentifier =
R"""(Enables use of identifiers with to_string. It requires ADL to work.
So, it should be invoked as: ``to_string(id);`` and should be preceded
by ``using std::to_string``.)""";
    } to_string;
    // Symbol: drake::uhash
    struct /* uhash */ {
      // Source: drake/common/hash.h
      const char* doc =
R"""(A hashing functor, somewhat like ``std::hash``. Given an item of type
``T``, applies hash_append to it, directing the bytes to append into
the given ``HashAlgorithm``, and then finally returning the
algorithm's result.)""";
      // Symbol: drake::uhash::operator()
      struct /* operator_call */ {
        // Source: drake/common/hash.h
        const char* doc = R"""()""";
      } operator_call;
      // Symbol: drake::uhash::result_type
      struct /* result_type */ {
        // Source: drake/common/hash.h
        const char* doc = R"""()""";
      } result_type;
    } uhash;
    // Symbol: drake::unused
    struct /* unused */ {
      // Source: drake/common/unused.h
      const char* doc =
R"""(Documents the argument(s) as unused, placating GCC's
-Wunused-parameter warning. This can be called within function bodies
to mark that certain parameters are unused.

When possible, removing the unused parameter is better than placating
the warning. However, in some cases the parameter is part of a virtual
API or template concept that is used elsewhere, so we can't remove it.
In those cases, this function might be an appropriate work-around.

Here's rough advice on how to fix Wunused-parameter warnings:

(1) If the parameter can be removed entirely, prefer that as the first
choice. (This may not be possible if, e.g., a method must match some
virtual API or template concept.)

(2) Unless the parameter name has acute value, prefer to omit the name
of the parameter, leaving only the type, e.g.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    void Print(const State& state) override { /* No state to print. */ }

.. raw:: html

    </details>

changes to


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    void Print(const State&) override { /* No state to print. */}

.. raw:: html

    </details>

This no longer triggers the warning and further makes it clear that a
parameter required by the API is definitively unused in the function.

This is an especially good solution in the context of method
definitions (vs declarations); the parameter name used in a definition
is entirely irrelevant to Doxygen and most readers.

(3) When leaving the parameter name intact has acute value, it is
acceptable to keep the name and mark it ``unused``. For example, when
the name appears as part of a virtual method's base class declaration,
the name is used by Doxygen to document the method, e.g.,


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    /** Sets the default State of a System.  This default implementation is to
        set all zeros.  Subclasses may override to use non-zero defaults.  The
        custom defaults may be based on the given ``context``, when relevant.  
    virtual void SetDefault(const Context<T>& context, State<T>* state) const {
      unused(context);
      state->SetZero();
    }

.. raw:: html

    </details>)""";
    } unused;
  } drake;
} pydrake_doc_common;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
