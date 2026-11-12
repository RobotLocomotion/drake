/** @file
 Doxygen-only documentation for @ref system_multithreading. */

#pragma once

// Putting this document in drake::systems namespace makes the links work.
namespace drake {
namespace systems {

/** @defgroup system_multithreading Multithreading with Systems and Contexts
    @ingroup systems
    @brief Recommended threading patterns and memory allocation considerations.

@section system_multithreading_context_per_thread One Context per thread

Drake's recommended approach to concurrent computation is to share one fully
constructed System, such as a
@ref drake::multibody::MultibodyPlant "MultibodyPlant" or Diagram, while giving
each worker thread its own Context. Each worker can independently change its
%Context's time, state, inputs, and parameters and evaluate results using its
own cache. For a %Diagram, each worker should have a separate root %Context tree
and use subsystem contexts from that tree. Leave the shared %System unchanged
during concurrent use, and observe any class-specific thread-safety
restrictions.

@section system_multithreading_shared_context Sharing a frozen Context

A `const Context&` does not by itself permit concurrent evaluation: evaluating a
quantity can update the %Context's cache. To share a single %Context for
read-only computations, first initialize its values and, on one thread, evaluate
every quantity the readers will need. Then call ContextBase::FreezeCache() on
the root %Context before starting the readers. An evaluation requiring a cache
update will throw. Keep the %Context's values unchanged until all readers
finish; only then call ContextBase::UnfreezeCache() before making changes.
Repeat the preparation and freezing before the next reader phase. Freezing
guards cache updates; it does not lock the %Context or make its state and inputs
immutable.

Shared frozen contexts support computations that only read %Context values and
already-computed cache entries. Some `const` methods also require writable cache
storage for temporary computations and therefore cannot run with the cache
frozen. Each worker also needs private output and scratch objects wherever an
API writes to caller-provided storage.

@section system_multithreading_allocation Heap allocation and workspace

Heap allocation and deallocation are thread-safe, but allocator synchronization
can cause contention and variable execution time. In performance-critical loops,
preallocate reusable workspace, or allocate it during an initial warm-up.
%System authors should place persistent computation workspace in %Context cache
entries rather than mutable %System members or shared static buffers. This
preserves the one-%Context-per-thread approach while avoiding repeated
allocations. Preallocation alone does not make shared storage safe, and cache
freezing does not guarantee allocation-free execution. See also the <a
href="https://drake.mit.edu/styleguide/cppguide.html#Memory%20Allocation">Memory
Allocation section of the C++ style guide</a>.
*/

}  // namespace systems
}  // namespace drake
