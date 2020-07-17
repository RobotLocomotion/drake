#pragma once

#include <iostream>

namespace drake {
namespace geometry {
namespace internal {

// TODO(SeanCurtis-TRI): Update this to have an additional classification: kBoth
//  when we have the need from the algorithm. For example: when we have two
//  very stiff objects, we'd want to process them as soft. But when one
//  very stiff and one very soft object interact, it might make sense to
//  consider the stiff object as effectively rigid and simplify the computation.
//  In this case, the object would get two representations.
/* Classification of the type of representation a shape has for the
 hydroelastic contact model: rigid or soft.  */
enum class HydroelasticType { kUndefined, kRigid, kSoft };

/* Streaming operator for writing hydroelastic type to output stream.  */
std::ostream& operator<<(std::ostream& out, const HydroelasticType& type);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
