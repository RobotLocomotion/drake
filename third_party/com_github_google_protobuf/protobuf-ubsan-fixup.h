#ifndef PROTOBUF_UBSAN_FIXUP_H
#define PROTOBUF_UBSAN_FIXUP_H

// This code is pulled from upstream Protobuf:
// https://github.com/google/protobuf/blob/4fc9304/src/google/protobuf/generated_message_util.h#L80
// This is necessary because Protobuf 2.6 (which Drake is currently using from
// the system) won't test cleanly under UBSan.  The way this works is that
// the drake_cc_proto_library() skylark function splices this header file into
// any generated protobuf headers, so that
// the GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET macros is redefined to
// pass UBSan.  This can be removed if the system version of protobuf is moved
// to a newer version.

#ifdef GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET
#undef GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET
#endif

// Returns the offset of the given field within the given aggregate type.
// This is equivalent to the ANSI C offsetof() macro.  However, according
// to the C++ standard, offsetof() only works on POD types, and GCC
// enforces this requirement with a warning.  In practice, this rule is
// unnecessarily strict; there is probably no compiler or platform on
// which the offsets of the direct fields of a class are non-constant.
// Fields inherited from superclasses *can* have non-constant offsets,
// but that's not what this macro will be used for.
#if defined(__clang__)
// For Clang we use __builtin_offsetof() and suppress the warning,
// to avoid Control Flow Integrity and UBSan vptr sanitizers from
// crashing while trying to validate the invalid reinterpet_casts.
#define GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(TYPE, FIELD)  \
  _Pragma("clang diagnostic push")                                   \
  _Pragma("clang diagnostic ignored \"-Winvalid-offsetof\"")         \
  __builtin_offsetof(TYPE, FIELD)                                    \
  _Pragma("clang diagnostic pop")
#else
// Note that we calculate relative to the pointer value 16 here since if we
// just use zero, GCC complains about dereferencing a NULL pointer.  We
// choose 16 rather than some other number just in case the compiler would
// be confused by an unaligned pointer.
#define GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(TYPE, FIELD)  \
  static_cast< ::google::protobuf::uint32>(                           \
      reinterpret_cast<const char*>(                                 \
          &reinterpret_cast<const TYPE*>(16)->FIELD) -               \
      reinterpret_cast<const char*>(16))
#endif

#endif
