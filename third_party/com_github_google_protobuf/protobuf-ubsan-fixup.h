#ifndef PROTOBUF_UBSAN_FIXUP_H
#define PROTOBUF_UBSAN_FIXUP_H

/*
Copyright 2014, Google Inc.  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

    * Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
copyright notice, this list of conditions and the following disclaimer
in the documentation and/or other materials provided with the
distribution.
    * Neither the name of Google Inc. nor the names of its
contributors may be used to endorse or promote products derived from
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

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
