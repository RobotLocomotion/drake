set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Keep CMAKE_CXX_FLAGS in sync with CXX_FLAGS, CLANG_FLAGS, and GCC_FLAGS in
# tools/skylark/drake_cc.bzl.
if(CMAKE_CXX_COMPILER_ID STREQUAL AppleClang OR CMAKE_CXX_COMPILER_ID STREQUAL Clang)
  set(CXX_FLAGS
    -Werror=all
    -Werror=ignored-qualifiers
    -Werror=inconsistent-missing-override
    -Werror=non-virtual-dtor
    -Werror=old-style-cast
    -Werror=overloaded-virtual
    -Werror=return-stack-address
    -Werror=shadow
    -Werror=sign-compare
  )
elseif(CMAKE_CXX_COMPILER_ID STREQUAL GNU)
  set(CXX_FLAGS
    -Werror=all
    -Werror=extra
    -Werror=ignored-qualifiers
    -Werror=logical-op
    -Werror=non-virtual-dtor
    -Werror=old-style-cast
    -Werror=overloaded-virtual
    -Werror=return-local-addr
    -Werror=unused-but-set-parameter
  )
endif()

foreach(CXX_FLAG ${CXX_FLAGS})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX_FLAG}")
endforeach()
