This include directory is necessary to prevent Catkin from using the wrong
version of gtest. Without this directory, the following linker error will occur:

    /usr/include/gtest/gtest.h:1333: undefined reference to
    `testing::internal::EqFailure(char const*, char const*,
    testing::internal::String const&, testing::internal::String const&, bool)'


This is because /usr/include/gtest/gtest.h is used instead of
`drake-distro/externals/googletest/googletest/include/gtest/gtest.h`.
