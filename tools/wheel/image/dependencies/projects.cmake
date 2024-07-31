if(NOT APPLE)
    # libxcrypt
    set(libxcrypt_version 4.4.25)
    set(libxcrypt_url "https://github.com/besser82/libxcrypt/archive/v${libxcrypt_version}/libxcrypt-${libxcrypt_version}.tar.gz")
    set(libxcrypt_md5 "4828b1530f5bf35af0b45b35acc4db1d")
    list(APPEND ALL_PROJECTS libxcrypt)
endif()

# zlib
set(zlib_version 1.2.11)
set(zlib_url "https://github.com/madler/zlib/archive/v${zlib_version}.zip")
set(zlib_md5 "9d6a627693163bbbf3f26403a3a0b0b1")
set(zlib_dlname "zlib-${zlib_version}.zip")
list(APPEND ALL_PROJECTS zlib)

# eigen
set(eigen_version 3.4.0)
set(eigen_url "https://gitlab.com/libeigen/eigen/-/archive/${eigen_version}/eigen-${eigen_version}.tar.gz")
set(eigen_md5 "4c527a9171d71a72a9d4186e65bea559")
set(eigen_dlname "eigen-${eigen_version}.tar.gz")
list(APPEND ALL_PROJECTS eigen)

# lapack (blas)
if(NOT APPLE)
  set(lapack_version 3.10.0)
  set(lapack_url "https://github.com/Reference-LAPACK/lapack/archive/v${lapack_version}.tar.gz")
  set(lapack_md5 "d70fc27a8bdebe00481c97c728184f09")
  list(APPEND ALL_PROJECTS lapack)
endif()

# ipopt (requires mumps)
if(APPLE_ARM64)
    set(mumps_version 5.4.1)  # Latest available in Ubuntu.
    set(mumps_url
        "http://archive.ubuntu.com/ubuntu/pool/universe/m/mumps/mumps_${mumps_version}.orig.tar.gz"
        "http://mumps.enseeiht.fr/MUMPS_${mumps_version}.tar.gz"
    )
    set(mumps_md5 "93be789bf9c6c341a78c16038da3241b")
    set(mumps_dlname "mumps-${mumps_version}.tar.gz")
    list(APPEND ALL_PROJECTS mumps)

    # This must match the version in tools/workspace/ipopt_internal_fromsource.
    # The matching is automatically enforced by a linter script.
    set(ipopt_version 3.14.16)
    set(ipopt_url "https://github.com/coin-or/Ipopt/archive/refs/tags/releases/${ipopt_version}.tar.gz")
    set(ipopt_md5 "f94822be08b1f6e109261f305799b0ae")
    set(ipopt_dlname "ipopt-${ipopt_version}.tar.gz")
    list(APPEND ALL_PROJECTS ipopt)
endif()
