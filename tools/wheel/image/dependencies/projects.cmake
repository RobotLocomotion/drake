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

# bzip2
set(bzip2_version 1.0.8)
set(bzip2_url "https://sourceware.org/pub/bzip2/bzip2-${bzip2_version}.tar.gz")
set(bzip2_md5 "67e051268d0c475ea773822f7500d0e5")
list(APPEND ALL_PROJECTS bzip2)

# xz
set(xz_version 5.2.5)
set(xz_url "https://tukaani.org/xz/xz-${xz_version}.tar.gz")
set(xz_md5 "0d270c997aff29708c74d53f599ef717")
list(APPEND ALL_PROJECTS xz)

# png
set(png_version 1.6.38)
set(png_url "https://downloads.sourceforge.net/project/libpng/libpng16/${png_version}/libpng-${png_version}.tar.xz")
set(png_md5 "122e6b7837811698563083b352bc8ca2")
list(APPEND ALL_PROJECTS png)

# eigen
if(APPLE)
    # This version mimics homebrew.
    set(eigen_version 3.4.0)
    set(eigen_url "https://gitlab.com/libeigen/eigen/-/archive/${eigen_version}/eigen-${eigen_version}.tar.gz")
    set(eigen_md5 "4c527a9171d71a72a9d4186e65bea559")
else()
    # This version mimics Ubuntu 20.04 (Focal).
    set(eigen_version 3.3.7)
    set(eigen_url "https://gitlab.com/libeigen/eigen/-/archive/${eigen_version}/eigen-${eigen_version}.tar.gz")
    set(eigen_md5 "9e30f67e8531477de4117506fe44669b")
endif()
set(eigen_dlname "eigen-${eigen_version}.tar.gz")
list(APPEND ALL_PROJECTS eigen)

# lapack (blas)
set(lapack_version 3.10.0)
set(lapack_url "https://github.com/Reference-LAPACK/lapack/archive/v${lapack_version}.tar.gz")
set(lapack_md5 "d70fc27a8bdebe00481c97c728184f09")
list(APPEND ALL_PROJECTS lapack)

# coinutils
set(coinutils_version 2.11.4)
set(coinutils_url "https://github.com/coin-or/CoinUtils/archive/refs/tags/releases/${coinutils_version}.tar.gz")
set(coinutils_md5 "dcdb2a327344014de9d8b050575fa90b")
set(coinutils_dlname "coinutils-${coinutils_version}.tar.gz")
list(APPEND ALL_PROJECTS coinutils)

# clp
set(clp_version 1.17.5)
set(clp_url "https://github.com/coin-or/Clp/archive/refs/tags/releases/${clp_version}.tar.gz")
set(clp_md5 "f7c25af22d2f03398cbbdf38c8b4f6fd")
set(clp_dlname "clp-${clp_version}.tar.gz")
list(APPEND ALL_PROJECTS clp)

# ipopt (requires mumps)
if(APPLE)
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
    set(ipopt_version 3.14.12)
    set(ipopt_url "https://github.com/coin-or/Ipopt/archive/refs/tags/releases/${ipopt_version}.tar.gz")
    set(ipopt_md5 "b2bcb362be4c10eccde02829d3025faa")
    set(ipopt_dlname "ipopt-${ipopt_version}.tar.gz")
    list(APPEND ALL_PROJECTS ipopt)
endif()
