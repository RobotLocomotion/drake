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
set(xz_url "https://drake-mirror.csail.mit.edu/other/xz/xz-${xz_version}.tar.gz")
set(xz_md5 "0d270c997aff29708c74d53f599ef717")
list(APPEND ALL_PROJECTS xz)

# libjpeg-turbo
set(libjpeg-turbo_version 2.1.4)
set(libjpeg-turbo_url "https://github.com/libjpeg-turbo/libjpeg-turbo/archive/refs/tags/${libjpeg-turbo_version}.tar.gz")
set(libjpeg-turbo_md5 "357dc26a802c34387512a42697846d16")
list(APPEND ALL_PROJECTS libjpeg-turbo)

# png
set(png_version 1.6.38)
set(png_url "https://downloads.sourceforge.net/project/libpng/libpng16/${png_version}/libpng-${png_version}.tar.xz")
set(png_md5 "122e6b7837811698563083b352bc8ca2")
list(APPEND ALL_PROJECTS png)

# TODO(jwnimmer-tri) When we purge `libtiff`, we can also purge `xz`, above.
# The only user of `xz` is `libtiff`.
#
# libtiff
set(libtiff_version 4.1.0)
set(libtiff_url "http://download.osgeo.org/libtiff/tiff-${libtiff_version}.tar.gz")
set(libtiff_md5 "2165e7aba557463acc0664e71a3ed424")
list(APPEND ALL_PROJECTS libtiff)

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
