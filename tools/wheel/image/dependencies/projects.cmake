if(NOT APPLE)
    # patchelf
    set(patchelf_version 0.12)
    set(patchelf_url "https://github.com/NixOS/patchelf/archive/${patchelf_version}/patchelf-${patchelf_version}.tar.gz")
    set(patchelf_md5 "b9d1161e52e2f342598deabf7d85ed24")
    list(APPEND ALL_PROJECTS patchelf)

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

# lz4
set(lz4_version 1.9.3)
set(lz4_url "https://github.com/lz4/lz4/archive/v${lz4_version}/lz4-${lz4_version}.tar.gz")
set(lz4_md5 "3a1ab1684e14fc1afc66228ce61b2db3")
list(APPEND ALL_PROJECTS lz4)

# xz
set(xz_version 5.2.5)
set(xz_url "https://tukaani.org/xz/xz-${xz_version}.tar.gz")
set(xz_md5 "0d270c997aff29708c74d53f599ef717")
list(APPEND ALL_PROJECTS xz)

# libjpeg-turbo
set(libjpeg-turbo_version 1.4.0)
set(libjpeg-turbo_url "http://sourceforge.net/projects/libjpeg-turbo/files/libjpeg-turbo-${libjpeg-turbo_version}.tar.gz")
set(libjpeg-turbo_md5 "039153dabe61e1ac8d9323b5522b56b0")
list(APPEND ALL_PROJECTS libjpeg-turbo)

# png
set(png_version 1.6.19)
set(png_archive_version 16)
set(png_url "http://sourceforge.net/projects/libpng/files/libpng${png_archive_version}/older-releases/${png_version}/libpng-${png_version}.tar.gz")
set(png_md5 "3121bdc77c365a87e054b9f859f421fe")
list(APPEND ALL_PROJECTS png)

# libtiff
set(libtiff_version 4.1.0)
set(libtiff_url "http://download.osgeo.org/libtiff/tiff-${libtiff_version}.tar.gz")
set(libtiff_md5 "2165e7aba557463acc0664e71a3ed424")
list(APPEND ALL_PROJECTS libtiff)

# tinyxml2
set(tinyxml2_version 7.0.1)
set(tinyxml2_url "https://github.com/leethomason/tinyxml2/archive/${tinyxml2_version}.zip")
set(tinyxml2_md5 "03ad292c4b6454702c0cc22de0d196ad")
set(tinyxml2_dlname "tinyXML2.zip")
list(APPEND ALL_PROJECTS tinyxml2)

# msgpack
set(msgpack_version 3.1.0)
set(msgpack_url "https://github.com/msgpack/msgpack-c/releases/download/cpp-${msgpack_version}/msgpack-${msgpack_version}.tar.gz")
set(msgpack_md5 "57bdba5ae83831c7c04aad39e479b225")
list(APPEND ALL_PROJECTS msgpack)

# gflags
set(gflags_version 2.2.1)
set(gflags_url "https://github.com/gflags/gflags/archive/v${gflags_version}.tar.gz")
set(gflags_md5 "b98e772b4490c84fc5a87681973f75d1")
set(gflags_dlname "gflags-${gflags_version}.tar.gz")
list(APPEND ALL_PROJECTS gflags)

# eigen
set(eigen_version 3.3.7)
set(eigen_url "https://gitlab.com/libeigen/eigen/-/archive/${eigen_version}/eigen-${eigen_version}.tar.gz")
set(eigen_md5 "9e30f67e8531477de4117506fe44669b")
set(eigen_dlname "eigen-${eigen_version}.tar.gz")
list(APPEND ALL_PROJECTS eigen)

# double-conversion
set(double-conversion_version 3.1.5)
set(double-conversion_url "https://github.com/google/double-conversion/archive/v${double-conversion_version}/double-conversion-${double-conversion_version}.tar.gz")
set(double-conversion_md5 "e94d3a33a417e692e5600e75019f0272")
list(APPEND ALL_PROJECTS double-conversion)

# lapack (blas)
set(lapack_version 3.10.0)
set(lapack_url "https://github.com/Reference-LAPACK/lapack/archive/v${lapack_version}.tar.gz")
set(lapack_md5 "d70fc27a8bdebe00481c97c728184f09")
list(APPEND ALL_PROJECTS lapack)

# suitesparse
set(suitesparse_version 4.4.5)
set(suitesparse_url "http://faculty.cse.tamu.edu/davis/SuiteSparse/SuiteSparse-${suitesparse_version}.tar.gz")
set(SuiteSparse_md5 "a2926c27f8a5285e4a10265cc68bbc18")
list(APPEND ALL_PROJECTS suitesparse)

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

if(APPLE)
    set(mumps_version 5.4.0)  # Current version in Homebrew.
    set(mumps_url "http://mumps.enseeiht.fr/MUMPS_${mumps_version}.tar.gz")
    set(mumps_md5 "808178997dc571c748e9cf0cabf9a26e")
    set(mumps_dlname "mumps-${mumps_version}.tar.gz")
    list(APPEND ALL_PROJECTS mumps)
endif()

# ipopt
set(ipopt_version 3.11.9)
set(ipopt_url "https://github.com/coin-or/Ipopt/archive/refs/tags/releases/${ipopt_version}.tar.gz")
set(ipopt_md5 "55275c202072ad30db25d2b723ef9b7a")
set(ipopt_dlname "ipopt-${ipopt_version}.tar.gz")
list(APPEND ALL_PROJECTS ipopt)
