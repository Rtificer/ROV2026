# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/artificer/Desktop/ROV2026/src/build/rov_control/_deps/qdldl-src"
  "/home/artificer/Desktop/ROV2026/src/build/rov_control/_deps/qdldl-build"
  "/home/artificer/Desktop/ROV2026/src/build/rov_control/_deps/qdldl-subbuild/qdldl-populate-prefix"
  "/home/artificer/Desktop/ROV2026/src/build/rov_control/_deps/qdldl-subbuild/qdldl-populate-prefix/tmp"
  "/home/artificer/Desktop/ROV2026/src/build/rov_control/_deps/qdldl-subbuild/qdldl-populate-prefix/src/qdldl-populate-stamp"
  "/home/artificer/Desktop/ROV2026/src/build/rov_control/_deps/qdldl-subbuild/qdldl-populate-prefix/src"
  "/home/artificer/Desktop/ROV2026/src/build/rov_control/_deps/qdldl-subbuild/qdldl-populate-prefix/src/qdldl-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/artificer/Desktop/ROV2026/src/build/rov_control/_deps/qdldl-subbuild/qdldl-populate-prefix/src/qdldl-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/artificer/Desktop/ROV2026/src/build/rov_control/_deps/qdldl-subbuild/qdldl-populate-prefix/src/qdldl-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
