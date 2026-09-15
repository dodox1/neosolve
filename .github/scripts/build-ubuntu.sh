#!/bin/sh -xe

# Both ways: the OCC code is most of what this fork adds, and the OCC-off build
# is what breaks when something new is used outside HAVE_OPENCASCADE.
build() {
    cmake \
      -S . -B "$1" \
      -DCMAKE_BUILD_TYPE="Debug" \
      -DENABLE_OPENMP="ON" \
      -DENABLE_SANITIZERS="ON" \
      -DUSE_OPENCASCADE="$2"
    cmake --build "$1" -j$(nproc)
}

build build-noocc OFF
build build-occ ON
# Skip visual tests on CI - rendering differs between environments
