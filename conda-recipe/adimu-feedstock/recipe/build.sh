#!/bin/sh

set -euo pipefail

rm -rf build
mkdir build
cd build

cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$PREFIX ..
# Make all, run tests, then install
make -j${CPU_COUNT} all VERBOSE=1
# ctest -V
make -j${CPU_COUNT} install
