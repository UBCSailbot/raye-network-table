#!/usr/bin/env bash
set -e

# Install protobuf locally
# inside the folder lib/protobuf/cmake/build
SCRIPTDIR="$(cd "$(dirname "$0")" && pwd)"
pushd $SCRIPTDIR/../../lib/protobuf
./autogen.sh
./configure --prefix="$(pwd)/cmake/build"
make
if make check ; then
    make install
fi
popd
