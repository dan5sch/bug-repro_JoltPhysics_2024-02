#!/bin/bash

set -ue

REPO_ROOT="$(git rev-parse --show-toplevel)"
BUILD_DIR=build

cd "${REPO_ROOT}"

if [ -d "${BUILD_DIR}" ]; then
    echo "build directory '${BUILD_DIR}' already exists -- doing nothing" 1>&2
    exit 1
fi

export CC=/usr/bin/clang
export CXX=/usr/bin/clang++

mkdir "${BUILD_DIR}"
cd "${BUILD_DIR}"
cmake -DCMAKE_BUILD_TYPE=Release ..
