#!/bin/bash
set -e

# Get current directory
SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"

# Create build folder
BUILD_FOLDER="$SCRIPT_DIR/build"

if [ ! -d $BUILD_FOLDER ] ; then
    mkdir $BUILD_FOLDER
fi

# Set Docker run base command
DOCKER_IMG_NAME=carlosgalvezp/sdcnd_p5
DOCKER_RUN_BASE="docker run --rm=true                            \
                            --volume=$SCRIPT_DIR:$SCRIPT_DIR     \
                            --workdir=$BUILD_FOLDER              \
                            --user=$UID:$GROUPS                  \
                            $DOCKER_IMG_NAME"

# Run CMake from Docker image
$DOCKER_RUN_BASE cmake -Dbuild_tests=ON ..

# Run make from Docker image
$DOCKER_RUN_BASE make
