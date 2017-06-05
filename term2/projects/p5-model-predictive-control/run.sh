#!/bin/bash
set -e

# Get current directory
SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"

# Get build folder
BUILD_FOLDER="$SCRIPT_DIR/build"

if [ ! -d "$BUILD_FOLDER" ] ; then
    echo "You must build first! Use:"
    echo "$ ./build.sh"
    exit 1
fi

BINARY_NAME=mpc

if [ ! -f "$BUILD_FOLDER/$BINARY_NAME" ] ; then
    echo "Could not find binary \"$BINARY_NAME\" in $BUILD_FOLDER"
    echo "Make sure you build successfully with:"
    echo "$ ./build.sh"
    exit 1
fi

# Set Docker run base command
DOCKER_IMG_NAME=carlosgalvezp/sdcnd_p5
DOCKER_RUN_BASE="docker run --rm=true                             \
                            --volume=$SCRIPT_DIR:$SCRIPT_DIR      \
                            --workdir=$BUILD_FOLDER               \
                            --user=$UID:$GROUPS                   \
                            --network=host                        \
                            $DOCKER_IMG_NAME"

# Run binary from within Docker
$DOCKER_RUN_BASE ./$BINARY_NAME
