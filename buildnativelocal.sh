#!/bin/bash

OS=`uname -s | tail -1`

VCPKG_TRIPLET=""
ARCH=""
ARCH_FLAGS=""

a="/$0"; a=${a%/*}; a=${a#/}; a=${a:-.}; BASEDIR=$(cd "$a"; pwd)

mkdir .vcpkg
git clone https://github.com/Microsoft/vcpkg.git ./.vcpkg/vcpkg --depth 1

if [ "$OS" = "Darwin" ];
then
    echo "MacOS"
    if [ "$1" = "x86_64" ]; then
        VCPKG_TRIPLET="x64-osx-release"
        ARCH="x86_64"
    elif [ "$1" = "arm64" ]; then
        VCPKG_TRIPLET="arm64-osx"
        ARCH="arm64"
    else
        ARCH=`uname -m | tail -1`
        if [ "$ARCH" = "x86_64" ]; then
            VCPKG_TRIPLET="x64-osx-release"
        elif [ "$ARCH" = "arm64" ]; then
            VCPKG_TRIPLET="arm64-osx"
        else
            echo "bad arch $ARCH a"
        fi
    fi

    ARCH_FLAGS="-DCMAKE_OSX_ARCHITECTURES=$ARCH"

else
    echo "Linux"
    sudo apt-get install -y gperf fontconfig libgles2-mesa-dev libgtk2.0-dev
    VCPKG_TRIPLET="x64-linux-release"
fi

./.vcpkg/vcpkg/bootstrap-vcpkg.sh

export VCPKG_NUGET_REPOSITORY="https://github.com/aardvark-community/MiniCV"
./.vcpkg/vcpkg/vcpkg install OpenCV --triplet $VCPKG_TRIPLET 

rm -dfr src/MiniCVNative/build
cmake -S src/MiniCVNative/ -B src/MiniCVNative/build $ARCH_FLAGS \
    -DCMAKE_TOOLCHAIN_FILE="$BASEDIR/.vcpkg/vcpkg/scripts/buildsystems/vcpkg.cmake" \
    -DVCPKG_TARGET_TRIPLET=$VCPKG_TRIPLET \
    -DCMAKE_BUILD_TYPE=Release

cmake --build src/MiniCVNative/build --config Release --target install