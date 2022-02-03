@echo off

cmd /C "rmdir /S /Q .vcpkg"

git clone https://github.com/Microsoft/vcpkg.git ./.vcpkg/vcpkg --depth 1
echo set(VCPKG_BUILD_TYPE release) >> .vcpkg\vcpkg\triplets\community\x64-windows-static-md.cmake
cmd /C ".vcpkg\vcpkg\bootstrap-vcpkg.bat -disableMetrics"


SET VCPKG_NUGET_REPOSITORY="https://github.com/aardvark-community/MiniCV"
.vcpkg\vcpkg\vcpkg.exe install OpenCV --triplet x64-windows-static-md --binarysource='clear;nuget,Github,readwrite'

cmake -S src\MiniCVNative -B src\MiniCVNative\build -DCMAKE_TOOLCHAIN_FILE="%~dp0\.vcpkg\vcpkg\scripts\buildsystems\vcpkg.cmake" -DVCPKG_TARGET_TRIPLET=x64-windows-static-md -DCMAKE_BUILD_TYPE=Release
cmake --build src\MiniCVNative\build --config Release --target install