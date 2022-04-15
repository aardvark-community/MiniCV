@echo off

.config/nuget.exe setApiKey -Source GitHub $GITHUB_TOKEN -NonInteractive
cmd /C "rmdir /S /Q .vcpkg"

git clone https://github.com/Microsoft/vcpkg.git ./.vcpkg/vcpkg --depth 1
copy .vcpkg\vcpkg\triplets\community\x64-windows-static-md.cmake .vcpkg\vcpkg\triplets\community\x64-windows-static-md-rel.cmake
echo set(VCPKG_BUILD_TYPE release) >> .vcpkg\vcpkg\triplets\community\x64-windows-static-md-rel.cmake
cmd /C ".vcpkg\vcpkg\bootstrap-vcpkg.bat -disableMetrics"


SET VCPKG_NUGET_REPOSITORY=https://github.com/aardvark-community/MiniCV
.vcpkg\vcpkg\vcpkg.exe install OpenCV[nonfree] --triplet x64-windows-static-md-rel --binarysource="clear;nuget,Github,readwrite;nugettimeout,1000"

cmake -S src\MiniCVNative -B src\MiniCVNative\build -DCMAKE_TOOLCHAIN_FILE="%~dp0\.vcpkg\vcpkg\scripts\buildsystems\vcpkg.cmake" -DVCPKG_TARGET_TRIPLET=x64-windows-static-md-rel -DCMAKE_BUILD_TYPE=Release
cmake --build src\MiniCVNative\build --config Release --target install