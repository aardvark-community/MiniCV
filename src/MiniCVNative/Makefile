all: deploy

deploy: MiniCVNative.dylib
	cp MiniCVNative.dylib ../../lib/Native/MiniCV/mac/AMD64/MiniCVNative.dylib

MiniCVNative.o: MiniCVNative.cpp MiniCVNative.h
	g++ -std=c++11 -fPIC -c MiniCVNative.cpp -I/Users/atti/proj/MiniCV/include -o MiniCVNative.o

fivepoint.o: fivepoint.cpp fivepoint.h
	g++ -std=c++11 -fPIC -c fivepoint.cpp -I/Users/atti/proj/MiniCV/include -o fivepoint.o

ap3p.o: ap3p.cpp ap3p.h
	g++ -std=c++11 -fPIC -c ap3p.cpp -I/Users/atti/proj/MiniCV/include -o ap3p.o

MiniCVNative.dylib: MiniCVNative.o fivepoint.o ap3p.o
	g++ MiniCVNative.o fivepoint.o ap3p.o -shared -o MiniCVNative.dylib ../../lib/Native/MiniCV/mac/AMD64/libopencv_features2d.3.4.9.dylib ../../lib/Native/MiniCV/mac/AMD64/libopencv_core.3.4.9.dylib ../../lib/Native/MiniCV/mac/AMD64/libopencv_calib3d.3.4.9.dylib ../../lib/Native/MiniCV/mac/AMD64/libopencv_imgproc.3.4.9.dylib ../../lib/Native/MiniCV/mac/AMD64/libopencv_imgcodecs.3.4.9.dylib

.PHONY clean:
	rm -fr *.o MiniCVNative.dylib