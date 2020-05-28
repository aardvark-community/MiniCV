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
	g++ -shared -o MiniCVNative.dylib ap3p.o fivepoint.o MiniCVNative.o \
	-L/usr/local/Cellar/opencv@3/3.4.10_2/lib \
	-std=c++11 \
	-exported_symbols_list exports.txt \
	-Wl,-exported_symbols_list,exports.txt \
	-fdata-sections -ffunction-sections \
	-dead_strip \
	-Wl,-dead_strip \
	-dead_strip_dylibs \
	-lpthread \
	-lz \
	-framework OpenCL \
	/usr/local/Cellar/opencv@3/3.4.10_2/lib/libippicv.a \
	/usr/local/Cellar/opencv@3/3.4.10_2/lib/libippiw.a \
	/usr/local/Cellar/opencv@3/3.4.10_2/lib/libopencv_calib3d.a \
	/usr/local/Cellar/opencv@3/3.4.10_2/lib/libopencv_core.a \
	/usr/local/Cellar/opencv@3/3.4.10_2/lib/libopencv_features2d.a \
	/usr/local/Cellar/opencv@3/3.4.10_2/lib/libopencv_imgproc.a \
	/usr/local/Cellar/libomp/10.0.0/lib/libomp.a \
	/usr/local/Cellar/gcc/9.3.0_1/lib/gcc/9/gcc/x86_64-apple-darwin19/9.3.0/libgcc.a \
	/usr/local/Cellar/gcc/9.3.0_1/lib/gcc/9/libquadmath.a \
	/usr/local/Cellar/gcc/9.3.0_1/lib/gcc/9/libgfortran.a \
	/usr/local/Cellar/openblas/0.3.9/lib/libopenblas.a \
	/usr/local/Cellar/tbb/2020_U2/lib/libtbb.a \
	/usr/local/Cellar/tbb/2020_U2/lib/libtbbmalloc.a 

.PHONY clean:
	rm -fr *.o MiniCVNative.dylib