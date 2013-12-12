CC = g++
ifeq ($(shell sw_vers 2>/dev/null | grep Mac | awk '{ print $$2}'),Mac)
	CFLAGS = -g -I./include/ -I/usr/X11/include -I/usr/local/include -I/usr/local/include/eigen3 -IirrKlang-1.4.0/include -DOSX -Wno-deprecated-declarations -O2
	LDFLAGS = -lm -lstdc++ -framework GLUT -framework OpenGL \
	 	-L"/System/Library/Frameworks/OpenGL.framework/Libraries" \
	 	-lfreeimage ./FreeImage/libfreeimage.a -L./FreeImage \
	 	-L"/usr/lib" irrKlang-1.4.0/bin/linux-gcc/libIrrKlang.so -pthread
else
	CFLAGS = -g -DGL_GLEXT_PROTOTYPES -Iglut-3.7.6-bin\
		-IirrKlang-1.4.0/include -I/usr/local/include -I/usr/include -I/home/ff/cs184/eigen -I/usr/include/eigen3 -pthread -O3 
	LDFLAGS = -ljpeg -lGL -lGLU -lm -lstdc++ -lglut -lfreeimage -L./FreeImage -L"/usr/lib" irrKlang-1.4.0/bin/linux-gcc/libIrrKlang.so -pthread

endif

RM = /bin/rm -f 
all: main
main: main.o 
	$(CC) $(CFLAGS) -o main main.o $(LDFLAGS)
main.o: main.cpp
	$(CC) $(CFLAGS) -c main.cpp -o main.o
clean: 
	$(RM) *.o main
