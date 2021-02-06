
CC = clang++
STANDARD = -std=c++11
LIBRARY = -stdlib=libc++
INCLUDE = -I.
CFLAGS = -g -Wall
LDFLAGS = -L. -L/usr/lib
LDLIBS = -lc -lm

.c.o:
		$(CC) $(STANDARD) $(LIBRARY) $(INCLUDE) $(CFLAGS) -c $<

all: main

main: main.o
		$(CC) -o $@ $^ $(LDFLAGS) $(LDLIBS)
		
clean:
		rm -f *.o
		rm -f main