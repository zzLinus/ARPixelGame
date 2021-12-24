CC = g++
CFLAGS = `pkg-config --cflags --libs opencv4` -lX11 -lGL -lpthread -lpng -lstdc++fs -g

ARGame: main.o
	${CC} ${CFLAGS} main.o -o ARGame

main.o: main.cpp
	${CC} ${CFLAGS} -c main.cpp
