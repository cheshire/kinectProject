CC = g++

CFLAGS = `pkg-config opencv --cflags --libs` -Wall

default: playground

playground: playground.cc
	$(CC) $(CFLAGS) -o playground.out playground.cc
