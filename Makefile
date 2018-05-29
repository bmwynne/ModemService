CC = gcc
OBJ = main.o
HEADER = include/header.h 
CFLAGS = -Wall -c

INCLUDE_DIR=./osx


lib: mdm_service.o cellular.o gps.o power.o
	ar rc libmodemlib.a mdm_service.o cellular.o gps.o power.o

mdm_service.o: mdm_service.c mdm_service.h cellular.h gps.h power.h mdm_util.h
	$(CC) $(CFLAGS) $<

cellular.o: cellular.c cellular.h mdm_util.h
	$(CC) $(CFLAGS) $<

gps.o: gps.c gps.h mdm_util.h
	$(CC) $(CFLAGS) $<

power.o: power.c power.h mdm_util.h
	$(CC) $(CFLAGS) $<

clean:
	rm *.o
