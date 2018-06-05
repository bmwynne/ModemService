CC = gcc
OBJ = mdm_service.o cellular.o gps.o power.o
CFLAGS = -Wall -c


libmodem.a: $(OBJ)
	ar rc $@ $(OBJ)

mdm_service.o: mdm_service.c mdm_service.h cellular.h gps.h power.h
	$(CC) $(CFLAGS) $<

cellular.o: cellular.c cellular.h 
	$(CC) $(CFLAGS) $<

gps.o: gps.c gps.h 
	$(CC) $(CFLAGS) $<

power.o: power.c power.h 
	$(CC) $(CFLAGS) $<

clean:
	rm *.o libmodem*

