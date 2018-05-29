all: main.c lib
	gcc -Wall -o test main.c -L /mnt/c/Users/Will/Documents/MdmLib/ -l mdmlib

lib: mdm_service.o cellular.o gps.o power.o
	ar rc libmdmlib.a mdm_service.o cellular.o gps.o power.o

mdm_service.o: mdm_service.c mdm_service.h cellular.h gps.h power.h mdm_util.h
	gcc -Wall -c mdm_service.c

cellular.o: cellular.c cellular.h mdm_util.h
	gcc -Wall -c cellular.c

gps.o: gps.c gps.h mdm_util.h
	gcc -Wall -c gps.c

power.o: power.c power.h mdm_util.h
	gcc -Wall -c power.c

clean:
	rm *.o

