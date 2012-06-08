LIBDIR="/home/johannes/DAQ/lib64"

all: timedDAQ

timedDAQ: timedDAQ.c
	gcc  -Wall -pedantic -lhid timedDAQ.c $(LIBDIR)/pmd.o $(LIBDIR)/libusb-0.1.so.4 $(LIBDIR)/libmcchid.a -o timedDAQ
