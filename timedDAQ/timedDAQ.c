#include <asm/types.h>
#include <sys/time.h>
#include <signal.h>
#include <unistd.h>
#include <stdio.h>
#include "pmd.h"
#include "usb-1208FS.h"

static HIDInterface * hid[4];
int debug = 1;

void usb1208fs_dset(unsigned short data) {
	usbDOut_USB1208FS(hid[0], DIO_PORTA, (__u8) data);
	usbDOut_USB1208FS(hid[0], DIO_PORTA, 0);
	if(debug)
		printf("%u\t", data);
}

int usb1208fs_init() {
	hid_return ret;
	int i;
	int interface;
	ret = hid_init();
	if(ret != HID_RET_SUCCESS) {
		fprintf(stderr, "hid_init failed with return code %d\n", ret);
		return -1;
	}

	for(i = 0; i <=3; i++) {
		interface = PMD_Find_Interface(&hid[i], i, USB1208FS_PID);
		if(interface < 0) {
			fprintf(stderr, "USB 1208FS not found.\n");
			return -1;
		}
		else {
			if(debug) 
				printf("USB 1208FS Device found! Interface = %d\n", interface);
		}
	}

	usbDConfigPort_USB1208FS(hid[0], DIO_PORTA, DIO_DIR_OUT);
	usbDConfigPort_USB1208FS(hid[0], DIO_PORTB, DIO_DIR_IN);
	usbDOut_USB1208FS(hid[0], DIO_PORTA, 0);
	usbDOut_USB1208FS(hid[0], DIO_PORTA, 0);
	return 0;
}

int usb1208fs_deinit() {
	int i = 0;
	hid_return ret;
	for(i = 0; i <= 3; i++) {
		ret = hid_close(hid[i]);
		if( ret != HID_RET_SUCCESS) {
			fprintf(stderr, "hid_close failed with return code %d\n", ret);
			return -1;
		}
		hid_delete_HIDInterface(&hid[i]);
	}
	
	ret = hid_cleanup();
	if(ret != HID_RET_SUCCESS) {
		fprintf(stderr, "hid_cleanup failed with return code %d\n", ret);
		return -1;
	}	
	return 0;
}

void handler(int signal) {
	printf("Closing...\n");
	usb1208fs_deinit();
	exit(0);
}

int main(int argc, char *argv[]) {
	struct sigaction act;
	unsigned short z[] = {1,2,4,8,16,32,64,128};
	unsigned short sig = 255;
	unsigned int numStamps;
	struct timeval tv1, tv2;
	long t1, t2;
	int i;

/* This sets up the correct deinitialization procedure */
	act.sa_handler = handler;
	sigaction(SIGINT, &act, NULL);
	sigaction(SIGTERM, &act, NULL);

/* Initialize */	
	usb1208fs_init();
	printf("Initial signature: %u\t%u\n", sig, sig);
	usb1208fs_dset(sig);
	usb1208fs_dset(sig);

/* Write, make a protocol */
	i = 0;
/*Stamp counting starts with 0 */
	numStamps = 0;	
	if(debug)
		printf("\ndata\t");
	printf("Stamp\tpre\tpost\n");
	while(1) {
		gettimeofday(&tv1, NULL);
		usb1208fs_dset(z[i]);
		gettimeofday(&tv2, NULL);
		t1 = tv1.tv_sec * 1000000 + tv1.tv_usec;
		t2 = tv2.tv_sec * 1000000 + tv2.tv_usec;
		printf("%i\t%lu\t%lu\n", numStamps, t1, t2);
		numStamps++;
		i++;
		i %= 8;
		gettimeofday(&tv2, NULL);
		t2 = (1 - tv2.tv_sec) * 1000000 - tv2.tv_usec + t1;
		if(t2 > 0)
			usleep(t2);
	}
	return 0;
}
