#include "mex.h"
#include <stdlib.h>
#include <stdio.h>
#include <asm/types.h>

#include "pmd.h"
#include "usb-1208FS.h"

int init(HIDInterface * hid[]) {
	hid_return ret;
	int i;
	int interface;
	ret = hid_init();
	if(ret != HID_RET_SUCCESS) {
		mexErrMsgTxt("hid_init failed!\n");
		return -1;
	}

	for(i = 0; i <=3; i++) {
		interface = PMD_Find_Interface(&hid[i], i, USB1208FS_PID);
		if(interface < 0) {
			fprintf(stderr, "USB 1208FS not found.\n");
			exit(1);
		}
		else {
			mexPrintf("USB 1208FS Device found! Interface = %d\n", interface);
			/*mexMakeMemoryPersistent(&hid[i]);*/
		}
	}
	usbDConfigPort_USB1208FS(hid[0], DIO_PORTA, DIO_DIR_OUT);
	usbDConfigPort_USB1208FS(hid[0], DIO_PORTB, DIO_DIR_IN);
	usbDOut_USB1208FS(hid[0], DIO_PORTA, 0);
	usbDOut_USB1208FS(hid[0], DIO_PORTA, 0);
	return 0;
}

void writeData(HIDInterface * hid[], unsigned int data) {
	usbDOut_USB1208FS(hid[0], DIO_PORTA, (__u8) data);
	mexPrintf("Wrote %x\n", data);
}

int deinit(HIDInterface * hid[]) {
	int i = 0;
	hid_return ret;
	for(i = 0; i <= 3; i++) {
		ret = hid_close(hid[i]);
		if( ret != HID_RET_SUCCESS) {
			fprintf(stderr, "hid_close failed with return code %d\n", ret);
			return 1;
		}
		hid_delete_HIDInterface(&hid[i]);
	}
	
	ret = hid_cleanup();
	if(ret != HID_RET_SUCCESS) {
		fprintf(stderr, "hid_cleanup failed with return code %d\n", ret);
		return 1;
	}
	return 0;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) {
	HIDInterface * hid[4];
	unsigned int temp;
	init(hid);	
	mexPrintf("Enter data... ");
	writeData(hid, 0x12);
	deinit(hid);
	return;
}


