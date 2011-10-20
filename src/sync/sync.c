#include <asm/types.h>
#include "pmd.h"
#include "usb-1208FS.h"

static HIDInterface * hid[4];
int debug = 1;

void usb1208fs_dset(unsigned short data) {
	usbDOut_USB1208FS(hid[0], DIO_PORTA, (__u8) data);
	if(debug)
		printf("wrote %x\n", data);
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

int main(int argc, char *argv[]) {
	usb1208fs_init();
	usb1208fs_dset(12);
	usb1208fs_deinit();
	return 0;
}
