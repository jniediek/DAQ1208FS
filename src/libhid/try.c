#include "pmd.h"
#include "usb-1208FS.h"

int main(int argc, char * argv[]) {
	hid_return ret;
	int i;
	HIDInterface* hid[4];
	
	ret = hid_init();
	if (ret != HID_RET_SUCCESS) {
		fprintf(stderr, "hid_init failed: %d\n", ret);
		return -1;
	}

	for (i = 0; i <= 3; i++ ) {
		interface = PMD_Find_Interface(&hid[i], i, USB1208FS_PID);
		if(interface < 0) {
			fprintf(stderr, "USB 1208FS not found.\n");
			exit(1);
		}
		else {
			printf("USB 1208FS found. Interface = %d\n", interface);
		}
	}

	usbDConfigPort_USB1208FS(hid[0], DIO_PORTA, DIO_DIR_OUT);
	usbDOut_USB1208FS(hid[0], DIO_PORTA, 0);
	usbDOut_USB1208FS(hid[0], DIO_PORTA, 0);


