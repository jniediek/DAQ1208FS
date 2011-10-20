#include <octave/oct.h>
#include <asm/types.h>
#include <stdbool.h>

extern "C" {
#include "../../include/pmd.h"
#include "../../include/usb-1208FS.h"
}

static HIDInterface * hid[4];

int init() {
	hid_return ret;
	int i;
	int interface;
	ret = hid_init();
	if(ret != HID_RET_SUCCESS) {
		error("hid_init failed with return code %i", ret);
		return -1;
	}

	for(i = 0; i <=3; i++) {
		interface = PMD_Find_Interface(&hid[i], i, USB1208FS_PID);
		if(interface < 0) {
			error("USB 1208FS not found.");
			return -1;
		}
		else {
			octave_stdout << "USB 1208FS Device found! Interface = "  <<  interface << ".\n";
		}
	}
	usbDConfigPort_USB1208FS(hid[0], DIO_PORTA, DIO_DIR_OUT);
	usbDConfigPort_USB1208FS(hid[0], DIO_PORTB, DIO_DIR_IN);
	usbDOut_USB1208FS(hid[0], DIO_PORTA, 0);
	usbDOut_USB1208FS(hid[0], DIO_PORTA, 0);
	return 0;
}

void writeData(unsigned int data) {
	usbDOut_USB1208FS(hid[0], DIO_PORTA, (__u8) data);
	octave_stdout << "Wrote " << data << "\n";
}

int deinit() {
	int i = 0;
	hid_return ret;
	for(i = 0; i <= 3; i++) {
		ret = hid_close(hid[i]);
		if( ret != HID_RET_SUCCESS) {
		error("hid_close failed with return code %i.", ret);
		return -1;
		}
		hid_delete_HIDInterface(&hid[i]);
	}
	
	ret = hid_cleanup();
	if(ret != HID_RET_SUCCESS) {
		error("hid_cleanup failed with return code %i.", ret);
		return -1;
	}
	return 0;
}

DEFUN_DLD(daq1208fs, args, nargout, "daq1208fs interface" ) {
	int nargin, retval;
	static int initialized = 0;
	charMatrix ch;
	retval = 0;

	nargin = args.length();

	if ( nargin < 1 ) {
		error("daq1208fs: input argument required.");
		return octave_value_list();
	}

	ch = args(0).char_matrix_value();

	if ( nargin == 2 && ch(0) == 'w' ) {
		if ( initialized )
		       if ( args(1).is_real_scalar() ) 
				writeData( (unsigned int) args(1).int_value() );
		       else {
			       error("input argument is not a scalar");
			       retval = -1;
		       }
		else { 
			error("USB 1208FS Device has not been initialized.\n");
			retval = -1;
		}
	}
	else if ( nargin == 1 && (ch(0) == 'i' || ch(0) == 'd') ) {
		switch(ch(0)) {
			case 'i':
				if ( !initialized ) {
					retval = init(); 
					if ( retval == 0 )
						initialized = 1;
				}
				else {
					error("USB 1208FS Device has already been initialized.\n");
					retval = -1;
				}
				break;
			case 'd' :
				if ( initialized ) {
					retval = deinit();
					if ( retval == 0 )
						initialized = 0;
				}
				else {
					error("USB 1208FS Device has not been initialized.\n");
					retval = -1;
				}
		}
	}
	else
		error("Usage:\ndaq1208fs('i') to initialize\ndaq1208fs('w', DATA) to write DATA\ndaq1208fs('d') to deinitialize");
	return octave_value(retval);
}
