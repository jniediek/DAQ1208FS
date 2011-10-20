#include <Python.h>
#include <asm/types.h>

#include "pmd.h"
#include "usb-1208FS.h"

static HIDInterface * hid[4];

static PyObject * usb1208fs_dset(PyObject *self, PyObject *args) {
	unsigned short data;
	if(!PyArg_ParseTuple(args, "i", &data))
		return NULL;
	usbDOut_USB1208FS(hid[0], DIO_PORTA, (__u8) data);
	printf("Wrote %x\n", data);
	return Py_BuildValue("i",0);
}

static PyObject * usb1208fs_init(PyObject *self, PyObject *args) {
	hid_return ret;
	int i;
	int interface;
	ret = hid_init();
	if(ret != HID_RET_SUCCESS) {
		fprintf(stderr, "hid_init failed with return code %d\n", ret);
		return Py_BuildValue("i",-1);
	}

	for(i = 0; i <=3; i++) {
		interface = PMD_Find_Interface(&hid[i], i, USB1208FS_PID);
		if(interface < 0) {
			fprintf(stderr, "USB 1208FS not found.\n");
			exit(1);
		}
		else {
			printf("USB 1208FS Device found! Interface = %d\n", interface);
		}
	}

	usbDConfigPort_USB1208FS(hid[0], DIO_PORTA, DIO_DIR_OUT);
	usbDConfigPort_USB1208FS(hid[0], DIO_PORTB, DIO_DIR_IN);
	usbDOut_USB1208FS(hid[0], DIO_PORTA, 0);
	usbDOut_USB1208FS(hid[0], DIO_PORTA, 0);
	return Py_BuildValue("i", 0);
}

static PyObject * usb1208fs_deinit(PyObject *self, PyObject *args) {
	int i = 0;
	hid_return ret;
	for(i = 0; i <= 3; i++) {
		ret = hid_close(hid[i]);
		if( ret != HID_RET_SUCCESS) {
			fprintf(stderr, "hid_close failed with return code %d\n", ret);
			return Py_BuildValue("i",1);
		}
		hid_delete_HIDInterface(&hid[i]);
	}
	
	ret = hid_cleanup();
	if(ret != HID_RET_SUCCESS) {
		fprintf(stderr, "hid_cleanup failed with return code %d\n", ret);
		return Py_BuildValue("i",1);
	}
	return Py_BuildValue("i", 0);
}

static PyMethodDef usb1208fsMethods[] = {
	{ "dset", usb1208fs_dset, METH_VARARGS, "Set digital ports on DAQ device device."},
	{ "init", usb1208fs_init, METH_VARARGS, "Initialize DAQ device."},
	{ "deinit", usb1208fs_deinit, METH_VARARGS, "Deinitialize DAQ device."},
	{NULL, NULL, 0 ,NULL}
};

PyMODINIT_FUNC initusb1208fs(void) {
	(void) Py_InitModule("usb1208fs", usb1208fsMethods);
}
